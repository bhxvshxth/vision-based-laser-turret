#include "config.h"
#if ENABLE_WEB_DEBUG
#include <WiFi.h>
#include <WebServer.h>
#include "esp_camera.h"
#include "img_converters.h"
#include "web_debug.h"
#include "turret_state.h"
#include "rangefinder.h"

#if __has_include("secrets.h")
#include "secrets.h"
#endif
#ifndef WIFI_SSID
#define WIFI_SSID ""
#define WIFI_PASS ""
#endif

static WebServer server(80);

static const char PAGE[] = R"HTML(<!doctype html>
<html><head><meta name="viewport" content="width=device-width,initial-scale=1">
<title>turret</title>
<style>
body{margin:0;background:#000;color:#aaa;font:12px monospace;display:flex;flex-direction:column;align-items:center;padding-top:24px}
img{width:640px;max-width:96vw;image-rendering:pixelated;border:1px solid #222;background:#111}
#s{width:640px;max-width:96vw;margin-top:8px;white-space:pre}
button{background:#000;color:#aaa;border:1px solid #333;font:12px monospace;padding:4px 10px;margin:8px 4px 0 0;cursor:pointer}
</style></head><body>
<img id="v">
<div id="s"></div>
<div><button onclick="c('calibrate')">calibrate</button><button onclick="c('arm')">arm</button><button onclick="c('disarm')">disarm</button><button onclick="c('reset')">reset bias</button><button onclick="c('clearcal')">clear cal</button></div>
<script>
const v=document.getElementById('v'),s=document.getElementById('s');
function next(){v.src='/snapshot.jpg?'+Date.now()}
v.onload=()=>setTimeout(next,100);v.onerror=()=>setTimeout(next,1000);next();
function f(n){return n==null?'--':n.toFixed(1)}
async function poll(){try{const j=await (await fetch('/status')).json();
s.textContent=(j.armed?'ARMED':'SAFE')+'  laser '+(j.laser?'on':'off')+'  '+(j.locked?'LOCKED':'')+'  cal '+(j.calibrated?'sweep':'default')+'  '+f(j.fps)+' fps\n'+
'ball '+(j.ball?f(j.ball.x)+','+f(j.ball.y):'--')+'   dot '+(j.dot?f(j.dot.x)+','+f(j.dot.y):'--')+'   err '+(j.err<0?'--':f(j.err))+' px\n'+
'pan '+f(j.pan)+'  tilt '+f(j.tilt)+'  bias '+f(j.biasPan)+','+f(j.biasTilt)+'\n'+
'range '+(j.range==null?'--':f(j.range)+' cm')+' ('+j.rangeSensor+')'+(j.target?'   target x '+f(j.target.x)+' y '+f(j.target.y)+' z '+f(j.target.z)+' cm':'');
}catch(e){}setTimeout(poll,250)}
poll();
function c(x){fetch('/cmd?c='+x)}
</script></body></html>)HTML";

static void handleRoot() { server.send(200, "text/html", PAGE); }

static void handleSnapshot() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) { server.send(503, "text/plain", "no frame"); return; }
  Detections d = visionDetect(fb);
  visionDrawOverlay(fb, d);
  uint8_t* jpg = nullptr;
  size_t len = 0;
  bool ok = frame2jpg(fb, 80, &jpg, &len);
  esp_camera_fb_return(fb);
  if (!ok) { server.send(500, "text/plain", "jpeg failed"); return; }
  server.sendHeader("Cache-Control", "no-store");
  server.send_P(200, "image/jpeg", (const char*)jpg, len);
  free(jpg);
}

static void handleStatus() {
  const TurretState& t = turret;
  char ball[64] = "null", dot[64] = "null", range[24] = "null", target[96] = "null";
  if (t.det.ball.found)  snprintf(ball, sizeof(ball), "{\"x\":%.1f,\"y\":%.1f,\"area\":%d}", t.det.ball.x, t.det.ball.y, t.det.ball.area);
  if (t.det.laser.found) snprintf(dot,  sizeof(dot),  "{\"x\":%.1f,\"y\":%.1f}", t.det.laser.x, t.det.laser.y);
  if (rangeValid()) {
    float d = rangeCm();
    snprintf(range, sizeof(range), "%.1f", d);
    if (t.locked) {
      // Position of the ball relative to the turret pivot, from the aim angles + distance.
      float az = radians(t.pan - PAN_HOME_DEG), el = radians(t.tilt - TILT_HOME_DEG);
      snprintf(target, sizeof(target), "{\"x\":%.1f,\"y\":%.1f,\"z\":%.1f}",
               d * cosf(el) * sinf(az), d * sinf(el), d * cosf(el) * cosf(az));
    }
  }
  char buf[640];
  snprintf(buf, sizeof(buf),
    "{\"armed\":%s,\"laser\":%s,\"locked\":%s,\"calibrated\":%s,\"fps\":%.1f,"
    "\"pan\":%.2f,\"tilt\":%.2f,\"biasPan\":%.2f,\"biasTilt\":%.2f,\"err\":%.1f,"
    "\"ball\":%s,\"dot\":%s,\"range\":%s,\"rangeSensor\":\"%s\",\"target\":%s}",
    t.armed ? "true" : "false", t.laserOn ? "true" : "false", t.locked ? "true" : "false",
    cal.fromSweep ? "true" : "false", t.fps, t.pan, t.tilt, t.biasPan, t.biasTilt, t.errPx,
    ball, dot, range, rangeName(), target);
  server.send(200, "application/json", buf);
}

static void handleCmd() {
  String c = server.arg("c");
  if (c == "calibrate")     turret.reqCalibrate = true;
  else if (c == "arm")      turret.armed = true;
  else if (c == "disarm")   turret.armed = false;
  else if (c == "reset")    turret.reqResetBias = true;
  else if (c == "clearcal") turret.reqClearCal = true;
  else { server.send(400, "text/plain", "unknown command"); return; }
  server.send(200, "text/plain", "ok");
}

void webDebugBegin() {
  bool connected = false;
  if (strlen(WIFI_SSID) > 0) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.printf("[wifi] connecting to %s", WIFI_SSID);
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) { delay(250); Serial.print("."); }
    Serial.println();
    connected = WiFi.status() == WL_CONNECTED;
  }
  IPAddress ip;
  if (connected) {
    ip = WiFi.localIP();
  } else {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    ip = WiFi.softAPIP();
    Serial.printf("[wifi] started access point \"%s\" (password %s)\n", AP_SSID, AP_PASS);
  }
  WiFi.setSleep(false);
  server.on("/", handleRoot);
  server.on("/snapshot.jpg", handleSnapshot);
  server.on("/status", handleStatus);
  server.on("/cmd", handleCmd);
  server.begin();
  Serial.printf("[wifi] debug page: http://%s/\n", ip.toString().c_str());
}

void webDebugLoop() { server.handleClient(); }

#endif
