// Laptop demo. Uses the laptop camera as the turret's fixed camera: click any
// object to make it the target, and a simulated pan/tilt laser chases it.
// Tracking runs on a 160x120 copy of each frame with the same detection and
// control logic as the ESP32 firmware (tracker.js).
// No camera / permission denied -> falls back to a still frame with a
// draggable tennis ball.
(() => {
  const DW = 640, DH = 480;          // display
  const PW = 160, PH = 120;          // processing (matches FRAMESIZE_QQVGA)
  const S = DW / PW;
  const BALL_R = 26;
  const TICK_MS = 50;                // ~20 fps, roughly what the ESP32-CAM manages
  const SERVO_DEG_PER_S = 400;       // SG90 ~0.1s / 60deg

  const cam = document.getElementById('cam');
  const ctx = cam.getContext('2d');
  const proc = document.createElement('canvas');
  proc.width = PW; proc.height = PH;
  const pctx = proc.getContext('2d', { willReadFrequently: true });

  let mode = 'starting';             // 'starting' | 'camera' | 'still'
  let target = { ...Tracker.DEFAULT_TARGET };
  let targetColor = null;            // swatch of the picked colour
  let message = '';                  // short OSD notice
  let messageUntil = 0;
  let cleanFrame = null;             // last processing frame without the laser

  // ---------------- camera ----------------
  const video = document.createElement('video');
  video.muted = true; video.playsInline = true;

  async function startCamera() {
    try {
      const stream = await navigator.mediaDevices.getUserMedia({
        video: { width: { ideal: 640 }, height: { ideal: 480 }, facingMode: 'user' }, audio: false,
      });
      video.srcObject = stream;
      await video.play();
      mode = 'camera';
      notice('click an object to make it the target', 4000);
    } catch (e) {
      mode = 'still';
      notice('no camera - using a still frame', 4000);
    }
  }

  // Draw the video cropped to 4:3 and mirrored (like a mirror, so clicks feel natural).
  function drawVideo(g, w, h) {
    const vw = video.videoWidth, vh = video.videoHeight;
    if (!vw) return;
    const k = Math.max(w / vw, h / vh), sw = w / k, sh = h / k;
    g.save();
    g.translate(w, 0); g.scale(-1, 1);
    g.drawImage(video, (vw - sw) / 2, (vh - sh) / 2, sw, sh, 0, 0, w, h);
    g.restore();
  }

  // ---------------- still scene (fallback) ----------------
  const scene = document.createElement('canvas');
  scene.width = DW; scene.height = DH;
  const sctx = scene.getContext('2d');

  function drawDefaultScene() {
    const g = sctx;
    let gr = g.createLinearGradient(0, 0, DW, 340);
    gr.addColorStop(0, '#8a867f'); gr.addColorStop(1, '#67635d');
    g.fillStyle = gr; g.fillRect(0, 0, DW, 345);

    gr = g.createLinearGradient(0, 345, 0, DH);
    gr.addColorStop(0, '#514b45'); gr.addColorStop(1, '#3a3530');
    g.fillStyle = gr; g.fillRect(0, 345, DW, DH - 345);
    g.strokeStyle = 'rgba(0,0,0,0.12)'; g.lineWidth = 1;
    for (let i = -6; i <= 6; i++) {
      g.beginPath(); g.moveTo(DW / 2 + i * 40, 345); g.lineTo(DW / 2 + i * 150, DH); g.stroke();
    }
    g.fillStyle = '#8f8b84'; g.fillRect(0, 336, DW, 10);

    g.fillStyle = '#5d5852'; g.fillRect(470, 70, 116, 268);
    g.strokeStyle = '#9a958d'; g.lineWidth = 8; g.strokeRect(466, 66, 124, 274);
    g.fillStyle = '#8a857d'; g.fillRect(568, 205, 8, 4);

    g.fillStyle = '#3a3531'; g.fillRect(56, 168, 150, 172);
    g.fillStyle = '#2b2724';
    for (const y of [212, 258, 304]) g.fillRect(62, y, 138, 4);
    const books = [['#5b5f66', 8], ['#6b5d55', 11], ['#4f5a55', 7], ['#76726b', 9], ['#5a524d', 12]];
    let x = 66;
    for (const [c, w] of books) { g.fillStyle = c; g.fillRect(x, 222, w, 36); x += w + 2; }
    x = 70;
    for (const [c, w] of books.slice().reverse()) { g.fillStyle = c; g.fillRect(x, 270, w, 34); x += w + 3; }

    gr = g.createRadialGradient(240, 90, 10, 240, 90, 420);
    gr.addColorStop(0, 'rgba(255,255,255,0.10)'); gr.addColorStop(1, 'rgba(255,255,255,0)');
    g.fillStyle = gr; g.fillRect(0, 0, DW, DH);
    finishScene();
  }

  function finishScene() {
    const g = sctx;
    const v = g.createRadialGradient(DW / 2, DH / 2, DH * 0.35, DW / 2, DH / 2, DW * 0.62);
    v.addColorStop(0, 'rgba(0,0,0,0)'); v.addColorStop(1, 'rgba(0,0,0,0.45)');
    g.fillStyle = v; g.fillRect(0, 0, DW, DH);
    const img = g.getImageData(0, 0, DW, DH), d = img.data;
    for (let i = 0; i < d.length; i += 4) {
      const n = (Math.random() - 0.5) * 14;
      d[i] += n; d[i + 1] += n; d[i + 2] += n;
    }
    g.putImageData(img, 0, 0);
  }

  function useImage(img) {
    const k = Math.max(DW / img.width, DH / img.height);
    const w = img.width * k, h = img.height * k;
    sctx.drawImage(img, (DW - w) / 2, (DH - h) / 2, w, h);
    finishScene();
  }

  const ball = { x: DW * 0.62, y: DH * 0.48 };

  // ---------------- turret model ----------------
  // The simulated turret sits a little off from the camera and isn't
  // perfectly aligned, so the controller's rough map is wrong. The dot
  // feedback fixes it - same as on the real hardware.
  const F = (DW / 2) / Math.tan(33 * Math.PI / 180);   // ~66deg HFOV
  function laserPixel(pan, tilt) {
    const az = (pan - 90) * Math.PI / 180 + 0.035;
    const el = (tilt - 90) * Math.PI / 180 - 0.02;
    return [DW / 2 + F * Math.tan(az) + 12, DH / 2 + F * Math.tan(el) / Math.cos(az) + 8];
  }

  const ctl = new Tracker.Controller(PW, PH);
  const servo = { pan: 90, tilt: 90 };
  let det = { ball: { found: false }, laser: { found: false } };

  // ---------------- drawing ----------------
  function drawBall(g, x, y, r) {
    g.fillStyle = 'rgba(0,0,0,0.25)';
    g.beginPath(); g.ellipse(x + r * 0.25, y + r * 0.9, r * 0.9, r * 0.3, 0, 0, Math.PI * 2); g.fill();
    const gr = g.createRadialGradient(x - r * 0.35, y - r * 0.4, r * 0.1, x, y, r);
    gr.addColorStop(0, '#e4f06a'); gr.addColorStop(0.7, '#cfe03f'); gr.addColorStop(1, '#a4b42a');
    g.fillStyle = gr;
    g.beginPath(); g.arc(x, y, r, 0, Math.PI * 2); g.fill();
    if (r > 8) {
      g.save();
      g.beginPath(); g.arc(x, y, r, 0, Math.PI * 2); g.clip();
      g.strokeStyle = 'rgba(245,248,225,0.85)'; g.lineWidth = r * 0.07;
      g.beginPath(); g.arc(x - r * 1.15, y, r * 0.85, -0.9, 0.9); g.stroke();
      g.beginPath(); g.arc(x + r * 1.15, y, r * 0.85, Math.PI - 0.9, Math.PI + 0.9); g.stroke();
      g.restore();
    }
  }

  function drawLaser(g, x, y, scale) {
    const gr = g.createRadialGradient(x, y, 0, x, y, 11 * scale);
    gr.addColorStop(0, 'rgba(255,60,40,0.55)'); gr.addColorStop(1, 'rgba(255,0,0,0)');
    g.fillStyle = gr;
    g.beginPath(); g.arc(x, y, 11 * scale, 0, Math.PI * 2); g.fill();
    g.fillStyle = 'rgb(255,35,30)';
    g.beginPath(); g.arc(x, y, Math.max(1.4, 5 * scale), 0, Math.PI * 2); g.fill();
    g.fillStyle = 'rgba(255,235,230,0.9)';
    g.beginPath(); g.arc(x, y, 1.8 * scale, 0, Math.PI * 2); g.fill();
  }

  function drawFrame(g, w, h) {
    if (mode === 'camera') drawVideo(g, w, h);
    else {
      g.drawImage(scene, 0, 0, w, h);
      if (mode === 'still') drawBall(g, ball.x * w / DW, ball.y * h / DH, BALL_R * w / DW);
    }
  }

  function brackets(g, x, y, r) {
    const k = r * 0.45;
    g.beginPath();
    for (const [sx, sy] of [[-1, -1], [1, -1], [1, 1], [-1, 1]]) {
      const cx = x + sx * r, cy = y + sy * r;
      g.moveTo(cx, cy - sy * k); g.lineTo(cx, cy); g.lineTo(cx - sx * k, cy);
    }
    g.stroke();
  }

  const RESET_BTN = { x: DW - 12 - 96, y: 26, w: 96, h: 18 };
  let hoverReset = false;
  const inReset = (x, y) => x >= RESET_BTN.x && x <= RESET_BTN.x + RESET_BTN.w && y >= RESET_BTN.y && y <= RESET_BTN.y + RESET_BTN.h;

  function resetTarget() {
    target = { ...Tracker.DEFAULT_TARGET };
    targetColor = null;
    ctl.resetBias();
    notice('target reset to tennis ball', 1500);
  }

  function notice(text, ms) { message = text; messageUntil = performance.now() + ms; }

  const pad = (n) => String(n).padStart(2, '0');
  function osd(g, now) {
    g.font = '11px ui-monospace, Menlo, monospace';
    g.fillStyle = 'rgba(225,225,225,0.85)';
    g.textBaseline = 'top';
    g.textAlign = 'left';
    g.fillText('CAM0  160x120', 12, 10);
    const t = new Date();
    g.textAlign = 'right';
    g.fillText(`${t.getFullYear()}-${pad(t.getMonth() + 1)}-${pad(t.getDate())} ${pad(t.getHours())}:${pad(t.getMinutes())}:${pad(t.getSeconds())}`, DW - 12, 10);

    g.textBaseline = 'bottom';
    g.textAlign = 'left';
    const state = !det.ball.found ? 'SEARCH' : ctl.locked ? 'LOCK' : 'TRACK';
    const err = ctl.errPx < 0 ? '--' : ctl.locked && det.dotOnTarget ? 'ON' : (ctl.errPx * S).toFixed(1);
    g.fillText(`${state.padEnd(7)}PAN ${servo.pan.toFixed(1).padStart(5)}  TILT ${servo.tilt.toFixed(1).padStart(5)}  ERR ${err}`, 12, DH - 10);

    g.textAlign = 'right';
    g.fillStyle = 'rgba(200,200,200,0.45)';
    let hint = mode === 'camera' ? 'click an object to track it' : 'drag the ball / drop an image';
    if (now < messageUntil) { hint = message; g.fillStyle = 'rgba(230,230,230,0.85)'; }
    g.fillText(hint, DW - 12, DH - 10);

    if (mode === 'camera') {
      const b = RESET_BTN;
      g.strokeStyle = hoverReset ? 'rgba(255,255,255,0.8)' : 'rgba(255,255,255,0.35)';
      g.lineWidth = 1;
      g.strokeRect(b.x + 0.5, b.y + 0.5, b.w, b.h);
      g.fillStyle = hoverReset ? 'rgba(255,255,255,0.95)' : 'rgba(225,225,225,0.75)';
      g.textAlign = 'center'; g.textBaseline = 'middle';
      g.fillText('RESET TARGET', b.x + b.w / 2, b.y + b.h / 2 + 1);

      g.textAlign = 'left';
      g.textBaseline = 'top';
      g.fillStyle = 'rgba(225,225,225,0.85)';
      g.fillText('TARGET', 12, 26);
      g.fillStyle = targetColor || '#cfe03f';
      g.fillRect(62, 27, 10, 10);
      g.strokeStyle = 'rgba(255,255,255,0.4)'; g.lineWidth = 1;
      g.strokeRect(61.5, 26.5, 11, 11);
    }
  }

  function render(now) {
    if (mode === 'starting') {
      ctx.fillStyle = '#000'; ctx.fillRect(0, 0, DW, DH);
      ctx.font = '11px ui-monospace, Menlo, monospace';
      ctx.fillStyle = 'rgba(200,200,200,0.6)';
      ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
      ctx.fillText('waiting for camera...', DW / 2, DH / 2);
      return;
    }
    drawFrame(ctx, DW, DH);
    if (ctl.laserOn) {
      const [lx, ly] = laserPixel(servo.pan, servo.tilt);
      drawLaser(ctx, lx, ly, 1);
    }
    ctx.lineWidth = 1;
    ctx.strokeStyle = ctl.locked ? 'rgba(255,255,255,0.9)' : 'rgba(255,255,255,0.55)';
    if (det.ball.found) {
      const r = Math.sqrt(det.ball.area / Math.PI) * S + 8;
      brackets(ctx, det.ball.x * S, det.ball.y * S, r);
    }
    if (det.laser.found) {
      ctx.strokeStyle = 'rgba(255,255,255,0.5)';
      const x = det.laser.x * S, y = det.laser.y * S;
      ctx.beginPath();
      ctx.moveTo(x - 6, y); ctx.lineTo(x - 3, y); ctx.moveTo(x + 3, y); ctx.lineTo(x + 6, y);
      ctx.moveTo(x, y - 6); ctx.lineTo(x, y - 3); ctx.moveTo(x, y + 3); ctx.lineTo(x, y + 6);
      ctx.stroke();
    }
    osd(ctx, now);
  }

  // ---------------- loop ----------------
  function tick(now) {
    drawFrame(pctx, PW, PH);
    cleanFrame = pctx.getImageData(0, 0, PW, PH);
    if (ctl.laserOn) {
      const [lx, ly] = laserPixel(servo.pan, servo.tilt);
      drawLaser(pctx, lx / S, ly / S, 1 / S);
    }
    det = Tracker.detect(pctx.getImageData(0, 0, PW, PH).data, PW, PH, target, ctl.predictDot());
    ctl.step(det, now);
  }

  let last = performance.now(), acc = 0;
  function frame(now) {
    const dt = Math.min(100, now - last);
    last = now;
    if (mode !== 'starting') {
      const step = SERVO_DEG_PER_S * dt / 1000;
      servo.pan += Math.max(-step, Math.min(step, ctl.pan - servo.pan));
      servo.tilt += Math.max(-step, Math.min(step, ctl.tilt - servo.tilt));
      acc += dt;
      while (acc >= TICK_MS) { acc -= TICK_MS; tick(now); }
    }
    render(now);
    requestAnimationFrame(frame);
  }

  // ---------------- picking a target ----------------
  // Sample a small patch around the click and build a colour range from it.
  function pickTarget(x, y) {
    if (!cleanFrame) return;
    const cx = Math.round(x / S), cy = Math.round(y / S);
    let sx = 0, sy = 0, ss = 0, sv = 0, n = 0, rr = 0, gg = 0, bb = 0;
    for (let dy = -2; dy <= 2; dy++) {
      for (let dx = -2; dx <= 2; dx++) {
        const px = cx + dx, py = cy + dy;
        if (px < 0 || py < 0 || px >= PW || py >= PH) continue;
        const j = (py * PW + px) * 4, d = cleanFrame.data;
        const [h, s, v] = Tracker.hsv(d[j], d[j + 1], d[j + 2]);
        const a = h * Math.PI / 180;            // hue is circular - average as a vector
        sx += Math.cos(a) * s; sy += Math.sin(a) * s;
        ss += s; sv += v; rr += d[j]; gg += d[j + 1]; bb += d[j + 2]; n++;
      }
    }
    if (!n) return;
    const s = ss / n, v = sv / n;
    if (s < 60 || v < 40) {
      notice(v < 40 ? 'too dark - pick something brighter' : 'not colourful enough - pick something brighter', 2500);
      return;
    }
    let h = Math.atan2(sy, sx) * 180 / Math.PI;
    if (h < 0) h += 360;
    const W = 14;
    target = {
      hMin: Math.round((h - W + 360) % 360),
      hMax: Math.round((h + W) % 360),
      sMin: Math.round(Math.max(50, s * 0.55)),
      vMin: Math.round(Math.max(40, v * 0.45)),
    };
    targetColor = `rgb(${rr / n | 0},${gg / n | 0},${bb / n | 0})`;
    ctl.resetBias();
    notice('target set', 1500);
  }

  // ---------------- input ----------------
  let drag = null;
  function pos(e) {
    const r = cam.getBoundingClientRect();
    return [(e.clientX - r.left) * DW / r.width, (e.clientY - r.top) * DH / r.height];
  }
  const clampBall = () => {
    ball.x = Math.max(BALL_R, Math.min(DW - BALL_R, ball.x));
    ball.y = Math.max(BALL_R, Math.min(DH - BALL_R, ball.y));
  };
  cam.addEventListener('pointerdown', (e) => {
    const [x, y] = pos(e);
    if (mode === 'camera') { if (inReset(x, y)) resetTarget(); else pickTarget(x, y); return; }
    if (mode !== 'still') return;
    if (Math.hypot(x - ball.x, y - ball.y) > BALL_R * 1.3) { ball.x = x; ball.y = y; clampBall(); }
    drag = [x - ball.x, y - ball.y];
    cam.setPointerCapture(e.pointerId);
    cam.classList.add('dragging');
  });
  cam.addEventListener('pointermove', (e) => {
    const [x, y] = pos(e);
    hoverReset = mode === 'camera' && inReset(x, y);
    cam.style.cursor = hoverReset ? 'pointer' : '';
    if (!drag) return;
    ball.x = x - drag[0]; ball.y = y - drag[1];
    clampBall();
  });
  const end = () => { drag = null; cam.classList.remove('dragging'); };
  window.addEventListener('keydown', (e) => { if (e.key === 'r' && mode === 'camera') resetTarget(); });
  cam.addEventListener('pointerup', end);
  cam.addEventListener('pointercancel', end);

  window.addEventListener('dragover', (e) => e.preventDefault());
  window.addEventListener('drop', (e) => {
    e.preventDefault();
    if (mode !== 'still') return;
    const file = e.dataTransfer.files[0];
    if (!file || !file.type.startsWith('image/')) return;
    const rd = new FileReader();
    rd.onload = () => { const img = new Image(); img.onload = () => useImage(img); img.src = rd.result; };
    rd.readAsDataURL(file);
  });

  drawDefaultScene();
  requestAnimationFrame(frame);
  if (navigator.mediaDevices && navigator.mediaDevices.getUserMedia) startCamera();
  else { mode = 'still'; notice('no camera - using a still frame', 4000); }
})();
