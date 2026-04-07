// helper $ to shorten getElementById calls
const $ = (id) => document.getElementById(id);

/* ===================== STARFIELD ===================== */
(function() {
  const c = $('stars');
  const ctx = c.getContext('2d');
  let stars = [];
  function resize() {
    c.width = window.innerWidth;
    c.height = window.innerHeight;
    ctx.fillStyle = '#050a14';
    ctx.fillRect(0, 0, c.width, c.height);
    stars = Array.from({length:200}, () => ({
      x: Math.random()*c.width,
      y: Math.random()*c.height,
      r: Math.random()*1.5,
      a: Math.random()
    }));
    draw();
  }
  function draw() {
    ctx.fillRect(0, 0, c.width, c.height);
    stars.forEach(s => {
      ctx.beginPath();
      ctx.arc(s.x, s.y, s.r, 0, Math.PI*2);
      ctx.fillStyle = `rgba(200,224,255,${s.a})`;
      ctx.fill();
    });
  }
  window.addEventListener('resize', resize);
  resize();
})();



/* ===================== SIM STATE ===================== */
let gravity = 1.62;
let maxThrust = 3.5;
let initAlt = 500;
let setpoint = 400;
let infiniteFuel = true;

let Kp = 1.0, Ki = 0.1, Kd = 2.0;
let mode = 'manual';
let manualThrust = 0;

// Cascade PD+PID gains
// outer PD: altitude error → velocity setpoint
let Kp_outer = 0.05;   // maps altitude error to velocity command
let Kd_outer = 0.1;    // damps altitude oscillation
// inner PID: velocity error → thrust
let Kp_inner = 1.5;    // velocity tracking
let Ki_inner = 0.3;    // eliminates velocity steady-state error
let Kd_inner = 0.5;    // damps velocity oscillation

// Cascade state
let prevAltError = 0;
let integralVelErr = 0, prevVelErr = 0;
// Internals exposed for HUD
let cascadeVelSetpoint = 0, cascadeVelError = 0;
let cascadePouter = 0, cascadeDouter = 0;
let cascadePinner = 0, cascadeIinner = 0, cascadeDinner = 0;

// First-order thrust dynamics
let thrustDynamicsEnabled = false;
let dynK   = 1.0;
let dynTau = 0.5;
let realizedThrust = 0;

// Simulation state
let altitude, velocity, acceleration, fuel, elapsedTime;
let integralErr = 0, prevError = 0;
let pidPterm = 0, pidIterm = 0, pidDterm = 0, pidOutput = 0;
let running = false;
let landed = false;
let crashed = false;

// commanded thrust (output of controller / manual input, before actuator dynamics)
let commandedThrust = 0;

// History buffers
const HIST = 300;
let thrustCmdHistory  = [];
let thrustRealHistory = [];

// Data log
let dataLog = [];

/* ===================== CANVAS ===================== */
const simCanvas = $('sim');
const simCtx = simCanvas.getContext('2d');
const thrustChartCanvas = $('thrustChart');
const thrustCtx = thrustChartCanvas.getContext('2d');

function resizeSim() {
  const wrap = $('canvasWrap');
  simCanvas.width = wrap.clientWidth;
  simCanvas.height = wrap.clientHeight;
  thrustChartCanvas.width = thrustChartCanvas.offsetWidth;
  thrustChartCanvas.height = thrustChartCanvas.offsetHeight;
}
window.addEventListener('resize', resizeSim);

/* ===================== DYNAMICS TOGGLE ===================== */
function onDynamicsToggle() {
  thrustDynamicsEnabled = $('dynamicsChk').checked;
  $('dynamicsBadge').classList.toggle('on', thrustDynamicsEnabled);
  $('dynKSlider').disabled   = !thrustDynamicsEnabled;
  $('dynTauSlider').disabled = !thrustDynamicsEnabled;
  if (!thrustDynamicsEnabled) realizedThrust = commandedThrust;
}

/* ===================== INIT ===================== */
function resetSim() {
  altitude = initAlt;
  velocity = 0;
  acceleration = 0;
  fuel = 1.0;
  elapsedTime = 0;
  integralErr = 0; prevError = 0;
  prevAltError = 0; integralVelErr = 0; prevVelErr = 0;
  cascadeVelSetpoint = 0; cascadeVelError = 0;
  pidPterm = 0; pidIterm = 0; pidDterm = 0; pidOutput = 0;
  thrustCmdHistory  = [];
  thrustRealHistory = [];
  dataLog = [];
  landed = false; crashed = false;
  manualThrust = 0;
  commandedThrust = 0;
  realizedThrust = 0;
  $('thrustSlider').value = 0;
  $('overlay').classList.add('hidden');
  running = true;
}

/* ===================== MODE ===================== */
function setMode(m) {
  mode = m;
  // console.log(`Mode set to ${m}`);
  integralErr = 0; prevError = 0;
  prevAltError = 0; integralVelErr = 0; prevVelErr = 0;
  document.querySelectorAll('.mode-btn').forEach(b => b.classList.remove('active'));
  $('btn-'+m.toLowerCase()).classList.add('active');
  $('hdrMode').textContent = m.toUpperCase();
  $('hdrMode').style.color = m === 'manual' ? 'var(--accent2)' : m === 'bangbang' ? 'var(--warn)' : 'var(--accent)';

  const isManual    = m === 'manual';
  const isP         = m === 'P';
  const isPD        = m === 'PD';
  const isPID       = m === 'PID';
  const isCascade   = m === 'cascade';
  const isBangBang  = m === 'bangbang';

  $('manualSection').style.display    = isManual   ? '' : 'none';
  $('pidSection').style.display       = isP || isPD || isPID ? '' : 'none';  
  $('cascadeSection').style.display   = isCascade ? '' : 'none';  
  $('bangbangSection').style.display  = isBangBang ? '' : 'none';

  $('kiRow').style.display = isPID || isCascade ? '' : 'none';
  $('kdRow').style.display = isPD  || isPID || isCascade ? '' : 'none'; 
}

function setGrav(g) {
  gravity = g;
  $('gravSlider').value = g;
  $('gravVal').textContent = g.toFixed(2);
}

function updateFuelUI() {
  $('infiniteFuelBadge').style.display = infiniteFuel ? 'inline' : 'none';
}

/* ===================== PID ===================== */
function computePID(dt) {
  const error = setpoint - altitude;
  const useP = (mode === 'P' || mode === 'PD' || mode === 'PID' || mode === 'PD+PID');
  const useI = (mode === 'PID' || mode === 'cascade');
  const useD = (mode === 'PD' || mode === 'PID' || mode === 'cascade');

  // Only accumulate integral when below setpoint (error > 0) — lander can only thrust upward,
  // so accumulating negative error while above setpoint has no authority and causes windup.
  // Also skip accumulation while falling fast toward setpoint to avoid suppressing D term.
  if (useI) {
    const approachingFromAbove = error < 0 && velocity < -0.5;
    if (!approachingFromAbove) {
      integralErr += error * dt;
      integralErr = Math.max(-500, Math.min(500, integralErr));
    }
  }
  const derivative = (error - prevError) / dt;
  prevError = error;

  pidPterm = useP ? Kp * error : 0;
  pidIterm = useI ? Ki * integralErr : 0;
  pidDterm = useD ? Kd * derivative : 0;
  pidOutput = pidPterm + pidIterm + pidDterm;

  const hoverFrac = gravity / maxThrust;
  let thrustFrac = hoverFrac + pidOutput / maxThrust;
  thrustFrac = Math.max(0, Math.min(1, thrustFrac));
  return thrustFrac;
}

/* ===================== CASCADE PD + PID =====================
 *  Outer PD:  altitude error  → commanded velocity setpoint
 *  Inner PID: velocity error  → thrust
 *
 *  Velocity profile: linearly scales max descent rate with altitude
 *  so impact velocity is always well under 5 m/s regardless of gains.
 * ============================================================ */
function computeCommandedVelocity(alt) {
  const maxDescent  = 12.0;   // m/s — max descent rate at high altitude
  const landingSpeed = 2.0;   // m/s — target speed near ground
  const transitionAlt = 80;   // m  — below this, bleed to landing speed

  if (alt > transitionAlt) {
    const t = Math.min(1, alt / initAlt);
    return -(landingSpeed + (maxDescent - landingSpeed) * t);
  } else {
    const t = alt / transitionAlt;
    return -(landingSpeed * t + 0.5 * (1 - t));
  }
}

function computeCascade(dt) {
  // --- Outer PD: altitude → velocity setpoint ---
  const altError = setpoint - altitude;
  const altD     = (altError - prevAltError) / dt;
  prevAltError   = altError;

  cascadePouter = Kp_outer * altError;
  cascadeDouter = Kd_outer * altD;
  const pdVelCmd = cascadePouter + cascadeDouter;

  // Clamp velocity command by safety descent profile
  const profileVel   = computeCommandedVelocity(altitude);
  cascadeVelSetpoint = Math.max(profileVel, Math.min(0, pdVelCmd));

  // --- Inner PID: velocity → thrust ---
  cascadeVelError = cascadeVelSetpoint - velocity;

  // Integral: only accumulate when we have authority (not falling freely above setpoint)
  const approachingFromAbove = altError < 0 && velocity < -0.5;
  if (!approachingFromAbove) {
    integralVelErr += cascadeVelError * dt;
    integralVelErr  = Math.max(-50, Math.min(50, integralVelErr));
  }

  const velD = (cascadeVelError - prevVelErr) / dt;
  prevVelErr = cascadeVelError;

  cascadePinner = Kp_inner * cascadeVelError;
  cascadeIinner = Ki_inner * integralVelErr;
  cascadeDinner = Kd_inner * velD;

  const hoverFrac = gravity / maxThrust;
  let thrustFrac  = hoverFrac
                  + cascadePinner / maxThrust
                  + cascadeIinner / maxThrust
                  + cascadeDinner / maxThrust;

  return Math.max(0, Math.min(1, thrustFrac));
}

/* ===================== BANG-BANG ===================== */
const BB_TARGET_ALT  = 5;
const BB_MAX_IMPACT  = 5;
const BB_AIM_IMPACT  = 3.5;
const BB_SAFETY_M    = 2.0;

function computeBangBang() {
  const netBrakeAccel = maxThrust - gravity;
  if (netBrakeAccel <= 0) return 0;

  const downSpeed = -velocity;
  const aimSpeed = Math.min(BB_AIM_IMPACT, BB_MAX_IMPACT);
  
  // Braking distance needed to reduce downSpeed to aim impact speed
  // v² - u² = 2·a·d  →  d = (downSpeed² - aimSpeed²) / (2·netBrakeAccel)
  // Note that you could calculate switching altitude once using this formula:
  // h_switch = (v₀² - v_aim² + 2·g·h₀ + 2·netBrakeAccel·h_target) / (2·(g + netBrakeAccel))
  // If anything perturbs the trajectory — a keyboard nudge in manual-hybrid mode, floating point drift, 
  // or the sub-stepping near the ground — the pre-computed switch point becomes stale and the lander 
  // either brakes too late (crash) or too early (hovers wastefully).
  // The continuous recalculation below is essentially a closed-loop correction that makes the controller 
  // robust to disturbances. Computing once is open-loop and fragile.  
  const brakingDist = Math.max(0,
    (downSpeed * downSpeed - aimSpeed * aimSpeed) / (2 * netBrakeAccel)
  );
  const switchAlt = BB_TARGET_ALT + brakingDist + BB_SAFETY_M;

  $('bbNetAccel').textContent  = netBrakeAccel.toFixed(3) + ' m/s²';
  $('bbSwitchAlt').textContent  = switchAlt.toFixed(1);
  $('bbSwitchAlt2').textContent = switchAlt.toFixed(1) + ' m';

  if (altitude <= switchAlt) {
    $('bbPhase').textContent = 'BRAKING';
    $('bbPhase').style.color = 'var(--accent)';
    return 1.0;
  } else {
    $('bbPhase').textContent = 'FREEFALL';
    $('bbPhase').style.color = 'var(--accent2)';
    return 0.0;
  }
}

/* ===================== FIRST-ORDER THRUST DYNAMICS =====================
 *  Euler discretisation of:  τ·dy/dt = K·u − y
 *  → y[n+1] = y[n] + (dt/τ) · (K·u[n] − y[n])
 * ===================================================================== */
function applyThrustDynamics(commanded, dt) {
  if (!thrustDynamicsEnabled) return commanded;
  realizedThrust += (dt / dynTau) * (dynK * commanded - realizedThrust);
  realizedThrust = Math.max(0, Math.min(1, realizedThrust));
  return realizedThrust;
}

/* ===================== PHYSICS ===================== */
let currentThrust = 0;

function physicsStep(elapsed) {
  let thrustFrac;
  if (mode === 'manual') {
    thrustFrac = fuel > 0 ? manualThrust : 0;
  } else if (mode === 'bangbang') {
    thrustFrac = fuel > 0 ? computeBangBang() : 0;
  } else if (mode === 'cascade') {
    thrustFrac = fuel > 0 ? computeCascade(elapsed) : 0;
  } else {
    thrustFrac = fuel > 0 ? computePID(elapsed) : 0;
  }
  commandedThrust = thrustFrac;

  const realized = applyThrustDynamics(commandedThrust, elapsed);
  currentThrust = realized;

  if (!infiniteFuel) fuel = Math.max(0, fuel - realized * fuelConsumptionRate * elapsed * 60);

  const thrustAccel = realized * maxThrust;
  acceleration = thrustAccel - gravity;
  velocity += acceleration * elapsed;
  altitude += velocity * elapsed;
  if (altitude > 1200) { altitude = 1200; velocity = Math.min(velocity, 0); }
}

const fuelConsumptionRate = 0.0001;

/* ===================== DRAW ===================== */
function draw() {
  const W = simCanvas.width, H = simCanvas.height;
  const ctx = simCtx;
  ctx.clearRect(0, 0, W, H);

  const groundY = H - 60;
  const gradient = ctx.createLinearGradient(0, groundY, 0, H);
  gradient.addColorStop(0, '#1a3a6a');
  gradient.addColorStop(1, '#050a14');
  ctx.fillStyle = gradient;
  ctx.fillRect(0, groundY, W, H - groundY);

  ctx.strokeStyle = '#00d4ff';
  ctx.lineWidth = 2;
  ctx.shadowColor = '#00d4ff';
  ctx.shadowBlur = 10;
  ctx.beginPath(); ctx.moveTo(0, groundY); ctx.lineTo(W, groundY); ctx.stroke();
  ctx.shadowBlur = 0;

  const padW = 100, padX = W/2 - padW/2;
  ctx.fillStyle = 'rgba(0,212,255,0.15)';
  ctx.fillRect(padX, groundY - 4, padW, 4);
  ctx.strokeStyle = '#00d4ff';
  ctx.lineWidth = 1;
  ctx.setLineDash([6,4]);
  ctx.strokeRect(padX, groundY - 4, padW, 4);
  ctx.setLineDash([]);

  if (mode !== 'manual' && mode !== 'bangbang') {
    const spY = altToY(setpoint, H, groundY);
    ctx.strokeStyle = 'rgba(255,204,0,0.5)';
    ctx.lineWidth = 1;
    ctx.setLineDash([8,6]);
    ctx.beginPath(); ctx.moveTo(0, spY); ctx.lineTo(W, spY); ctx.stroke();
    ctx.setLineDash([]);
    ctx.fillStyle = 'rgba(255,204,0,0.7)';
    ctx.font = '11px Share Tech Mono';
    ctx.fillText(`TARGET: ${setpoint}m`, 8, spY - 4);
  }

  if (mode === 'bangbang') {
    const netBrakeAccel = maxThrust - gravity;
    if (netBrakeAccel > 0) {
      const downSpeed = -velocity;
      const brakingDist = Math.max(0,
        (downSpeed * downSpeed - BB_AIM_IMPACT * BB_AIM_IMPACT) / (2 * netBrakeAccel)
      );
      const switchAlt = BB_TARGET_ALT + brakingDist + BB_SAFETY_M;
      const swY = altToY(switchAlt, H, groundY);
      ctx.strokeStyle = 'rgba(255,204,0,0.7)';
      ctx.lineWidth = 1.5;
      ctx.setLineDash([6,4]);
      ctx.beginPath(); ctx.moveTo(0, swY); ctx.lineTo(W, swY); ctx.stroke();
      ctx.setLineDash([]);
      ctx.fillStyle = 'rgba(255,204,0,0.85)';
      ctx.font = '11px Share Tech Mono';
      ctx.fillText(`SWITCH: ${switchAlt.toFixed(1)}m`, 8, swY - 4);
    }
  }

  const FEET_OFFSET = 18;
  const landerX = W / 2;
  const landerY = altToY(altitude, H, groundY) - FEET_OFFSET;
  drawLander(ctx, landerX, landerY, currentThrust);
  drawRuler(ctx, W, H, groundY);

  if (Math.abs(velocity) > 0.5) {
    const vScale = 8;
    const arrowLen = Math.min(Math.abs(velocity) * vScale, 60);
    const dir = velocity > 0 ? -1 : 1;
    ctx.strokeStyle = velocity > 0 ? '#39ff14' : '#ff6b35';
    ctx.lineWidth = 2;
    ctx.shadowColor = velocity > 0 ? '#39ff14' : '#ff6b35';
    ctx.shadowBlur = 6;
    ctx.beginPath();
    ctx.moveTo(landerX + 30, landerY);
    ctx.lineTo(landerX + 30, landerY + dir * arrowLen);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(landerX + 30, landerY + dir * arrowLen);
    ctx.lineTo(landerX + 25, landerY + dir * (arrowLen - 10));
    ctx.lineTo(landerX + 35, landerY + dir * (arrowLen - 10));
    ctx.closePath();
    ctx.fillStyle = velocity > 0 ? '#39ff14' : '#ff6b35';
    ctx.fill();
    ctx.shadowBlur = 0;
  }
}

function altToY(alt, H, groundY) {
  const maxAlt = 900;
  return groundY - (alt / maxAlt) * (groundY - 40);
}

function drawLander(ctx, x, y, thrustFrac) {
  ctx.save();
  ctx.translate(x, y);

  if (thrustFrac > 0.01) {
    const flameH = 20 + thrustFrac * 40 + Math.random() * 10;
    const flameGrad = ctx.createLinearGradient(0, 0, 0, flameH);
    flameGrad.addColorStop(0, `rgba(255,150,50,${0.9 * thrustFrac})`);
    flameGrad.addColorStop(0.5, `rgba(255,80,20,${0.6 * thrustFrac})`);
    flameGrad.addColorStop(1, `rgba(255,40,10,0)`);
    ctx.fillStyle = flameGrad;
    ctx.shadowColor = '#ff6b35';
    ctx.shadowBlur = 20 * thrustFrac;
    ctx.beginPath();
    ctx.moveTo(-8, 12);
    ctx.quadraticCurveTo(0, 12 + flameH * 0.7, 0, 12 + flameH);
    ctx.quadraticCurveTo(0, 12 + flameH * 0.7, 8, 12);
    ctx.closePath();
    ctx.fill();
    ctx.shadowBlur = 0;
  }

  ctx.strokeStyle = '#00d4ff';
  ctx.lineWidth = 2;
  ctx.shadowColor = '#00d4ff';
  ctx.shadowBlur = 8;
  ctx.fillStyle = '#0d1f3c';
  ctx.beginPath();
  ctx.roundRect(-18, -20, 36, 32, 4);
  ctx.fill();
  ctx.stroke();

  ctx.beginPath();
  ctx.arc(0, -6, 8, 0, Math.PI*2);
  ctx.fillStyle = 'rgba(0,212,255,0.15)';
  ctx.fill();
  ctx.strokeStyle = '#00d4ff';
  ctx.lineWidth = 1.5;
  ctx.stroke();

  ctx.strokeStyle = '#4a6a9a';
  ctx.lineWidth = 2;
  ctx.shadowBlur = 0;
  ctx.beginPath(); ctx.moveTo(-18, 8); ctx.lineTo(-30, 18); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(-30, 18); ctx.lineTo(-36, 18); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(18, 8); ctx.lineTo(30, 18); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(30, 18); ctx.lineTo(36, 18); ctx.stroke();

  ctx.shadowBlur = 0;
  ctx.fillStyle = 'rgba(200,224,255,0.8)';
  ctx.font = '11px Share Tech Mono';
  ctx.textAlign = 'center';
  ctx.fillText(`${altitude.toFixed(1)}m`, 0, -28);
  ctx.textAlign = 'left';

  ctx.restore();
}

function drawRuler(ctx, W, H, groundY) {
  const maxAlt = 900;
  ctx.strokeStyle = 'rgba(74,106,154,0.4)';
  ctx.fillStyle = 'rgba(74,106,154,0.7)';
  ctx.font = '10px Share Tech Mono';
  ctx.lineWidth = 1;
  for (let a = 0; a <= maxAlt; a += 100) {
    const y = altToY(a, H, groundY);
    ctx.setLineDash([3,6]);
    ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(W, y); ctx.stroke();
    ctx.setLineDash([]);
    ctx.fillText(`${a}m`, 4, y - 2);
  }
}

/* ===================== CHARTS ===================== */
function updateCharts() {
  const c = thrustChartCanvas, ctx = thrustCtx;
  const W = c.width, H = c.height;
  ctx.clearRect(0,0,W,H);
  ctx.fillStyle = '#050a14';
  ctx.fillRect(0,0,W,H);

  if (thrustCmdHistory.length < 2) return;

  ctx.beginPath();
  thrustCmdHistory.forEach((v,i) => {
    const x = (i/(HIST-1))*W;
    const y = H - v*H;
    i===0 ? ctx.moveTo(x,y) : ctx.lineTo(x,y);
  });
  ctx.lineTo(W, H); ctx.lineTo(0, H); ctx.closePath();
  ctx.fillStyle = 'rgba(255,107,53,0.12)';
  ctx.fill();
  ctx.strokeStyle = 'rgba(255,107,53,0.4)';
  ctx.lineWidth = 1;
  ctx.beginPath();
  thrustCmdHistory.forEach((v,i) => {
    const x = (i/(HIST-1))*W;
    const y = H - v*H;
    i===0 ? ctx.moveTo(x,y) : ctx.lineTo(x,y);
  });
  ctx.stroke();

  const grad = ctx.createLinearGradient(0,0,0,H);
  grad.addColorStop(0, 'rgba(255,107,53,0.7)');
  grad.addColorStop(1, 'rgba(255,107,53,0.05)');
  ctx.beginPath();
  thrustRealHistory.forEach((v,i) => {
    const x = (i/(HIST-1))*W;
    const y = H - v*H;
    i===0 ? ctx.moveTo(x,y) : ctx.lineTo(x,y);
  });
  ctx.lineTo(W, H); ctx.lineTo(0, H); ctx.closePath();
  ctx.fillStyle = grad; ctx.fill();
  ctx.strokeStyle = '#ff6b35'; ctx.lineWidth = 1.5;
  ctx.beginPath();
  thrustRealHistory.forEach((v,i) => {
    const x = (i/(HIST-1))*W;
    const y = H - v*H;
    i===0 ? ctx.moveTo(x,y) : ctx.lineTo(x,y);
  });
  ctx.stroke();
}

/* ===================== HUD ===================== */
function updateHUD() {
  const velAbs = Math.abs(velocity);
  const velClass = velAbs < 5 ? 'ok' : velAbs < 15 ? 'warn' : 'danger';
  const altClass = altitude > 50 ? 'ok' : altitude > 10 ? 'warn' : 'danger';
  const fuelClass = fuel > 0.5 ? 'ok' : fuel > 0.2 ? 'warn' : 'danger';

  $('hdrAlt').textContent = altitude.toFixed(1) + ' m';
  $('hdrAlt').className = altClass;
  $('hdrVel').textContent = velocity.toFixed(2) + ' m/s';
  $('hdrVel').className = velClass;
  $('hdrThrust').textContent =
    (commandedThrust*100).toFixed(0) + '% / ' + (currentThrust*100).toFixed(0) + '%';
  $('hdrFuel').textContent = infiniteFuel ? '∞' : (fuel*100).toFixed(1) + '%';
  $('hdrFuel').className = infiniteFuel ? 'ok' : fuelClass;
  $('telFuel').textContent = infiniteFuel ? '∞ (unlimited)' : (fuel*100).toFixed(1) + '%';

  if (mode === 'manual') {
    const pct = (manualThrust*100).toFixed(0);
    $('thrustPct').textContent = pct;
    $('thrustSlider').value = manualThrust * 100;
  }

  $('dualBarCmd').style.width  = (commandedThrust*100) + '%';
  $('dualBarReal').style.width = (currentThrust*100) + '%';

  const lag = commandedThrust - currentThrust;
  $('telThrustCmd').textContent  = (commandedThrust*100).toFixed(1) + '%';
  $('telThrustReal').textContent = (currentThrust*100).toFixed(1) + '%';
  $('telThrustLag').textContent  = (lag*100).toFixed(1) + '%';

  $('telAlt').textContent = altitude.toFixed(2) + ' m';
  $('telVel').textContent = velocity.toFixed(3) + ' m/s';
  $('telAcc').textContent = acceleration.toFixed(3) + ' m/s²';
  $('telTime').textContent = elapsedTime.toFixed(1) + ' s';
  $('logCount').textContent = dataLog.length.toLocaleString() + ' frames recorded';

  if (mode !== 'manual' && mode !== 'bangbang') {
    if (mode === 'cascade') {
      $('liveError').textContent  = (setpoint - altitude).toFixed(3);
      $('livePterm').textContent  = `P=${cascadePouter.toFixed(3)} D=${cascadeDouter.toFixed(3)}`;
      $('liveIterm').textContent  = `vCmd=${cascadeVelSetpoint.toFixed(2)} m/s`;
      $('liveDterm').textContent  = `vErr=${cascadeVelError.toFixed(3)}`;
      $('liveOutput').textContent = `P=${cascadePinner.toFixed(3)} I=${cascadeIinner.toFixed(3)} D=${cascadeDinner.toFixed(3)}`;
      const clamped = Math.max(0, Math.min(1, (gravity/maxThrust)
        + cascadePinner/maxThrust + cascadeIinner/maxThrust + cascadeDinner/maxThrust));
      $('liveClamped').textContent = (clamped*100).toFixed(1) + '%';
    } else {
      const error = setpoint - altitude;
      $('liveError').textContent = error.toFixed(3);
      $('livePterm').textContent = pidPterm.toFixed(4);
      $('liveIterm').textContent = pidIterm.toFixed(4);
      $('liveDterm').textContent = pidDterm.toFixed(4);
      $('liveOutput').textContent = pidOutput.toFixed(4);
      const clamped = Math.max(0, Math.min(1, (gravity/maxThrust) + pidOutput/maxThrust));
      $('liveClamped').textContent = (clamped*100).toFixed(1) + '%';
    }
  } else {
    ['liveError','livePterm','liveIterm','liveDterm','liveOutput','liveClamped'].forEach(id => {
      $(id).textContent = 'N/A';
    });
  }
}

/* ===================== OVERLAY ===================== */
function showOverlay(success, speed) {
  const ov = $('overlay');
  const title = $('overlayTitle');
  const msg = $('overlayMsg');
  ov.classList.remove('hidden');
  if (success) {
    title.textContent = '✓ LANDED';
    title.className = 'success';
    msg.textContent = `Safe landing! Impact velocity: ${speed.toFixed(2)} m/s • Time: ${elapsedTime.toFixed(1)}s`;
  } else {
    title.textContent = '✗ CRASHED';
    title.className = 'fail';
    msg.textContent = `Impact velocity: ${speed.toFixed(2)} m/s (max 5 m/s) • ${elapsedTime.toFixed(1)}s`;
  }
}

function showFlash(id) {
  const el = $(id);
  el.classList.remove('show');
  void el.offsetWidth;
  el.classList.add('show');
  setTimeout(() => el.classList.remove('show'), 800);
}

/* ===================== KEYBOARD ===================== */
const keysDown = {};
window.addEventListener('keydown', e => {
  keysDown[e.code] = true;
  if (e.code === 'Space') { manualThrust = 0; e.preventDefault(); }
});
window.addEventListener('keyup', e => { keysDown[e.code] = false; });

function handleKeys() {
  if (mode !== 'manual') return;
  if (keysDown['ArrowUp'] || keysDown['KeyW']) {
    manualThrust = Math.min(1, manualThrust + 0.01);
  }
  if (keysDown['ArrowDown'] || keysDown['KeyS']) {
    manualThrust = Math.max(0, manualThrust - 0.01);
  }
}

/* ===================== MAIN LOOP ===================== */
let lastPhysicsTime = null;
function mainLoop(ts) {
  handleKeys();
  if (!lastPhysicsTime) lastPhysicsTime = ts;
  const elapsed = Math.min((ts - lastPhysicsTime)/1000, 0.05);
  lastPhysicsTime = ts;

  if (running && !landed && !crashed) {
    if (altitude < 10) {
      const subSteps = 5;
      const subDt = elapsed / subSteps;
      for (let i = 0; i < subSteps; i++) {
        physicsStep(subDt);
      }
    } else {
      physicsStep(elapsed);
    }
    elapsedTime += elapsed;

    thrustCmdHistory.push(commandedThrust);
    thrustRealHistory.push(currentThrust);
    if (thrustCmdHistory.length  > HIST) thrustCmdHistory.shift();
    if (thrustRealHistory.length > HIST) thrustRealHistory.shift();

    const logError = (mode === 'manual' || mode === 'bangbang') ? 0 : (setpoint - altitude);
    dataLog.push([
      elapsedTime.toFixed(4),
      mode,
      gravity.toFixed(4),
      altitude.toFixed(4),
      velocity.toFixed(4),
      logError.toFixed(4),
      (commandedThrust * 100).toFixed(2),
      (currentThrust * 100).toFixed(2),
      (fuel * 100).toFixed(2)
    ]);

    if (altitude <= 4) {
      altitude = 4;
      const impactSpeed = Math.abs(velocity);
      velocity = 0;
      running = false;
      if (impactSpeed <= 5) {
        landed = true;
        showOverlay(true, impactSpeed);
        showFlash('flash');
      } else {
        crashed = true;
        showOverlay(false, impactSpeed);
        showFlash('crashFlash');
      }
    }
  }

  if (simCanvas.width > 0) {
    draw();
    updateHUD();
    updateCharts();
  }

  requestAnimationFrame(mainLoop);
}

/* ===================== DATA EXPORT ===================== */
function downloadLog() {
  if (dataLog.length === 0) {
    alert('No data recorded yet. Start the simulation first.');
    return;
  }
  const header = '%time(s)\tmode\tgravity(m/s2)\theight(m)\tvelocity(m/s)\terror(m)\tthrust_commanded(%)\tthrust_realized(%)\tfuel(%)';
  const rows = dataLog.map(r => r.join('\t'));
  const content = [header, ...rows].join('\n');
  const blob = new Blob([content], { type: 'text/plain' });
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = `pid_lander_log_${new Date().toISOString().slice(0,19).replace(/[T:]/g,'-')}.txt`;
  a.click();
  URL.revokeObjectURL(url);
}

/* ===================== BOOT ===================== */
window.addEventListener('load', () => {
  resizeSim();
  resetSim();
  $('infiniteFuelChk').checked = true;
  updateFuelUI();
  requestAnimationFrame(mainLoop);
});
window.addEventListener('resize', resizeSim);
