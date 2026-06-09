const canvas = document.getElementById("scene");
const ctx = canvas.getContext("2d");

const pointCountEl = document.getElementById("pointCount");
const statusEl = document.getElementById("status");

const createBtn = document.getElementById("createBtn");
const resetBtn = document.getElementById("resetBtn");
const clearTraceBtn = document.getElementById("clearTraceBtn");
const modeBtn = document.getElementById("modeBtn");
const canvasWrapEl = document.querySelector(".canvas-wrap");

const state = {
  targetPoints: [],
  mechanism: null,
  trace: [],
  running: false,
  viewMode: "engineering",
  lastTime: 0,
  omega: 0.55,
  lastB: null,
  lastPose: null,
  viewport: {
    width: 0,
    height: 0,
  },
  camera: {
    x: 0,
    y: 0,
    scale: 1,
    targetX: 0,
    targetY: 0,
    targetScale: 1,
    initialized: false,
  },
};

const SOLVER_CONFIG = {
  coarse: {
    n: 52,
    threshold: 0.02,
    maxAbsScale: 1.8,
    minKeepFrac: 0.08,
    maxKeep: 50,
    refineIters: 10,
  },
  medium: {
    n: 84,
    threshold: 0.008,
    maxAbsScale: 1.9,
    minKeepFrac: 0.06,
    maxKeep: 90,
    refineIters: 14,
  },
  fine: {
    n: 124,
    threshold: 0.0028,
    maxAbsScale: 2.0,
    minKeepFrac: 0.045,
    maxKeep: 130,
    refineIters: 18,
  },
};

function setStatus(text) {
  statusEl.textContent = text;
}

function isPresentationMode() {
  return state.viewMode === "presentation";
}

function updateModeUI() {
  if (!modeBtn) {
    return;
  }
  const presentation = isPresentationMode();
  modeBtn.textContent = presentation ? "Mode: Presentation" : "Mode: Engineering";
  modeBtn.setAttribute("aria-pressed", presentation ? "true" : "false");
  modeBtn.classList.toggle("active", presentation);
  if (canvasWrapEl) {
    canvasWrapEl.classList.toggle("presentation", presentation);
  }
}

function resizeCanvas() {
  const dpr = window.devicePixelRatio || 1;
  const rect = canvas.getBoundingClientRect();
  canvas.width = Math.floor(rect.width * dpr);
  canvas.height = Math.floor(rect.height * dpr);
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

  state.viewport.width = rect.width;
  state.viewport.height = rect.height;

  if (!state.camera.initialized) {
    const cx = rect.width / 2;
    const cy = rect.height / 2;
    state.camera.x = cx;
    state.camera.y = cy;
    state.camera.targetX = cx;
    state.camera.targetY = cy;
    state.camera.scale = 1;
    state.camera.targetScale = 1;
    state.camera.initialized = true;
  }
}

function v(x, y) {
  return { x, y };
}

function add(a, b) {
  return v(a.x + b.x, a.y + b.y);
}

function sub(a, b) {
  return v(a.x - b.x, a.y - b.y);
}

function mul(a, s) {
  return v(a.x * s, a.y * s);
}

function dot(a, b) {
  return a.x * b.x + a.y * b.y;
}

function cross(a, b) {
  return a.x * b.y - a.y * b.x;
}

function length(a) {
  return Math.hypot(a.x, a.y);
}

function dist(a, b) {
  return length(sub(a, b));
}

function rotate(p, ang) {
  const c = Math.cos(ang);
  const s = Math.sin(ang);
  return v(c * p.x - s * p.y, s * p.x + c * p.y);
}

function angleOf(p) {
  return Math.atan2(p.y, p.x);
}

function clamp(val, minVal, maxVal) {
  return Math.max(minVal, Math.min(maxVal, val));
}

function lerp(a, b, t) {
  return a + (b - a) * t;
}

function worldToScreen(p) {
  const { width, height } = state.viewport;
  const { x, y, scale } = state.camera;
  return v(
    (p.x - x) * scale + width / 2,
    (p.y - y) * scale + height / 2
  );
}

function screenToWorld(p) {
  const { width, height } = state.viewport;
  const { x, y, scale } = state.camera;
  return v(
    (p.x - width / 2) / scale + x,
    (p.y - height / 2) / scale + y
  );
}

function pushEnvelopePoints(out, center, radius) {
  out.push(v(center.x + radius, center.y));
  out.push(v(center.x - radius, center.y));
  out.push(v(center.x, center.y + radius));
  out.push(v(center.x, center.y - radius));
}

function collectFocusPoints() {
  const points = state.targetPoints.slice();

  if (state.mechanism) {
    const mech = state.mechanism;
    points.push(mech.A, mech.B);

    const aLocalMag = length(mech.aLocal);
    const bLocalMag = length(mech.bLocal);
    const reachA = mech.rA + aLocalMag + 46;
    const reachB = mech.rB + bLocalMag + 46;
    pushEnvelopePoints(points, mech.A, reachA);
    pushEnvelopePoints(points, mech.B, reachB);
  }

  if (state.lastPose) {
    points.push(state.lastPose.aWorld, state.lastPose.bWorld, state.lastPose.couplerPoint);
  }

  if (state.trace.length > 0) {
    const stride = Math.max(1, Math.floor(state.trace.length / 90));
    for (let i = 0; i < state.trace.length; i += stride) {
      points.push(state.trace[i]);
    }
  }

  return points;
}

function computeCameraTarget() {
  const { width, height } = state.viewport;
  if (width <= 0 || height <= 0) {
    return;
  }

  const points = collectFocusPoints();
  if (points.length === 0) {
    const cx = width / 2;
    const cy = height / 2;
    state.camera.targetX = cx;
    state.camera.targetY = cy;
    state.camera.targetScale = 1;
    return;
  }

  let minX = Infinity;
  let minY = Infinity;
  let maxX = -Infinity;
  let maxY = -Infinity;

  for (const p of points) {
    minX = Math.min(minX, p.x);
    minY = Math.min(minY, p.y);
    maxX = Math.max(maxX, p.x);
    maxY = Math.max(maxY, p.y);
  }

  const boxW = Math.max(120, maxX - minX);
  const boxH = Math.max(120, maxY - minY);
  const pad = 76;
  const sx = (width - pad * 2) / boxW;
  const sy = (height - pad * 2) / boxH;
  let targetScale = Math.min(sx, sy) * 0.92;
  targetScale = clamp(targetScale, 0.32, 2.6);

  state.camera.targetX = (minX + maxX) / 2;
  state.camera.targetY = (minY + maxY) / 2;
  state.camera.targetScale = targetScale;
}

function updateCamera(dt) {
  computeCameraTarget();

  if (!state.camera.initialized) {
    return;
  }

  const settle = clamp(dt * 6.5, 0.08, 0.24);
  state.camera.x = lerp(state.camera.x, state.camera.targetX, settle);
  state.camera.y = lerp(state.camera.y, state.camera.targetY, settle);
  state.camera.scale = lerp(state.camera.scale, state.camera.targetScale, settle);
}

function wrapAngle(a) {
  let out = a;
  while (out > Math.PI) {
    out -= 2 * Math.PI;
  }
  while (out < -Math.PI) {
    out += 2 * Math.PI;
  }
  return out;
}

function normalizeAlphas(alphas) {
  const base = alphas[0];
  return alphas.map((a) => wrapAngle(a - base));
}

function det4(m) {
  const a = m.map((row) => row.slice());
  let det = 1;
  for (let i = 0; i < 4; i += 1) {
    let pivot = i;
    for (let r = i + 1; r < 4; r += 1) {
      if (Math.abs(a[r][i]) > Math.abs(a[pivot][i])) {
        pivot = r;
      }
    }
    if (Math.abs(a[pivot][i]) < 1e-12) {
      return 0;
    }
    if (pivot !== i) {
      [a[i], a[pivot]] = [a[pivot], a[i]];
      det *= -1;
    }
    const pivotVal = a[i][i];
    det *= pivotVal;
    for (let r = i + 1; r < 4; r += 1) {
      const factor = a[r][i] / pivotVal;
      for (let c = i; c < 4; c += 1) {
        a[r][c] -= factor * a[i][c];
      }
    }
  }
  return det;
}

function circumcenter(p1, p2, p3) {
  const d = 2 * (
    p1.x * (p2.y - p3.y) +
    p2.x * (p3.y - p1.y) +
    p3.x * (p1.y - p2.y)
  );

  if (Math.abs(d) < 1e-9) {
    return null;
  }

  const p1Sq = p1.x * p1.x + p1.y * p1.y;
  const p2Sq = p2.x * p2.x + p2.y * p2.y;
  const p3Sq = p3.x * p3.x + p3.y * p3.y;

  const ux = (
    p1Sq * (p2.y - p3.y) +
    p2Sq * (p3.y - p1.y) +
    p3Sq * (p1.y - p2.y)
  ) / d;

  const uy = (
    p1Sq * (p3.x - p2.x) +
    p2Sq * (p1.x - p3.x) +
    p3Sq * (p2.x - p1.x)
  ) / d;

  return v(ux, uy);
}

function circleIntersections(c1, r1, c2, r2) {
  const d = dist(c1, c2);
  if (d < 1e-9) {
    return [];
  }
  if (d > r1 + r2 + 1e-8 || d < Math.abs(r1 - r2) - 1e-8) {
    return [];
  }

  const a = (r1 * r1 - r2 * r2 + d * d) / (2 * d);
  const hSq = r1 * r1 - a * a;
  if (hSq < -1e-8) {
    return [];
  }

  const h = Math.sqrt(Math.max(0, hSq));
  const dir = mul(sub(c2, c1), 1 / d);
  const pMid = add(c1, mul(dir, a));
  const perp = v(-dir.y, dir.x);

  return [
    add(pMid, mul(perp, h)),
    add(pMid, mul(perp, -h)),
  ];
}

function estimateOrientations(points) {
  const p1 = points[0];
  const p2 = points[1];
  const p3 = points[2];
  const p4 = points[3];

  const a1 = angleOf(sub(p2, p1));
  const a2 = angleOf(sub(p3, p1));
  const a3 = angleOf(sub(p4, p2));
  const a4 = angleOf(sub(p4, p3));

  return [a1, a2, a3, a4];
}

function circleCondition(points, alphas, q) {
  const rowData = points.map((p, i) => {
    const qi = add(p, rotate(q, alphas[i]));
    return [qi.x * qi.x + qi.y * qi.y, qi.x, qi.y, 1];
  });

  return det4(rowData);
}

function sampleCirclePointLocus(points, alphas, span, options) {
  const samples = [];
  const maxAbs = span * options.maxAbsScale;
  const n = options.n;
  const step = (2 * maxAbs) / (n - 1);
  const norm = Math.max(1, span ** 6);
  const threshold = options.threshold;

  for (let iy = 0; iy < n; iy += 1) {
    const y = -maxAbs + iy * step;
    for (let ix = 0; ix < n; ix += 1) {
      const x = -maxAbs + ix * step;
      const q = v(x, y);
      const f = circleCondition(points, alphas, q) / norm;
      if (Math.abs(f) < threshold) {
        samples.push(q);
      }
    }
  }

  const filtered = [];
  const minKeepDist = span * options.minKeepFrac;

  for (const p of samples) {
    let tooClose = false;
    for (const k of filtered) {
      if (dist(p, k) < minKeepDist) {
        tooClose = true;
        break;
      }
    }
    if (!tooClose) {
      filtered.push(p);
    }
    if (filtered.length > options.maxKeep) {
      break;
    }
  }

  return filtered;
}

function refineCirclePointRoot(points, alphas, span, q0, maxIters) {
  const norm = Math.max(1, span ** 6);
  const eps = Math.max(1e-4, span * 8e-5);
  let q = v(q0.x, q0.y);

  for (let iter = 0; iter < maxIters; iter += 1) {
    const f = circleCondition(points, alphas, q) / norm;
    if (Math.abs(f) < 1e-8) {
      break;
    }

    const fx1 = circleCondition(points, alphas, v(q.x + eps, q.y)) / norm;
    const fx0 = circleCondition(points, alphas, v(q.x - eps, q.y)) / norm;
    const fy1 = circleCondition(points, alphas, v(q.x, q.y + eps)) / norm;
    const fy0 = circleCondition(points, alphas, v(q.x, q.y - eps)) / norm;

    const dfdx = (fx1 - fx0) / (2 * eps);
    const dfdy = (fy1 - fy0) / (2 * eps);
    const gradSq = dfdx * dfdx + dfdy * dfdy;

    if (gradSq < 1e-14) {
      break;
    }

    let stepScale = f / gradSq;
    stepScale = clamp(stepScale, -span * 0.15, span * 0.15);
    q = v(q.x - dfdx * stepScale, q.y - dfdy * stepScale);
  }

  return q;
}

function dedupePoints(points, minDist) {
  const out = [];
  for (const p of points) {
    let close = false;
    for (const q of out) {
      if (dist(p, q) < minDist) {
        close = true;
        break;
      }
    }
    if (!close) {
      out.push(p);
    }
  }
  return out;
}

function evaluatePair(points, alphas, aLocal, bLocal, span) {
  const aWorld = points.map((p, i) => add(p, rotate(aLocal, alphas[i])));
  const bWorld = points.map((p, i) => add(p, rotate(bLocal, alphas[i])));

  const A = circumcenter(aWorld[0], aWorld[1], aWorld[2]);
  const B = circumcenter(bWorld[0], bWorld[1], bWorld[2]);

  if (!A || !B) {
    return null;
  }

  const rA = dist(A, aWorld[0]);
  const rB = dist(B, bWorld[0]);

  const errA = Math.abs(dist(A, aWorld[3]) - rA);
  const errB = Math.abs(dist(B, bWorld[3]) - rB);
  const spacing = dist(aLocal, bLocal);
  const ground = dist(A, B);

  if (spacing < 0.1 * span || ground < 0.1 * span) {
    return null;
  }

  const score = errA + errB + 0.003 * (1 / spacing + 1 / ground);

  return {
    score,
    errA,
    errB,
    A,
    B,
    aLocal,
    bLocal,
    alphas,
  };
}

function findBestForAlphas(points, alphas, span, options) {
  const raw = sampleCirclePointLocus(points, alphas, span, options);
  if (raw.length < 2) {
    return null;
  }

  const refined = raw.map((q) => refineCirclePointRoot(points, alphas, span, q, options.refineIters));
  const candidates = dedupePoints(refined, span * options.minKeepFrac * 0.8);
  if (candidates.length < 2) {
    return null;
  }

  let best = null;
  for (let i = 0; i < candidates.length; i += 1) {
    for (let j = i + 1; j < candidates.length; j += 1) {
      const evalRes = evaluatePair(points, alphas, candidates[i], candidates[j], span);
      if (!evalRes) {
        continue;
      }
      if (!best || evalRes.score < best.score) {
        best = evalRes;
      }
    }
  }

  return best;
}

function tangentBasedOrientations(points) {
  const alphas = [];
  for (let i = 0; i < points.length; i += 1) {
    const prev = points[(i + points.length - 1) % points.length];
    const next = points[(i + 1) % points.length];
    alphas.push(angleOf(sub(next, prev)));
  }
  return alphas;
}

function searchBestAlphas(points, span, seed) {
  let current = normalizeAlphas(seed);
  let best = findBestForAlphas(points, current, span, SOLVER_CONFIG.medium);

  let step = Math.PI / 5;
  while (step > 0.008) {
    let improved = false;
    for (let k = 1; k < 4; k += 1) {
      for (const dir of [-1, 1]) {
        const trial = current.slice();
        trial[k] += dir * step;
        const normTrial = normalizeAlphas(trial);
        const evalRes = findBestForAlphas(points, normTrial, span, SOLVER_CONFIG.coarse);
        if (!evalRes) {
          continue;
        }
        if (!best || evalRes.score < best.score) {
          best = evalRes;
          current = normTrial;
          improved = true;
        }
      }
    }
    if (!improved) {
      step *= 0.58;
    }
  }

  if (!best) {
    return null;
  }

  // Re-evaluate the optimized orientation set with a tighter solve.
  const fine = findBestForAlphas(points, current, span, SOLVER_CONFIG.fine);
  return fine || best;
}

function synthesizeMechanism(points) {
  let span = 1;
  for (let i = 0; i < points.length; i += 1) {
    for (let j = i + 1; j < points.length; j += 1) {
      span = Math.max(span, dist(points[i], points[j]));
    }
  }

  const seeds = [
    normalizeAlphas(estimateOrientations(points)),
    normalizeAlphas(tangentBasedOrientations(points)),
  ];

  // Add light randomized starts around the two deterministic seeds.
  for (let s = 0; s < 2; s += 1) {
    for (let i = 0; i < 10; i += 1) {
      const base = seeds[s];
      const jitter = base.map((a, idx) => (
        idx === 0 ? 0 : a + (Math.random() * 2 - 1) * 0.55
      ));
      seeds.push(normalizeAlphas(jitter));
    }
  }

  let best = null;
  for (const seed of seeds) {
    const candidate = searchBestAlphas(points, span, seed);
    if (!candidate) {
      continue;
    }
    if (!best || candidate.score < best.score) {
      best = candidate;
    }
  }

  if (!best) {
    return { ok: false, reason: "No high-accuracy Burmester solution found for this point set." };
  }

  const tol = Math.max(0.6, span * 0.0025);
  if (best.errA > tol || best.errB > tol) {
    return {
      ok: false,
      reason: "Point-set is ill-conditioned for pin-point closure. Try a less singular 4-point pattern.",
    };
  }

  const alphas = best.alphas;
  const a0 = add(points[0], rotate(best.aLocal, alphas[0]));
  const b0 = add(points[0], rotate(best.bLocal, alphas[0]));

  const thetaRef = points.map((_, i) => {
    const ai = add(points[i], rotate(best.aLocal, alphas[i]));
    return angleOf(sub(ai, best.A));
  });

  return {
    ok: true,
    A: best.A,
    B: best.B,
    aLocal: best.aLocal,
    bLocal: best.bLocal,
    alphas,
    a0,
    b0,
    rA: dist(best.A, a0),
    rB: dist(best.B, b0),
    couplerLen: dist(best.aLocal, best.bLocal),
    localAngleAB: angleOf(sub(best.bLocal, best.aLocal)),
    theta0: angleOf(sub(a0, best.A)),
    thetaRef,
    closureError: best.errA + best.errB,
  };
}

function solvePoseAtTheta(mech, theta, prevB) {
  const aWorld = add(mech.A, v(mech.rA * Math.cos(theta), mech.rA * Math.sin(theta)));

  const intersections = circleIntersections(aWorld, mech.couplerLen, mech.B, mech.rB);
  if (intersections.length === 0) {
    return null;
  }

  let bWorld = intersections[0];
  if (intersections.length === 2) {
    if (prevB) {
      bWorld = dist(intersections[0], prevB) < dist(intersections[1], prevB)
        ? intersections[0]
        : intersections[1];
    } else {
      const score0 = dot(sub(intersections[0], aWorld), sub(mech.b0, mech.a0));
      const score1 = dot(sub(intersections[1], aWorld), sub(mech.b0, mech.a0));
      bWorld = score0 > score1 ? intersections[0] : intersections[1];
    }
  }

  const phi = angleOf(sub(bWorld, aWorld)) - mech.localAngleAB;
  const couplerPoint = add(aWorld, rotate(mul(mech.aLocal, -1), phi));

  return {
    aWorld,
    bWorld,
    couplerPoint,
    phi,
  };
}

function runSynthesis() {
  if (state.targetPoints.length !== 4) {
    setStatus("Select exactly 4 points first.");
    return;
  }

  setStatus("Running high-accuracy Burmester synthesis...");
  let res;
  try {
    res = synthesizeMechanism(state.targetPoints);
  } catch (error) {
    state.mechanism = null;
    state.running = false;
    setStatus(`Synthesis crashed: ${error.message}`);
    return;
  }
  if (!res.ok) {
    state.mechanism = null;
    state.running = false;
    setStatus(`Synthesis failed: ${res.reason}`);
    return;
  }

  state.mechanism = res;
  state.trace = [];
  state.running = true;
  state.lastB = res.b0;
  state.lastPose = solvePoseAtTheta(res, res.theta0, res.b0);

  if (!state.lastPose) {
    state.running = false;
    setStatus("Synthesis produced an invalid initial pose.");
    return;
  }

  const info = `Mechanism synthesized. Residual closure error: ${res.closureError.toFixed(4)} px`;
  setStatus(info);
}

function drawScreenPoint(p, radius, fill, stroke, strokeWidth = 1.3) {
  ctx.beginPath();
  ctx.arc(p.x, p.y, radius, 0, Math.PI * 2);
  ctx.fillStyle = fill;
  ctx.fill();
  if (stroke) {
    ctx.strokeStyle = stroke;
    ctx.lineWidth = strokeWidth;
    ctx.stroke();
  }
}

function drawWorldPoint(p, radius, fill, stroke, strokeWidth = 1.3) {
  drawScreenPoint(worldToScreen(p), radius, fill, stroke, strokeWidth);
}

function drawWorldLine(a, b, color, width = 2, dash = null) {
  const as = worldToScreen(a);
  const bs = worldToScreen(b);
  ctx.beginPath();
  ctx.moveTo(as.x, as.y);
  ctx.lineTo(bs.x, bs.y);
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  if (dash) {
    ctx.setLineDash(dash);
  }
  ctx.stroke();
  if (dash) {
    ctx.setLineDash([]);
  }
}

function drawWorldCircle(center, radius, color, width = 1.4, dash = null) {
  const cs = worldToScreen(center);
  const sr = radius * state.camera.scale;
  ctx.beginPath();
  ctx.arc(cs.x, cs.y, sr, 0, Math.PI * 2);
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  if (dash) {
    ctx.setLineDash(dash);
  }
  ctx.stroke();
  if (dash) {
    ctx.setLineDash([]);
  }
}

function isMajorGridLine(value, step) {
  const ratio = value / step;
  return Math.abs(ratio - Math.round(ratio)) < 1e-6;
}

function drawSceneGrid() {
  const { width, height } = state.viewport;
  if (width <= 0 || height <= 0) {
    return;
  }

  const minorStep = 40;
  const majorStep = minorStep * 5;
  const topLeft = screenToWorld(v(0, 0));
  const bottomRight = screenToWorld(v(width, height));

  const minX = Math.min(topLeft.x, bottomRight.x);
  const maxX = Math.max(topLeft.x, bottomRight.x);
  const minY = Math.min(topLeft.y, bottomRight.y);
  const maxY = Math.max(topLeft.y, bottomRight.y);

  let count = 0;
  const maxLines = 260;

  const startX = Math.floor(minX / minorStep) * minorStep;
  const endX = Math.ceil(maxX / minorStep) * minorStep;
  const majorColor = isPresentationMode()
    ? "rgba(70, 110, 102, 0.14)"
    : "rgba(65, 95, 89, 0.22)";
  const minorColor = isPresentationMode()
    ? "rgba(70, 110, 102, 0.05)"
    : "rgba(65, 95, 89, 0.1)";
  const majorWidth = isPresentationMode() ? 1 : 1.3;
  const minorWidth = 1;

  for (let x = startX; x <= endX && count < maxLines; x += minorStep) {
    const sx = worldToScreen(v(x, 0)).x;
    const major = isMajorGridLine(x, majorStep);
    ctx.beginPath();
    ctx.moveTo(sx, 0);
    ctx.lineTo(sx, height);
    ctx.strokeStyle = major ? majorColor : minorColor;
    ctx.lineWidth = major ? majorWidth : minorWidth;
    ctx.stroke();
    count += 1;
  }

  const startY = Math.floor(minY / minorStep) * minorStep;
  const endY = Math.ceil(maxY / minorStep) * minorStep;
  for (let y = startY; y <= endY && count < maxLines; y += minorStep) {
    const sy = worldToScreen(v(0, y)).y;
    const major = isMajorGridLine(y, majorStep);
    ctx.beginPath();
    ctx.moveTo(0, sy);
    ctx.lineTo(width, sy);
    ctx.strokeStyle = major ? majorColor : minorColor;
    ctx.lineWidth = major ? majorWidth : minorWidth;
    ctx.stroke();
    count += 1;
  }
}

function drawGroundPivot(p, label) {
  const s = worldToScreen(p);

  ctx.fillStyle = "rgba(22, 31, 34, 0.88)";
  ctx.fillRect(s.x - 19, s.y + 12, 38, 7);

  ctx.beginPath();
  ctx.moveTo(s.x - 14, s.y + 12);
  ctx.lineTo(s.x + 14, s.y + 12);
  ctx.lineTo(s.x, s.y + 1);
  ctx.closePath();
  ctx.fillStyle = "rgba(49, 63, 68, 0.9)";
  ctx.fill();

  drawScreenPoint(s, 8.2, "#0f1417", "#f1f6f4", 1.6);
  drawScreenPoint(s, 2.6, "#f4faf7");

  ctx.beginPath();
  ctx.moveTo(s.x - 4, s.y);
  ctx.lineTo(s.x + 4, s.y);
  ctx.moveTo(s.x, s.y - 4);
  ctx.lineTo(s.x, s.y + 4);
  ctx.strokeStyle = "rgba(169, 192, 186, 0.78)";
  ctx.lineWidth = 1;
  ctx.stroke();

  ctx.font = "700 11px Space Grotesk";
  ctx.fillStyle = "#1f2a2c";
  ctx.fillText(label, s.x - 4, s.y - 14);
}

function drawPresentationPivot(p, label) {
  const s = worldToScreen(p);
  drawScreenPoint(s, 9, "#10231f", "#ecf9f4", 1.4);
  drawScreenPoint(s, 3.2, "#ecf9f4");

  ctx.beginPath();
  ctx.arc(s.x, s.y, 15.5, 0, Math.PI * 2);
  ctx.strokeStyle = "rgba(95, 158, 141, 0.45)";
  ctx.lineWidth = 1.3;
  ctx.stroke();

  ctx.font = "700 11px Space Grotesk";
  ctx.fillStyle = "#1f2a2c";
  ctx.fillText(label, s.x - 4, s.y - 16);
}

function drawViewBadge() {
  const { width } = state.viewport;
  const modeLabel = isPresentationMode() ? "Presentation" : "Engineering";
  const zoomLabel = `${modeLabel} ${state.camera.scale.toFixed(2)}x`;
  ctx.font = "600 12px Space Grotesk";
  const padX = 11;
  const textW = ctx.measureText(zoomLabel).width;
  const boxW = textW + padX * 2;
  const x = width - boxW - 12;
  const y = 12;
  ctx.fillStyle = "rgba(253, 250, 244, 0.9)";
  ctx.strokeStyle = "rgba(110, 132, 125, 0.65)";
  ctx.lineWidth = 1;
  ctx.fillRect(x, y, boxW, 26);
  ctx.strokeRect(x, y, boxW, 26);
  ctx.fillStyle = "#223332";
  ctx.fillText(zoomLabel, x + padX, y + 17.5);
}

function drawTargets() {
  ctx.font = "700 12px Space Grotesk";
  state.targetPoints.forEach((p, idx) => {
    const s = worldToScreen(p);
    drawScreenPoint(s, 6.2, "#d3542b", "#6f2a13", 1.5);
    drawScreenPoint(s, 2.3, "#ffece2");
    ctx.fillStyle = "#162020";
    ctx.fillText(String(idx + 1), s.x + 9, s.y - 9);
  });

  if (state.targetPoints.length > 1) {
    for (let i = 0; i < state.targetPoints.length - 1; i += 1) {
      drawWorldLine(
        state.targetPoints[i],
        state.targetPoints[i + 1],
        "rgba(211, 84, 43, 0.5)",
        1.5,
        [5, 4]
      );
    }
  }
}

function drawMechanism() {
  if (!state.mechanism || !state.lastPose) {
    return;
  }

  const mech = state.mechanism;
  const { A, B } = mech;
  const { aWorld, bWorld, couplerPoint } = state.lastPose;
  const presentation = isPresentationMode();

  if (state.trace.length > 1) {
    const start = worldToScreen(state.trace[0]);
    ctx.beginPath();
    ctx.moveTo(start.x, start.y);
    for (let i = 1; i < state.trace.length; i += 1) {
      const p = worldToScreen(state.trace[i]);
      ctx.lineTo(p.x, p.y);
    }
    if (presentation) {
      ctx.strokeStyle = "rgba(26, 132, 114, 0.95)";
      ctx.lineWidth = 3.2;
      ctx.shadowColor = "rgba(22, 140, 118, 0.28)";
      ctx.shadowBlur = 9;
    } else {
      ctx.strokeStyle = "#1e7666";
      ctx.lineWidth = 2.5;
      ctx.shadowBlur = 0;
    }
    ctx.stroke();
    ctx.shadowBlur = 0;
  }

  if (!presentation) {
    drawWorldCircle(A, mech.rA, "rgba(51, 79, 102, 0.34)", 1.2, [8, 7]);
    drawWorldCircle(B, mech.rB, "rgba(51, 79, 102, 0.28)", 1.2, [8, 7]);
  }

  const as = worldToScreen(aWorld);
  const bs = worldToScreen(bWorld);
  const cs = worldToScreen(couplerPoint);
  ctx.beginPath();
  ctx.moveTo(as.x, as.y);
  ctx.lineTo(bs.x, bs.y);
  ctx.lineTo(cs.x, cs.y);
  ctx.closePath();
  ctx.fillStyle = presentation ? "rgba(31, 137, 118, 0.26)" : "rgba(30, 118, 102, 0.18)";
  ctx.fill();
  ctx.strokeStyle = presentation ? "rgba(19, 84, 72, 0.62)" : "rgba(19, 84, 72, 0.52)";
  ctx.lineWidth = presentation ? 1.6 : 1.4;
  ctx.stroke();

  if (presentation) {
    drawWorldLine(A, B, "rgba(36, 64, 60, 0.68)", 4.3);
    drawWorldLine(A, aWorld, "#2f556f", 3.9);
    drawWorldLine(aWorld, bWorld, "#177661", 4.6);
    drawWorldLine(bWorld, B, "#2f556f", 3.9);
    drawPresentationPivot(A, "A");
    drawPresentationPivot(B, "B");
  } else {
    drawWorldLine(A, B, "rgba(44, 53, 57, 0.75)", 4.8);
    drawWorldLine(A, aWorld, "#344f66", 3.8);
    drawWorldLine(aWorld, bWorld, "#1c6c5d", 4.2);
    drawWorldLine(bWorld, B, "#344f66", 3.8);
    drawGroundPivot(A, "A");
    drawGroundPivot(B, "B");
  }

  drawWorldPoint(aWorld, 5.5, "#f2b79e", "#7f3c25", 1.5);
  drawWorldPoint(bWorld, 5.5, "#f2b79e", "#7f3c25", 1.5);
  drawWorldPoint(
    couplerPoint,
    presentation ? 6.5 : 6.1,
    presentation ? "#178870" : "#1e7666",
    presentation ? "#043b32" : "#083d34",
    1.8
  );
}

function update(dt) {
  if (!state.running || !state.mechanism || !state.lastPose) {
    return;
  }

  const mech = state.mechanism;
  mech.theta0 += state.omega * dt;

  const pose = solvePoseAtTheta(mech, mech.theta0, state.lastB);
  if (!pose) {
    state.running = false;
    setStatus("Motion branch hit a non-assembly state. Click Create Synthesis again.");
    return;
  }

  state.lastB = pose.bWorld;
  state.lastPose = pose;
  state.trace.push(pose.couplerPoint);

  if (state.trace.length > 2400) {
    state.trace.shift();
  }
}

function render() {
  const { width, height } = state.viewport;
  ctx.clearRect(0, 0, width, height);

  drawSceneGrid();
  drawTargets();
  drawMechanism();
  drawViewBadge();

  pointCountEl.textContent = `${state.targetPoints.length} / 4`;
}

function loop(ts) {
  if (!state.lastTime) {
    state.lastTime = ts;
  }

  const dt = clamp((ts - state.lastTime) / 1000, 0, 0.05);
  state.lastTime = ts;

  update(dt);
  updateCamera(dt);
  render();
  requestAnimationFrame(loop);
}

canvas.addEventListener("click", (evt) => {
  const rect = canvas.getBoundingClientRect();
  const screen = v(evt.clientX - rect.left, evt.clientY - rect.top);
  const world = screenToWorld(screen);

  if (state.targetPoints.length < 4) {
    state.targetPoints.push(world);
    setStatus(`Point ${state.targetPoints.length} added.`);
    if (state.targetPoints.length === 4) {
      setStatus("4 points ready. Click Create Synthesis.");
    }
  } else {
    setStatus("Already 4 points selected. Use Reset Points to pick a new set.");
  }
});

createBtn.addEventListener("click", runSynthesis);

resetBtn.addEventListener("click", () => {
  state.targetPoints = [];
  state.mechanism = null;
  state.trace = [];
  state.running = false;
  state.lastB = null;
  state.lastPose = null;
  state.camera.targetScale = 1;
  setStatus("Points reset. Click 4 new target points.");
});

clearTraceBtn.addEventListener("click", () => {
  state.trace = [];
  setStatus("Trace cleared.");
});

if (modeBtn) {
  modeBtn.addEventListener("click", () => {
    state.viewMode = state.viewMode === "engineering" ? "presentation" : "engineering";
    updateModeUI();
    setStatus(
      state.viewMode === "presentation"
        ? "View mode switched to Presentation."
        : "View mode switched to Engineering."
    );
  });
}

window.addEventListener("resize", () => {
  resizeCanvas();
  render();
});

resizeCanvas();
updateModeUI();
requestAnimationFrame(loop);
