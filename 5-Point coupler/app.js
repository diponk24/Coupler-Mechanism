const TWO_PI = Math.PI * 2;
const POINT_COUNT = 5;

class Vec2 {
  static add(a, b) {
    return { x: a.x + b.x, y: a.y + b.y };
  }

  static sub(a, b) {
    return { x: a.x - b.x, y: a.y - b.y };
  }

  static scale(v, s) {
    return { x: v.x * s, y: v.y * s };
  }

  static dot(a, b) {
    return a.x * b.x + a.y * b.y;
  }

  static cross(a, b) {
    return a.x * b.y - a.y * b.x;
  }

  static length(v) {
    return Math.hypot(v.x, v.y);
  }

  static distance(a, b) {
    return Math.hypot(a.x - b.x, a.y - b.y);
  }

  static normalize(v) {
    const len = Vec2.length(v);
    if (len < 1e-12) {
      return { x: 1, y: 0 };
    }
    return { x: v.x / len, y: v.y / len };
  }

  static perp(v) {
    return { x: -v.y, y: v.x };
  }

  static angle(v) {
    return Math.atan2(v.y, v.x);
  }
}

function clamp(value, lo, hi) {
  return Math.min(hi, Math.max(lo, value));
}

function softplus(x) {
  if (x > 20) return x;
  if (x < -20) return Math.exp(x);
  return Math.log1p(Math.exp(x));
}

function invSoftplus(y) {
  if (y > 20) return y;
  return Math.log(Math.expm1(Math.max(y, 1e-9)));
}

function unwrapAngles(raw) {
  if (!raw.length) return [];
  const out = [raw[0]];
  for (let i = 1; i < raw.length; i += 1) {
    let value = raw[i];
    while (value - out[i - 1] > Math.PI) value -= TWO_PI;
    while (value - out[i - 1] < -Math.PI) value += TWO_PI;
    out.push(value);
  }
  return out;
}

function centroid(points) {
  const sum = points.reduce((acc, p) => Vec2.add(acc, p), { x: 0, y: 0 });
  return Vec2.scale(sum, 1 / Math.max(points.length, 1));
}

function boundingBox(points) {
  const xs = points.map((p) => p.x);
  const ys = points.map((p) => p.y);
  const minX = Math.min(...xs);
  const maxX = Math.max(...xs);
  const minY = Math.min(...ys);
  const maxY = Math.max(...ys);
  return {
    minX,
    maxX,
    minY,
    maxY,
    width: maxX - minX,
    height: maxY - minY,
    center: { x: (minX + maxX) * 0.5, y: (minY + maxY) * 0.5 },
    scale: Math.max(maxX - minX, maxY - minY, 1),
  };
}

function spreadRatio(points) {
  const c = centroid(points);
  let xx = 0;
  let xy = 0;
  let yy = 0;
  for (const p of points) {
    const dx = p.x - c.x;
    const dy = p.y - c.y;
    xx += dx * dx;
    xy += dx * dy;
    yy += dy * dy;
  }

  const tr = xx + yy;
  const det = xx * yy - xy * xy;
  const disc = Math.max(0, tr * tr - 4 * det);
  const s = Math.sqrt(disc);
  const l1 = 0.5 * (tr + s);
  const l2 = 0.5 * (tr - s);
  if (l1 < 1e-12) return 0;
  return l2 / l1;
}

function lineIntersection(p, r, q, s) {
  const denom = Vec2.cross(r, s);
  if (Math.abs(denom) < 1e-12) return null;
  const qp = Vec2.sub(q, p);
  const t = Vec2.cross(qp, s) / denom;
  return Vec2.add(p, Vec2.scale(r, t));
}

function perpendicularBisector(a, b) {
  const mid = Vec2.scale(Vec2.add(a, b), 0.5);
  const dir = Vec2.perp(Vec2.sub(b, a));
  return { point: mid, dir: Vec2.normalize(dir) };
}

function circumcenter(a, b, c) {
  const bis1 = perpendicularBisector(a, b);
  const bis2 = perpendicularBisector(b, c);
  return lineIntersection(bis1.point, bis1.dir, bis2.point, bis2.dir);
}

function circleIntersection(c1, r1, c2, r2, branch = 1) {
  const dVec = Vec2.sub(c2, c1);
  const d = Vec2.length(dVec);
  if (d < 1e-10) return null;

  const tol = 1e-7;
  if (d > r1 + r2 + tol) return null;
  if (d < Math.abs(r1 - r2) - tol) return null;

  const a = (r1 * r1 - r2 * r2 + d * d) / (2 * d);
  let h2 = r1 * r1 - a * a;
  if (h2 < 0 && h2 > -1e-8) h2 = 0;
  if (h2 < 0) return null;

  const h = Math.sqrt(h2);
  const u = Vec2.scale(dVec, 1 / d);
  const mid = Vec2.add(c1, Vec2.scale(u, a));
  const offset = Vec2.scale(Vec2.perp(u), h * (branch >= 0 ? 1 : -1));
  return Vec2.add(mid, offset);
}

function estimateAnglesFromCenter(center, points) {
  const raw = points.map((p) => Vec2.angle(Vec2.sub(p, center)));
  let unwrapped = unwrapAngles(raw);
  let span = unwrapped[unwrapped.length - 1] - unwrapped[0];

  if (Math.abs(span) < 1e-3) {
    return points.map((_, i) => i * 0.32);
  }

  if (span < 0) {
    unwrapped = unwrapped.map((value) => -value);
    span = -span;
  }

  if (span < 0.2) {
    const step = 0.22;
    return points.map((_, i) => i * step);
  }

  return unwrapped;
}

function solveLinearSystem(A, b) {
  const n = A.length;
  const M = A.map((row, i) => [...row, b[i]]);

  for (let col = 0; col < n; col += 1) {
    let pivot = col;
    for (let row = col + 1; row < n; row += 1) {
      if (Math.abs(M[row][col]) > Math.abs(M[pivot][col])) {
        pivot = row;
      }
    }

    if (Math.abs(M[pivot][col]) < 1e-12) {
      return null;
    }

    if (pivot !== col) {
      [M[pivot], M[col]] = [M[col], M[pivot]];
    }

    const invPivot = 1 / M[col][col];
    for (let j = col; j <= n; j += 1) {
      M[col][j] *= invPivot;
    }

    for (let row = 0; row < n; row += 1) {
      if (row === col) continue;
      const factor = M[row][col];
      if (Math.abs(factor) < 1e-16) continue;
      for (let j = col; j <= n; j += 1) {
        M[row][j] -= factor * M[col][j];
      }
    }
  }

  return M.map((row) => row[n]);
}

function fitQuality(maxError) {
  if (maxError < 1.5) return "high";
  if (maxError < 4.0) return "moderate";
  return "low";
}

function randNormal() {
  let u = 0;
  let v = 0;
  while (u === 0) u = Math.random();
  while (v === 0) v = Math.random();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(TWO_PI * v);
}

class FourBarMechanism {
  constructor(params) {
    this.O2 = params.O2;
    this.O4 = params.O4;
    this.r2 = params.r2;
    this.r3 = params.r3;
    this.r4 = params.r4;
    this.px = params.px;
    this.py = params.py;
    this.branch = params.branch;
    this.thetaList = params.thetaList;
  }

  solvePose(theta) {
    const A = {
      x: this.O2.x + this.r2 * Math.cos(theta),
      y: this.O2.y + this.r2 * Math.sin(theta),
    };

    const B = circleIntersection(A, this.r3, this.O4, this.r4, this.branch);
    if (!B) return null;

    const ab = Vec2.sub(B, A);
    const u = Vec2.normalize(ab);
    const n = Vec2.perp(u);

    const P = Vec2.add(A, Vec2.add(Vec2.scale(u, this.px), Vec2.scale(n, this.py)));
    return { A, B, P, theta };
  }

  precisionPoses() {
    const poses = [];
    for (const theta of this.thetaList) {
      const pose = this.solvePose(theta);
      if (!pose) return null;
      poses.push(pose);
    }
    return poses;
  }

  sampleCouplerPath(samples = 420) {
    const out = [];
    if (this.thetaList.length < 2) return out;
    const start = this.thetaList[0];
    const end = this.thetaList[this.thetaList.length - 1];
    const span = end - start;
    if (span < 1e-6) return out;

    const pad = Math.min(0.45 * Math.abs(span), 0.7);
    const t0 = start - pad;
    const t1 = end + pad;

    for (let i = 0; i <= samples; i += 1) {
      const t = i / samples;
      const theta = t0 + (t1 - t0) * t;
      const pose = this.solvePose(theta);
      if (pose) out.push(pose.P);
    }
    return out;
  }

  toJSON(errors, precisionTargets, construction, precisionPoses = null) {
    const poses = precisionPoses ?? this.precisionPoses() ?? [];
    return {
      mechanism: {
        fixedPivots: { O2: this.O2, O4: this.O4 },
        movingPivots: {
          inputPivotPath: poses.map((p) => p.A),
          outputPivotPath: poses.map((p) => p.B),
        },
        links: {
          crank: this.r2,
          coupler: this.r3,
          follower: this.r4,
          couplerPointOffset: { x: this.px, y: this.py },
          inputAngles: this.thetaList,
          assemblyBranch: this.branch,
        },
      },
      precisionTargets,
      synthesisError: errors,
      construction,
      generatedAt: new Date().toISOString(),
    };
  }
}

class FivePointSynthesisSolver {
  constructor() {
    this.maxRestarts = 220;
    this.maxIterations = 140;
    this.minStep = 0.03;
  }

  solve(points) {
    if (points.length !== POINT_COUNT) {
      return { ok: false, message: "Select exactly five precision points." };
    }

    const ratio = spreadRatio(points);
    if (ratio < 0.008) {
      return {
        ok: false,
        message: "Precision points are nearly collinear. Move at least one point away from the main line.",
      };
    }

    const candidates = [];

    const direct = this.solveForOrderedPoints(points);
    if (direct) candidates.push({ ...direct, reversed: false });

    const reversedPoints = [...points].reverse();
    const reverseCandidate = this.solveForOrderedPoints(reversedPoints);
    if (reverseCandidate) candidates.push({ ...reverseCandidate, reversed: true });

    if (!candidates.length) {
      return {
        ok: false,
        message: "No feasible four-bar mechanism was found for these five points.",
      };
    }

    candidates.sort((a, b) => a.score - b.score);
    const best = candidates[0];

    let precisionPoses = best.precisionPoses;
    let perPoint = [...best.errors.perPoint];
    if (best.reversed) {
      precisionPoses = [...precisionPoses].reverse();
      perPoint = [...perPoint].reverse();
      best.construction = this.computeConstructionData(points, precisionPoses);
    }

    const rms = Math.sqrt(perPoint.reduce((sum, e) => sum + e * e, 0) / perPoint.length);
    const max = Math.max(...perPoint);

    return {
      ok: true,
      mechanism: best.mechanism,
      precisionPoses,
      trace: best.mechanism.sampleCouplerPath(460),
      construction: best.construction,
      errors: {
        perPoint,
        rms,
        max,
      },
      message:
        max < 1.5
          ? "Exact-quality synthesis found. The coupler passes through all 5 points with very small residual error."
          : "Approximate synthesis found via Burmester-seeded nonlinear optimization.",
    };
  }

  solveForOrderedPoints(points) {
    const box = boundingBox(points);
    const seeds = this.buildInitialSeeds(points, box);
    let best = null;

    for (const seed of seeds) {
      const optimized = this.optimize(seed.x0, points, box.scale, seed.branch, {
        regWeight: 0.45,
        maxIterations: this.maxIterations,
      });
      if (!optimized) continue;
      const evaluated = this.evaluateVectorCandidate(optimized.x, optimized.branch, points, box.scale);
      if (!evaluated) continue;
      if (!best || evaluated.score < best.score) best = evaluated;
    }

    if (!best) return null;

    let polished = best;
    const regSchedule = [0.2, 0.08, 0.03, 0];
    for (const regWeight of regSchedule) {
      const refined = this.optimize(polished.x, points, box.scale, polished.branch, {
        regWeight,
        maxIterations: this.maxIterations + 40,
      });
      if (!refined) continue;
      const evaluated = this.evaluateVectorCandidate(refined.x, refined.branch, points, box.scale);
      if (evaluated && evaluated.score < polished.score) {
        polished = evaluated;
      }
    }

    // Extra strict-search phase: perturb around the current best and re-optimize.
    // This helps escape local minima and drive maximum point error down.
    const perturbBatches = [
      { count: 14, strength: 0.85, regWeight: 0.05 },
      { count: 14, strength: 0.45, regWeight: 0.02 },
      { count: 10, strength: 0.2, regWeight: 0 },
    ];
    for (const batch of perturbBatches) {
      for (let i = 0; i < batch.count; i += 1) {
        const guess = this.perturbVector(polished.x, box.scale, batch.strength);
        const refined = this.optimize(guess, points, box.scale, polished.branch, {
          regWeight: batch.regWeight,
          maxIterations: this.maxIterations + 55,
        });
        if (!refined) continue;
        const evaluated = this.evaluateVectorCandidate(refined.x, refined.branch, points, box.scale);
        if (evaluated && evaluated.score < polished.score) {
          polished = evaluated;
        }
      }
    }

    return {
      mechanism: polished.mechanism,
      precisionPoses: polished.precisionPoses,
      errors: polished.errors,
      compactness: polished.compactness,
      score: polished.score,
      construction: this.computeConstructionData(points, polished.precisionPoses),
    };
  }

  computeCompactness(mechanism, poses, points) {
    const all = [mechanism.O2, mechanism.O4];
    for (const pose of poses) {
      all.push(pose.A, pose.B, pose.P);
    }

    const box = boundingBox(all);
    const precisionBox = boundingBox(points);
    const c = centroid(points);
    const centerOffset = Vec2.distance(box.center, c) / precisionBox.scale;
    const extentRatio = Math.max(box.width, box.height) / precisionBox.scale;
    const g = Vec2.distance(mechanism.O2, mechanism.O4);
    const linkRatio = Math.max(mechanism.r2, mechanism.r3, mechanism.r4, g) / precisionBox.scale;

    const penalty =
      clamp((extentRatio - 2.2) / 1.3, 0, 10) * 0.6 +
      clamp((centerOffset - 1.1) / 1.2, 0, 10) * 0.9 +
      clamp((linkRatio - 2.5) / 1.5, 0, 10) * 0.55;

    return {
      extentRatio,
      centerOffset,
      linkRatio,
      penalty,
    };
  }

  evaluateVectorCandidate(x, branch, points, scale) {
    const mechanism = this.vectorToMechanism(x, branch, scale);
    const precisionPoses = mechanism.precisionPoses();
    if (!precisionPoses) return null;

    const perPoint = precisionPoses.map((pose, i) => Vec2.distance(pose.P, points[i]));
    const rms = Math.sqrt(perPoint.reduce((sum, e) => sum + e * e, 0) / perPoint.length);
    const max = Math.max(...perPoint);
    const errors = { perPoint, rms, max };
    const compactness = this.computeCompactness(mechanism, precisionPoses, points);

    // Precision is the primary target, especially worst-point error.
    const score = 2.0 * max + 0.85 * rms + 0.03 * compactness.penalty;

    return {
      x,
      branch,
      mechanism,
      precisionPoses,
      errors,
      compactness,
      score,
    };
  }

  buildInitialSeeds(points, box) {
    const seeds = [];
    const center = centroid(points);
    const s = box.scale;

    const construction = this.computeConstructionSkeleton(points);
    const hr = construction.hr ?? {
      x: center.x + 0.58 * s,
      y: center.y - 0.18 * s,
    };
    const hc = construction.hc ?? {
      x: center.x - 0.56 * s,
      y: center.y + 0.32 * s,
    };

    const geometricTheta = estimateAnglesFromCenter(hr, points);
    const circum = circumcenter(points[0], points[1], points[2]);
    const circumTheta = estimateAnglesFromCenter(circum ?? center, points);

    const baseParams = {
      O2: hc,
      O4: hr,
      r2: Math.max(18, s * 0.42),
      r3: Math.max(24, s * 0.92),
      r4: Math.max(20, s * 0.7),
      px: s * 0.36,
      py: s * 0.14,
      thetaList: geometricTheta,
    };

    for (const branch of [1, -1]) {
      seeds.push({ branch, x0: this.paramsToVector(baseParams, s) });
    }

    const altParams = {
      O2: {
        x: center.x - 0.72 * s,
        y: center.y + 0.12 * s,
      },
      O4: {
        x: center.x + 0.66 * s,
        y: center.y - 0.28 * s,
      },
      r2: Math.max(16, s * 0.32),
      r3: Math.max(26, s * 1.15),
      r4: Math.max(16, s * 0.72),
      px: s * 0.25,
      py: -s * 0.08,
      thetaList: circumTheta,
    };

    for (const branch of [1, -1]) {
      seeds.push({ branch, x0: this.paramsToVector(altParams, s) });
    }

    for (let i = 0; i < this.maxRestarts; i += 1) {
      const a1 = Math.random() * TWO_PI;
      const a2 = Math.random() * TWO_PI;
      const rO2 = s * (0.45 + Math.random() * 1.35);
      const rO4 = s * (0.45 + Math.random() * 1.35);

      const O2 = {
        x: center.x + rO2 * Math.cos(a1),
        y: center.y + rO2 * Math.sin(a1),
      };
      const O4 = {
        x: center.x + rO4 * Math.cos(a2),
        y: center.y + rO4 * Math.sin(a2),
      };

      const centerForTheta = Math.random() > 0.5 ? O4 : center;
      const thetaSeed = estimateAnglesFromCenter(centerForTheta, points).map(
        (value) => value * (0.7 + Math.random() * 0.9),
      );

      const params = {
        O2,
        O4,
        r2: Math.max(12, s * (0.18 + Math.random() * 1.45)),
        r3: Math.max(12, s * (0.3 + Math.random() * 2.0)),
        r4: Math.max(12, s * (0.18 + Math.random() * 1.45)),
        px: s * (-0.85 + Math.random() * 1.7),
        py: s * (-0.85 + Math.random() * 1.7),
        thetaList: thetaSeed,
      };

      seeds.push({
        branch: Math.random() > 0.5 ? 1 : -1,
        x0: this.paramsToVector(params, s),
      });
    }

    return seeds;
  }

  perturbVector(x, scale, strength) {
    const out = [...x];
    const s = strength;
    out[0] += randNormal() * 0.13 * scale * s;
    out[1] += randNormal() * 0.13 * scale * s;
    out[2] += randNormal() * 0.13 * scale * s;
    out[3] += randNormal() * 0.13 * scale * s;

    out[4] += randNormal() * 0.24 * s;
    out[5] += randNormal() * 0.24 * s;
    out[6] += randNormal() * 0.24 * s;

    out[7] += randNormal() * 0.14 * scale * s;
    out[8] += randNormal() * 0.14 * scale * s;

    out[9] += randNormal() * 0.17 * s;
    out[10] += randNormal() * 0.13 * s;
    out[11] += randNormal() * 0.13 * s;
    out[12] += randNormal() * 0.13 * s;
    out[13] += randNormal() * 0.13 * s;
    return out;
  }

  paramsToVector(params, scale) {
    const minLen = Math.max(8, 0.03 * scale);

    const theta = [...params.thetaList];
    if (theta.length !== POINT_COUNT) {
      theta.length = POINT_COUNT;
      for (let i = 0; i < POINT_COUNT; i += 1) {
        theta[i] = i * 0.28;
      }
    }

    const unwrapped = unwrapAngles(theta);
    while (unwrapped[unwrapped.length - 1] <= unwrapped[0]) {
      unwrapped[unwrapped.length - 1] += TWO_PI;
    }

    const d1 = Math.max(this.minStep, unwrapped[1] - unwrapped[0]);
    const d2 = Math.max(this.minStep, unwrapped[2] - unwrapped[1]);
    const d3 = Math.max(this.minStep, unwrapped[3] - unwrapped[2]);
    const d4 = Math.max(this.minStep, unwrapped[4] - unwrapped[3]);

    return [
      params.O2.x,
      params.O2.y,
      params.O4.x,
      params.O4.y,
      invSoftplus(Math.max(minLen, params.r2) - minLen),
      invSoftplus(Math.max(minLen, params.r3) - minLen),
      invSoftplus(Math.max(minLen, params.r4) - minLen),
      params.px,
      params.py,
      unwrapped[0],
      invSoftplus(d1 - this.minStep),
      invSoftplus(d2 - this.minStep),
      invSoftplus(d3 - this.minStep),
      invSoftplus(d4 - this.minStep),
    ];
  }

  vectorToMechanism(x, branch, scale) {
    const minLen = Math.max(8, 0.03 * scale);

    const r2 = minLen + softplus(x[4]);
    const r3 = minLen + softplus(x[5]);
    const r4 = minLen + softplus(x[6]);

    const theta0 = x[9];
    const d1 = this.minStep + softplus(x[10]);
    const d2 = this.minStep + softplus(x[11]);
    const d3 = this.minStep + softplus(x[12]);
    const d4 = this.minStep + softplus(x[13]);

    const thetaList = [
      theta0,
      theta0 + d1,
      theta0 + d1 + d2,
      theta0 + d1 + d2 + d3,
      theta0 + d1 + d2 + d3 + d4,
    ];

    return new FourBarMechanism({
      O2: { x: x[0], y: x[1] },
      O4: { x: x[2], y: x[3] },
      r2,
      r3,
      r4,
      px: x[7],
      py: x[8],
      branch,
      thetaList,
    });
  }

  residualVector(x, points, scale, branch, regWeight = 1) {
    const mechanism = this.vectorToMechanism(x, branch, scale);
    const residual = [];

    const poses = mechanism.precisionPoses();
    if (!poses) {
      for (let i = 0; i < POINT_COUNT; i += 1) {
        residual.push(20, 20);
      }
    } else {
      for (let i = 0; i < POINT_COUNT; i += 1) {
        const ex = (poses[i].P.x - points[i].x) / scale;
        const ey = (poses[i].P.y - points[i].y) / scale;
        const errMag = Math.hypot(ex, ey);
        // Adaptive reweighting tightens the worst-off precision points.
        const w = 1 + 7.5 * Math.pow(errMag, 1.35);
        residual.push(ex * w, ey * w);
        residual.push(errMag * errMag * 0.35);
      }
    }

    const center = centroid(points);
    const g = Vec2.distance(mechanism.O2, mechanism.O4);
    const minLink = Math.min(mechanism.r2, mechanism.r3, mechanism.r4);
    const maxLink = Math.max(mechanism.r2, mechanism.r3, mechanism.r4);
    const angleSpan = mechanism.thetaList[POINT_COUNT - 1] - mechanism.thetaList[0];

    // Regularization terms keep the optimization away from numerically fragile regions.
    residual.push(clamp((0.11 * scale - g) / scale, 0, 10) * 0.8 * regWeight);
    residual.push(clamp((g - 2.7 * scale) / scale, 0, 10) * 0.72 * regWeight);
    residual.push(clamp((maxLink - 2.8 * scale) / scale, 0, 10) * 0.68 * regWeight);
    residual.push(clamp((maxLink / Math.max(minLink, 1e-9) - 7.5) / 4, 0, 10) * 0.18 * regWeight);
    residual.push((Vec2.distance(mechanism.O2, center) / (6 * scale)) * 0.24 * regWeight);
    residual.push((Vec2.distance(mechanism.O4, center) / (6 * scale)) * 0.24 * regWeight);
    residual.push(clamp((angleSpan - TWO_PI * 1.65) / 2.4, 0, 10) * 0.12 * regWeight);

    if (poses) {
      for (const pose of poses) {
        const ba = Vec2.normalize(Vec2.sub(pose.A, pose.B));
        const bo4 = Vec2.normalize(Vec2.sub(mechanism.O4, pose.B));
        const sinMu = Math.abs(Vec2.cross(ba, bo4));
        residual.push(clamp((0.08 - sinMu) * 6.2, 0, 2.5) * regWeight);
      }
    }

    return residual;
  }

  optimize(x0, points, scale, branch, options = {}) {
    let x = [...x0];
    const n = x.length;
    const regWeight = options.regWeight ?? 1;
    const maxIterations = options.maxIterations ?? this.maxIterations;
    let lambda = 0.022;
    let prevCost = Infinity;

    for (let iter = 0; iter < maxIterations; iter += 1) {
      const r = this.residualVector(x, points, scale, branch, regWeight);
      const m = r.length;
      const cost = r.reduce((sum, value) => sum + value * value, 0) / m;

      if (Math.abs(prevCost - cost) < 1e-11) break;
      prevCost = cost;

      const J = Array.from({ length: m }, () => Array(n).fill(0));
      for (let j = 0; j < n; j += 1) {
        const eps = 1e-4 * (1 + Math.abs(x[j]));
        const xh = [...x];
        xh[j] += eps;
        const rh = this.residualVector(xh, points, scale, branch, regWeight);
        for (let i = 0; i < m; i += 1) {
          J[i][j] = (rh[i] - r[i]) / eps;
        }
      }

      const JTJ = Array.from({ length: n }, () => Array(n).fill(0));
      const JTr = Array(n).fill(0);

      for (let i = 0; i < m; i += 1) {
        for (let j = 0; j < n; j += 1) {
          JTr[j] += J[i][j] * r[i];
          for (let k = 0; k < n; k += 1) {
            JTJ[j][k] += J[i][j] * J[i][k];
          }
        }
      }

      for (let d = 0; d < n; d += 1) {
        JTJ[d][d] += lambda * (1 + JTJ[d][d]);
      }

      const delta = solveLinearSystem(JTJ, JTr.map((v) => -v));
      if (!delta) break;

      const stepNorm = Math.hypot(...delta);
      if (stepNorm < 1e-7) break;

      const candidate = x.map((value, i) => value + delta[i]);
      const rCandidate = this.residualVector(candidate, points, scale, branch, regWeight);
      const costCandidate = rCandidate.reduce((sum, value) => sum + value * value, 0) / rCandidate.length;

      if (costCandidate < cost) {
        x = candidate;
        lambda = Math.max(1e-5, lambda * 0.62);
      } else {
        lambda = Math.min(12, lambda * 2.35);
      }
    }

    const finalResidual = this.residualVector(x, points, scale, branch, regWeight);
    const cost = finalResidual.reduce((sum, value) => sum + value * value, 0) / finalResidual.length;

    return {
      x,
      branch,
      cost,
    };
  }

  computeConstructionSkeleton(points) {
    const c15 = perpendicularBisector(points[0], points[4]);
    const c23 = perpendicularBisector(points[1], points[2]);
    const hr = lineIntersection(c15.point, c15.dir, c23.point, c23.dir);

    const c12 = perpendicularBisector(points[0], points[1]);
    const c34 = perpendicularBisector(points[2], points[3]);
    const hc = lineIntersection(c12.point, c12.dir, c34.point, c34.dir);

    return { c15, c23, c12, c34, hr, hc };
  }

  computeRelativePoles(poses) {
    const poles = [];
    for (let i = 0; i < poses.length - 1; i += 1) {
      const bisA = perpendicularBisector(poses[i].A, poses[i + 1].A);
      const bisB = perpendicularBisector(poses[i].B, poses[i + 1].B);
      const pole = lineIntersection(bisA.point, bisA.dir, bisB.point, bisB.dir);
      if (pole) poles.push({ index: i + 1, point: pole });
    }
    return poles;
  }

  computeConstructionData(points, poses) {
    const skeleton = this.computeConstructionSkeleton(points);
    const poles = poses ? this.computeRelativePoles(poses) : [];

    let a12 = null;
    let a23 = null;
    let hcFromA = null;

    if (poses?.length === POINT_COUNT) {
      a12 = perpendicularBisector(poses[0].A, poses[1].A);
      a23 = perpendicularBisector(poses[1].A, poses[2].A);
      hcFromA = lineIntersection(a12.point, a12.dir, a23.point, a23.dir);
    }

    return {
      ...skeleton,
      a12,
      a23,
      hcFromA,
      poles,
    };
  }
}

class SynthesisApp {
  constructor() {
    this.canvas = document.getElementById("synthesisCanvas");
    this.ctx = this.canvas.getContext("2d");

    this.pointList = document.getElementById("pointList");
    this.metrics = document.getElementById("metrics");
    this.statusText = document.getElementById("statusText");

    this.recomputeBtn = document.getElementById("recomputeBtn");
    this.resetBtn = document.getElementById("resetBtn");
    this.toggleAnimBtn = document.getElementById("toggleAnimBtn");
    this.toggleConstructionBtn = document.getElementById("toggleConstructionBtn");
    this.exportJsonBtn = document.getElementById("exportJsonBtn");
    this.exportPngBtn = document.getElementById("exportPngBtn");
    this.themeBtn = document.getElementById("themeBtn");

    this.solver = new FivePointSynthesisSolver();

    this.points = [];
    this.solution = null;

    this.dragIndex = -1;
    this.hoverIndex = -1;

    this.isAnimating = true;
    this.showConstruction = false;
    this.animationClock = 0;
    this.lastTs = performance.now();

    this.bindEvents();
    this.resize();
    this.canvas.style.cursor = "crosshair";

    this.refreshPointList();
    this.refreshMetrics();
    this.setStatus("Click the canvas to place 5 precision points.", false);

    requestAnimationFrame((ts) => this.render(ts));
  }

  bindEvents() {
    window.addEventListener("resize", () => this.resize());

    this.canvas.addEventListener("pointerdown", (event) => this.onPointerDown(event));
    this.canvas.addEventListener("pointermove", (event) => this.onPointerMove(event));
    window.addEventListener("pointerup", () => this.onPointerUp());
    window.addEventListener("pointercancel", () => this.onPointerUp());

    // Fallback listeners improve reliability on browsers/environments
    // where pointer events are inconsistent for local file previews.
    this.canvas.addEventListener("mousedown", (event) => this.onPointerDown(event));
    window.addEventListener("mousemove", (event) => this.onPointerMove(event));
    window.addEventListener("mouseup", () => this.onPointerUp());

    this.canvas.addEventListener(
      "touchstart",
      (event) => {
        event.preventDefault();
        this.onPointerDown(event);
      },
      { passive: false },
    );
    this.canvas.addEventListener(
      "touchmove",
      (event) => {
        event.preventDefault();
        this.onPointerMove(event);
      },
      { passive: false },
    );
    window.addEventListener("touchend", () => this.onPointerUp());
    window.addEventListener("touchcancel", () => this.onPointerUp());

    this.recomputeBtn.addEventListener("click", () => this.recompute());
    this.resetBtn.addEventListener("click", () => this.reset());

    this.toggleAnimBtn.addEventListener("click", () => {
      this.isAnimating = !this.isAnimating;
      this.toggleAnimBtn.textContent = this.isAnimating ? "Pause" : "Play";
      this.toggleAnimBtn.classList.toggle("primary", !this.isAnimating);
    });

    this.toggleConstructionBtn.addEventListener("click", () => {
      this.showConstruction = !this.showConstruction;
      this.toggleConstructionBtn.classList.toggle("primary", this.showConstruction);
    });

    this.exportJsonBtn.addEventListener("click", () => this.exportJSON());
    this.exportPngBtn.addEventListener("click", () => this.exportPNG());

    this.themeBtn.addEventListener("click", () => {
      const next = document.body.dataset.theme === "dark" ? "light" : "dark";
      document.body.dataset.theme = next;
      this.themeBtn.textContent = next === "dark" ? "Light" : "Dark";
    });
  }

  resize() {
    const dpr = window.devicePixelRatio || 1;
    const rect = this.canvas.getBoundingClientRect();
    this.canvas.width = Math.floor(rect.width * dpr);
    this.canvas.height = Math.floor(rect.height * dpr);
    this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    this.width = rect.width;
    this.height = rect.height;
  }

  canvasPointFromEvent(event) {
    let source = event;
    if (event.touches?.length) source = event.touches[0];
    else if (event.changedTouches?.length) source = event.changedTouches[0];

    const rect = this.canvas.getBoundingClientRect();
    return {
      x: source.clientX - rect.left,
      y: source.clientY - rect.top,
    };
  }

  nearestPointIndex(p) {
    let best = -1;
    let bestDist = 13;
    for (let i = 0; i < this.points.length; i += 1) {
      const d = Vec2.distance(this.points[i], p);
      if (d < bestDist) {
        bestDist = d;
        best = i;
      }
    }
    return best;
  }

  onPointerDown(event) {
    const p = this.canvasPointFromEvent(event);
    const near = this.nearestPointIndex(p);

    if (near >= 0) {
      this.dragIndex = near;
      return;
    }

    if (this.points.length >= POINT_COUNT) {
      this.setStatus("Exactly 5 points are allowed. Drag any point to edit or click Reset.", true);
      return;
    }

    this.points.push(p);
    this.solution = null;
    this.refreshPointList();
    this.refreshMetrics();

    if (this.points.length === POINT_COUNT) {
      this.recompute();
    } else {
      this.setStatus(`Point ${this.points.length}/${POINT_COUNT} placed.`, false);
    }
  }

  onPointerMove(event) {
    const p = this.canvasPointFromEvent(event);

    if (this.dragIndex >= 0) {
      this.points[this.dragIndex] = p;
      this.solution = null;
      this.refreshPointList();
      this.refreshMetrics();
      this.setStatus("Dragging point. Release to recompute synthesis.", false);
      return;
    }

    this.hoverIndex = this.nearestPointIndex(p);
    this.canvas.style.cursor = this.hoverIndex >= 0 ? "grab" : "crosshair";
  }

  onPointerUp() {
    if (this.dragIndex >= 0) {
      this.dragIndex = -1;
      if (this.points.length === POINT_COUNT) {
        this.recompute();
      }
    }
    this.canvas.style.cursor = this.hoverIndex >= 0 ? "grab" : "crosshair";
  }

  reset() {
    this.points = [];
    this.solution = null;
    this.dragIndex = -1;
    this.hoverIndex = -1;

    this.refreshPointList();
    this.refreshMetrics();
    this.setStatus("Canvas reset. Click to place five precision points.", false);
  }

  recompute() {
    if (this.points.length !== POINT_COUNT) {
      this.setStatus("Select exactly 5 points before recomputing.", true);
      return;
    }

    this.setStatus("Synthesizing 4-bar linkage from 5 points...", false);

    const start = performance.now();
    const result = this.solver.solve(this.points);
    const elapsed = performance.now() - start;

    if (!result.ok) {
      this.solution = null;
      this.refreshMetrics();
      this.setStatus(result.message, true);
      return;
    }

    this.solution = {
      ...result,
      elapsed,
    };

    this.refreshMetrics();
    const quality = fitQuality(result.errors.max);
    this.setStatus(`${result.message} Fit quality: ${quality}.`, result.errors.max > 7);
  }

  exportJSON() {
    if (!this.solution?.mechanism) {
      this.setStatus("No synthesized mechanism to export.", true);
      return;
    }

    const data = this.solution.mechanism.toJSON(
      this.solution.errors,
      this.points,
      this.solution.construction,
      this.solution.precisionPoses,
    );

    const blob = new Blob([JSON.stringify(data, null, 2)], { type: "application/json" });
    const url = URL.createObjectURL(blob);

    const a = document.createElement("a");
    a.href = url;
    a.download = `five-point-fourbar-${Date.now()}.json`;
    a.click();

    URL.revokeObjectURL(url);
    this.setStatus("Synthesis JSON exported.", false);
  }

  exportPNG() {
    const a = document.createElement("a");
    a.href = this.canvas.toDataURL("image/png");
    a.download = `five-point-fourbar-${Date.now()}.png`;
    a.click();
    this.setStatus("Canvas snapshot exported.", false);
  }

  setStatus(text, isError) {
    this.statusText.textContent = text;
    this.statusText.classList.toggle("error", Boolean(isError));
  }

  refreshPointList() {
    this.pointList.innerHTML = "";

    this.points.forEach((p, i) => {
      const li = document.createElement("li");
      li.textContent = `C${i + 1}: (${p.x.toFixed(1)}, ${p.y.toFixed(1)})`;
      this.pointList.appendChild(li);
    });

    while (this.pointList.children.length < POINT_COUNT) {
      const li = document.createElement("li");
      li.textContent = "-";
      li.style.opacity = "0.42";
      this.pointList.appendChild(li);
    }
  }

  refreshMetrics() {
    this.metrics.innerHTML = "";

    const rows = [];
    if (this.solution?.mechanism) {
      const mech = this.solution.mechanism;
      const ground = Vec2.distance(mech.O2, mech.O4);
      const span = (mech.thetaList[POINT_COUNT - 1] - mech.thetaList[0]) * (180 / Math.PI);

      rows.push(["Ground", `${ground.toFixed(2)} px`]);
      rows.push(["Crank", `${mech.r2.toFixed(2)} px`]);
      rows.push(["Coupler", `${mech.r3.toFixed(2)} px`]);
      rows.push(["Follower", `${mech.r4.toFixed(2)} px`]);
      rows.push(["RMS Error", `${this.solution.errors.rms.toFixed(3)} px`]);
      rows.push(["Max Error", `${this.solution.errors.max.toFixed(3)} px`]);
      rows.push(["Input Span", `${span.toFixed(1)} deg`]);
      rows.push(["Branch", mech.branch > 0 ? "+" : "-"]);
      rows.push(["Solve Time", `${this.solution.elapsed.toFixed(1)} ms`]);
    } else {
      rows.push(["Ground", "-"]);
      rows.push(["Crank", "-"]);
      rows.push(["Coupler", "-"]);
      rows.push(["Follower", "-"]);
      rows.push(["RMS Error", "-"]);
      rows.push(["Max Error", "-"]);
      rows.push(["Input Span", "-"]);
      rows.push(["Branch", "-"]);
      rows.push(["Solve Time", "-"]);
    }

    for (const [key, value] of rows) {
      const dt = document.createElement("dt");
      dt.textContent = key;
      const dd = document.createElement("dd");
      dd.textContent = value;
      this.metrics.append(dt, dd);
    }
  }

  render(ts) {
    const dt = (ts - this.lastTs) * 0.001;
    this.lastTs = ts;
    if (this.isAnimating) this.animationClock += dt;

    const ctx = this.ctx;
    ctx.clearRect(0, 0, this.width, this.height);

    this.drawGrid();
    this.drawOverlayHints();
    this.drawPoints();

    if (this.solution?.mechanism) {
      this.drawSolution();
    }

    requestAnimationFrame((nextTs) => this.render(nextTs));
  }

  drawGrid() {
    const ctx = this.ctx;
    const cs = getComputedStyle(document.body);
    const minorColor = cs.getPropertyValue("--grid-minor").trim();
    const majorColor = cs.getPropertyValue("--grid-major").trim();

    const minor = 22;
    const major = 110;

    for (let x = 0; x <= this.width; x += minor) {
      ctx.strokeStyle = minorColor;
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(x + 0.5, 0);
      ctx.lineTo(x + 0.5, this.height);
      ctx.stroke();
    }

    for (let y = 0; y <= this.height; y += minor) {
      ctx.strokeStyle = minorColor;
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(0, y + 0.5);
      ctx.lineTo(this.width, y + 0.5);
      ctx.stroke();
    }

    for (let x = 0; x <= this.width; x += major) {
      ctx.strokeStyle = majorColor;
      ctx.lineWidth = 1.2;
      ctx.beginPath();
      ctx.moveTo(x + 0.5, 0);
      ctx.lineTo(x + 0.5, this.height);
      ctx.stroke();
    }

    for (let y = 0; y <= this.height; y += major) {
      ctx.strokeStyle = majorColor;
      ctx.lineWidth = 1.2;
      ctx.beginPath();
      ctx.moveTo(0, y + 0.5);
      ctx.lineTo(this.width, y + 0.5);
      ctx.stroke();
    }
  }

  drawOverlayHints() {
    if (this.points.length >= POINT_COUNT) return;

    const ctx = this.ctx;
    const remaining = POINT_COUNT - this.points.length;

    ctx.save();
    ctx.fillStyle = "rgba(5, 12, 20, 0.46)";
    ctx.fillRect(22, 22, 320, 66);

    ctx.strokeStyle = "rgba(86, 171, 255, 0.52)";
    ctx.lineWidth = 1;
    ctx.strokeRect(22.5, 22.5, 319, 65);

    ctx.fillStyle = "#dce9f7";
    ctx.font = '600 13px "IBM Plex Sans", "Avenir Next", sans-serif';
    ctx.fillText("5-Point Coupler Synthesis", 35, 48);

    ctx.fillStyle = "#a8bfd7";
    ctx.font = '12px "IBM Plex Sans", "Avenir Next", sans-serif';
    ctx.fillText(`Place ${remaining} more precision point${remaining > 1 ? "s" : ""}.`, 35, 69);

    ctx.restore();
  }

  drawPoints() {
    const ctx = this.ctx;
    ctx.font = '12px "IBM Plex Sans", "Avenir Next", sans-serif';

    for (let i = 0; i < this.points.length; i += 1) {
      const p = this.points[i];
      const isActive = i === this.dragIndex || i === this.hoverIndex;

      ctx.fillStyle = isActive ? "#ffdb8c" : "#ffb96e";
      ctx.strokeStyle = "rgba(0, 0, 0, 0.42)";
      ctx.lineWidth = 1.5;

      ctx.beginPath();
      ctx.arc(p.x, p.y, isActive ? 7.5 : 6.4, 0, TWO_PI);
      ctx.fill();
      ctx.stroke();

      ctx.fillStyle = "#f2f6fd";
      ctx.fillText(`C${i + 1}`, p.x + 10, p.y - 12);
      ctx.fillStyle = "#b0c3d8";
      ctx.fillText(`${p.x.toFixed(0)}, ${p.y.toFixed(0)}`, p.x + 10, p.y + 2);
    }
  }

  drawSolution() {
    const ctx = this.ctx;
    const { mechanism, trace, precisionPoses, errors, construction } = this.solution;

    if (trace.length > 1) {
      ctx.strokeStyle = "rgba(108, 212, 255, 0.78)";
      ctx.lineWidth = 2.5;
      ctx.beginPath();
      trace.forEach((p, i) => {
        if (i === 0) ctx.moveTo(p.x, p.y);
        else ctx.lineTo(p.x, p.y);
      });
      ctx.stroke();
    }

    for (let i = 0; i < precisionPoses.length; i += 1) {
      const target = this.points[i];
      const fit = precisionPoses[i].P;
      const error = errors.perPoint[i];

      ctx.strokeStyle = error < 2 ? "rgba(84, 214, 173, 0.65)" : "rgba(255, 136, 112, 0.8)";
      ctx.lineWidth = 1.25;
      ctx.beginPath();
      ctx.moveTo(target.x, target.y);
      ctx.lineTo(fit.x, fit.y);
      ctx.stroke();

      ctx.fillStyle = "rgba(111, 215, 255, 0.95)";
      ctx.beginPath();
      ctx.arc(fit.x, fit.y, 3.2, 0, TWO_PI);
      ctx.fill();
    }

    if (this.showConstruction) {
      this.drawConstructionOverlay(construction, precisionPoses, mechanism);
    }

    const start = mechanism.thetaList[0];
    const end = mechanism.thetaList[POINT_COUNT - 1];
    const motion = start + (end - start) * (0.5 + 0.5 * Math.sin(this.animationClock * 1.18));
    const pose = mechanism.solvePose(motion);
    if (!pose) return;

    this.drawLink(mechanism.O2, mechanism.O4, "#dde2ea", 4.2, [8, 6]);
    this.drawLink(mechanism.O2, pose.A, "#45c79d", 5);
    this.drawLink(pose.A, pose.B, "#55b9ff", 5);
    this.drawLink(pose.B, mechanism.O4, "#ef8a72", 5);
    this.drawCouplerPlate(pose.A, pose.B, pose.P);

    this.drawPivot(mechanism.O2, "#ffffff", "Hc / O2");
    this.drawPivot(mechanism.O4, "#ffffff", "Hr / O4");
    this.drawPivot(pose.A, "#45c79d", "A");
    this.drawPivot(pose.B, "#ef8a72", "B");

    ctx.fillStyle = "#ffe07f";
    ctx.beginPath();
    ctx.arc(pose.P.x, pose.P.y, 6.4, 0, TWO_PI);
    ctx.fill();

    ctx.font = '12px "IBM Plex Sans", "Avenir Next", sans-serif';
    ctx.fillStyle = "#fff6d4";
    ctx.fillText("Coupler point", pose.P.x + 10, pose.P.y + 14);
  }

  drawConstructionOverlay(construction, precisionPoses, mechanism) {
    const ctx = this.ctx;

    const drawGuideLine = (line, color, label) => {
      if (!line) return;
      const d = Vec2.normalize(line.dir);
      const len = Math.hypot(this.width, this.height) * 1.2;
      const p1 = Vec2.add(line.point, Vec2.scale(d, -len));
      const p2 = Vec2.add(line.point, Vec2.scale(d, len));

      ctx.save();
      ctx.strokeStyle = color;
      ctx.lineWidth = 1.2;
      ctx.setLineDash([8, 7]);
      ctx.beginPath();
      ctx.moveTo(p1.x, p1.y);
      ctx.lineTo(p2.x, p2.y);
      ctx.stroke();
      ctx.restore();

      if (label) {
        ctx.fillStyle = color;
        ctx.font = '11px "IBM Plex Sans", "Avenir Next", sans-serif';
        ctx.fillText(label, line.point.x + 7, line.point.y - 8);
      }
    };

    drawGuideLine(construction.c15, "rgba(145, 227, 189, 0.78)", "c15");
    drawGuideLine(construction.c23, "rgba(145, 227, 189, 0.78)", "c23");
    drawGuideLine(construction.a12, "rgba(111, 197, 255, 0.72)", "a12");
    drawGuideLine(construction.a23, "rgba(111, 197, 255, 0.72)", "a23");

    if (construction.hr) {
      this.drawPivot(construction.hr, "#8df6ad", "Hr");
      ctx.save();
      ctx.strokeStyle = "rgba(141, 246, 173, 0.35)";
      ctx.lineWidth = 1.2;
      ctx.setLineDash([5, 6]);

      const r15 = Vec2.distance(construction.hr, this.points[0]);
      const r23 = Vec2.distance(construction.hr, this.points[1]);
      ctx.beginPath();
      ctx.arc(construction.hr.x, construction.hr.y, r15, 0, TWO_PI);
      ctx.stroke();

      ctx.beginPath();
      ctx.arc(construction.hr.x, construction.hr.y, r23, 0, TWO_PI);
      ctx.stroke();
      ctx.restore();
    }

    if (construction.hcFromA) {
      this.drawPivot(construction.hcFromA, "#8bc8ff", "Hc");
    }

    if (construction.poles?.length) {
      ctx.fillStyle = "#ffc769";
      ctx.font = '11px "IBM Plex Sans", "Avenir Next", sans-serif';

      construction.poles.forEach((pole) => {
        ctx.beginPath();
        ctx.arc(pole.point.x, pole.point.y, 3.9, 0, TWO_PI);
        ctx.fill();
        ctx.fillText(`I${pole.index}`, pole.point.x + 6, pole.point.y - 6);
      });
    }

    ctx.save();
    ctx.strokeStyle = "rgba(100, 183, 255, 0.28)";
    ctx.lineWidth = 1.1;
    ctx.setLineDash([4, 7]);
    for (let i = 0; i < precisionPoses.length - 1; i += 1) {
      ctx.beginPath();
      ctx.moveTo(precisionPoses[i].A.x, precisionPoses[i].A.y);
      ctx.lineTo(precisionPoses[i + 1].A.x, precisionPoses[i + 1].A.y);
      ctx.moveTo(precisionPoses[i].B.x, precisionPoses[i].B.y);
      ctx.lineTo(precisionPoses[i + 1].B.x, precisionPoses[i + 1].B.y);
      ctx.stroke();
    }
    ctx.restore();

    ctx.save();
    ctx.strokeStyle = "rgba(255, 194, 100, 0.2)";
    ctx.lineWidth = 1.1;
    ctx.setLineDash([7, 8]);
    for (const pose of precisionPoses) {
      ctx.beginPath();
      ctx.arc(pose.A.x, pose.A.y, mechanism.r3, 0, TWO_PI);
      ctx.stroke();
    }
    ctx.restore();
  }

  drawPivot(point, color, label) {
    const ctx = this.ctx;
    ctx.fillStyle = color;
    ctx.strokeStyle = "rgba(0, 0, 0, 0.45)";
    ctx.lineWidth = 1.2;

    ctx.beginPath();
    ctx.arc(point.x, point.y, 5.6, 0, TWO_PI);
    ctx.fill();
    ctx.stroke();

    ctx.font = '11px "IBM Plex Sans", "Avenir Next", sans-serif';
    ctx.fillStyle = "#f0f5ff";
    ctx.fillText(label, point.x + 8, point.y - 8);
  }

  drawLink(a, b, color, width = 4.2, dash = []) {
    const ctx = this.ctx;
    ctx.save();
    ctx.strokeStyle = color;
    ctx.lineWidth = width;
    ctx.lineCap = "round";
    ctx.setLineDash(dash);
    ctx.beginPath();
    ctx.moveTo(a.x, a.y);
    ctx.lineTo(b.x, b.y);
    ctx.stroke();
    ctx.restore();
  }

  drawCouplerPlate(a, b, p) {
    const ctx = this.ctx;
    ctx.save();

    ctx.fillStyle = "rgba(115, 196, 255, 0.2)";
    ctx.strokeStyle = "rgba(115, 196, 255, 0.6)";
    ctx.lineWidth = 2;

    ctx.beginPath();
    ctx.moveTo(a.x, a.y);
    ctx.lineTo(b.x, b.y);
    ctx.lineTo(p.x, p.y);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();

    ctx.strokeStyle = "rgba(115, 196, 255, 0.7)";
    ctx.lineWidth = 2.4;
    ctx.beginPath();
    ctx.moveTo(a.x, a.y);
    ctx.lineTo(p.x, p.y);
    ctx.moveTo(b.x, b.y);
    ctx.lineTo(p.x, p.y);
    ctx.stroke();

    ctx.restore();
  }
}

new SynthesisApp();
