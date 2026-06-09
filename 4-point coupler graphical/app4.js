const canvas = document.getElementById("synthesisCanvas");
const ctx = canvas.getContext("2d");
const createButton = document.getElementById("createButton");
const playButton = document.getElementById("playButton");
const demoButton = document.getElementById("demoButton");
const resetButton = document.getElementById("resetButton");
const pivotSlider = document.getElementById("pivotSlider");
const couplerSlider = document.getElementById("couplerSlider");
const radiusRInput = document.getElementById("radiusR");
const radiusrInput = document.getElementById("radiusr");
const showGuidesInput = document.getElementById("showGuides");
const showFinalLinkageInput = document.getElementById("showFinalLinkage");
const statusList = document.getElementById("statusList");
const pointList = document.getElementById("pointList");
const feasibilityBadge = document.getElementById("feasibilityBadge");

const state = {
  points: [],
  draggingIndex: -1,
  construction: null,
  animationFrame: null,
  isPlaying: false,
  motionPhase: 0,
};

const POINT_RADIUS = 8;
const LABELS = ["C1", "C2", "C3", "C4"];
const ANIMATION_DURATION_MS = 10667;
const KEYFRAME_DWELL = 0.18;
const AUTO_SEARCH_STEPS = 9;
const AUTO_SEARCH_RADIUS_STEPS = 7;
const FALLBACK_SPREAD = 140;
const STRICT_SOLVER_ERROR_LIMIT = 10;
const GOOD_BADGE_ERROR_LIMIT = 1.5;

canvas.addEventListener("pointerdown", onPointerDown);
canvas.addEventListener("pointermove", onPointerMove);
canvas.addEventListener("pointerup", stopDragging);
canvas.addEventListener("pointerleave", stopDragging);
createButton.addEventListener("click", createConstruction);
playButton.addEventListener("click", togglePlayback);
demoButton.addEventListener("click", loadDemoPoints);
resetButton.addEventListener("click", resetAll);
[pivotSlider, couplerSlider, radiusRInput, radiusrInput, showGuidesInput, showFinalLinkageInput].forEach((input) =>
  input.addEventListener("input", () => {
    if (state.points.length === 4) {
      createConstructionFromControls();
    } else {
      drawScene();
      updateStatus();
    }
  }),
);

drawScene();
updateStatus();
updatePointList();

function onPointerDown(event) {
  const pos = getCanvasPoint(event);
  const hitIndex = state.points.findIndex((point) => distance(point, pos) < 16);

  if (hitIndex >= 0) {
    state.draggingIndex = hitIndex;
    return;
  }

  if (state.points.length < 4) {
    state.points.push(pos);
    state.construction = null;
    updateStatus();
    updatePointList();
    drawScene();
  }
}

function onPointerMove(event) {
  if (state.draggingIndex < 0) {
    return;
  }

  state.points[state.draggingIndex] = getCanvasPoint(event);
  state.construction = null;
  drawScene();
  updateStatus();
  updatePointList();
}

function stopDragging() {
  if (state.draggingIndex >= 0 && state.points.length === 4) {
    createConstruction();
  }
  state.draggingIndex = -1;
}

function createConstruction() {
  if (state.points.length < 4) {
    updateStatus("Place all four precision points before creating the construction.");
    drawScene();
    return;
  }

  const solved = resolveConstructionForPoints(state.points, getCurrentParams());
  if (solved?.params) {
    setControlValues(solved.params);
  }
  state.construction = solved.construction;

  updateStatus();
  updatePointList();
  drawScene();
}

function createConstructionFromControls() {
  if (state.points.length < 4) {
    return;
  }

  state.construction = computeConstructionFromParams(getCurrentParams(), state.points);
  updateStatus();
  updatePointList();
  drawScene();
}


function resolveConstructionForPoints(points, seedParams) {
  const direct = computeConstructionFromParams(seedParams, points);
  const directScore = isPlayableConstruction(direct) ? scoreConstruction(direct, points) : Infinity;

  const searched = findPlayableConstruction(points, seedParams);
  if (searched && searched.score < directScore) {
    return { construction: searched.construction, params: searched.params };
  }

  if (isPlayableConstruction(direct)) {
    direct.solverNote = "Used your current slider values (already feasible).";
    return { construction: direct, params: seedParams };
  }

  if (searched) {
    return { construction: searched.construction, params: searched.params };
  }

  return {
    construction: buildFallbackConstruction(points),
    params: seedParams,
  };
}


function getCurrentParams() {
  return {
    pivotOffset: Number(pivotSlider.value),
    couplerOffset: Number(couplerSlider.value),
    radiusR: Number(radiusRInput.value),
    radiusr: Number(radiusrInput.value),
  };
}

function setControlValues(params) {
  pivotSlider.value = clamp(Math.round(params.pivotOffset), -220, 220);
  couplerSlider.value = clamp(Math.round(params.couplerOffset), -220, 220);
  radiusRInput.value = clamp(Math.round(params.radiusR), 80, 520);
  radiusrInput.value = clamp(Math.round(params.radiusr), 40, 320);
}

function computeConstructionFromParams(params, points) {
  const [c1, , c3] = points;
  const c13 = midpointNormal(c1, c3);
  const hr = pointOnLine(c13.midpoint, c13.unitNormal, params.pivotOffset);
  const radiusR = params.radiusR;
  const radiusr = params.radiusr;
  const aPoints = resolveAPoints(hr, radiusR, c1, c3, radiusr);
  const a13 = aPoints.a1 && aPoints.a3 ? midpointNormal(aPoints.a1, aPoints.a3) : null;
  const hc = a13 ? pointOnLine(a13.midpoint, a13.unitNormal, params.couplerOffset) : null;
  const crankRadius = hc && aPoints.a1 ? distance(hc, aPoints.a1) : null;
  const secondaryPoints =
    hc && crankRadius
      ? resolveSecondaryAPoints(hc, crankRadius, points[1], points[3], radiusr, aPoints)
      : null;
  const followerPoints =
    secondaryPoints?.a2 && secondaryPoints?.a4
      ? {
          p2: rigidTransformPoint(aPoints.a1, points[0], hr, secondaryPoints.a2, points[1]),
          p4: rigidTransformPoint(aPoints.a1, points[0], hr, secondaryPoints.a4, points[3]),
        }
      : null;
  const b1 =
    followerPoints?.p2 && followerPoints?.p4 ? circumcenter(hr, followerPoints.p2, followerPoints.p4) : null;
  const mechanismPositions =
    b1 && secondaryPoints?.a2 && secondaryPoints?.a4
      ? buildMechanismPositions(aPoints, secondaryPoints, points, b1)
      : null;
  const linkLengths =
    mechanismPositions?.length === 4 ? deriveLinkLengths(mechanismPositions, hr, hc) : null;
  const traceSamples =
    mechanismPositions?.length === 4 ? sampleCouplerTrace(mechanismPositions, hr, hc, linkLengths) : [];
  const precisionErrors =
    mechanismPositions?.length === 4
      ? measurePrecisionErrors(mechanismPositions, points, hr, hc)
      : null;

  return {
    c13,
    hr,
    radiusR,
    radiusr,
    aPoints,
    a13,
    hc,
    crankRadius,
    secondaryPoints,
    followerPoints,
    b1,
    mechanismPositions,
    linkLengths,
    traceSamples,
    precisionErrors,
  };
}

function isPlayableConstruction(construction) {
  return Boolean(construction?.mechanismPositions?.length === 4 && construction?.b1);
}

function resetAll() {
  state.points = [];
  state.construction = null;
  state.draggingIndex = -1;
  state.motionPhase = 0;
  stopPlayback();
  updateStatus();
  updatePointList();
  drawScene();
}

function updateStatus(message) {
  const items = [];

  if (message) {
    items.push(message);
  } else if (state.points.length < 4) {
    items.push(`Select precision point ${LABELS[state.points.length]} on the canvas.`);
  } else {
    items.push("Choose C1 and C3, then construct the mid-normal c13.");
    items.push("Locate HR on c13 using the pivot slider.");
    items.push(`Draw an arc of radius R = ${Number(radiusRInput.value)} centered at HR.`);
    items.push(
      `Draw arcs of radius r = ${Number(radiusrInput.value)} from C1 and C3 to cut the previous arc.`,
    );

    if (state.construction?.aPoints?.validPair) {
      items.push("A1 and A3 were found at the arc intersections.");
      items.push("Construct the mid-normal a13 of segment A1A3 so it passes through HR.");
      items.push("Locate HC on a13, then draw the crank circle through A1 and A3.");
      if (state.construction?.secondaryPoints?.a2 && state.construction?.secondaryPoints?.a4) {
        items.push("Use C2 and C4 as centers with radius r to locate A2 and A4 on the crank circle.");
        items.push("Transfer the coupler rigid-body motion to get inverted follower points 2 and 4.");
        if (state.construction?.b1) {
          items.push("Locate rocker tip B1 so HR, 2 and 4 rotate about it.");
          items.push("Form the linkage HC-A, A-C, C-B, and B-HR and animate the synthesized motion.");
          items.push("Animation now slows down and dwells briefly at each precision point.");
          if (state.construction?.solverNote) {
            items.push(state.construction.solverNote);
          }
          if (state.construction?.precisionErrors) {
            items.push(
              `Key-position error: max ${state.construction.precisionErrors.maxError.toFixed(2)} px.`,
            );
          }
        } else {
          items.push("The current geometry does not yield a stable rocker center B1.");
        }
      } else {
        items.push("Adjust HC or r until the crank circle is cut by circles from C2 and C4.");
      }
    } else {
      items.push("Current R and r do not produce both A1 and A3 intersections. Adjust the sliders.");
    }
  }

  statusList.innerHTML = items.map((item) => `<li>${item}</li>`).join("");
  updateFeasibilityBadge();
}

function drawScene() {
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  drawBackdrop();
  drawPrecisionTargets();

  if (showGuidesInput.checked && state.construction) {
    drawConstructionGuides(state.construction);
  }

  if (showFinalLinkageInput.checked && state.construction?.mechanismPositions?.length) {
    drawFinalMechanism(state.construction);
  }

  drawPoints();
}

function drawBackdrop() {
  ctx.save();
  ctx.fillStyle = "rgba(250, 246, 239, 0.84)";
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  ctx.restore();
}

function updatePointList() {
  pointList.innerHTML = LABELS.map((label, index) => {
    const point = state.points[index];
    const coords = point ? `x: ${Math.round(point.x)}, y: ${Math.round(point.y)}` : "not placed";
    return `<div class="point-row"><strong>${label}</strong><span>${coords}</span></div>`;
  }).join("");
}

function updateFeasibilityBadge() {
  feasibilityBadge.className = "badge";

  if (state.points.length < 4) {
    const remaining = 4 - state.points.length;
    feasibilityBadge.classList.add("badge-neutral");
    feasibilityBadge.textContent = `Waiting for ${remaining} point${remaining === 1 ? "" : "s"}`;
    return;
  }

  if (
    state.construction?.b1 &&
    state.construction?.secondaryPoints?.a2 &&
    state.construction?.secondaryPoints?.a4 &&
    state.construction?.precisionErrors?.maxError <= GOOD_BADGE_ERROR_LIMIT
  ) {
    feasibilityBadge.classList.add("badge-good");
    feasibilityBadge.textContent = "Accurate at C1-C4";
    return;
  }

  feasibilityBadge.classList.add("badge-warn");
  feasibilityBadge.textContent = "Adjust for accuracy";
}

function findPlayableConstruction(points, seedParams) {
  const seedConstruction = computeConstructionFromParams(seedParams, points);
  let best = isPlayableConstruction(seedConstruction)
    ? { params: seedParams, construction: seedConstruction, score: scoreConstruction(seedConstruction, points) }
    : null;

  const pivotOffsets = buildRange(-220, 220, AUTO_SEARCH_STEPS);
  const couplerOffsets = buildRange(-220, 220, AUTO_SEARCH_STEPS);
  const radiiR = buildRange(90, 520, AUTO_SEARCH_RADIUS_STEPS);
  const radiir = buildRange(50, 320, AUTO_SEARCH_RADIUS_STEPS);

  for (const pivotOffset of pivotOffsets) {
    for (const couplerOffset of couplerOffsets) {
      for (const radiusR of radiiR) {
        for (const radiusr of radiir) {
          const params = { pivotOffset, couplerOffset, radiusR, radiusr };
          const construction = computeConstructionFromParams(params, points);
          if (!isPlayableConstruction(construction)) {
            continue;
          }
          if (construction.precisionErrors?.maxError > STRICT_SOLVER_ERROR_LIMIT) {
            continue;
          }

          const score = scoreConstruction(construction, points);
          if (!best || score < best.score) {
            best = { params, construction, score };
          }
        }
      }
    }
  }

  if (!best) {
    return null;
  }

  const refined = refineAroundBest(best.params, points);
  const winner = refined && refined.score < best.score ? refined : best;
  winner.construction.solverNote = "Auto-solved mechanism parameters for this point set.";
  return { params: winner.params, construction: winner.construction, score: winner.score };
}


function refineAroundBest(center, points) {
  let best = null;
  const pivotOffsets = buildRange(
    clamp(center.pivotOffset - 80, -220, 220),
    clamp(center.pivotOffset + 80, -220, 220),
    7,
  );
  const couplerOffsets = buildRange(
    clamp(center.couplerOffset - 80, -220, 220),
    clamp(center.couplerOffset + 80, -220, 220),
    7,
  );
  const radiiR = buildRange(
    clamp(center.radiusR - 80, 80, 520),
    clamp(center.radiusR + 80, 80, 520),
    6,
  );
  const radiir = buildRange(
    clamp(center.radiusr - 70, 40, 320),
    clamp(center.radiusr + 70, 40, 320),
    6,
  );

  for (const pivotOffset of pivotOffsets) {
    for (const couplerOffset of couplerOffsets) {
      for (const radiusR of radiiR) {
        for (const radiusr of radiir) {
          const params = { pivotOffset, couplerOffset, radiusR, radiusr };
          const construction = computeConstructionFromParams(params, points);
          if (!isPlayableConstruction(construction)) {
            continue;
          }

          if (construction.precisionErrors?.maxError > STRICT_SOLVER_ERROR_LIMIT) {
            continue;
          }

          const score = scoreConstruction(construction, points);
          if (!best || score < best.score) {
            best = { params, construction, score };
          }
        }
      }
    }
  }

  return best;
}

function scoreConstruction(construction, points) {
  const closure = construction.precisionErrors?.maxError ?? 9999;
  const acLengths = construction.mechanismPositions.map((pose) => distance(pose.a, pose.c));
  const bcLengths = construction.mechanismPositions.map((pose) => distance(pose.b, pose.c));
  const acVar = normalizedSpread(acLengths);
  const bcVar = normalizedSpread(bcLengths);

  const signs = construction.mechanismPositions.map((pose) => signedTriangleArea(pose.a, pose.b, pose.c));
  const signFlipPenalty = signs.some((v) => Math.sign(v || signs[0]) !== Math.sign(signs[0] || 1)) ? 2000 : 0;

  const minArea = Math.min(...signs.map((v) => Math.abs(v)));
  const areaPenalty = minArea < 50 ? (50 - minArea) * 10 : 0;

  return closure * 900 + acVar * 450 + bcVar * 450 + signFlipPenalty * 2 + areaPenalty;
}

function normalizedSpread(values) {
  const min = Math.min(...values);
  const max = Math.max(...values);
  const mean = values.reduce((sum, value) => sum + value, 0) / Math.max(1, values.length);
  return (max - min) / Math.max(1, mean);
}

function buildRange(min, max, steps) {
  if (steps <= 1) {
    return [min];
  }

  const values = [];
  for (let i = 0; i < steps; i += 1) {
    values.push(min + ((max - min) * i) / (steps - 1));
  }
  return values;
}

function buildFallbackConstruction(points) {
  const [c1, c2, c3, c4] = points;
  const centroid = {
    x: (c1.x + c2.x + c3.x + c4.x) / 4,
    y: (c1.y + c2.y + c3.y + c4.y) / 4,
  };

  const c13 = midpointNormal(c1, c3);
  const hr = { x: centroid.x + FALLBACK_SPREAD, y: centroid.y + FALLBACK_SPREAD * 0.65 };
  const hc = { x: centroid.x - FALLBACK_SPREAD, y: centroid.y + FALLBACK_SPREAD * 0.35 };

  const aRadius = Math.max(70, averageDistance(points, hc) * 0.55);
  const bRadius = Math.max(65, distance(hr, hc) * 0.42);

  const a1 = pointFromAngle(hc, aRadius, angleBetween(hc, c1) - 0.85);
  const a2 = pointFromAngle(hc, aRadius, angleBetween(hc, c2) - 0.85);
  const a3 = pointFromAngle(hc, aRadius, angleBetween(hc, c3) - 0.85);
  const a4 = pointFromAngle(hc, aRadius, angleBetween(hc, c4) - 0.85);

  const b1 = pointFromAngle(hr, bRadius, angleBetween(hr, c1) + 0.9);
  const b2 = rigidTransformPoint(a1, c1, b1, a2, c2);
  const b3 = rigidTransformPoint(a1, c1, b1, a3, c3);
  const b4 = rigidTransformPoint(a1, c1, b1, a4, c4);

  const mechanismPositions = [
    { a: a1, b: b1, c: c1 },
    { a: a2, b: b2, c: c2 },
    { a: a3, b: b3, c: c3 },
    { a: a4, b: b4, c: c4 },
  ];

  const aPoints = { a1, a3, validPair: true };
  const secondaryPoints = { a2, a4 };
  const a13 = midpointNormal(a1, a3);
  const radiusR = distance(hr, a1);
  const radiusr = distance(c1, a1);
  const traceSamples = sampleCouplerTrace(mechanismPositions, hr, hc);
  const precisionErrors = measurePrecisionErrors(mechanismPositions, points, hr, hc);

  return {
    c13,
    hr,
    radiusR,
    radiusr,
    aPoints,
    a13,
    hc,
    crankRadius: distance(hc, a1),
    secondaryPoints,
    followerPoints: {
      p2: rigidTransformPoint(a1, c1, hr, a2, c2),
      p4: rigidTransformPoint(a1, c1, hr, a4, c4),
    },
    b1,
    mechanismPositions,
    traceSamples,
    precisionErrors,
    solverNote: "Fallback mechanism used for this random point set. Try Auto Create again for stricter geometry.",
  };
}

function averageDistance(points, center) {
  return points.reduce((sum, point) => sum + distance(point, center), 0) / Math.max(1, points.length);
}

function loadDemoPoints() {
  stopPlayback();
  state.points = [
    { x: 238, y: 258 },
    { x: 402, y: 160 },
    { x: 592, y: 176 },
    { x: 716, y: 250 },
  ];
  state.construction = null;
  state.motionPhase = 0;
  createConstruction();
}

function drawPrecisionTargets() {
  if (!state.points.length) {
    return;
  }

  ctx.save();
  ctx.strokeStyle = "rgba(15, 118, 110, 0.28)";
  ctx.lineWidth = 2;
  ctx.setLineDash([7, 10]);
  ctx.beginPath();
  ctx.moveTo(state.points[0].x, state.points[0].y);
  for (let i = 1; i < state.points.length; i += 1) {
    ctx.lineTo(state.points[i].x, state.points[i].y);
  }
  ctx.stroke();
  ctx.restore();
}

function drawFinalMechanism(construction) {
  const { mechanismPositions, hc, hr } = construction;
  drawReferenceLinkageSweep(mechanismPositions);
  const pose = interpolateMechanismPose(mechanismPositions, state.motionPhase);
  if (pose) {
    drawLinkagePose(pose, hc, hr, true);
  }
}

function drawConstructionGuides(construction) {
  const {
    c13,
    hr,
    radiusR,
    radiusr,
    aPoints,
    a13,
    hc,
    crankRadius,
    secondaryPoints,
    followerPoints,
    b1,
    mechanismPositions,
    traceSamples,
  } = construction;

  ctx.save();
  ctx.strokeStyle = "rgba(47, 38, 27, 0.35)";
  ctx.lineWidth = 2;
  ctx.setLineDash([8, 8]);
  drawInfiniteLine(c13.midpoint, c13.unitNormal);
  ctx.restore();

  ctx.save();
  ctx.strokeStyle = "rgba(180, 83, 9, 0.6)";
  ctx.lineWidth = 2.5;
  ctx.beginPath();
  ctx.arc(hr.x, hr.y, radiusR, 0, Math.PI * 2);
  ctx.stroke();
  ctx.restore();

  ctx.save();
  ctx.strokeStyle = "rgba(15, 118, 110, 0.45)";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.arc(state.points[0].x, state.points[0].y, radiusr, 0, Math.PI * 2);
  ctx.stroke();
  ctx.beginPath();
  ctx.arc(state.points[2].x, state.points[2].y, radiusr, 0, Math.PI * 2);
  ctx.stroke();
  ctx.restore();

  drawPivot(hr);

  if (a13) {
    ctx.save();
    ctx.strokeStyle = "rgba(107, 114, 128, 0.75)";
    ctx.lineWidth = 2;
    ctx.setLineDash([10, 8]);
    drawInfiniteLine(a13.midpoint, a13.unitNormal);
    ctx.restore();

    drawLabel(a13.midpoint, "a13", { dx: 12, dy: -12, color: "#6b7280" });
  }

  if (hc && crankRadius) {
    ctx.save();
    ctx.strokeStyle = "rgba(15, 118, 110, 0.55)";
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    ctx.arc(hc.x, hc.y, crankRadius, 0, Math.PI * 2);
    ctx.stroke();
    ctx.restore();

    drawPivot(hc, "HC");
  }

  if (followerPoints?.p2) {
    drawMechanismPoint(followerPoints.p2, "2");
  }

  if (followerPoints?.p4) {
    drawMechanismPoint(followerPoints.p4, "4");
  }

  if (traceSamples?.length) {
    drawTrace(traceSamples);
  }

  if (b1) {
    drawPivot(b1, "B1");
  }

  ctx.save();
  ctx.strokeStyle = "rgba(47, 38, 27, 0.38)";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(state.points[0].x, state.points[0].y);
  ctx.lineTo(state.points[2].x, state.points[2].y);
  ctx.stroke();
  ctx.restore();

  drawLabel(c13.midpoint, "c13", { dx: 10, dy: -10, color: "#6d624f" });

  if (aPoints.a1 && aPoints.a3) {
    ctx.save();
    ctx.strokeStyle = "rgba(107, 114, 128, 0.45)";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(aPoints.a1.x, aPoints.a1.y);
    ctx.lineTo(aPoints.a3.x, aPoints.a3.y);
    ctx.stroke();
    ctx.restore();

    drawConstructionPoint(aPoints.a1, "A1");
    drawConstructionPoint(aPoints.a3, "A3");
  }

  if (secondaryPoints?.a2) {
    drawConstructionPoint(secondaryPoints.a2, "A2");
  }

  if (secondaryPoints?.a4) {
    drawConstructionPoint(secondaryPoints.a4, "A4");
  }

}

function drawPoints() {
  state.points.forEach((point, index) => {
    ctx.save();
    ctx.fillStyle = index === 0 || index === 2 ? "#0f766e" : "#7c6f57";
    ctx.beginPath();
    ctx.arc(point.x, point.y, POINT_RADIUS, 0, Math.PI * 2);
    ctx.fill();
    ctx.restore();

    drawLabel(point, LABELS[index], { dx: 12, dy: -14, color: "#2f261b" });
  });
}

function drawPivot(point, label = "HR") {
  ctx.save();
  ctx.fillStyle = "#6b7280";
  ctx.beginPath();
  ctx.arc(point.x, point.y, 16, 0, Math.PI * 2);
  ctx.fill();
  ctx.fillStyle = "#f8f5ef";
  ctx.beginPath();
  ctx.arc(point.x, point.y, 6, 0, Math.PI * 2);
  ctx.fill();
  ctx.restore();

  drawLabel(point, label, { dx: 18, dy: 10, color: "#2f261b" });
}

function drawConstructionPoint(point, label) {
  ctx.save();
  ctx.strokeStyle = "#b45309";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(point.x - 10, point.y - 10);
  ctx.lineTo(point.x + 10, point.y + 10);
  ctx.moveTo(point.x - 10, point.y + 10);
  ctx.lineTo(point.x + 10, point.y - 10);
  ctx.stroke();
  ctx.restore();

  drawLabel(point, label, { dx: 12, dy: -12, color: "#b45309" });
}

function drawMechanismPoint(point, label, color = "#84cc16") {
  ctx.save();
  ctx.fillStyle = color;
  ctx.beginPath();
  ctx.arc(point.x, point.y, 7, 0, Math.PI * 2);
  ctx.fill();
  ctx.restore();

  if (label) {
    drawLabel(point, label, { dx: 12, dy: -12, color: "#1f2937" });
  }
}

function drawJoint(point, color, radius = 6) {
  ctx.save();
  ctx.fillStyle = color;
  ctx.beginPath();
  ctx.arc(point.x, point.y, radius, 0, Math.PI * 2);
  ctx.fill();
  ctx.restore();
}

function drawCouplerPatch(points, fill = "rgba(190, 24, 93, 0.18)", stroke = "rgba(37, 99, 235, 0.45)") {
  ctx.save();
  ctx.fillStyle = fill;
  ctx.strokeStyle = stroke;
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(points[0].x, points[0].y);
  for (let i = 1; i < points.length; i += 1) {
    ctx.lineTo(points[i].x, points[i].y);
  }
  ctx.closePath();
  ctx.fill();
  ctx.stroke();
  ctx.restore();
}

function drawTrace(points) {
  ctx.save();
  ctx.strokeStyle = "rgba(14, 116, 144, 0.35)";
  ctx.lineWidth = 2;
  ctx.setLineDash([4, 8]);
  ctx.beginPath();
  ctx.moveTo(points[0].x, points[0].y);
  for (let i = 1; i < points.length; i += 1) {
    ctx.lineTo(points[i].x, points[i].y);
  }
  ctx.stroke();
  ctx.restore();
}

function drawLinkagePose(pose, hc, hr, highlight = false) {
  const stroke = highlight ? "rgba(37, 99, 235, 0.96)" : "rgba(59, 130, 246, 0.14)";
  const fill = highlight ? "rgba(59, 130, 246, 0.14)" : "rgba(96, 165, 250, 0.04)";

  drawCouplerPatch([pose.a, pose.c, pose.b], fill, stroke);

  ctx.save();
  ctx.strokeStyle = stroke;
  ctx.lineWidth = highlight ? 6 : 2;
  ctx.lineCap = "round";
  ctx.lineJoin = "round";
  ctx.beginPath();
  ctx.moveTo(hc.x, hc.y);
  ctx.lineTo(pose.a.x, pose.a.y);
  ctx.lineTo(pose.c.x, pose.c.y);
  ctx.lineTo(pose.b.x, pose.b.y);
  ctx.lineTo(hr.x, hr.y);
  ctx.stroke();
  ctx.restore();

  drawJoint(pose.a, stroke, highlight ? 6 : 4);
  drawJoint(pose.b, stroke, highlight ? 6 : 4);
  drawMechanismPoint(pose.c, "", highlight ? "#1d4ed8" : "rgba(37, 99, 235, 0.55)");
}

function drawReferenceLinkageSweep(positions) {
  positions.forEach((pose) => {
    drawLinkagePose(pose, state.construction.hc, state.construction.hr, false);
  });
}

function drawLabel(point, text, options) {
  ctx.save();
  ctx.font = "24px Georgia";
  ctx.fillStyle = options.color;
  ctx.fillText(text, point.x + options.dx, point.y + options.dy);
  ctx.restore();
}

function drawInfiniteLine(origin, direction) {
  const far = 1600;
  ctx.beginPath();
  ctx.moveTo(origin.x - direction.x * far, origin.y - direction.y * far);
  ctx.lineTo(origin.x + direction.x * far, origin.y + direction.y * far);
  ctx.stroke();
}

function midpointNormal(a, b) {
  const midpoint = {
    x: (a.x + b.x) / 2,
    y: (a.y + b.y) / 2,
  };
  const dx = b.x - a.x;
  const dy = b.y - a.y;
  const length = Math.hypot(dx, dy) || 1;
  return {
    midpoint,
    unitNormal: {
      x: -dy / length,
      y: dx / length,
    },
  };
}

function pointOnLine(origin, direction, offset) {
  return {
    x: origin.x + direction.x * offset,
    y: origin.y + direction.y * offset,
  };
}

function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

function resolveAPoints(hr, radiusR, c1, c3, radiusr) {
  const pairOne = intersectTwoCircles(hr, radiusR, c1, radiusr);
  const pairTwo = intersectTwoCircles(hr, radiusR, c3, radiusr);
  const a1 = pickPreferredIntersection(pairOne, c3);
  const a3 = pickPreferredIntersection(pairTwo, c1);

  return {
    validPair: Boolean(a1 && a3),
    a1,
    a3,
  };
}

function resolveSecondaryAPoints(hc, crankRadius, c2, c4, radiusr, basePoints) {
  const a2Candidates = intersectTwoCircles(hc, crankRadius, c2, radiusr);
  const a4Candidates = intersectTwoCircles(hc, crankRadius, c4, radiusr);
  const a2 = pickPreferredIntersection(a2Candidates, basePoints.a3 || basePoints.a1);
  const a4 = pickPreferredIntersection(a4Candidates, basePoints.a1 || basePoints.a3);

  return { a2, a4 };
}

function rigidTransformPoint(aStart, cStart, targetPoint, aEnd, cEnd) {
  const startVector = {
    x: cStart.x - aStart.x,
    y: cStart.y - aStart.y,
  };
  const endVector = {
    x: cEnd.x - aEnd.x,
    y: cEnd.y - aEnd.y,
  };
  const startAngle = Math.atan2(startVector.y, startVector.x);
  const endAngle = Math.atan2(endVector.y, endVector.x);
  const rotation = endAngle - startAngle;
  const local = {
    x: targetPoint.x - aStart.x,
    y: targetPoint.y - aStart.y,
  };
  const rotated = rotateVector(local, rotation);

  return {
    x: aEnd.x + rotated.x,
    y: aEnd.y + rotated.y,
  };
}

function buildMechanismPositions(aPoints, secondaryPoints, cPoints, b1) {
  const aPos = [aPoints.a1, secondaryPoints.a2, aPoints.a3, secondaryPoints.a4];
  const b2 = rigidTransformPoint(aPoints.a1, cPoints[0], b1, secondaryPoints.a2, cPoints[1]);
  const b3 = rigidTransformPoint(aPoints.a1, cPoints[0], b1, aPoints.a3, cPoints[2]);
  const b4 = rigidTransformPoint(aPoints.a1, cPoints[0], b1, secondaryPoints.a4, cPoints[3]);
  const bPos = [b1, b2, b3, b4];

  const couplerAC = distance(aPos[0], cPoints[0]);
  const couplerBC = distance(bPos[0], cPoints[0]);
  const preferredSide = Math.sign(signedTriangleArea(aPos[0], bPos[0], cPoints[0])) || 1;

  const positions = [];
  for (let i = 0; i < 4; i += 1) {
    const candidates = intersectTwoCircles(aPos[i], couplerAC, bPos[i], couplerBC);
    if (!candidates.length) {
      return null;
    }

    const c = selectCouplerCandidate(candidates, aPos[i], bPos[i], cPoints[i], preferredSide);
    positions.push({ a: aPos[i], b: bPos[i], c });
  }

  return positions;
}


function sampleCouplerTrace(positions, hr, hc, linkLengths = null) {
  const samples = [];
  const lengths = linkLengths || deriveLinkLengths(positions, hr, hc);
  for (let i = 0; i < positions.length; i += 1) {
    const start = positions[i];
    const end = positions[(i + 1) % positions.length];
    for (let step = 0; step <= 24; step += 1) {
      const t = step / 24;
      const pose = interpolatePoseFromAngles(start, end, t, hr, hc, lengths);
      if (pose?.c) {
        samples.push(pose.c);
      }
    }
  }
  return samples;
}

function deriveLinkLengths(positions, hr, hc) {
  const reference = positions[0];
  return {
    crankLength: distance(hc, reference.a),
    rockerLength: distance(hr, reference.b),
    couplerAB: distance(reference.a, reference.b),
    couplerAC: distance(reference.a, reference.c),
    couplerBC: distance(reference.b, reference.c),
    couplerSign: Math.sign(signedTriangleArea(reference.a, reference.b, reference.c)) || 1,
  };
}

function interpolateMechanismPose(positions, phase) {
  if (!positions?.length) {
    return null;
  }

  const segmentCount = positions.length;
  const scaled = ((phase % 1) + 1) % 1 * segmentCount;
  const index = Math.floor(scaled);
  const localT = scaled - index;
  const start = positions[index];
  const end = positions[(index + 1) % segmentCount];
  const lengths = state.construction?.linkLengths || deriveLinkLengths(positions, state.construction.hr, state.construction.hc);

  if (localT <= KEYFRAME_DWELL) {
    return clonePose(start);
  }

  if (localT >= 1 - KEYFRAME_DWELL) {
    return clonePose(end);
  }

  const shapedT = applyKeyframeDwell(localT);
  return interpolatePoseFromAngles(start, end, shapedT, state.construction.hr, state.construction.hc, lengths);
}

function interpolatePoseFromAngles(start, end, t, hr, hc, lengths = null) {
  if (t <= 0) {
    return clonePose(start);
  }

  if (t >= 1) {
    return clonePose(end);
  }

  const link = lengths || deriveLinkLengths([start], hr, hc);
  const angleA = lerpAngle(angleBetween(hc, start.a), angleBetween(hc, end.a), t);
  const angleB = lerpAngle(angleBetween(hr, start.b), angleBetween(hr, end.b), t);

  const a = pointFromAngle(hc, link.crankLength, angleA);
  const bTarget = pointFromAngle(hr, link.rockerLength, angleB);
  const bCandidates = intersectTwoCircles(a, link.couplerAB, hr, link.rockerLength);
  const b = bCandidates.length ? pickClosestPoint(bCandidates, bTarget) : bTarget;

  const target = lerpPoint(start.c, end.c, t);
  const cCandidates = intersectTwoCircles(a, link.couplerAC, b, link.couplerBC);
  const c = cCandidates.length
    ? selectCouplerCandidate(cCandidates, a, b, target, link.couplerSign)
    : target;

  return { a, b, c };
}


function pointFromAngle(origin, radius, angle) {
  return {
    x: origin.x + radius * Math.cos(angle),
    y: origin.y + radius * Math.sin(angle),
  };
}

function angleBetween(origin, point) {
  return Math.atan2(point.y - origin.y, point.x - origin.x);
}

function lerpAngle(start, end, t) {
  let delta = end - start;
  while (delta > Math.PI) delta -= Math.PI * 2;
  while (delta < -Math.PI) delta += Math.PI * 2;
  return start + delta * t;
}

function lerpPoint(a, b, t) {
  return {
    x: a.x + (b.x - a.x) * t,
    y: a.y + (b.y - a.y) * t,
  };
}

function lerpSignedValue(start, end, t) {
  return start + (end - start) * t;
}

function clonePose(pose) {
  return {
    a: { ...pose.a },
    b: { ...pose.b },
    c: { ...pose.c },
  };
}

function smoothStep(t) {
  return t * t * (3 - 2 * t);
}

function applyKeyframeDwell(t) {
  const dwell = KEYFRAME_DWELL;
  const movingSpan = 1 - dwell * 2;

  if (t <= dwell) {
    return 0;
  }

  if (t >= 1 - dwell) {
    return 1;
  }

  return smoothStep((t - dwell) / movingSpan);
}

function togglePlayback() {
  if (!state.construction?.mechanismPositions?.length) {
    return;
  }

  if (state.isPlaying) {
    stopPlayback();
    drawScene();
    return;
  }

  state.isPlaying = true;
  playButton.textContent = "Pause";
  const startTime = performance.now() - state.motionPhase * ANIMATION_DURATION_MS;

  const tick = (time) => {
    if (!state.isPlaying) {
      return;
    }
    state.motionPhase = ((time - startTime) % ANIMATION_DURATION_MS) / ANIMATION_DURATION_MS;
    drawScene();
    state.animationFrame = requestAnimationFrame(tick);
  };

  state.animationFrame = requestAnimationFrame(tick);
}

function stopPlayback() {
  state.isPlaying = false;
  playButton.textContent = "Play";
  if (state.animationFrame) {
    cancelAnimationFrame(state.animationFrame);
    state.animationFrame = null;
  }
}

function rotateVector(vector, angle) {
  const cos = Math.cos(angle);
  const sin = Math.sin(angle);
  return {
    x: vector.x * cos - vector.y * sin,
    y: vector.x * sin + vector.y * cos,
  };
}

function selectCouplerCandidate(candidates, a, b, target, preferredSide) {
  const sideThreshold = 1e-3;
  const preferred = candidates.filter((candidate) => {
    const side = signedTriangleArea(a, b, candidate);
    if (Math.abs(preferredSide) < sideThreshold) {
      return true;
    }
    return Math.sign(side || preferredSide) === Math.sign(preferredSide);
  });

  const pool = preferred.length ? preferred : candidates;
  return [...pool].sort((left, right) => distance(left, target) - distance(right, target))[0];
}

function signedTriangleArea(a, b, c) {
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

function measurePrecisionErrors(positions, targetPoints, hr, hc) {
  if (!positions?.length) {
    return { values: [], maxError: Infinity };
  }

  const acLength = distance(positions[0].a, positions[0].c);
  const bcLength = distance(positions[0].b, positions[0].c);
  const crankLength = distance(hc, positions[0].a);
  const rockerLength = distance(hr, positions[0].b);

  const errors = positions.map((pose, index) => {
    const target = targetPoints[index];
    const a = pointFromAngle(hc, crankLength, angleBetween(hc, pose.a));
    const b = pointFromAngle(hr, rockerLength, angleBetween(hr, pose.b));
    const candidates = intersectTwoCircles(a, acLength, b, bcLength);
    if (!candidates.length) {
      return 9999;
    }

    const closest = pickClosestPoint(candidates, target);
    return distance(closest, target);
  });

  return {
    values: errors,
    maxError: Math.max(...errors),
  };
}

function pickClosestPoint(points, target) {
  return [...points].sort((left, right) => distance(left, target) - distance(right, target))[0];
}

function circumcenter(a, b, c) {
  const d =
    2 *
    (a.x * (b.y - c.y) +
      b.x * (c.y - a.y) +
      c.x * (a.y - b.y));

  if (Math.abs(d) < 1e-6) {
    return null;
  }

  const ax2ay2 = a.x * a.x + a.y * a.y;
  const bx2by2 = b.x * b.x + b.y * b.y;
  const cx2cy2 = c.x * c.x + c.y * c.y;

  return {
    x:
      (ax2ay2 * (b.y - c.y) +
        bx2by2 * (c.y - a.y) +
        cx2cy2 * (a.y - b.y)) /
      d,
    y:
      (ax2ay2 * (c.x - b.x) +
        bx2by2 * (a.x - c.x) +
        cx2cy2 * (b.x - a.x)) /
      d,
  };
}

function pickPreferredIntersection(candidates, referencePoint) {
  if (!candidates.length) {
    return null;
  }

  return [...candidates].sort((left, right) => {
    const byReference = distance(right, referencePoint) - distance(left, referencePoint);
    if (Math.abs(byReference) > 1) {
      return byReference;
    }
    return right.y - left.y;
  })[0];
}

function intersectTwoCircles(c0, r0, c1, r1) {
  const dx = c1.x - c0.x;
  const dy = c1.y - c0.y;
  const d = Math.hypot(dx, dy);

  if (d === 0 || d > r0 + r1 || d < Math.abs(r0 - r1)) {
    return [];
  }

  const a = (r0 * r0 - r1 * r1 + d * d) / (2 * d);
  const hSq = r0 * r0 - a * a;
  if (hSq < 0) {
    return [];
  }

  const h = Math.sqrt(hSq);
  const xm = c0.x + (a * dx) / d;
  const ym = c0.y + (a * dy) / d;

  const rx = (-dy * h) / d;
  const ry = (dx * h) / d;

  return [
    { x: xm + rx, y: ym + ry },
    { x: xm - rx, y: ym - ry },
  ];
}

function distance(a, b) {
  return Math.hypot(a.x - b.x, a.y - b.y);
}

function getCanvasPoint(event) {
  const rect = canvas.getBoundingClientRect();
  const scaleX = canvas.width / rect.width;
  const scaleY = canvas.height / rect.height;
  return {
    x: (event.clientX - rect.left) * scaleX,
    y: (event.clientY - rect.top) * scaleY,
  };
}
