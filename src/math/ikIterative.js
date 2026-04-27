/**
 * ikIterative.js — Jacobian-transpose iterative IK for 6-DOF robots
 *
 * Works for any DH-parameterised robot (no robot-specific geometry needed).
 * Used as the primary solver when the analytical solution is unavailable or
 * when the target is near a singularity.
 *
 * Algorithm: Jacobian-transpose gradient descent
 *   q_{n+1} = q_n + α · Jᵀ · e
 * where e = (target − current TCP position).
 *
 * Convergence: stops when position error < eps mm or maxIter exceeded.
 *
 * No imports — forwardKinematics6 is inlined below as a local function.
 */

// ── Inlined FK (from fk6dof.js) ───────────────────────────────────────────────

function mat4mul(A, B) {
  const C = new Array(16).fill(0);
  for (let col = 0; col < 4; col++) {
    for (let row = 0; row < 4; row++) {
      let sum = 0;
      for (let k = 0; k < 4; k++) sum += A[k * 4 + row] * B[col * 4 + k];
      C[col * 4 + row] = sum;
    }
  }
  return C;
}

function mat4identity() {
  return [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1];
}

function dhTransform(theta, d, a, alpha) {
  const ct = Math.cos(theta), st = Math.sin(theta);
  const ca = Math.cos(alpha), sa = Math.sin(alpha);
  // prettier-ignore
  return [ct, st, 0, 0, -st*ca, ct*ca, sa, 0, st*sa, -ct*sa, ca, 0, a*ct, a*st, d, 1];
}

function fkPosition(angles, dhParams) {
  let T = mat4identity();
  for (let i = 0; i < angles.length; i++) {
    const { a, d, alpha, thetaOffset } = dhParams[i];
    T = mat4mul(T, dhTransform(angles[i] + thetaOffset, d, a, alpha));
  }
  return { x: T[12], y: T[13], z: T[14] };
}

// Safe O(1) wrap — no while loop, immune to Inf/NaN input.
function normalizeAngle(a) {
  if (!isFinite(a)) return 0;
  // Equivalent to fmod but symmetric around 0
  return a - 2 * Math.PI * Math.round(a / (2 * Math.PI));
}

// ── Solver ────────────────────────────────────────────────────────────────────

/**
 * Jacobian-transpose iterative IK (position only).
 *
 * @param {{ x, y, z }}  targetPos     - Desired TCP position (mm)
 * @param {number[]}     initialAngles - Starting joint angles (radians)
 * @param {object[]}     dhParams      - DH parameter table (6 entries)
 * @param {object}       [opts]        - Optional tuning parameters
 * @param {number}       [opts.maxIter=200] - Max iterations
 * @param {number}       [opts.stepSize=0.8] - Gradient step scale
 * @param {number}       [opts.eps=0.5]     - Convergence tolerance (mm)
 * @returns {{ angles: number[], error: number, iterations: number }}
 *   angles     — final joint angles (radians)
 *   error      — residual position error (mm)
 *   iterations — actual iterations used
 */
export function solveIKIterative(targetPos, initialAngles, dhParams, {
  maxIter  = 200,
  stepSize = 0.8,
  eps      = 0.5,
} = {}) {
  let q = [...initialAngles];
  const h = 1e-4;  // finite-difference step (radians)
  let lastError = Infinity;
  let iter = 0;

  for (; iter < maxIter; iter++) {
    const p = fkPosition(q, dhParams);
    const ex = targetPos.x - p.x;
    const ey = targetPos.y - p.y;
    const ez = targetPos.z - p.z;
    lastError = Math.sqrt(ex*ex + ey*ey + ez*ez);
    if (lastError < eps) break;

    // Jacobian-transpose update: dqᵢ += (∂p/∂qᵢ) · err
    const scale = stepSize * Math.min(1, lastError / 50);
    for (let i = 0; i < q.length; i++) {
      const qp = [...q]; qp[i] += h;
      const pp = fkPosition(qp, dhParams);
      const dqi = ((pp.x - p.x) * ex + (pp.y - p.y) * ey + (pp.z - p.z) * ez) / h;
      // Clip individual joint step to ±0.5 rad to prevent gradient blow-up
      const step = Math.max(-0.5, Math.min(0.5, scale * dqi));
      q[i] = normalizeAngle(q[i] + step);
    }
  }

  // Final error measurement
  const pFinal = fkPosition(q, dhParams);
  const ef = Math.hypot(
    targetPos.x - pFinal.x,
    targetPos.y - pFinal.y,
    targetPos.z - pFinal.z,
  );

  return { angles: q, error: ef, iterations: iter };
}
