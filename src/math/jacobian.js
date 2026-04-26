/**
 * jacobian.js — Numerical Jacobian for 6-DOF robots
 *
 * NOTE: The Jacobian is used for EDUCATIONAL DISPLAY only in ArmIQ.
 * The actual IK solver (ik6dof.js) uses closed-form analytical solution.
 *
 * The 6×6 geometric Jacobian J(q) relates joint velocities q̇ to
 * end-effector velocity Ẋ:
 *
 *   Ẋ = J(q) · q̇
 *
 * Computed via finite differences on the FK function.
 *
 * No import statements — FK math is inlined below as local functions.
 */

// ── Inlined DH/FK math (from fk6dof.js) ──────────────────────────────────────

function mat4mul(A, B) {
  const C = new Array(16).fill(0);
  for (let col = 0; col < 4; col++) {
    for (let row = 0; row < 4; row++) {
      let sum = 0;
      for (let k = 0; k < 4; k++) sum += A[k*4+row] * B[col*4+k];
      C[col*4+row] = sum;
    }
  }
  return C;
}

function mat4identity() { return [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]; }

function dhTransform(theta, d, a, alpha) {
  const ct = Math.cos(theta), st = Math.sin(theta);
  const ca = Math.cos(alpha), sa = Math.sin(alpha);
  // prettier-ignore
  return [ct, st, 0, 0, -st*ca, ct*ca, sa, 0, st*sa, -ct*sa, ca, 0, a*ct, a*st, d, 1];
}

/**
 * Compute T₀⁶ forward kinematics for a 6-DOF robot.
 * @param {number[]} angles   - [θ1…θ6] in radians
 * @param {object[]} dhParams - DH table (6 entries with a, d, alpha, thetaOffset)
 * @returns {{ position: {x,y,z}, R: number[] }}
 */
function forwardKinematics6(angles, dhParams) {
  let T = mat4identity();
  for (let i = 0; i < 6; i++) {
    const { a, d, alpha, thetaOffset } = dhParams[i];
    T = mat4mul(T, dhTransform(angles[i] + thetaOffset, d, a, alpha));
  }
  const position = { x: T[12], y: T[13], z: T[14] };
  const R = [T[0],T[1],T[2], T[4],T[5],T[6], T[8],T[9],T[10]];
  return { position, R };
}

const DELTA = 1e-5;  // finite difference step (radians)

/**
 * Compute the 6×6 Jacobian matrix numerically.
 *
 * @param {number[]} angles   - Current joint angles [θ1…θ6] (radians)
 * @param {object[]} dhParams - DH parameter table (6 entries)
 * @returns {number[][]} 6×6 Jacobian matrix (rows: [vx,vy,vz,wx,wy,wz],
 *                                             cols: joints 1–6)
 */
export function computeJacobian(angles, dhParams) {
  const J = Array.from({ length: 6 }, () => new Array(6).fill(0));

  // Nominal FK
  const { T: T0, position: p0, R: R0 } = forwardKinematics6(angles, dhParams);

  for (let i = 0; i < 6; i++) {
    // Perturb joint i by +DELTA
    const anglesPlus = [...angles];
    anglesPlus[i] += DELTA;

    const { position: pp, R: Rp } = forwardKinematics6(anglesPlus, dhParams);

    // Linear velocity column (finite difference of position)
    J[0][i] = (pp.x - p0.x) / DELTA;
    J[1][i] = (pp.y - p0.y) / DELTA;
    J[2][i] = (pp.z - p0.z) / DELTA;

    // Angular velocity column derived from skew-symmetric part of ΔR·R₀ᵀ
    // dR/dθᵢ ≈ (Rp − R0) / DELTA
    // ω extracted from skew symmetric: [R dot Rᵀ]
    // Simplified extraction: use the fact that for revolute joints,
    // ωᵢ = zᵢ₋₁ (unit vector along joint i axis in world frame)
    // We extract this from the partial derivative of R.
    // For small DELTA: ΔR ≈ [ω]× · R₀ · DELTA
    // So ΔR · R₀ᵀ ≈ [ω]× · DELTA
    const dRR = mulRRt(Rp, R0);
    // Extract skew-symmetric part and read ω
    J[3][i] = (dRR[7] - dRR[5]) / (2 * DELTA);  // ωx
    J[4][i] = (dRR[2] - dRR[6]) / (2 * DELTA);  // ωy
    J[5][i] = (dRR[3] - dRR[1]) / (2 * DELTA);  // ωz
  }

  return J;
}

/** Multiply two 3×3 row-major matrices A × Bᵀ. */
function mulRRt(A, B) {
  const C = new Array(9).fill(0);
  for (let r = 0; r < 3; r++) {
    for (let c = 0; c < 3; c++) {
      for (let k = 0; k < 3; k++) {
        C[r * 3 + c] += A[r * 3 + k] * B[c * 3 + k];  // B transposed: swap c,k
      }
    }
  }
  return C;
}

/**
 * Compute condition number of the Jacobian (using singular values approximation).
 * Condition number = σ_max / σ_min — indicates proximity to singularity.
 *
 * @param {number[][]} J - 6×6 Jacobian matrix
 * @returns {{ cond: number, det: number, manipulability: number }}
 */
export function jacobianMetrics(J) {
  // Full SVD on a 6×6 is expensive. For display we use the Frobenius norm
  // and the determinant of J·Jᵀ (Yoshikawa's manipulability measure).
  //
  // Manipulability: w = sqrt(det(J·Jᵀ))
  // For a square Jacobian: w = |det(J)|

  const det = det6(J);
  const manipulability = Math.abs(det);

  // Condition approximated as max-row-norm / min-row-norm (rough but fast)
  const rowNorms = J.map(row => Math.sqrt(row.reduce((s, v) => s + v * v, 0)));
  const maxN = Math.max(...rowNorms);
  const minN = Math.min(...rowNorms.filter(n => n > 1e-12));
  const cond = minN > 0 ? maxN / minN : Infinity;

  return { cond, det, manipulability };
}

/**
 * Compute determinant of a 6×6 matrix via LU decomposition.
 * Used for Jacobian singularity analysis.
 *
 * @param {number[][]} M - 6×6 matrix
 * @returns {number} Determinant
 */
function det6(M) {
  const n = 6;
  // Copy into flat array for in-place LU
  const A = M.map(row => [...row]);
  let sign = 1;

  for (let i = 0; i < n; i++) {
    // Partial pivoting
    let maxRow = i;
    for (let k = i + 1; k < n; k++) {
      if (Math.abs(A[k][i]) > Math.abs(A[maxRow][i])) maxRow = k;
    }
    if (maxRow !== i) {
      [A[i], A[maxRow]] = [A[maxRow], A[i]];
      sign *= -1;
    }
    if (Math.abs(A[i][i]) < 1e-14) return 0;  // singular

    for (let k = i + 1; k < n; k++) {
      const factor = A[k][i] / A[i][i];
      for (let j = i; j < n; j++) {
        A[k][j] -= factor * A[i][j];
      }
    }
  }

  let d = sign;
  for (let i = 0; i < n; i++) d *= A[i][i];
  return d;
}
