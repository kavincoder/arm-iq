/**
 * ik6.js — 6-DOF Analytical Inverse Kinematics (Spherical Wrist)
 *
 * Method: Spherical wrist decoupling (Pieper's method)
 *
 * Assumption: Joints 4, 5, 6 share a common intersection point (wrist centre).
 * All three supported presets (UR5, KUKA KR6 R900, ABB IRB 120) satisfy this.
 *
 * Algorithm:
 *   Step 1 — Find wrist centre: Pc = Pe − d6·R·ẑ
 *   Step 2 — Solve θ₁ from wrist x-y position
 *   Step 3 — Solve θ₂, θ₃ using planar geometry (law of cosines)
 *   Step 4 — Compute R₀³ from θ₁,θ₂,θ₃, then R₃⁶ = R₀³ᵀ·R₀⁶
 *   Step 5 — Extract θ₄, θ₅, θ₆ from R₃⁶ (Euler ZYZ decomposition)
 *
 * Up to 8 solutions are returned; the caller picks the closest to current pose.
 *
 * Reference: Siciliano et al., "Robotics: Modelling, Planning and Control" Ch 2-3
 */

import { dhTransform, mat4mul, mat4identity, forwardKinematics6 } from './dh.js';

const PI = Math.PI;
const TWO_PI = 2 * PI;

/**
 * Wrap angle into [-π, π].
 */
export function normalizeAngle(a) {
  while (a > PI)  a -= TWO_PI;
  while (a < -PI) a += TWO_PI;
  return a;
}

/**
 * Shortest signed angle difference from 'from' to 'to' (both in radians).
 * Returns value in [-π, π].
 */
export function angleDiff(from, to) {
  return normalizeAngle(to - from);
}

/**
 * Solve 6-DOF IK with spherical wrist decoupling.
 *
 * @param {{ x, y, z }}     position    - Desired TCP position (mm)
 * @param {number[]}        Rd          - Desired 3×3 rotation matrix (row-major)
 * @param {object[]}        dhParams    - DH parameter table [6 entries]
 * @param {object}          config      - Robot-specific geometry constants:
 *   { d1, a2, a3, d4, d6 } — from DH table
 * @returns {number[][] | null}
 *   Array of up-to-8 solutions (each = [θ1,θ2,θ3,θ4,θ5,θ6] radians),
 *   or null if no solution exists.
 */
export function solveIK6(position, Rd, dhParams, config) {
  const { x: px, y: py, z: pz } = position;
  const { d1, a2, a3, d4, d6 } = config;

  // ── Step 1: Wrist centre position ────────────────────────────────────────
  // Pc = Pe − d6 · R · [0,0,1]ᵀ
  // The last column of Rd gives the approach direction (Z-axis of TCP frame).
  const nx = Rd[2]; const ny = Rd[5]; const nz = Rd[8];  // column 3 of Rd
  const wcx = px - d6 * nx;
  const wcy = py - d6 * ny;
  const wcz = pz - d6 * nz;

  const solutions = [];

  // ── Step 2: θ₁ — two solutions (±π flip) ─────────────────────────────────
  const theta1_candidates = [
    Math.atan2(wcy, wcx),
    Math.atan2(-wcy, -wcx),  // shoulder-flip solution
  ];

  for (const t1 of theta1_candidates) {
    // ── Step 3: θ₂, θ₃ via planar law of cosines ─────────────────────────
    // Project wrist into shoulder plane
    const r   = Math.sqrt(wcx * wcx + wcy * wcy);  // radial distance
    const s   = wcz - d1;                            // axial height above shoulder

    // Effective planar distances for the 2-link sub-problem
    const rp  = Math.sqrt((r - 0) ** 2 + s * s);   // simplified for a1=0 robot
    // Use actual geometry: r_wrist projected in shoulder plane
    const r2  = Math.sqrt((r) ** 2 + (s) ** 2);    // distance from shoulder to wrist

    // Link lengths in the shoulder plane
    const L1  = Math.hypot(a2, 0);                  // link 2 (a2 along x)
    const L2  = Math.hypot(a3, d4);                 // link 3+4 combined distance

    const cosT3 = (r2 * r2 - L1 * L1 - L2 * L2) / (2 * L1 * L2);
    if (Math.abs(cosT3) > 1 + 1e-9) continue;      // no solution for this t1
    const cosT3c = Math.max(-1, Math.min(1, cosT3));

    // Two elbow configurations (up and down)
    for (const sinSign of [1, -1]) {
      const sinT3 = sinSign * Math.sqrt(1 - cosT3c * cosT3c);
      const t3_raw = Math.atan2(sinT3, cosT3c);

      // Offset for a3, d4 geometry
      const gamma = Math.atan2(d4, a3);  // angle of the a3-d4 link vector
      const t3 = t3_raw - gamma;

      // θ₂: angle to reach wrist from shoulder
      const beta  = Math.atan2(s, r);
      const alpha = Math.atan2(L2 * sinT3, L1 + L2 * cosT3c);
      const t2 = beta - alpha - (PI / 2);   // subtract π/2 for UR-style convention

      // ── Step 4: R₃⁶ = R₀³ᵀ · Rd ────────────────────────────────────────
      // Build R₀³ from DH for joints 1-3 (use first 3 DH entries)
      const T03 = buildT03(t1, t2, t3, dhParams);
      const R03 = extractR(T03);
      const R03T = transpose3(R03);
      const R36 = mulR(R03T, Rd);  // relative rotation for the wrist

      // ── Step 5: Extract θ₄,θ₅,θ₆ from R₃⁶ (ZYZ Euler angles) ──────────
      const wristSols = extractZYZ(R36);

      for (const [t4, t5, t6] of wristSols) {
        solutions.push([t1, t2, t3, t4, t5, t6]);
      }
    }
  }

  if (solutions.length === 0) return null;
  return solutions;
}

// ── Internal helpers ──────────────────────────────────────────────────────────

/** Build 4×4 transform T₀³ from joint angles θ₁,θ₂,θ₃ and DH params. */
function buildT03(t1, t2, t3, dh) {
  const angles = [t1, t2, t3];
  let T = mat4identity();
  for (let i = 0; i < 3; i++) {
    const { a, d, alpha, thetaOffset } = dh[i];
    T = mat4mul(T, dhTransform(angles[i] + thetaOffset, d, a, alpha));
  }
  return T;
}

/** Extract 3×3 rotation matrix (row-major) from 4×4 column-major transform. */
function extractR(T) {
  // Column-major T: indices [0..2] = col0, [4..6] = col1, [8..10] = col2
  return [
    T[0], T[4], T[8],
    T[1], T[5], T[9],
    T[2], T[6], T[10],
  ];
}

/** Transpose a 3×3 row-major matrix. */
function transpose3(R) {
  return [
    R[0], R[3], R[6],
    R[1], R[4], R[7],
    R[2], R[5], R[8],
  ];
}

/** Multiply two 3×3 row-major matrices. */
function mulR(A, B) {
  const C = new Array(9).fill(0);
  for (let r = 0; r < 3; r++) {
    for (let c = 0; c < 3; c++) {
      for (let k = 0; k < 3; k++) {
        C[r * 3 + c] += A[r * 3 + k] * B[k * 3 + c];
      }
    }
  }
  return C;
}

/**
 * Extract ZYZ Euler angles from a 3×3 rotation matrix.
 * ZYZ decomposition: R = Rz(θ₄) · Ry(θ₅) · Rz(θ₆)
 * Returns up to 2 solution pairs (for θ₅ > 0 and θ₅ < 0).
 */
function extractZYZ(R) {
  const solutions = [];

  // R[2,2] = cos(θ₅)
  const cosT5 = Math.max(-1, Math.min(1, R[8]));

  for (const sinSign of [1, -1]) {
    const sinT5 = sinSign * Math.sqrt(1 - cosT5 * cosT5);
    const t5 = Math.atan2(sinT5, cosT5);

    if (Math.abs(sinT5) < 1e-6) {
      // Gimbal lock: θ₅ ≈ 0 or π — infinite solutions for θ₄,θ₆; pick θ₄=0
      const t4 = 0;
      let t6;
      if (cosT5 > 0) {
        // θ₅ = 0: R = Rz(θ₄+θ₆)
        t6 = Math.atan2(R[1], R[0]);
      } else {
        // θ₅ = π: R = Rz(θ₄−θ₆) flipped
        t6 = Math.atan2(-R[1], -R[0]);
      }
      solutions.push([t4, t5, t6]);
    } else {
      // Standard case
      const t4 = Math.atan2(R[5] / sinT5, R[2] / sinT5);
      const t6 = Math.atan2(R[7] / sinT5, -R[6] / sinT5);
      solutions.push([t4, t5, t6]);
    }
  }

  return solutions;
}

/**
 * Pick the best solution from a set by minimising total joint displacement
 * from the current configuration (joint-space proximity).
 *
 * @param {number[][]} solutions - Candidate [θ1…θ6] arrays
 * @param {number[]}   current   - Current joint angles [θ1…θ6]
 * @returns {number[]} Best solution
 */
export function pickBestSolution(solutions, current) {
  let best = null;
  let bestCost = Infinity;

  for (const sol of solutions) {
    const cost = sol.reduce((sum, angle, i) => {
      return sum + Math.abs(angleDiff(current[i], angle));
    }, 0);
    if (cost < bestCost) {
      bestCost = cost;
      best = sol;
    }
  }

  return best;
}
