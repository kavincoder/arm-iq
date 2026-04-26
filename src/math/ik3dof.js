/**
 * ik3dof.js — 3-DOF Planar Inverse Kinematics
 *
 * Method: Geometric decomposition (closed-form, exact)
 *
 * Strategy:
 *   1. The end-effector orientation φ (phi) is a free parameter the user controls.
 *   2. Back-compute the wrist point: (wx, wy) = (x − L3·cos φ, y − L3·sin φ)
 *   3. Solve the 2-link sub-problem to (wx, wy) for θ₁, θ₂ (inlined below)
 *   4. θ₃ = φ − θ₁ − θ₂   (angle-sum constraint for planar chain)
 *
 * Returns two solutions (elbow-up and elbow-down), or null if unreachable.
 *
 * No import statements — all dependencies are inlined as local functions.
 */

// ── Inlined 2-DOF IK solver (law of cosines) ─────────────────────────────────
// Avoids import dependency on ik2dof.js while keeping this file pure.
// Identical logic to solveIK2 in ik2dof.js.
function solveIK2Local(x, y, L1, L2) {
  const d2 = x * x + y * y;
  const d  = Math.sqrt(d2);
  const maxReach = L1 + L2;
  const minReach = Math.abs(L1 - L2);
  if (d > maxReach + 1e-9 || d < minReach - 1e-9) return null;

  const cosT2  = (d2 - L1 * L1 - L2 * L2) / (2 * L1 * L2);
  const cosT2c = Math.max(-1, Math.min(1, cosT2));
  const sinT2_up = Math.sqrt(1 - cosT2c * cosT2c);
  const sinT2_dn = -sinT2_up;

  return [sinT2_up, sinT2_dn].map(s2 => {
    const t2 = Math.atan2(s2, cosT2c);
    const k1 = L1 + L2 * cosT2c;
    const k2 = L2 * s2;
    const t1 = Math.atan2(y, x) - Math.atan2(k2, k1);
    return { t1, t2 };
  });
}

/**
 * Solve 3-DOF planar IK.
 *
 * @param {number} x    - Target x in mm
 * @param {number} y    - Target y in mm
 * @param {number} phi  - Desired end-effector orientation in radians
 * @param {number} L1   - Link 1 length (mm)
 * @param {number} L2   - Link 2 length (mm)
 * @param {number} L3   - Link 3 / wrist length (mm)
 * @returns {{ t1, t2, t3, wx, wy }[] | null}
 *   Array of up-to-2 solutions, or null if unreachable.
 */
export function solveIK3(x, y, phi, L1, L2, L3) {
  // Step 1: back-compute wrist position by removing link 3 along orientation φ
  const wx = x - L3 * Math.cos(phi);
  const wy = y - L3 * Math.sin(phi);

  // Step 2: solve the 2-link IK to the wrist point
  const sols2 = solveIK2Local(wx, wy, L1, L2);
  if (!sols2) return null;  // wrist point is unreachable

  // Step 3: compute θ₃ for each sub-solution
  return sols2.map(({ t1, t2 }) => {
    const t3 = phi - t1 - t2;   // angle-sum closure constraint
    return { t1, t2, t3, wx, wy };
  });
}

// FK function lives in fk3dof.js — re-exported here for backward compatibility.
export { solveFK3 } from './fk3dof.js';
