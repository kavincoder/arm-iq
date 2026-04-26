/**
 * ik3.js — 3-DOF Planar Inverse Kinematics
 *
 * Method: Geometric decomposition (closed-form, exact)
 *
 * Strategy:
 *   1. The end-effector orientation φ (phi) is a free parameter the user controls.
 *   2. Back-compute the wrist point: (wx, wy) = (x − L3·cos φ, y − L3·sin φ)
 *   3. Solve the 2-link sub-problem to (wx, wy) for θ₁, θ₂   (reuses ik2.js)
 *   4. θ₃ = φ − θ₁ − θ₂   (angle-sum constraint for planar chain)
 *
 * Returns two solutions (elbow-up and elbow-down), or null if unreachable.
 */

import { solveIK2 } from './ik2dof.js';

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
  const sols2 = solveIK2(wx, wy, L1, L2);
  if (!sols2) return null;  // wrist point is unreachable

  // Step 3: compute θ₃ for each sub-solution
  return sols2.map(({ t1, t2 }) => {
    const t3 = phi - t1 - t2;   // angle-sum closure constraint
    return { t1, t2, t3, wx, wy };
  });
}

// FK function lives in fk3dof.js — re-exported here for backward compatibility.
export { solveFK3 } from './fk3dof.js';
