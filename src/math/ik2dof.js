/**
 * ik2.js — 2-DOF Planar Inverse Kinematics
 *
 * Method: Law of Cosines (closed-form, exact)
 *
 * Given target (x, y) and link lengths L1, L2, solve for joint angles θ1, θ2
 * using the geometric triangle formed by origin → elbow → tip.
 *
 * Returns TWO solutions (elbow-up and elbow-down), or null if unreachable.
 */

/**
 * Solve 2-DOF planar IK.
 *
 * @param {number} x      - Target x in mm (workspace frame, origin at base joint)
 * @param {number} y      - Target y in mm
 * @param {number} L1     - Link 1 length in mm
 * @param {number} L2     - Link 2 length in mm
 * @returns {{ t1: number, t2: number }[] | null}
 *   Array of [elbowUp, elbowDown] solutions (t1, t2 in radians),
 *   or null if the target is outside the reachable workspace.
 */
export function solveIK2(x, y, L1, L2) {
  const d2 = x * x + y * y;          // squared distance to target
  const d  = Math.sqrt(d2);          // distance to target

  // Workspace check: inner dead-zone + outer reach limit
  const maxReach = L1 + L2;
  const minReach = Math.abs(L1 - L2);
  if (d > maxReach + 1e-9 || d < minReach - 1e-9) return null;

  // Law of cosines: cos θ₂ = (d² − L1² − L2²) / (2·L1·L2)
  const cosT2 = (d2 - L1 * L1 - L2 * L2) / (2 * L1 * L2);

  // Clamp numerical noise to [-1, 1] before acos
  const cosT2c = Math.max(-1, Math.min(1, cosT2));
  const sinT2_up = Math.sqrt(1 - cosT2c * cosT2c);  // positive → elbow up
  const sinT2_dn = -sinT2_up;                        // negative → elbow down

  // Build both solutions
  return [sinT2_up, sinT2_dn].map(s2 => {
    const t2 = Math.atan2(s2, cosT2c);

    // k1, k2 decompose the direction to the target
    const k1 = L1 + L2 * cosT2c;
    const k2 = L2 * s2;

    // θ₁ = atan2(y, x) − atan2(k2, k1)
    const t1 = Math.atan2(y, x) - Math.atan2(k2, k1);

    return { t1, t2 };
  });
}

// FK functions live in fk2dof.js — re-exported here for backward compatibility
// so existing imports of { solveFK2 } from 'ik2dof.js' continue to work.
export { solveFK2, elbowPos2 } from './fk2dof.js';
