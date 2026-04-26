/**
 * fk2dof.js — 2-DOF Planar Forward Kinematics
 *
 * Given joint angles and link lengths, computes joint and end-effector positions.
 * Pure function — no imports, no DOM, no side effects.
 */

/**
 * Compute end-effector position for a 2-DOF planar arm.
 * Used to verify IK solution accuracy: FK(IK(target)) ≈ target.
 *
 * @param {number} t1 - Joint 1 angle (radians)
 * @param {number} t2 - Joint 2 angle (radians)
 * @param {number} L1 - Link 1 length (mm)
 * @param {number} L2 - Link 2 length (mm)
 * @returns {{ x: number, y: number }} End-effector (tip) position in mm
 */
export function solveFK2(t1, t2, L1, L2) {
  const elbowX = L1 * Math.cos(t1);
  const elbowY = L1 * Math.sin(t1);
  const tipX   = elbowX + L2 * Math.cos(t1 + t2);
  const tipY   = elbowY + L2 * Math.sin(t1 + t2);
  return { x: tipX, y: tipY };
}

/**
 * Compute elbow joint position for a 2-DOF planar arm.
 *
 * @param {number} t1 - Joint 1 angle (radians)
 * @param {number} L1 - Link 1 length (mm)
 * @returns {{ x: number, y: number }} Elbow position in mm
 */
export function elbowPos2(t1, L1) {
  return {
    x: L1 * Math.cos(t1),
    y: L1 * Math.sin(t1),
  };
}
