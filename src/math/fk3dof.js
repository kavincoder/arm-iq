/**
 * fk3dof.js — 3-DOF Planar Forward Kinematics
 *
 * Given joint angles and link lengths, computes all joint positions and
 * the end-effector pose (position + orientation angle phi).
 * Pure function — no imports, no DOM, no side effects.
 */

/**
 * Forward kinematics for a 3-DOF planar arm.
 * Returns all joint positions and end-effector pose.
 *
 * @param {number} t1 - Joint 1 angle (radians)
 * @param {number} t2 - Joint 2 angle (radians)
 * @param {number} t3 - Joint 3 angle (radians)
 * @param {number} L1 - Link 1 length (mm)
 * @param {number} L2 - Link 2 length (mm)
 * @param {number} L3 - Link 3 length (mm)
 * @returns {{ j1: {x,y}, j2: {x,y}, tip: {x,y}, phi: number }}
 *   j1 = elbow 1 position, j2 = elbow 2 position,
 *   tip = end-effector position, phi = end-effector orientation (radians)
 */
export function solveFK3(t1, t2, t3, L1, L2, L3) {
  const j1 = {
    x: L1 * Math.cos(t1),
    y: L1 * Math.sin(t1),
  };
  const j2 = {
    x: j1.x + L2 * Math.cos(t1 + t2),
    y: j1.y + L2 * Math.sin(t1 + t2),
  };
  const tip = {
    x: j2.x + L3 * Math.cos(t1 + t2 + t3),
    y: j2.y + L3 * Math.sin(t1 + t2 + t3),
  };
  return { j1, j2, tip, phi: t1 + t2 + t3 };
}
