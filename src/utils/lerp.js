/**
 * lerp.js — Animation utilities for smooth joint angle interpolation
 *
 * Uses linear interpolation (lerp) on each frame via requestAnimationFrame.
 * angleDiff ensures we always take the shortest angular path (e.g. going from
 * 170° to -170° rotates -20°, not +340°).
 */

import { normalizeAngle } from '../math/ik6dof.js';

/**
 * Lerp a scalar value toward a target.
 * @param {number} current
 * @param {number} target
 * @param {number} alpha  - Fraction per frame (0 = no movement, 1 = instant)
 * @returns {number}
 */
export function lerp(current, target, alpha) {
  return current + (target - current) * alpha;
}

/**
 * Lerp an angle toward a target using the shortest arc.
 * Avoids 359° → 0° going the long way around.
 *
 * @param {number} current - Current angle (radians)
 * @param {number} target  - Target angle (radians)
 * @param {number} alpha   - Lerp factor per frame
 * @returns {number} New angle (radians)
 */
export function lerpAngle(current, target, alpha) {
  const diff = normalizeAngle(target - current);
  return current + diff * alpha;
}

/**
 * Lerp all joints in an array toward their targets.
 *
 * @param {number[]} current - Current joint angles (radians)
 * @param {number[]} target  - Target joint angles (radians)
 * @param {number}   alpha   - Per-frame lerp factor (default: 0.12)
 * @returns {number[]} New joint angles
 */
export function lerpAngles(current, target, alpha = 0.12) {
  return current.map((a, i) => lerpAngle(a, target[i], alpha));
}

/**
 * Returns true if all angles are within tolerance of their targets.
 * Used to stop the animation loop when the arm has settled.
 *
 * @param {number[]} current
 * @param {number[]} target
 * @param {number}   tol - Convergence tolerance in radians (default: 0.0001)
 */
export function anglesSettled(current, target, tol = 0.0001) {
  return current.every((a, i) => Math.abs(normalizeAngle(target[i] - a)) < tol);
}
