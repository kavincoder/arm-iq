/**
 * format.js — Number formatting helpers for the calculation panel
 *
 * All number displays use fixed decimal places so the text width stays
 * constant even as values update at 60fps (prevents layout jitter).
 * Combined with font-variant-numeric: tabular-nums in CSS.
 */

/**
 * Format a length value (mm) with 2 decimal places.
 * @param {number} v
 * @returns {string}
 */
export function fmtMM(v) {
  return (typeof v === 'number' ? v : 0).toFixed(2);
}

/**
 * Format an angle in radians as degrees with 2 decimal places.
 * @param {number} rad
 * @returns {string} e.g. "45.00"
 */
export function fmtDeg(rad) {
  return (rad * 180 / Math.PI).toFixed(2);
}

/**
 * Format a raw degree value with 2 decimal places.
 * @param {number} deg
 * @returns {string}
 */
export function fmtDegRaw(deg) {
  return (typeof deg === 'number' ? deg : 0).toFixed(2);
}

/**
 * Format a dimensionless ratio (e.g. cos θ) with 4 decimal places.
 * @param {number} v
 * @returns {string}
 */
export function fmtRatio(v) {
  return (typeof v === 'number' ? v : 0).toFixed(4);
}

/**
 * Format a very small number in scientific notation if < 0.01,
 * otherwise fixed 4 decimal places. Used for FK error display.
 * @param {number} v
 * @returns {string}
 */
export function fmtError(v) {
  const a = Math.abs(v);
  if (a === 0) return '0.0000';
  if (a < 0.0001) return v.toExponential(2);
  return a.toFixed(4);
}

/**
 * Convert radians to degrees (number).
 * @param {number} rad
 * @returns {number}
 */
export function toDeg(rad) {
  return rad * 180 / Math.PI;
}

/**
 * Convert degrees to radians (number).
 * @param {number} deg
 * @returns {number}
 */
export function toRad(deg) {
  return deg * Math.PI / 180;
}
