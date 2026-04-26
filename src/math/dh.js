/**
 * dh.js — Denavit-Hartenberg (DH) Transforms + Forward Kinematics
 *
 * Standard DH convention (Craig notation):
 *
 *   Tᵢ₋₁ᵢ = Rotz(θᵢ) · Tz(dᵢ) · Tx(aᵢ) · Rotx(αᵢ)
 *
 * Represented as a 4×4 homogeneous matrix stored as a flat Float64Array [16].
 * Column-major so it maps directly to Three.js Matrix4 if needed.
 *
 * For 6-DOF robots:
 *   T₀⁶ = T₀¹ · T₁² · T₂³ · T₃⁴ · T₄⁵ · T₅⁶
 *   Position  p = T₀⁶[0:3, 3]
 *   Rotation  R = T₀⁶[0:3, 0:3]
 */

/**
 * Multiply two 4×4 matrices (column-major flat arrays).
 * @param {number[]} A - 4×4 matrix, 16 elements
 * @param {number[]} B - 4×4 matrix, 16 elements
 * @returns {number[]} A × B
 */
export function mat4mul(A, B) {
  const C = new Array(16).fill(0);
  for (let col = 0; col < 4; col++) {
    for (let row = 0; row < 4; row++) {
      let sum = 0;
      for (let k = 0; k < 4; k++) {
        sum += A[k * 4 + row] * B[col * 4 + k];
      }
      C[col * 4 + row] = sum;
    }
  }
  return C;
}

/**
 * Identity 4×4 matrix (column-major).
 */
export function mat4identity() {
  // prettier-ignore
  return [
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1,
  ];
}

/**
 * Compute one DH transform matrix Tᵢ₋₁ᵢ.
 *
 * @param {number} theta  - Joint variable θᵢ (radians) + thetaOffset
 * @param {number} d      - Link offset dᵢ (mm)
 * @param {number} a      - Link length aᵢ (mm)
 * @param {number} alpha  - Twist angle αᵢ (radians)
 * @returns {number[]} 4×4 homogeneous transform (column-major, 16 elements)
 */
export function dhTransform(theta, d, a, alpha) {
  const ct = Math.cos(theta);
  const st = Math.sin(theta);
  const ca = Math.cos(alpha);
  const sa = Math.sin(alpha);

  // Standard DH homogeneous transform matrix (column-major):
  //
  //  [ ct   -st·ca    st·sa   a·ct ]
  //  [ st    ct·ca   -ct·sa   a·st ]
  //  [  0     sa       ca       d  ]
  //  [  0      0        0       1  ]
  //
  // prettier-ignore
  return [
    ct,       st,       0,  0,
   -st * ca,  ct * ca,  sa, 0,
    st * sa, -ct * sa,  ca, 0,
    a * ct,   a * st,   d,  1,
  ];
}

/**
 * Compute the full forward kinematic chain T₀⁶ for a 6-DOF robot.
 *
 * @param {number[]} angles  - Joint angles [θ1…θ6] in radians
 * @param {object[]} dhParams - DH parameter table from presets/
 *   Each entry: { a, d, alpha, thetaOffset }
 * @returns {{
 *   T: number[],       // Final 4×4 transform T₀⁶ (column-major)
 *   Ts: number[][],    // All intermediate transforms [T₀¹, T₀², …, T₀⁶]
 *   position: {x,y,z} // End-effector position (mm)
 * }}
 */
export function forwardKinematics6(angles, dhParams) {
  let T = mat4identity();
  const Ts = [];

  for (let i = 0; i < 6; i++) {
    const { a, d, alpha, thetaOffset } = dhParams[i];
    const theta = angles[i] + thetaOffset;
    const Ti = dhTransform(theta, d, a, alpha);
    T = mat4mul(T, Ti);
    Ts.push([...T]);  // store snapshot T₀ⁱ
  }

  // Extract position from final column (column-major: indices 12, 13, 14)
  const position = { x: T[12], y: T[13], z: T[14] };

  // Extract 3×3 rotation matrix from T₀⁶
  const R = [
    T[0], T[1], T[2],
    T[4], T[5], T[6],
    T[8], T[9], T[10],
  ];

  return { T, Ts, position, R };
}

/**
 * Extract ZYX Euler angles (roll, pitch, yaw) from a 3×3 rotation matrix.
 * Used to convert FK result to human-readable orientation.
 *
 * @param {number[]} R - 3×3 rotation matrix (row-major, 9 elements)
 * @returns {{ roll, pitch, yaw }} in radians
 */
export function rotMatToEulerZYX(R) {
  // R = Rz(yaw)·Ry(pitch)·Rx(roll)
  const pitch = Math.atan2(-R[2], Math.sqrt(R[0] * R[0] + R[1] * R[1]));
  const roll  = Math.atan2(R[5], R[8]);
  const yaw   = Math.atan2(R[1], R[0]);
  return { roll, pitch, yaw };
}
