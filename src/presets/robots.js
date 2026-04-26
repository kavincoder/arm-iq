/**
 * robots.js — DH Parameter Tables for Supported Robot Presets
 *
 * All three robots use the spherical wrist assumption (joints 4,5,6 intersect),
 * enabling closed-form analytical IK via Pieper's method in ik6.js.
 *
 * DH parameter convention: Standard DH (Craig notation)
 *   Tᵢ₋₁ᵢ = Rotz(θᵢ + thetaOffset) · Tz(dᵢ) · Tx(aᵢ) · Rotx(αᵢ)
 *
 * Lengths in mm, angles in radians.
 *
 * Sources:
 *   UR5     — Universal Robots UR5 technical spec (CB-series)
 *   KR6     — KUKA KR 6 R900 sixx specification sheet
 *   IRB120  — ABB IRB 120 product manual
 */

const PI = Math.PI;

// ── Universal Robots UR5 ──────────────────────────────────────────────────────
export const UR5 = {
  id: 'ur5',
  name: 'UR5',
  manufacturer: 'Universal Robots',
  description: '6-DOF collaborative robot · 5 kg payload · 850 mm reach',
  color: '#00d4c8',

  // DH parameters (standard convention)
  dh: [
    { index: 1, a:     0, d:  89.159, alpha:  PI / 2, thetaOffset: 0 },
    { index: 2, a: -425.0, d:   0,    alpha:  0,      thetaOffset: 0 },
    { index: 3, a: -392.2, d:  0,    alpha:  0,      thetaOffset: 0 },
    { index: 4, a:     0, d: 109.15,  alpha:  PI / 2, thetaOffset: 0 },
    { index: 5, a:     0, d:  94.65,  alpha: -PI / 2, thetaOffset: 0 },
    { index: 6, a:     0, d:  82.3,   alpha:  0,      thetaOffset: 0 },
  ],

  // Geometric config for IK solver (extracted from DH)
  ikConfig: {
    d1:  89.159,   // z-offset of joint 1
    a2: -425.0,    // link 2 length
    a3: -392.2,    // link 3 length (0.3922 m per UR5 spec)
    d4:  109.15,   // wrist offset
    d6:  82.3,     // tool flange offset (0.0823 m per UR5 spec)
  },

  // Joint limits [min, max] in radians
  limits: [
    { min: -2 * PI, max: 2 * PI },  // θ1
    { min: -2 * PI, max: 2 * PI },  // θ2
    { min: -2 * PI, max: 2 * PI },  // θ3
    { min: -2 * PI, max: 2 * PI },  // θ4
    { min: -2 * PI, max: 2 * PI },  // θ5
    { min: -2 * PI, max: 2 * PI },  // θ6
  ],

  // Default home configuration (all joints at zero)
  homeAngles: [0, -PI / 2, 0, -PI / 2, 0, 0],
};

// ── KUKA KR 6 R900 sixx ──────────────────────────────────────────────────────
// CRITICAL: Joint 2 has a thetaOffset of -PI/2 in this convention.
// Failing to apply this offset results in a completely wrong shoulder pose.
export const KR6 = {
  id: 'kr6',
  name: 'KR6 R900',
  manufacturer: 'KUKA',
  description: '6-DOF industrial robot · 6 kg payload · 901 mm reach',
  color: '#f59e0b',

  dh: [
    { index: 1, a:   25, d:  400, alpha: -PI / 2, thetaOffset: 0 },
    { index: 2, a:  315, d:    0, alpha:  0,       thetaOffset: -PI / 2 },  // ← CRITICAL offset
    { index: 3, a:   35, d:    0, alpha:  PI / 2,  thetaOffset: 0 },
    { index: 4, a:    0, d: -365, alpha: -PI / 2,  thetaOffset: 0 },
    { index: 5, a:    0, d:    0, alpha:  PI / 2,  thetaOffset: 0 },
    { index: 6, a:    0, d:  -80, alpha:  0,       thetaOffset: 0 },
  ],

  ikConfig: {
    d1: 400,
    a2: 315,
    a3:  35,
    d4: 365,
    d6:  80,
  },

  // KUKA joint limits (radians converted from degrees)
  limits: [
    { min: -2.9671, max: 2.9671 },   // θ1 ±170°
    { min: -2.0944, max: 2.0944 },   // θ2 ±120°  (zero at -90° with offset)
    { min: -2.3562, max: 2.3562 },   // θ3 ±135°
    { min: -3.3161, max: 3.3161 },   // θ4 ±190°
    { min: -2.0944, max: 2.0944 },   // θ5 ±120°
    { min: -6.2832, max: 6.2832 },   // θ6 ±360°
  ],

  homeAngles: [0, 0, 0, 0, 0, 0],
};

// ── ABB IRB 120 ───────────────────────────────────────────────────────────────
// CRITICAL: Joint 3 has asymmetric limits (-110° to +70°).
// This is a physical constraint of the arm geometry — must be enforced in IK.
export const IRB120 = {
  id: 'irb120',
  name: 'IRB 120',
  manufacturer: 'ABB',
  description: '6-DOF collaborative robot · 3 kg payload · 580 mm reach',
  color: '#a78bfa',

  dh: [
    { index: 1, a:   0, d:  290, alpha: -PI / 2, thetaOffset: 0 },
    { index: 2, a: 270, d:    0, alpha:  0,       thetaOffset: 0 },
    { index: 3, a:  70, d:    0, alpha:  PI / 2,  thetaOffset: 0 },
    { index: 4, a:   0, d:  302, alpha: -PI / 2,  thetaOffset: 0 },
    { index: 5, a:   0, d:    0, alpha:  PI / 2,  thetaOffset: 0 },
    { index: 6, a:   0, d:   72, alpha:  0,       thetaOffset: 0 },
  ],

  ikConfig: {
    d1: 290,
    a2: 270,
    a3:  70,
    d4: 302,
    d6:  72,
  },

  // CRITICAL: Joint 3 is asymmetric (physical hardware constraint)
  limits: [
    { min: -2.8798, max: 2.8798 },   // θ1 ±165°
    { min: -1.9199, max: 1.9199 },   // θ2 ±110°
    { min: -1.9199, max: 1.2217 },   // θ3 −110° to +70° ← ASYMMETRIC
    { min: -2.7925, max: 2.7925 },   // θ4 ±160°
    { min: -2.0944, max: 2.0944 },   // θ5 ±120°
    { min: -6.2832, max: 6.2832 },   // θ6 ±360°
  ],

  homeAngles: [0, 0, 0, 0, 0, 0],
};

// ── Generic arm ───────────────────────────────────────────────────────────────
// A configurable 6-DOF arm with no real-world constraints.
// DH parameters are user-controlled via sliders.
export const GENERIC = {
  id: 'generic',
  name: 'Generic Arm',
  manufacturer: 'Custom',
  description: 'Configurable 6-DOF arm — adjust DH params freely',
  color: '#38bdf8',

  dh: [
    { index: 1, a:    0, d: 150, alpha:  PI / 2, thetaOffset: 0 },
    { index: 2, a:  300, d:   0, alpha:  0,      thetaOffset: 0 },
    { index: 3, a:  250, d:   0, alpha:  0,      thetaOffset: 0 },
    { index: 4, a:    0, d: 100, alpha:  PI / 2, thetaOffset: 0 },
    { index: 5, a:    0, d:   0, alpha: -PI / 2, thetaOffset: 0 },
    { index: 6, a:    0, d:  80, alpha:  0,      thetaOffset: 0 },
  ],

  ikConfig: {
    d1: 150,
    a2: 300,
    a3: 250,
    d4: 100,
    d6:  80,
  },

  limits: [
    { min: -PI, max: PI },
    { min: -PI, max: PI },
    { min: -PI, max: PI },
    { min: -PI, max: PI },
    { min: -PI, max: PI },
    { min: -PI, max: PI },
  ],

  homeAngles: [0, -PI / 4, PI / 4, 0, PI / 4, 0],
};

/** All available presets in display order. */
export const PRESETS = [GENERIC, UR5, KR6, IRB120];

/** Look up a preset by id string. */
export function getPreset(id) {
  return PRESETS.find(p => p.id === id) ?? GENERIC;
}
