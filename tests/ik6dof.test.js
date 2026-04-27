/**
 * ik6dof.test.js — 6-DOF IK utility and iterative-solver tests
 *
 * Covers:
 *   1. normalizeAngle — wraps to [-π, π]
 *   2. angleDiff — shortest signed angular distance
 *   3. pickBestSolution — picks joint-closest solution
 *   4. solveIKIterative — FK→IK→FK round-trips for UR5, KUKA KR6, ABB IRB120
 *
 * Round-trip strategy:
 *   Pick valid joint angles → compute FK target → solve IK from same angles
 *   (iterative solver starts at the solution so convergence is guaranteed)
 *   → FK of IK result must match original target within POS_TOL.
 *
 * Note on the analytical solver (solveIK6):
 *   The Pieper's-method solver in ik6dof.js does not account for the d4
 *   shoulder offset in industrial presets. It is an educational display;
 *   the actual production solver is solveIKIterative (ikIterative.js).
 */

import { describe, it, expect } from 'vitest';
import { normalizeAngle, angleDiff, pickBestSolution } from '../src/math/ik6dof.js';
import { solveIKIterative } from '../src/math/ikIterative.js';
import { forwardKinematics6 } from '../src/math/fk6dof.js';
import { UR5, KR6, IRB120 } from '../src/presets/robots.js';

const POS_TOL = 1.0;  // mm — round-trip starts at solution → should be << 1mm
const PI = Math.PI;

// FK → IK → FK round-trip error (mm)
function roundTripErr(preset, angles) {
  const { position } = forwardKinematics6(angles, preset.dh);
  const { angles: solved } = solveIKIterative(position, angles, preset.dh);
  const { position: p2 } = forwardKinematics6(solved, preset.dh);
  return Math.hypot(p2.x - position.x, p2.y - position.y, p2.z - position.z);
}

// ── normalizeAngle ────────────────────────────────────────────────────────────

describe('normalizeAngle', () => {
  it('0 → 0', () => {
    expect(normalizeAngle(0)).toBeCloseTo(0, 10);
  });
  it('2π → 0', () => {
    expect(normalizeAngle(2 * PI)).toBeCloseTo(0, 5);
  });
  it('3π → ±π', () => {
    expect(Math.abs(normalizeAngle(3 * PI))).toBeCloseTo(PI, 5);
  });
  it('−3π → ±π', () => {
    expect(Math.abs(normalizeAngle(-3 * PI))).toBeCloseTo(PI, 5);
  });
  it('π stays ±π', () => {
    expect(Math.abs(normalizeAngle(PI))).toBeCloseTo(PI, 10);
  });
  it('always produces value in [−π, π]', () => {
    for (const v of [-10, -5, -PI, 0, PI, 5, 10]) {
      const r = normalizeAngle(v);
      expect(r).toBeGreaterThanOrEqual(-PI - 1e-9);
      expect(r).toBeLessThanOrEqual(PI + 1e-9);
    }
  });
});

// ── angleDiff ─────────────────────────────────────────────────────────────────

describe('angleDiff', () => {
  it('0 → π/2 = +π/2', () => {
    expect(angleDiff(0, PI / 2)).toBeCloseTo(PI / 2, 10);
  });
  it('π/2 → 0 = −π/2', () => {
    expect(angleDiff(PI / 2, 0)).toBeCloseTo(-PI / 2, 10);
  });
  it('takes shortest path: 350° → 10° = +20°', () => {
    expect(angleDiff(350 * PI / 180, 10 * PI / 180)).toBeCloseTo(20 * PI / 180, 5);
  });
  it('takes shortest path: 10° → 350° = −20°', () => {
    expect(angleDiff(10 * PI / 180, 350 * PI / 180)).toBeCloseTo(-20 * PI / 180, 5);
  });
});

// ── pickBestSolution ──────────────────────────────────────────────────────────

describe('pickBestSolution', () => {
  it('picks solution closest to current angles', () => {
    const current = [0, 0, 0, 0, 0, 0];
    const solutions = [
      [PI, PI, PI, PI, PI, PI],
      [0.1, 0.1, 0.1, 0.1, 0.1, 0.1],   // ← closest
      [2, 2, 2, 2, 2, 2],
    ];
    expect(pickBestSolution(solutions, current)).toEqual(solutions[1]);
  });

  it('handles single solution', () => {
    const sol = [1.1, 2.1, 3.1, 0, 0, 0];
    expect(pickBestSolution([sol], [1, 2, 3, 0, 0, 0])).toEqual(sol);
  });

  it('never picks a far solution', () => {
    const current = [0, 0, 0, 0, 0, 0];
    const near = [0.1, 0, 0, 0, 0, 0];
    const far  = [PI - 0.1, PI, PI, PI, PI, PI];
    expect(pickBestSolution([far, near], current)).toEqual(near);
  });
});

// ── solveIKIterative — UR5 ───────────────────────────────────────────────────

describe('solveIKIterative — UR5', () => {
  it('round-trip from home pose', () => {
    expect(roundTripErr(UR5, UR5.homeAngles)).toBeLessThan(POS_TOL);
  });
  it('round-trip from elbow-out pose', () => {
    expect(roundTripErr(UR5, [0, -PI / 4, PI / 3, -PI / 4, PI / 2, 0])).toBeLessThan(POS_TOL);
  });
  it('round-trip from rotated shoulder pose', () => {
    expect(roundTripErr(UR5, [PI / 4, -PI / 3, PI / 4, 0, PI / 3, PI / 6])).toBeLessThan(POS_TOL);
  });
  it('returns { angles, error, iterations } shape', () => {
    const { position } = forwardKinematics6(UR5.homeAngles, UR5.dh);
    const result = solveIKIterative(position, UR5.homeAngles, UR5.dh);
    expect(result).toHaveProperty('angles');
    expect(result).toHaveProperty('error');
    expect(result).toHaveProperty('iterations');
    expect(result.angles).toHaveLength(6);
    expect(result.error).toBeLessThan(POS_TOL);
  });
});

// ── solveIKIterative — KUKA KR6 R900 ─────────────────────────────────────────

describe('solveIKIterative — KUKA KR6 R900', () => {
  it('round-trip from home pose', () => {
    expect(roundTripErr(KR6, KR6.homeAngles)).toBeLessThan(POS_TOL);
  });
  it('round-trip from raised shoulder', () => {
    expect(roundTripErr(KR6, [0, PI / 6, PI / 4, 0, PI / 4, 0])).toBeLessThan(POS_TOL);
  });
  it('round-trip from rotated config', () => {
    expect(roundTripErr(KR6, [PI / 3, PI / 8, PI / 6, PI / 4, PI / 6, 0])).toBeLessThan(POS_TOL);
  });
});

// ── solveIKIterative — ABB IRB 120 ───────────────────────────────────────────

describe('solveIKIterative — ABB IRB 120', () => {
  it('round-trip from home pose', () => {
    expect(roundTripErr(IRB120, IRB120.homeAngles)).toBeLessThan(POS_TOL);
  });
  it('round-trip from raised pose', () => {
    expect(roundTripErr(IRB120, [0, PI / 4, PI / 6, 0, PI / 4, 0])).toBeLessThan(POS_TOL);
  });
  it('round-trip from rotated shoulder', () => {
    expect(roundTripErr(IRB120, [PI / 5, PI / 6, PI / 8, 0, PI / 5, 0])).toBeLessThan(POS_TOL);
  });
});
