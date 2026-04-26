/**
 * ik2.test.js — Unit tests for 2-DOF IK/FK math
 *
 * Tests:
 *   1. solveIK2 returns correct solutions for a known target
 *   2. FK(IK(target)) ≈ target within 0.001mm
 *   3. Returns null when target is out of reach
 *   4. Handles edge case: target at maximum reach (arm fully extended)
 *   5. Elbow-up and elbow-down solutions are geometrically different
 */

import { describe, it, expect } from 'vitest';
import { solveIK2, solveFK2 } from '../src/math/ik2.js';

const L1 = 100;
const L2 = 80;
const TOL = 0.001;  // mm — FK round-trip tolerance

describe('solveIK2', () => {
  it('returns two solutions for a reachable target', () => {
    const sols = solveIK2(100, 60, L1, L2);
    expect(sols).not.toBeNull();
    expect(sols).toHaveLength(2);
  });

  it('FK(IK(target)) round-trip is within 0.001mm for elbow-up', () => {
    const target = { x: 100, y: 60 };
    const sols = solveIK2(target.x, target.y, L1, L2);
    const { t1, t2 } = sols[0];   // elbow-up
    const fk = solveFK2(t1, t2, L1, L2);
    expect(Math.abs(fk.x - target.x)).toBeLessThan(TOL);
    expect(Math.abs(fk.y - target.y)).toBeLessThan(TOL);
  });

  it('FK(IK(target)) round-trip is within 0.001mm for elbow-down', () => {
    const target = { x: 100, y: 60 };
    const sols = solveIK2(target.x, target.y, L1, L2);
    const { t1, t2 } = sols[1];   // elbow-down
    const fk = solveFK2(t1, t2, L1, L2);
    expect(Math.abs(fk.x - target.x)).toBeLessThan(TOL);
    expect(Math.abs(fk.y - target.y)).toBeLessThan(TOL);
  });

  it('returns null when target is beyond max reach', () => {
    // Target at (300, 0) — L1+L2 = 180, so clearly unreachable
    const result = solveIK2(300, 0, L1, L2);
    expect(result).toBeNull();
  });

  it('returns null when target is inside inner dead-zone', () => {
    // Target very close to origin — L1-L2 = 20, target at r=5 is inside
    const result = solveIK2(5, 0, L1, L2);
    expect(result).toBeNull();
  });

  it('handles target exactly at maximum reach (arm fully extended)', () => {
    // At exactly r = L1+L2, cos2 = 1 → t2 = 0
    const x = L1 + L2;
    const sols = solveIK2(x, 0, L1, L2);
    expect(sols).not.toBeNull();
    const { t1, t2 } = sols[0];
    const fk = solveFK2(t1, t2, L1, L2);
    expect(Math.abs(fk.x - x)).toBeLessThan(TOL);
    expect(Math.abs(fk.y - 0)).toBeLessThan(TOL);
  });

  it('elbow-up and elbow-down give different configurations', () => {
    const target = { x: 100, y: 80 };
    const sols = solveIK2(target.x, target.y, L1, L2);
    // θ₂ should have opposite sign for elbow-up vs elbow-down
    expect(Math.sign(sols[0].t2)).not.toBe(Math.sign(sols[1].t2));
  });

  it('works for negative x target (second quadrant)', () => {
    const target = { x: -80, y: 60 };
    const sols = solveIK2(target.x, target.y, L1, L2);
    expect(sols).not.toBeNull();
    const { t1, t2 } = sols[0];
    const fk = solveFK2(t1, t2, L1, L2);
    expect(Math.abs(fk.x - target.x)).toBeLessThan(TOL);
    expect(Math.abs(fk.y - target.y)).toBeLessThan(TOL);
  });
});
