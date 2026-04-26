/**
 * fk2dof.test.js — Unit tests for 2-DOF Forward Kinematics
 *
 * Tests:
 *   1. Both joints at 0 → tip at (L1+L2, 0) (arm fully extended along +X)
 *   2. Joint 1 at PI/2, joint 2 at 0 → tip at (0, L1+L2)
 *   3. Both joints at PI/4 → verify against manual trig calculation
 *   4. FK(IK(target)[0]) round-trips back to target within 0.001mm
 */

import { describe, it, expect } from 'vitest';
import { solveFK2 } from '../src/math/fk2dof.js';
import { solveIK2 } from '../src/math/ik2dof.js';

const L1 = 100;
const L2 = 80;
const TOL = 0.001;

describe('solveFK2', () => {
  it('both joints at 0 → tip at (L1+L2, 0)', () => {
    const { x, y } = solveFK2(0, 0, L1, L2);
    expect(Math.abs(x - (L1 + L2))).toBeLessThan(TOL);
    expect(Math.abs(y - 0)).toBeLessThan(TOL);
  });

  it('joint1=PI/2, joint2=0 → tip at (0, L1+L2)', () => {
    const { x, y } = solveFK2(Math.PI / 2, 0, L1, L2);
    expect(Math.abs(x - 0)).toBeLessThan(TOL);
    expect(Math.abs(y - (L1 + L2))).toBeLessThan(TOL);
  });

  it('both joints at PI/4 → matches manual trig', () => {
    const t1 = Math.PI / 4;
    const t2 = Math.PI / 4;
    // Elbow at (L1·cos(t1), L1·sin(t1))
    const elbowX = L1 * Math.cos(t1);
    const elbowY = L1 * Math.sin(t1);
    // Tip at elbow + L2·(cos(t1+t2), sin(t1+t2))
    const expectedX = elbowX + L2 * Math.cos(t1 + t2);
    const expectedY = elbowY + L2 * Math.sin(t1 + t2);
    const { x, y } = solveFK2(t1, t2, L1, L2);
    expect(Math.abs(x - expectedX)).toBeLessThan(TOL);
    expect(Math.abs(y - expectedY)).toBeLessThan(TOL);
  });

  it('FK(IK(target)[0]) round-trip within 0.001mm', () => {
    const target = { x: 120, y: 60 };
    const sols = solveIK2(target.x, target.y, L1, L2);
    expect(sols).not.toBeNull();
    const { t1, t2 } = sols[0];   // elbow-up solution
    const { x, y } = solveFK2(t1, t2, L1, L2);
    expect(Math.abs(x - target.x)).toBeLessThan(TOL);
    expect(Math.abs(y - target.y)).toBeLessThan(TOL);
  });
});
