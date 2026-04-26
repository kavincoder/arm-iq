/**
 * ik3.test.js — Unit tests for 3-DOF IK/FK math
 */

import { describe, it, expect } from 'vitest';
import { solveIK3, solveFK3 } from '../src/math/ik3.js';

const L1 = 90, L2 = 75, L3 = 55;
const TOL = 0.001;

describe('solveIK3', () => {
  it('returns solutions for a reachable target', () => {
    const sols = solveIK3(130, 70, 0.436, L1, L2, L3);
    expect(sols).not.toBeNull();
    expect(sols.length).toBeGreaterThan(0);
  });

  it('FK(IK(target)) round-trip within 0.001mm', () => {
    const [x, y, phi] = [120, 50, 0.3];
    const sols = solveIK3(x, y, phi, L1, L2, L3);
    expect(sols).not.toBeNull();
    const { t1, t2, t3 } = sols[0];
    const { tip } = solveFK3(t1, t2, t3, L1, L2, L3);
    expect(Math.abs(tip.x - x)).toBeLessThan(TOL);
    expect(Math.abs(tip.y - y)).toBeLessThan(TOL);
  });

  it('θ₃ = φ − θ₁ − θ₂ (angle-sum constraint)', () => {
    const phi = 0.5;
    const sols = solveIK3(100, 60, phi, L1, L2, L3);
    expect(sols).not.toBeNull();
    const { t1, t2, t3 } = sols[0];
    expect(Math.abs((t1 + t2 + t3) - phi)).toBeLessThan(TOL);
  });

  it('returns null when wrist point is unreachable', () => {
    // Target 500mm away — impossible for this arm
    const result = solveIK3(500, 0, 0, L1, L2, L3);
    expect(result).toBeNull();
  });
});
