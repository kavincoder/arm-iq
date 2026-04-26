/**
 * dh.test.js — Unit tests for DH transforms and forward kinematics
 */

import { describe, it, expect } from 'vitest';
import { dhTransform, mat4mul, mat4identity, forwardKinematics6 } from '../src/math/fk6dof.js';
import { UR5 } from '../src/presets/robots.js';

const TOL = 0.001;

describe('mat4mul', () => {
  it('identity × identity = identity', () => {
    const I = mat4identity();
    const result = mat4mul(I, I);
    I.forEach((v, i) => expect(Math.abs(result[i] - v)).toBeLessThan(1e-12));
  });
});

describe('dhTransform', () => {
  it('all-zero parameters gives identity-like transform', () => {
    const T = dhTransform(0, 0, 0, 0);
    // Top-left 3×3 should be identity for θ=α=0
    expect(Math.abs(T[0] - 1)).toBeLessThan(TOL);  // col0 row0
    expect(Math.abs(T[5] - 1)).toBeLessThan(TOL);  // col1 row1
    expect(Math.abs(T[10] - 1)).toBeLessThan(TOL); // col2 row2
  });
});

describe('forwardKinematics6 (UR5 home position)', () => {
  it('returns a valid TCP position for UR5 home angles', () => {
    const { position } = forwardKinematics6(UR5.homeAngles, UR5.dh);
    // Position should be finite numbers (exact values depend on DH convention)
    expect(isFinite(position.x)).toBe(true);
    expect(isFinite(position.y)).toBe(true);
    expect(isFinite(position.z)).toBe(true);
  });

  it('returns 6 intermediate transforms', () => {
    const { Ts } = forwardKinematics6(UR5.homeAngles, UR5.dh);
    expect(Ts).toHaveLength(6);
  });
});
