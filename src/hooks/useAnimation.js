/**
 * useAnimation.js — requestAnimationFrame-based smooth angle animation
 *
 * Manages a looping rAF that lerps current joint angles toward a target.
 * Stops automatically when all joints have settled (within tolerance).
 *
 * Usage:
 *   const { angles, setTarget } = useAnimation(initialAngles);
 *   // Call setTarget(newAngles) whenever IK produces a new solution
 */

import { useRef, useState, useEffect, useCallback } from 'react';
import { lerpAngles, anglesSettled } from '../utils/lerp.js';

const LERP_ALPHA = 0.14;   // interpolation speed per frame (14% per frame ≈ ~100ms settle)
const SETTLE_TOL = 0.0002; // radians — convergence threshold

/**
 * @param {number[]} initialAngles - Starting joint configuration
 * @returns {{
 *   angles: number[],            // current (animated) joint angles
 *   setTarget: (a: number[]) => void,  // trigger animation to a new target
 *   isAnimating: boolean,        // true while lerping
 * }}
 */
export function useAnimation(initialAngles) {
  const [angles, setAngles] = useState([...initialAngles]);
  const [isAnimating, setIsAnimating] = useState(false);

  // Refs hold mutable animation state without triggering re-renders mid-frame
  const currentRef = useRef([...initialAngles]);
  const targetRef  = useRef([...initialAngles]);
  const rafRef     = useRef(null);

  // Cancel any pending rAF on unmount
  useEffect(() => {
    return () => {
      if (rafRef.current) cancelAnimationFrame(rafRef.current);
    };
  }, []);

  const tick = useCallback(() => {
    const next = lerpAngles(currentRef.current, targetRef.current, LERP_ALPHA);
    currentRef.current = next;
    setAngles([...next]);

    if (anglesSettled(next, targetRef.current, SETTLE_TOL)) {
      // Snap exactly to target and stop
      currentRef.current = [...targetRef.current];
      setAngles([...targetRef.current]);
      setIsAnimating(false);
      rafRef.current = null;
    } else {
      rafRef.current = requestAnimationFrame(tick);
    }
  }, []);

  const setTarget = useCallback((newTarget) => {
    targetRef.current = [...newTarget];

    // Start the loop if not already running
    if (!rafRef.current) {
      setIsAnimating(true);
      rafRef.current = requestAnimationFrame(tick);
    }
  }, [tick]);

  return { angles, setTarget, isAnimating };
}
