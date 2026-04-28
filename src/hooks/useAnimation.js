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

const LERP_RATE  = 0.14;   // fraction to close per 1/60 s frame — frame-rate independent
const TARGET_DT  = 1000 / 60; // ms per frame at 60 fps
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
  const currentRef  = useRef([...initialAngles]);
  const targetRef   = useRef([...initialAngles]);
  const rafRef      = useRef(null);
  const lastTimeRef = useRef(null); // previous rAF timestamp for dt calculation

  // Cancel any pending rAF on unmount
  useEffect(() => {
    return () => {
      if (rafRef.current) cancelAnimationFrame(rafRef.current);
    };
  }, []);

  const tick = useCallback((timestamp) => {
    // Time-normalised alpha: same wall-clock speed regardless of rAF frame rate.
    // Cap dt at 100ms so a long tab-sleep doesn't cause a jarring jump.
    const dt = lastTimeRef.current == null
      ? TARGET_DT
      : Math.min(timestamp - lastTimeRef.current, 100);
    lastTimeRef.current = timestamp;
    const alpha = 1 - Math.pow(1 - LERP_RATE, dt / TARGET_DT);

    const next = lerpAngles(currentRef.current, targetRef.current, alpha);
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
    lastTimeRef.current = null; // reset so first tick uses one TARGET_DT step

    // Start the loop if not already running
    if (!rafRef.current) {
      setIsAnimating(true);
      rafRef.current = requestAnimationFrame(tick);
    }
  }, [tick]);

  return { angles, setTarget, isAnimating };
}
