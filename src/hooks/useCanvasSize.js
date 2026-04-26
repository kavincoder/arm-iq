/**
 * useCanvasSize.js — Responsive canvas sizing via ResizeObserver
 *
 * Watches a container element and returns its current pixel dimensions.
 * The canvas/SVG renderers subscribe to these dimensions so they fill
 * their parent containers without overflow.
 *
 * Usage:
 *   const { ref, width, height } = useCanvasSize();
 *   return <div ref={ref}><svg width={width} height={height} .../></div>
 */

import { useState, useEffect, useRef, useCallback } from 'react';

/**
 * @returns {{
 *   ref: React.RefCallback,
 *   width: number,
 *   height: number,
 * }}
 */
export function useCanvasSize() {
  const [size, setSize] = useState({ width: 800, height: 500 });
  const observerRef = useRef(null);

  // Use a callback ref so we immediately observe whatever element is mounted
  const ref = useCallback((node) => {
    if (observerRef.current) {
      observerRef.current.disconnect();
      observerRef.current = null;
    }
    if (!node) return;

    // Take initial size from bounding rect
    const rect = node.getBoundingClientRect();
    if (rect.width > 0 && rect.height > 0) {
      setSize({ width: rect.width, height: rect.height });
    }

    // Then watch for future resizes
    const ro = new ResizeObserver(entries => {
      for (const entry of entries) {
        const { width, height } = entry.contentRect;
        if (width > 0 && height > 0) {
          setSize({ width, height });
        }
      }
    });
    ro.observe(node);
    observerRef.current = ro;
  }, []);

  // Cleanup on unmount
  useEffect(() => {
    return () => {
      if (observerRef.current) observerRef.current.disconnect();
    };
  }, []);

  return { ref, width: size.width, height: size.height };
}
