/**
 * ScopeReticle.jsx — Scope / crosshair target marker (SVG)
 *
 * Renders 4 L-shaped corner brackets + inner circle + center dot.
 * Used on all three canvases to mark the target position.
 *
 * Inspired by camera focus assist brackets and radar target cursors.
 *
 * Props:
 *   cx, cy  — center position in SVG pixels
 *   color   — stroke color (amber for reachable, red for out-of-reach)
 *   size    — outer bracket diameter in pixels
 */

export function ScopeReticle({ cx, cy, color = '#f59e0b', size = 28 }) {
  const half = size / 2;
  const arm  = 7;   // bracket arm length
  const sw   = 1.5; // stroke width

  return (
    <g transform={`translate(${cx}, ${cy})`}>
      {/* Top-left L bracket */}
      <path
        d={`M ${-half} ${-half + arm} L ${-half} ${-half} L ${-half + arm} ${-half}`}
        stroke={color} strokeWidth={sw} fill="none"
      />
      {/* Top-right L bracket */}
      <path
        d={`M ${half - arm} ${-half} L ${half} ${-half} L ${half} ${-half + arm}`}
        stroke={color} strokeWidth={sw} fill="none"
      />
      {/* Bottom-right L bracket */}
      <path
        d={`M ${half} ${half - arm} L ${half} ${half} L ${half - arm} ${half}`}
        stroke={color} strokeWidth={sw} fill="none"
      />
      {/* Bottom-left L bracket */}
      <path
        d={`M ${-half + arm} ${half} L ${-half} ${half} L ${-half} ${half - arm}`}
        stroke={color} strokeWidth={sw} fill="none"
      />
      {/* Inner reference circle */}
      <circle cx={0} cy={0} r={3.5} stroke={color} strokeWidth={sw} fill="none" />
      {/* Center dot */}
      <circle cx={0} cy={0} r={0.8} fill={color} />
    </g>
  );
}
