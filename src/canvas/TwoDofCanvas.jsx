/**
 * TwoDofCanvas.jsx — 2-DOF planar arm SVG renderer
 *
 * Renders the arm, workspace rings, axes, grid, joint angle arcs,
 * end-effector glow, and scope-reticle target — all as inline SVG
 * (no Canvas 2D API, so no pixel rasterisation artifacts at any DPI).
 *
 * Interaction: click anywhere on the SVG to set a new target position.
 * The IK is solved instantly; animation happens in the parent section.
 *
 * Props:
 *   width, height — canvas dimensions (px) from useCanvasSize
 *   L1, L2        — link lengths (mm)
 *   angles        — { t1, t2 } current (animated) joint angles (radians)
 *   target        — { x, y } target in mm-space
 *   reachable     — boolean: whether target is within workspace
 *   onTargetClick — (x, y) => void: called with mm coords when user clicks
 */

import { ScopeReticle } from './ScopeReticle.jsx';

export function TwoDofCanvas({
  width = 760, height = 520,
  L1 = 100, L2 = 80,
  angles = { t1: 0, t2: 0 },
  target = { x: 110, y: 60 },
  reachable = true,
  onTargetClick,
}) {
  // Scale factor: how many pixels per mm
  // Center of SVG is at origin (base joint)
  const scale = Math.min(width, height) / (2 * (L1 + L2) * 1.35);
  const ox = width / 2;
  const oy = height * 0.85;  // origin near bottom so arm has full upward space

  // Transform mm → SVG pixel coords
  const sx = (mm) => ox + mm * scale;
  const sy = (mm) => oy - mm * scale;   // Y flipped (SVG down = positive)

  const { t1, t2 } = angles;

  // Joint positions in mm-space
  const elbow = {
    x: L1 * Math.cos(t1),
    y: L1 * Math.sin(t1),
  };
  const tip = {
    x: elbow.x + L2 * Math.cos(t1 + t2),
    y: elbow.y + L2 * Math.sin(t1 + t2),
  };

  // Workspace radii in pixels
  const outerR = (L1 + L2) * scale;
  const innerR = Math.abs(L1 - L2) * scale;

  // Axis half-extents
  const axHX = (width - 60) / 2;
  const axHY = (height - 60) / 2;

  // Axis tick marks every 30mm
  const ticks = [];
  for (let i = -8; i <= 8; i++) {
    if (i === 0) continue;
    const v = i * 30;
    const xpx = ox + v * scale;
    if (xpx > 20 && xpx < width - 20) ticks.push({ kind: 'x', px: xpx, label: v });
    const ypx = oy - v * scale;
    if (ypx > 20 && ypx < height - 20) ticks.push({ kind: 'y', py: ypx, label: v });
  }

  // Grid lines (24px pitch = ~10mm at typical scale)
  const gridCols = Math.floor(width / 24) + 2;
  const gridRows = Math.floor(height / 24) + 2;

  // Handle click on canvas — convert pixel → mm
  const handleClick = (e) => {
    if (!onTargetClick) return;
    const rect = e.currentTarget.getBoundingClientRect();
    const px = e.clientX - rect.left;
    const py = e.clientY - rect.top;
    const mmX = (px - ox) / scale;
    const mmY = (oy - py) / scale;
    onTargetClick(mmX, mmY);
  };

  // Arc path for joint angle indicator
  const describeArc = (cx, cy, r, startA, endA) => {
    const toSVG = (a) => ({ x: cx + r * Math.cos(-a), y: cy + r * Math.sin(-a) });
    const s = toSVG(endA);
    const e = toSVG(startA);
    const large = Math.abs(endA - startA) > Math.PI ? 1 : 0;
    return `M ${s.x} ${s.y} A ${r} ${r} 0 ${large} 0 ${e.x} ${e.y}`;
  };

  return (
    <svg
      width={width} height={height}
      viewBox={`0 0 ${width} ${height}`}
      style={{ display: 'block', cursor: 'crosshair' }}
      onClick={handleClick}
    >
      <defs>
        {/* End-effector radial glow */}
        <radialGradient id="eeGlow2" cx="50%" cy="50%" r="50%">
          <stop offset="0%"   stopColor="rgba(0,212,200,0.7)" />
          <stop offset="60%"  stopColor="rgba(0,212,200,0.18)" />
          <stop offset="100%" stopColor="rgba(0,212,200,0)" />
        </radialGradient>
      </defs>

      {/* ── CAD dot grid ── */}
      <g opacity="0.5">
        {Array.from({ length: gridCols }).map((_, i) => (
          <line key={`vg${i}`} x1={i * 24} y1={0} x2={i * 24} y2={height}
            stroke="rgba(255,255,255,0.025)" strokeWidth="1" />
        ))}
        {Array.from({ length: gridRows }).map((_, i) => (
          <line key={`hg${i}`} x1={0} y1={i * 24} x2={width} y2={i * 24}
            stroke="rgba(255,255,255,0.025)" strokeWidth="1" />
        ))}
      </g>

      {/* ── Axes ── */}
      <line x1={ox - axHX} y1={oy} x2={ox + axHX} y2={oy}
        stroke="rgba(255,255,255,0.12)" strokeWidth="1" strokeDasharray="4 4" />
      <line x1={ox} y1={oy - axHY} x2={ox} y2={oy + axHY}
        stroke="rgba(255,255,255,0.12)" strokeWidth="1" strokeDasharray="4 4" />

      {/* Axis labels */}
      <text x={ox + axHX - 16} y={oy - 6} fontFamily="JetBrains Mono" fontSize="9" fill="#484f58">+X</text>
      <text x={ox + 6}         y={oy - axHY + 12} fontFamily="JetBrains Mono" fontSize="9" fill="#484f58">+Y</text>
      <text x={ox - axHX + 4}  y={oy - 6} fontFamily="JetBrains Mono" fontSize="9" fill="#484f58">-X</text>
      <text x={ox - 22}        y={oy + axHY - 4}  fontFamily="JetBrains Mono" fontSize="9" fill="#484f58">-Y</text>

      {/* Tick marks */}
      {ticks.map((t, i) => t.kind === 'x' ? (
        <g key={`tx${i}`}>
          <line x1={t.px} y1={oy - 3} x2={t.px} y2={oy + 3} stroke="rgba(255,255,255,0.18)" strokeWidth="1" />
          <text x={t.px} y={oy + 14} fontFamily="JetBrains Mono" fontSize="8" fill="#484f58" textAnchor="middle">{t.label}</text>
        </g>
      ) : (
        <g key={`ty${i}`}>
          <line x1={ox - 3} y1={t.py} x2={ox + 3} y2={t.py} stroke="rgba(255,255,255,0.18)" strokeWidth="1" />
          <text x={ox - 6} y={t.py + 3} fontFamily="JetBrains Mono" fontSize="8" fill="#484f58" textAnchor="end">{t.label}</text>
        </g>
      ))}

      {/* ── Workspace rings ── */}
      <circle cx={ox} cy={oy} r={outerR}
        fill="rgba(0,212,200,0.025)" stroke="#00a89e" strokeWidth="1" strokeDasharray="6 4" />
      {innerR > 2 && (
        <circle cx={ox} cy={oy} r={innerR}
          fill="#080c10" stroke="rgba(0,212,200,0.3)" strokeWidth="1" strokeDasharray="4 4" />
      )}
      <text
        x={ox + outerR * 0.707 + 6} y={oy - outerR * 0.707 - 4}
        fontFamily="JetBrains Mono" fontSize="9" fill="#00a89e" letterSpacing="0.06em"
      >
        WORKSPACE · R={L1 + L2}mm
      </text>

      {/* ── Target → tip distance line ── */}
      <line
        x1={sx(tip.x)} y1={sy(tip.y)}
        x2={sx(target.x)} y2={sy(target.y)}
        stroke="rgba(245,158,11,0.4)" strokeWidth="1" strokeDasharray="4 4"
      />

      {/* ── Links ── */}
      {/* Link 1 — teal */}
      <line x1={sx(0)} y1={sy(0)} x2={sx(elbow.x)} y2={sy(elbow.y)}
        stroke="#00d4c8" strokeWidth="8" strokeLinecap="round" />
      {/* Link 2 — sky blue */}
      <line x1={sx(elbow.x)} y1={sy(elbow.y)} x2={sx(tip.x)} y2={sy(tip.y)}
        stroke="#38bdf8" strokeWidth="7" strokeLinecap="round" />

      {/* ── Joint angle arc (θ₁ at base) ── */}
      <path
        d={describeArc(ox, oy, 28, 0, t1)}
        fill="none" stroke="rgba(0,212,200,0.5)" strokeWidth="1" strokeDasharray="2 3"
      />
      <text x={ox + 36} y={oy - 8} fontFamily="JetBrains Mono" fontSize="9" fill="#00d4c8">θ₁</text>

      {/* ── Base joint ── */}
      <circle cx={sx(0)} cy={sy(0)} r={10} fill="#0d1117" stroke="#30363d" strokeWidth="2" />
      <circle cx={sx(0)} cy={sy(0)} r={2.5} fill="#30363d" />

      {/* ── Elbow joint ── */}
      <circle cx={sx(elbow.x)} cy={sy(elbow.y)} r={8} fill="#161b22" stroke="#00a89e" strokeWidth="1.5" />

      {/* ── End-effector with glow ── */}
      <circle cx={sx(tip.x)} cy={sy(tip.y)} r={22} fill="url(#eeGlow2)" />
      <circle cx={sx(tip.x)} cy={sy(tip.y)} r={10} fill="white" stroke="#00d4c8" strokeWidth="2" />
      <circle cx={sx(tip.x)} cy={sy(tip.y)} r={4} fill="#00d4c8" />

      {/* End-effector coordinate callout */}
      <g transform={`translate(${sx(tip.x) + 16}, ${sy(tip.y) - 30})`}>
        <rect x={0} y={0} width={120} height={36} rx={3} fill="#1c2330" stroke="#30363d" />
        <text x={8} y={14} fontFamily="Space Grotesk" fontSize="8.5" fill="#8b949e" letterSpacing="0.08em">END EFFECTOR</text>
        <text x={8} y={28} fontFamily="JetBrains Mono" fontSize="10.5" fill="#00d4c8">
          {tip.x.toFixed(1)}, {tip.y.toFixed(1)}
          <tspan fill="#484f58"> mm</tspan>
        </text>
      </g>

      {/* ── Target reticle ── */}
      <ScopeReticle
        cx={sx(target.x)} cy={sy(target.y)}
        color={reachable ? '#f59e0b' : '#ef4444'}
        size={28}
      />

      {/* Target label */}
      <text
        x={sx(target.x) - 64} y={sy(target.y) + 26}
        fontFamily="JetBrains Mono" fontSize="9" fill={reachable ? '#f59e0b' : '#ef4444'}
      >
        T ({target.x.toFixed(1)}, {target.y.toFixed(1)})
      </text>
    </svg>
  );
}
