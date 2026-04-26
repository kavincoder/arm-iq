/**
 * ThreeDofCanvas.jsx — 3-DOF planar arm SVG renderer
 *
 * Extends the 2-DOF canvas with a third link and a draggable phi (φ)
 * orientation handle at the target position.
 *
 * Interaction:
 *   - Click canvas → set new target position
 *   - Drag the φ handle → change desired end-effector orientation
 *
 * Props:
 *   width, height — canvas dimensions
 *   L1, L2, L3   — link lengths (mm)
 *   angles        — { t1, t2, t3 } current animated joint angles (radians)
 *   target        — { x, y, phi } target pose (mm + radians for phi)
 *   reachable     — boolean
 *   onTargetClick — (x, y) => void
 *   onPhiChange   — (phi: number) => void: called when phi handle is dragged
 */

import { ScopeReticle } from './ScopeReticle.jsx';
import { useRef } from 'react';

export function ThreeDofCanvas({
  width = 760, height = 540,
  L1 = 90, L2 = 75, L3 = 55,
  angles = { t1: 0, t2: 0, t3: 0 },
  target = { x: 130, y: 70, phi: 0 },
  reachable = true,
  onTargetClick,
  onPhiChange,
}) {
  const scale = Math.min(width, height) / (2 * (L1 + L2 + L3) * 1.25);
  const ox = width / 2;
  const oy = height / 2 + height * 0.08;

  const sx = (mm) => ox + mm * scale;
  const sy = (mm) => oy - mm * scale;

  const { t1, t2, t3 } = angles;

  // Joint positions
  const j1 = { x: L1 * Math.cos(t1),         y: L1 * Math.sin(t1) };
  const j2 = { x: j1.x + L2 * Math.cos(t1 + t2), y: j1.y + L2 * Math.sin(t1 + t2) };
  const tip = { x: j2.x + L3 * Math.cos(t1 + t2 + t3), y: j2.y + L3 * Math.sin(t1 + t2 + t3) };

  const outerR = (L1 + L2 + L3) * scale;
  const innerR = Math.max(0, Math.abs(L1 - L2) - L3) * scale;

  // Phi handle position (36px from target along orientation direction)
  const phiHandleDist = 36;
  const phiHandle = {
    px: sx(target.x) + Math.cos(target.phi) * phiHandleDist,
    py: sy(target.y) - Math.sin(target.phi) * phiHandleDist,
  };

  const isDraggingPhi = useRef(false);

  // Click handler: prevent phi-handle drags from triggering target relocation
  const handleClick = (e) => {
    if (isDraggingPhi.current) { isDraggingPhi.current = false; return; }
    if (!onTargetClick) return;
    const rect = e.currentTarget.getBoundingClientRect();
    const mmX = (e.clientX - rect.left - ox) / scale;
    const mmY = (oy - (e.clientY - rect.top)) / scale;
    onTargetClick(mmX, mmY);
  };

  // Phi handle drag: compute angle from target to mouse
  const handlePhiMouseDown = (e) => {
    e.stopPropagation();
    isDraggingPhi.current = true;
    const svg = e.currentTarget.closest('svg');
    const rect = svg.getBoundingClientRect();

    const onMove = (ev) => {
      const mx = ev.clientX - rect.left;
      const my = ev.clientY - rect.top;
      const dx = (mx - ox) / scale - target.x;      // mm from target
      const dy = (oy - my) / scale - target.y;
      const newPhi = Math.atan2(dy, dx);
      if (onPhiChange) onPhiChange(newPhi);
    };

    const onUp = () => {
      window.removeEventListener('mousemove', onMove);
      window.removeEventListener('mouseup', onUp);
    };

    window.addEventListener('mousemove', onMove);
    window.addEventListener('mouseup', onUp);
  };

  const gridCols = Math.floor(width / 24) + 2;
  const gridRows = Math.floor(height / 24) + 2;

  return (
    <svg
      width={width} height={height}
      viewBox={`0 0 ${width} ${height}`}
      style={{ display: 'block', cursor: 'crosshair' }}
      onClick={handleClick}
    >
      <defs>
        <radialGradient id="eeGlow3" cx="50%" cy="50%" r="50%">
          <stop offset="0%"   stopColor="rgba(0,212,200,0.7)" />
          <stop offset="60%"  stopColor="rgba(0,212,200,0.18)" />
          <stop offset="100%" stopColor="rgba(0,212,200,0)" />
        </radialGradient>
      </defs>

      {/* CAD grid */}
      <g opacity="0.5">
        {Array.from({ length: gridCols }).map((_, i) => (
          <line key={`vg${i}`} x1={i*24} y1={0} x2={i*24} y2={height}
            stroke="rgba(255,255,255,0.025)" strokeWidth="1" />
        ))}
        {Array.from({ length: gridRows }).map((_, i) => (
          <line key={`hg${i}`} x1={0} y1={i*24} x2={width} y2={i*24}
            stroke="rgba(255,255,255,0.025)" strokeWidth="1" />
        ))}
      </g>

      {/* Axes */}
      <line x1={20} y1={oy} x2={width-20} y2={oy}
        stroke="rgba(255,255,255,0.12)" strokeWidth="1" strokeDasharray="4 4"/>
      <line x1={ox} y1={20} x2={ox} y2={height-20}
        stroke="rgba(255,255,255,0.12)" strokeWidth="1" strokeDasharray="4 4"/>

      {/* Workspace ring */}
      <circle cx={ox} cy={oy} r={outerR}
        fill="rgba(0,212,200,0.025)" stroke="#00a89e" strokeWidth="1" strokeDasharray="6 4"/>
      {innerR > 2 && (
        <circle cx={ox} cy={oy} r={innerR}
          fill="#080c10" stroke="rgba(0,212,200,0.3)" strokeWidth="1" strokeDasharray="4 4"/>
      )}
      <text x={ox + outerR * 0.707 + 6} y={oy - outerR * 0.707 - 4}
        fontFamily="JetBrains Mono" fontSize="9" fill="#00a89e" letterSpacing="0.06em">
        WORKSPACE · R={L1+L2+L3}mm
      </text>

      {/* Distance line tip → target */}
      <line x1={sx(tip.x)} y1={sy(tip.y)} x2={sx(target.x)} y2={sy(target.y)}
        stroke="rgba(245,158,11,0.4)" strokeWidth="1" strokeDasharray="4 4"/>

      {/* Links */}
      <line x1={sx(0)} y1={sy(0)} x2={sx(j1.x)} y2={sy(j1.y)}
        stroke="#00d4c8" strokeWidth="8" strokeLinecap="round"/>
      <line x1={sx(j1.x)} y1={sy(j1.y)} x2={sx(j2.x)} y2={sy(j2.y)}
        stroke="#38bdf8" strokeWidth="7" strokeLinecap="round"/>
      <line x1={sx(j2.x)} y1={sy(j2.y)} x2={sx(tip.x)} y2={sy(tip.y)}
        stroke="#a78bfa" strokeWidth="6" strokeLinecap="round"/>

      {/* Joints */}
      <circle cx={sx(0)} cy={sy(0)} r={10} fill="#0d1117" stroke="#30363d" strokeWidth="2"/>
      <circle cx={sx(0)} cy={sy(0)} r={2.5} fill="#30363d"/>
      <circle cx={sx(j1.x)} cy={sy(j1.y)} r={8} fill="#161b22" stroke="#00a89e" strokeWidth="1.5"/>
      <circle cx={sx(j2.x)} cy={sy(j2.y)} r={8} fill="#161b22" stroke="#00a89e" strokeWidth="1.5"/>

      {/* End-effector glow */}
      <circle cx={sx(tip.x)} cy={sy(tip.y)} r={22} fill="url(#eeGlow3)"/>
      <circle cx={sx(tip.x)} cy={sy(tip.y)} r={10} fill="white" stroke="#00d4c8" strokeWidth="2"/>
      <circle cx={sx(tip.x)} cy={sy(tip.y)} r={4} fill="#00d4c8"/>

      {/* Orientation arrow (EE pointing direction) */}
      <line
        x1={sx(tip.x)} y1={sy(tip.y)}
        x2={sx(tip.x) + Math.cos(t1+t2+t3)*28}
        y2={sy(tip.y) - Math.sin(t1+t2+t3)*28}
        stroke="#a78bfa" strokeWidth="1.5"/>

      {/* EE callout */}
      <g transform={`translate(${sx(tip.x) + 16}, ${sy(tip.y) - 38})`}>
        <rect x={0} y={0} width={132} height={48} rx={3} fill="#1c2330" stroke="#30363d"/>
        <text x={8} y={14} fontFamily="Space Grotesk" fontSize="8.5" fill="#8b949e" letterSpacing="0.08em">END EFFECTOR</text>
        <text x={8} y={28} fontFamily="JetBrains Mono" fontSize="10.5" fill="#00d4c8">
          {tip.x.toFixed(1)}, {tip.y.toFixed(1)} <tspan fill="#484f58">mm</tspan>
        </text>
        <text x={8} y={42} fontFamily="JetBrains Mono" fontSize="10" fill="#a78bfa">
          φ {(target.phi * 180 / Math.PI).toFixed(1)}<tspan fill="#484f58">°</tspan>
        </text>
      </g>

      {/* Target reticle */}
      <ScopeReticle cx={sx(target.x)} cy={sy(target.y)} color={reachable ? '#f59e0b' : '#ef4444'} size={28}/>

      {/* Phi orientation handle (draggable) */}
      <line
        x1={sx(target.x)} y1={sy(target.y)}
        x2={phiHandle.px} y2={phiHandle.py}
        stroke="#f59e0b" strokeWidth="1.5" strokeDasharray="2 3"/>
      <circle
        cx={phiHandle.px} cy={phiHandle.py} r={5}
        fill="#080c10" stroke="#f59e0b" strokeWidth="1.5"
        style={{ cursor: 'grab' }}
        onMouseDown={handlePhiMouseDown}
        onClick={e => e.stopPropagation()}
      />

      <text x={sx(target.x) - 64} y={sy(target.y) + 26}
        fontFamily="JetBrains Mono" fontSize="9" fill={reachable ? '#f59e0b' : '#ef4444'}>
        T ({target.x.toFixed(1)}, {target.y.toFixed(1)})
      </text>
    </svg>
  );
}
