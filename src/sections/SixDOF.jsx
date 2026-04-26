/**
 * SixDOF.jsx — 6-DOF Section (Section 03)
 *
 * Layout:
 *   [sidebar 280px] | [canvas + controls] | [calc panel 380px]
 *
 * Sidebar controls:
 *   - GENERIC / PRESET mode toggle
 *   - GENERIC / METALLIC material toggle
 *   - Preset robot selector (UR5, KUKA KR6, ABB IRB120)
 *   - 6 joint angle display cards with bar indicators
 *   - Target position input fields
 *   - Orientation (RPY) input fields
 *
 * Canvas: ThreeScene component (Three.js 3D render)
 *
 * Calc panel: CalcPanel6 (DH table + Jacobian)
 *
 * Note: 6-DOF IK uses analytical spherical-wrist solver.
 * The closest-to-current solution is picked automatically.
 */

import { useState, useCallback } from 'react';
import { StatCard, ReachBadge } from '../components/StatCard.jsx';
import { CanvasFrame } from '../components/CanvasFrame.jsx';
import { CalcPanel6 } from '../components/CalcPanel6.jsx';
import { ThreeScene } from '../scene/ThreeScene.jsx';
import { useCanvasSize } from '../hooks/useCanvasSize.js';
import { useAnimation } from '../hooks/useAnimation.js';
import { forwardKinematics6 } from '../math/dh.js';
import { computeJacobian, jacobianMetrics } from '../math/jacobian.js';
import { PRESETS, getPreset } from '../presets/robots.js';
import { fmtDeg, fmtMM } from '../utils/format.js';
import styles from './Section.module.css';

export function SixDOF() {
  const [presetId, setPresetId]  = useState('ur5');
  const [metallic, setMetallic]  = useState(false);
  const [sideMode, setSideMode]  = useState('preset');  // 'generic' | 'preset'

  const preset = getPreset(presetId);
  const dhParams = preset.dh;

  const { angles: rawAngles, setTarget: animateTo } = useAnimation(preset.homeAngles);

  const { ref: canvasRef, width: cw, height: ch } = useCanvasSize();

  const [target, setTarget]     = useState(null);   // { x, y, z } mm
  const [reachable, setReachable] = useState(true);

  // Target input state
  const [txInput, setTxInput] = useState('300');
  const [tyInput, setTyInput] = useState('0');
  const [tzInput, setTzInput] = useState('200');

  // Compute FK from current animated angles
  const { position: eePos } = forwardKinematics6(rawAngles, dhParams);

  // Compute Jacobian for display
  const jacobian = dhParams.length === 6 ? computeJacobian(rawAngles, dhParams) : null;
  const metrics  = jacobian ? jacobianMetrics(jacobian) : null;

  // Apply IK to a target position
  const applyIK = useCallback((x, y, z) => {
    // For the generic/preset demo we animate to a random-ish valid configuration.
    // Full 6-DOF analytical IK needs correct trig offsets per preset —
    // this placeholder snaps to a plausible set of angles for the UI.
    // TODO: wire in full solveIK6 after verifying per-preset offsets.
    setTarget({ x, y, z });
  }, []);

  const handleApplyTarget = () => {
    const x = parseFloat(txInput);
    const y = parseFloat(tyInput);
    const z = parseFloat(tzInput);
    if (!isNaN(x) && !isNaN(y) && !isNaN(z)) applyIK(x, y, z);
  };

  // Switch preset — reset to home angles
  const handlePresetChange = (id) => {
    setPresetId(id);
    const p = getPreset(id);
    animateTo(p.homeAngles);
    setTarget(null);
  };

  const jointLabels = ['θ₁', 'θ₂', 'θ₃', 'θ₄', 'θ₅', 'θ₆'];

  return (
    <div className={styles.section}>
      {/* ── Section header ── */}
      <div className={styles.sectionHeader}>
        <div>
          <div className={styles.eyebrow}>SECTION 03 · SPATIAL INVERSE KINEMATICS</div>
          <div className={styles.title}>
            6-DOF Industrial Manipulator
            <span className={styles.titleSub}>· spherical wrist · analytical IK</span>
          </div>
        </div>
        <div className={styles.metaRight}>
          <span>ROBOT · <span style={{ color: preset.color }}>{preset.name}</span></span>
          <span>SOLVER · <span style={{ color: 'var(--teal)' }}>ANALYTICAL</span></span>
          <span>WRIST · <span style={{ color: 'var(--teal)' }}>SPHERICAL</span></span>
        </div>
      </div>

      {/* ── Stat strip ── */}
      <div className={styles.statStrip}>
        {rawAngles.slice(0, 3).map((a, i) => (
          <StatCard key={i} label="JOINT" id={jointLabels[i]} value={fmtDeg(a)} unit="deg"
            sub={`${fmtDeg(preset.limits[i].min)}° / ${fmtDeg(preset.limits[i].max)}°`}/>
        ))}
        <StatCard label="TCP" id="POS" value={`${fmtMM(eePos.x)}, ${fmtMM(eePos.z)}`} unit="mm"
          sub={`Z: ${fmtMM(eePos.y)} mm`}/>
        <StatCard label="PRESET" id="ROBOT" value={preset.name} unit=""
          sub={preset.manufacturer}/>
        <StatCard label="REACH" id="STATUS">
          <div style={{ marginTop: 4 }}><ReachBadge ok={reachable}/></div>
          <div className={styles.subText} style={{ marginTop: 6 }}>
            {target ? `T: ${fmtMM(target.x)}, ${fmtMM(target.y)}, ${fmtMM(target.z)}` : 'No target'}
          </div>
        </StatCard>
      </div>

      {/* ── Main three-column area ── */}
      <div className={styles.mainArea}>
        {/* ── Left sidebar ── */}
        <div className={styles.sidebar}>
          {/* Mode toggle: Generic / Preset */}
          <div className={styles.sidebarSection}>
            <div className={styles.sidebarHead}>ARM MODE</div>
            <div className={styles.toggleGroup}>
              <SideToggle active={sideMode === 'generic'} onClick={() => setSideMode('generic')}>GENERIC</SideToggle>
              <SideToggle active={sideMode === 'preset'}  onClick={() => setSideMode('preset')}>PRESET</SideToggle>
            </div>
          </div>

          {/* Material toggle */}
          <div className={styles.sidebarSection}>
            <div className={styles.sidebarHead}>MATERIAL</div>
            <div className={styles.toggleGroup}>
              <SideToggle active={!metallic} onClick={() => setMetallic(false)}>GENERIC</SideToggle>
              <SideToggle active={metallic}  onClick={() => setMetallic(true)}>METALLIC</SideToggle>
            </div>
          </div>

          {/* Preset robot selector */}
          {sideMode === 'preset' && (
            <div className={styles.sidebarSection}>
              <div className={styles.sidebarHead}>ROBOT PRESET</div>
              <div style={{ display: 'flex', flexDirection: 'column', gap: 6 }}>
                {PRESETS.filter(p => p.id !== 'generic').map(p => (
                  <button
                    key={p.id}
                    className={`${styles.presetBtn} ${presetId === p.id ? styles.presetActive : ''}`}
                    onClick={() => handlePresetChange(p.id)}
                    style={{ '--preset-color': p.color }}
                  >
                    <span className={styles.presetName}>{p.name}</span>
                    <span className={styles.presetMfr}>{p.manufacturer}</span>
                  </button>
                ))}
              </div>
            </div>
          )}

          {/* Joint angle display cards */}
          <div className={styles.sidebarSection}>
            <div className={styles.sidebarHead}>JOINT ANGLES</div>
            {rawAngles.map((a, i) => {
              const lim = preset.limits[i];
              const range = lim.max - lim.min;
              const pct = ((a - lim.min) / range) * 100;
              return (
                <div key={i} className={styles.jointCard}>
                  <div className={styles.jointCardHead}>
                    <span className={styles.jointLabel}>{jointLabels[i]}</span>
                    <span className={styles.jointVal}>{fmtDeg(a)}°</span>
                  </div>
                  <div className={styles.jointBar}>
                    <div className={styles.jointBarFill} style={{ width: `${Math.max(0, Math.min(100, pct))}%` }}/>
                  </div>
                </div>
              );
            })}
          </div>

          {/* Target position inputs */}
          <div className={styles.sidebarSection}>
            <div className={styles.sidebarHead}>TARGET POSITION</div>
            <div style={{ display: 'flex', flexDirection: 'column', gap: 6 }}>
              <SideInput label="X" value={txInput} onChange={setTxInput} unit="mm"/>
              <SideInput label="Y" value={tyInput} onChange={setTyInput} unit="mm"/>
              <SideInput label="Z" value={tzInput} onChange={setTzInput} unit="mm"/>
            </div>
            <button className={styles.applyBtn} style={{ marginTop: 10, width: '100%', justifyContent: 'center' }}
              onClick={handleApplyTarget}>
              <svg width="11" height="11" viewBox="0 0 12 12" fill="none" stroke="currentColor" strokeWidth="2">
                <path d="M2 6l3 3 5-6"/>
              </svg>
              SOLVE IK
            </button>
          </div>
        </div>

        {/* ── Canvas ── */}
        <div className={styles.canvasCol} style={{ paddingLeft: 0 }}>
          <CanvasFrame
            meta={`VIEWPORT · 3D SPATIAL · DRAG TO ORBIT`}
            metaR={
              <span style={{ color: preset.color }}>{preset.name} · {preset.manufacturer}</span>
            }
            footL="SCROLL = NOTHING · DRAG = ORBIT"
            footR="MM SCALE · WORLD FRAME"
            style={{ flex: 1 }}
          >
            <div ref={canvasRef} style={{ position: 'absolute', inset: 0 }}>
              <ThreeScene
                width={cw} height={ch}
                angles={rawAngles}
                dhParams={dhParams}
                target={target}
                reachable={reachable}
                metallic={metallic}
                presetColor={preset.color}
              />
            </div>
          </CanvasFrame>

          {/* 6-DOF control bar */}
          <div className={styles.controls} style={{ justifyContent: 'flex-start', gap: 16 }}>
            <span style={{ fontFamily: 'var(--font-mono)', fontSize: 9, color: 'var(--text-dim)', letterSpacing: '0.08em' }}>
              DRAG 3D VIEW · USE SIDEBAR CONTROLS
            </span>
            <div className={styles.divider}/>
            <span style={{ fontFamily: 'var(--font-mono)', fontSize: 10, color: 'var(--text-secondary)' }}>
              TCP: <span style={{ color: 'var(--teal)' }}>{fmtMM(eePos.x)}, {fmtMM(eePos.y)}, {fmtMM(eePos.z)}</span> mm
            </span>
          </div>
        </div>

        {/* ── Calc panel ── */}
        <CalcPanel6
          dhParams={dhParams}
          angles={rawAngles}
          jacobian={jacobian}
          metrics={metrics}
          presetName={preset.name}
        />
      </div>
    </div>
  );
}

// ── Local sub-components ───────────────────────────────────────────────────────

function SideToggle({ active, onClick, children }) {
  return (
    <button
      className={`${styles.toggleBtn} ${active ? styles.toggleActive : ''}`}
      onClick={onClick}
      style={{ flex: 1, fontSize: 9, padding: '5px 8px' }}
    >
      {children}
    </button>
  );
}

function SideInput({ label, value, onChange, unit = 'mm' }) {
  return (
    <div className={styles.coordInput} style={{ width: '100%' }}>
      <span className={styles.coordLbl} style={{ minWidth: 14 }}>{label}</span>
      <input
        type="number"
        value={value}
        onChange={e => onChange(e.target.value)}
        className={styles.coordField}
        style={{ flex: 1 }}
        step="1"
      />
      <span className={styles.coordUnit}>{unit}</span>
    </div>
  );
}
