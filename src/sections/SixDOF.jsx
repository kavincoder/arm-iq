/**
 * SixDOF.jsx — 6-DOF Section (Section 03)
 *
 * Layout:
 *   [sidebar 280px] | [canvas + controls] | [calc panel 380px]
 *
 * How motion works (same pattern as 2-DOF / 3-DOF):
 *   • Joint sliders → drag to pose any joint → animateTo() → lerp animation
 *   • SOLVE IK button → iterative Jacobian-transpose IK → animateTo()
 *   • Preset switch → animateTo(homeAngles) → smooth reset to home
 *
 * IK solver: Jacobian-transpose gradient descent (position-only).
 * Universal — works for any DH parameterisation without robot-specific
 * geometric derivations. Runs in < 5ms for 100 iterations.
 */

import { useState, useCallback, useRef } from 'react';
import { StatCard, ReachBadge } from '../components/StatCard.jsx';
import { CanvasFrame } from '../components/CanvasFrame.jsx';
import { CalcPanel6 } from '../components/CalcPanel6.jsx';
import { ThreeScene } from '../scene/ThreeScene.jsx';
import { useCanvasSize } from '../hooks/useCanvasSize.js';
import { useAnimation } from '../hooks/useAnimation.js';
import { forwardKinematics6 } from '../math/dh.js';
import { computeJacobian, jacobianMetrics } from '../math/jacobian.js';
import { normalizeAngle } from '../math/ik6.js';
import { PRESETS, getPreset } from '../presets/robots.js';
import { fmtDeg, fmtMM, toRad } from '../utils/format.js';
import styles from './Section.module.css';

const JOINT_LABELS = ['θ₁', 'θ₂', 'θ₃', 'θ₄', 'θ₅', 'θ₆'];

// ── Jacobian-transpose iterative IK (position only) ───────────────────────────
// Finds joint angles so TCP reaches targetPos within tolerance.
// Stops after maxIter or when error < eps mm.
// Works for any 6-DOF robot — no robot-specific geometric derivation needed.
function solveIKIterative(targetPos, initialAngles, dhParams, {
  maxIter = 120,
  stepSize = 0.8,
  eps = 0.5,
} = {}) {
  let q = [...initialAngles];

  for (let iter = 0; iter < maxIter; iter++) {
    const { position: p } = forwardKinematics6(q, dhParams);
    const ex = targetPos.x - p.x;
    const ey = targetPos.y - p.y;
    const ez = targetPos.z - p.z;
    const errMag = Math.sqrt(ex*ex + ey*ey + ez*ez);
    if (errMag < eps) break;

    // Compute 3×6 position Jacobian via finite differences
    const h = 1e-4;  // radians
    const dq = new Array(6).fill(0);

    for (let i = 0; i < 6; i++) {
      const qp = [...q]; qp[i] += h;
      const pp = forwardKinematics6(qp, dhParams).position;
      // Jᵀ * err update: dqᵢ += (∂p/∂qᵢ) · err
      dq[i] = ((pp.x - p.x) * ex + (pp.y - p.y) * ey + (pp.z - p.z) * ez) / h;
    }

    // Adaptive step: scale down if errMag is already small
    const scale = stepSize * Math.min(1, errMag / 50);
    for (let i = 0; i < 6; i++) {
      q[i] += scale * dq[i];
      q[i] = normalizeAngle(q[i]);
    }
  }

  return q;
}

// Clamp angles to joint limits
function clampToLimits(angles, limits) {
  return angles.map((a, i) => Math.max(limits[i].min, Math.min(limits[i].max, a)));
}

// ── Component ──────────────────────────────────────────────────────────────────
export function SixDOF() {
  const [presetId, setPresetId] = useState('ur5');
  const [metallic, setMetallic] = useState(false);
  const [sideMode, setSideMode] = useState('preset');
  const [reachable, setReachable] = useState(true);
  const [target3D, setTarget3D] = useState(null);   // { x, y, z } mm for marker
  const [solveStatus, setSolveStatus] = useState('idle');  // 'idle' | 'ok' | 'fail'

  const preset   = getPreset(presetId);
  const dhParams = preset.dh;

  const { angles: rawAngles, setTarget: animateTo } = useAnimation([...preset.homeAngles]);
  const { ref: canvasRef, width: cw, height: ch } = useCanvasSize();

  // Target inputs
  const [txInput, setTxInput] = useState('300');
  const [tyInput, setTyInput] = useState('0');
  const [tzInput, setTzInput] = useState('400');

  // FK from current animated pose
  const { position: eePos } = forwardKinematics6(rawAngles, dhParams);

  // Jacobian for calc panel display
  const jacobian = dhParams.length === 6 ? computeJacobian(rawAngles, dhParams) : null;
  const metrics  = jacobian ? jacobianMetrics(jacobian) : null;

  // ── Joint slider handler ─────────────────────────────────────────────────────
  // Called when user drags one of the 6 joint sliders.
  // Immediately calls animateTo() so the arm lerps to the new configuration.
  const handleJointSlider = useCallback((jointIdx, deg) => {
    const rad = toRad(deg);
    const clamped = Math.max(
      preset.limits[jointIdx].min,
      Math.min(preset.limits[jointIdx].max, rad)
    );
    // Build target angles: keep all others, update this joint
    const newAngles = [...rawAngles];
    newAngles[jointIdx] = clamped;
    animateTo(newAngles);
  }, [rawAngles, preset.limits, animateTo]);

  // ── SOLVE IK ────────────────────────────────────────────────────────────────
  // Runs iterative Jacobian-transpose IK to find joint angles that reach (x,y,z).
  const handleSolveIK = useCallback(() => {
    const x = parseFloat(txInput);
    const y = parseFloat(tyInput);
    const z = parseFloat(tzInput);
    if (isNaN(x) || isNaN(y) || isNaN(z)) return;

    const targetPos = { x, y, z };

    // Solve IK starting from current pose (warm start = faster convergence)
    const solved = solveIKIterative(targetPos, rawAngles, dhParams);
    const clamped = clampToLimits(solved, preset.limits);

    // Verify solution quality (FK round-trip check)
    const { position: fkPos } = forwardKinematics6(clamped, dhParams);
    const err = Math.hypot(fkPos.x - x, fkPos.y - y, fkPos.z - z);
    const ok  = err < 5.0;  // within 5mm = acceptable

    setReachable(ok);
    setTarget3D(targetPos);
    setSolveStatus(ok ? 'ok' : 'fail');
    animateTo(clamped);  // ← THIS is what makes the arm move, just like 2-DOF
  }, [txInput, tyInput, tzInput, rawAngles, dhParams, preset.limits, animateTo]);

  // ── Preset switch ────────────────────────────────────────────────────────────
  const handlePresetChange = useCallback((id) => {
    setPresetId(id);
    const p = getPreset(id);
    animateTo([...p.homeAngles]);  // smooth lerp back to home position
    setTarget3D(null);
    setSolveStatus('idle');
    setReachable(true);
  }, [animateTo]);

  return (
    <div className={styles.section}>
      {/* ── Section header ── */}
      <div className={styles.sectionHeader}>
        <div>
          <div className={styles.eyebrow}>SECTION 03 · SPATIAL INVERSE KINEMATICS</div>
          <div className={styles.title}>
            6-DOF Industrial Manipulator
            <span className={styles.titleSub}>· spherical wrist · Jacobian IK</span>
          </div>
        </div>
        <div className={styles.metaRight}>
          <span>ROBOT · <span style={{ color: preset.color }}>{preset.name}</span></span>
          <span>SOLVER · <span style={{ color: 'var(--teal)' }}>JAC-TRANSPOSE</span></span>
          <span>TCP · <span style={{ color: 'var(--teal)' }}>
            {fmtMM(eePos.x)}, {fmtMM(eePos.y)}, {fmtMM(eePos.z)}
          </span> mm</span>
        </div>
      </div>

      {/* ── Stat strip ── */}
      <div className={styles.statStrip}>
        {rawAngles.slice(0, 3).map((a, i) => (
          <StatCard key={i} label="JOINT" id={JOINT_LABELS[i]}
            value={fmtDeg(a)} unit="deg"
            sub={`${fmtDeg(preset.limits[i].min)}° / ${fmtDeg(preset.limits[i].max)}°`}/>
        ))}
        <StatCard label="TCP" id="X,Y,Z"
          value={`${fmtMM(eePos.x)}, ${fmtMM(eePos.y)}`} unit="mm"
          sub={`Z: ${fmtMM(eePos.z)} mm`}/>
        <StatCard label="PRESET" id="ROBOT" value={preset.name} sub={preset.manufacturer}/>
        <StatCard label="IK" id="STATUS">
          <div style={{ marginTop: 4 }}>
            <ReachBadge ok={reachable}
              label={solveStatus === 'idle' ? 'READY' : reachable ? 'SOLVED' : 'UNREACHABLE'}/>
          </div>
          <div className={styles.subText} style={{ marginTop: 6 }}>
            {target3D
              ? `T: ${fmtMM(target3D.x)}, ${fmtMM(target3D.y)}, ${fmtMM(target3D.z)}`
              : 'Set target → SOLVE IK'}
          </div>
        </StatCard>
      </div>

      {/* ── Main three-column area ── */}
      <div className={styles.mainArea}>

        {/* ── Left sidebar ── */}
        <div className={styles.sidebar}>

          {/* Mode toggle */}
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

          {/* Robot preset selector */}
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

          {/* ── Interactive joint sliders ─────────────────────────────────────
               Each slider calls animateTo() on change, giving smooth animation
               exactly like clicking a target in the 2-DOF canvas.           */}
          <div className={styles.sidebarSection}>
            <div className={styles.sidebarHead}>JOINT ANGLES · DRAG TO MOVE</div>
            {rawAngles.map((a, i) => {
              const lim   = preset.limits[i];
              const limDegMin = lim.min * 180 / Math.PI;
              const limDegMax = lim.max * 180 / Math.PI;
              const aDeg  = a * 180 / Math.PI;
              const pct   = ((aDeg - limDegMin) / (limDegMax - limDegMin)) * 100;

              return (
                <div key={i} className={styles.jointCard}>
                  <div className={styles.jointCardHead}>
                    <span className={styles.jointLabel}>{JOINT_LABELS[i]}</span>
                    <span className={styles.jointVal}>{fmtDeg(a)}°</span>
                  </div>
                  {/* Progress bar doubles as a range input track */}
                  <div className={styles.jointBarWrap}>
                    <div className={styles.jointBar}>
                      <div
                        className={styles.jointBarFill}
                        style={{ width: `${Math.max(0, Math.min(100, pct))}%` }}
                      />
                    </div>
                    {/* Invisible range input overlaid on bar for interaction */}
                    <input
                      type="range"
                      min={limDegMin}
                      max={limDegMax}
                      step="0.5"
                      value={aDeg.toFixed(1)}
                      onChange={e => handleJointSlider(i, parseFloat(e.target.value))}
                      className={styles.jointSliderInput}
                      title={`${JOINT_LABELS[i]}: ${fmtDeg(a)}° (drag to move arm)`}
                    />
                  </div>
                </div>
              );
            })}
          </div>

          {/* Target position IK */}
          <div className={styles.sidebarSection}>
            <div className={styles.sidebarHead}>TARGET POSITION · IK SOLVE</div>
            <div style={{ display: 'flex', flexDirection: 'column', gap: 6 }}>
              <SideInput label="X" value={txInput} onChange={setTxInput} unit="mm"/>
              <SideInput label="Y" value={tyInput} onChange={setTyInput} unit="mm"/>
              <SideInput label="Z" value={tzInput} onChange={setTzInput} unit="mm"/>
            </div>
            <button
              className={styles.applyBtn}
              style={{ marginTop: 10, width: '100%', justifyContent: 'center' }}
              onClick={handleSolveIK}
            >
              <svg width="11" height="11" viewBox="0 0 12 12" fill="none" stroke="currentColor" strokeWidth="2">
                <path d="M2 6l3 3 5-6"/>
              </svg>
              SOLVE IK
            </button>
            <div style={{
              marginTop: 6, fontFamily: 'var(--font-mono)', fontSize: 9,
              color: 'var(--text-dim)', textAlign: 'center'
            }}>
              Or drag the joint sliders above ↑
            </div>
          </div>
        </div>

        {/* ── 3D Canvas ── */}
        <div className={styles.canvasCol} style={{ paddingLeft: 0 }}>
          <CanvasFrame
            meta="VIEWPORT · 3D SPATIAL · DRAG TO ORBIT"
            metaR={<span style={{ color: preset.color }}>{preset.name} · {preset.manufacturer}</span>}
            footL="DRAG = ORBIT CAMERA"
            footR="MM SCALE · WORLD FRAME"
            style={{ flex: 1 }}
          >
            <div ref={canvasRef} style={{ position: 'absolute', inset: 0 }}>
              <ThreeScene
                width={cw} height={ch}
                angles={rawAngles}
                dhParams={dhParams}
                target={target3D}
                reachable={reachable}
                metallic={metallic}
                presetColor={preset.color}
              />
            </div>
          </CanvasFrame>

          {/* Control bar */}
          <div className={styles.controls} style={{ justifyContent: 'flex-start', gap: 16 }}>
            <span style={{ fontFamily: 'var(--font-mono)', fontSize: 9, color: 'var(--text-dim)', letterSpacing: '0.08em' }}>
              JOINT SLIDERS IN SIDEBAR · OR TYPE TARGET + SOLVE IK
            </span>
            <div className={styles.divider}/>
            <span style={{ fontFamily: 'var(--font-mono)', fontSize: 10, color: 'var(--text-secondary)' }}>
              TCP: <span style={{ color: 'var(--teal)' }}>
                {fmtMM(eePos.x)}, {fmtMM(eePos.y)}, {fmtMM(eePos.z)}
              </span> mm
            </span>
            {/* Home button */}
            <button
              className={styles.toggleBtn}
              style={{ marginLeft: 'auto', marginRight: 8 }}
              onClick={() => { animateTo([...preset.homeAngles]); setTarget3D(null); setSolveStatus('idle'); }}
            >
              ↺ HOME
            </button>
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
