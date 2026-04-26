/**
 * TwoDOF.jsx — 2-DOF Section (Section 01)
 *
 * Layout (left-to-right):
 *   [canvas area (flex 1)] [calc panel (380px)]
 *
 * Canvas area layout (top-to-bottom):
 *   [stat strip] → [canvas frame] → [controls bar]
 *
 * State:
 *   L1, L2     — link lengths, controllable via sliders
 *   target     — { x, y } in mm, set by clicking canvas
 *   mode       — 'up' | 'down' (elbow configuration)
 *   animTarget — { t1, t2 } angles we're lerping toward
 *   angles     — live animated angles from useAnimation
 *
 * IK is solved instantly on each click. Animation happens via useAnimation.
 */

import { useState, useCallback } from 'react';
import { StatCard, ReachBadge } from '../components/StatCard.jsx';
import { CanvasFrame } from '../components/CanvasFrame.jsx';
import { CalcPanel2 } from '../components/CalcPanel2.jsx';
import { TwoDofCanvas } from '../canvas/TwoDofCanvas.jsx';
import { useCanvasSize } from '../hooks/useCanvasSize.js';
import { useAnimation } from '../hooks/useAnimation.js';
import { solveIK2 } from '../math/ik2.js';
import { fmtDeg, fmtMM } from '../utils/format.js';
import styles from './Section.module.css';

// Default configuration
const DEFAULT_L1 = 100;
const DEFAULT_L2 = 80;
const DEFAULT_TARGET = { x: 110, y: 60 };

export function TwoDOF() {
  const [L1, setL1] = useState(DEFAULT_L1);
  const [L2, setL2] = useState(DEFAULT_L2);
  const [target, setTarget] = useState(DEFAULT_TARGET);
  const [reachable, setReachable] = useState(true);
  const [mode, setMode] = useState('up');  // 'up' or 'down' (elbow config)

  // Input field states (string for controlled inputs)
  const [txInput, setTxInput] = useState(String(DEFAULT_TARGET.x));
  const [tyInput, setTyInput] = useState(String(DEFAULT_TARGET.y));

  // Initial angles from default target
  const initSols = solveIK2(DEFAULT_TARGET.x, DEFAULT_TARGET.y, DEFAULT_L1, DEFAULT_L2);
  const initAngles = initSols ? [initSols[0].t1, initSols[0].t2] : [0, 0];
  const { angles: rawAngles, setTarget: animateTo } = useAnimation(initAngles);
  const animAngles = { t1: rawAngles[0], t2: rawAngles[1] };

  const { ref: canvasRef, width: cw, height: ch } = useCanvasSize();

  // Solve IK and trigger animation when target changes
  const applyTarget = useCallback((x, y) => {
    const sols = solveIK2(x, y, L1, L2);
    setTarget({ x, y });
    setTxInput(x.toFixed(1));
    setTyInput(y.toFixed(1));

    if (!sols) {
      setReachable(false);
      return;
    }
    setReachable(true);
    const sol = mode === 'up' ? sols[0] : sols[1];
    animateTo([sol.t1, sol.t2]);
  }, [L1, L2, mode, animateTo]);

  // Re-solve when mode (elbow up/down) is toggled
  const toggleMode = useCallback(() => {
    const newMode = mode === 'up' ? 'down' : 'up';
    setMode(newMode);
    const sols = solveIK2(target.x, target.y, L1, L2);
    if (sols) {
      const sol = newMode === 'up' ? sols[0] : sols[1];
      animateTo([sol.t1, sol.t2]);
    }
  }, [mode, target, L1, L2, animateTo]);

  // Apply typed coordinate input
  const handleApply = () => {
    const x = parseFloat(txInput);
    const y = parseFloat(tyInput);
    if (!isNaN(x) && !isNaN(y)) applyTarget(x, y);
  };

  // Re-solve when link lengths change
  const handleL1Change = (v) => {
    setL1(v);
    const sols = solveIK2(target.x, target.y, v, L2);
    if (sols) {
      const sol = mode === 'up' ? sols[0] : sols[1];
      animateTo([sol.t1, sol.t2]);
      setReachable(true);
    } else {
      setReachable(false);
    }
  };

  const handleL2Change = (v) => {
    setL2(v);
    const sols = solveIK2(target.x, target.y, L1, v);
    if (sols) {
      const sol = mode === 'up' ? sols[0] : sols[1];
      animateTo([sol.t1, sol.t2]);
      setReachable(true);
    } else {
      setReachable(false);
    }
  };

  // Derived display values
  const r = Math.hypot(target.x, target.y);
  const rPct = Math.min(100, (r / (L1 + L2)) * 100);

  return (
    <div className={styles.section}>
      {/* ── Section header ── */}
      <div className={styles.sectionHeader}>
        <div>
          <div className={styles.eyebrow}>SECTION 01 · PLANAR INVERSE KINEMATICS</div>
          <div className={styles.title}>
            2-DOF Two-Link Manipulator
            <span className={styles.titleSub}>· law of cosines</span>
          </div>
        </div>
        <div className={styles.metaRight}>
          <span>SOLVER · <span style={{ color: 'var(--teal)' }}>COSINES</span></span>
          <span>ITER · <span style={{ color: 'var(--teal)' }}>1</span></span>
          <span>RESID · <span style={{ color: 'var(--teal)' }}>{'< 0.001 mm'}</span></span>
        </div>
      </div>

      {/* ── Stat strip ── */}
      <div className={styles.statStrip}>
        <StatCard label="JOINT" id="θ₁" value={fmtDeg(animAngles.t1)} unit="deg"
          sub={`Range: ±180°`} />
        <StatCard label="JOINT" id="θ₂" value={fmtDeg(animAngles.t2)} unit="deg"
          sub={`Range: ±180°`} />
        <StatCard label="TARGET" id="(X, Y)" value={`${fmtMM(target.x)}, ${fmtMM(target.y)}`} unit="mm"
          sub={`r = ${fmtMM(r)} mm`} />
        <StatCard label="ELBOW" id="MODE" value={mode.toUpperCase()} unit=""
          sub="Toggle to flip config" />
        <StatCard label="WORKSPACE" id="REACH" progress={rPct}>
          <div style={{ marginTop: 4 }}>
            <ReachBadge ok={reachable} />
          </div>
          <div className={styles.subText} style={{ marginTop: 6 }}>
            r/R = {(rPct / 100).toFixed(3)}
          </div>
        </StatCard>
      </div>

      {/* ── Main area ── */}
      <div className={styles.mainArea}>
        {/* Left: canvas + controls */}
        <div className={styles.canvasCol}>
          <CanvasFrame
            meta={`VIEWPORT · 2D PLANAR · 1px = ${(1 / (Math.min(cw, ch) / ((L1 + L2) * 2.7))).toFixed(2)} mm`}
            metaR={
              <span className="amber">TARGET <b>{fmtMM(target.x)}, {fmtMM(target.y)}</b></span>
            }
            footL={`GRID · 30 mm`}
            footR="CLICK CANVAS TO SET TARGET"
            style={{ flex: 1 }}
          >
            <div ref={canvasRef} style={{ position: 'absolute', inset: 0, display: 'flex', alignItems: 'center', justifyContent: 'center' }}>
              <TwoDofCanvas
                width={cw} height={ch}
                L1={L1} L2={L2}
                angles={animAngles}
                target={target}
                reachable={reachable}
                onTargetClick={applyTarget}
              />
            </div>
          </CanvasFrame>

          {/* Controls bar */}
          <div className={styles.controls}>
            {/* L1 slider */}
            <ArmSlider label="L₁" value={L1} min={30} max={200} unit="mm"
              onChange={handleL1Change} />
            {/* L2 slider */}
            <ArmSlider label="L₂" value={L2} min={30} max={200} unit="mm"
              onChange={handleL2Change} />

            <div className={styles.divider} />

            {/* Coordinate inputs */}
            <CoordIn label="X" value={txInput} onChange={setTxInput} />
            <CoordIn label="Y" value={tyInput} onChange={setTyInput} />

            {/* Elbow toggle */}
            <button
              className={`${styles.toggleBtn} ${mode === 'up' ? styles.toggleActive : ''}`}
              onClick={toggleMode}
            >
              ELBOW {mode === 'up' ? '↑ UP' : '↓ DOWN'}
            </button>

            {/* Apply button */}
            <button className={styles.applyBtn} onClick={handleApply}>
              <svg width="11" height="11" viewBox="0 0 12 12" fill="none" stroke="currentColor" strokeWidth="2">
                <path d="M2 6l3 3 5-6" />
              </svg>
              APPLY
            </button>
          </div>
        </div>

        {/* Right: calculation panel */}
        <CalcPanel2
          L1={L1} L2={L2}
          target={target}
          angles={animAngles}
          reachable={reachable}
        />
      </div>
    </div>
  );
}

// ── Local sub-components ───────────────────────────────────────────────────────

function ArmSlider({ label, value, min, max, unit, onChange }) {
  const pct = ((value - min) / (max - min)) * 100;
  return (
    <div className={styles.slider}>
      <span className={styles.sliderLbl}>{label}</span>
      <div className={styles.sliderTrack}>
        <div className={styles.sliderFill} style={{ width: `${pct}%` }} />
        <div className={styles.sliderThumb} style={{ left: `${pct}%` }} />
        <input
          type="range" min={min} max={max} value={value}
          onChange={e => onChange(Number(e.target.value))}
          className={styles.sliderInput}
        />
      </div>
      <span className={styles.sliderVal}>{value}<span className={styles.sliderUnit}> {unit}</span></span>
    </div>
  );
}

function CoordIn({ label, value, onChange }) {
  return (
    <div className={styles.coordInput}>
      <span className={styles.coordLbl}>{label}</span>
      <input
        type="number"
        value={value}
        onChange={e => onChange(e.target.value)}
        className={styles.coordField}
        step="0.1"
      />
      <span className={styles.coordUnit}>mm</span>
    </div>
  );
}
