/**
 * ThreeDOF.jsx — 3-DOF Section (Section 02)
 *
 * Extends the 2-DOF section with a third link (L3) and end-effector
 * orientation control (φ). The user can:
 *   - Click canvas to move target (x, y)
 *   - Drag the φ handle on the target reticle to set orientation
 *   - Type coordinates and φ in the control bar
 *   - Adjust link lengths via sliders
 */

import { useState, useCallback } from 'react';
import { StatCard, ReachBadge } from '../components/StatCard.jsx';
import { CanvasFrame } from '../components/CanvasFrame.jsx';
import { CalcPanel3 } from '../components/CalcPanel3.jsx';
import { ThreeDofCanvas } from '../canvas/ThreeDofCanvas.jsx';
import { useCanvasSize } from '../hooks/useCanvasSize.js';
import { useAnimation } from '../hooks/useAnimation.js';
import { solveIK3 } from '../math/ik3dof.js';
import { fmtDeg, fmtMM, toRad } from '../utils/format.js';
import styles from './Section.module.css';

const DEFAULT = { L1: 90, L2: 75, L3: 55, x: 130, y: 70, phi: 0.436 }; // phi ≈ 25°

export function ThreeDOF() {
  const [L1, setL1] = useState(DEFAULT.L1);
  const [L2, setL2] = useState(DEFAULT.L2);
  const [L3, setL3] = useState(DEFAULT.L3);
  const [target, setTarget] = useState({ x: DEFAULT.x, y: DEFAULT.y, phi: DEFAULT.phi });
  const [reachable, setReachable] = useState(true);

  const initSols = solveIK3(DEFAULT.x, DEFAULT.y, DEFAULT.phi, DEFAULT.L1, DEFAULT.L2, DEFAULT.L3);
  const initAngles = initSols ? [initSols[0].t1, initSols[0].t2, initSols[0].t3] : [0, 0, 0];
  const { angles: rawAngles, setTarget: animateTo } = useAnimation(initAngles);
  const animAngles = { t1: rawAngles[0], t2: rawAngles[1], t3: rawAngles[2] };

  const { ref: canvasRef, width: cw, height: ch } = useCanvasSize();

  const applyTarget = useCallback((x, y, phi = target.phi) => {
    const sols = solveIK3(x, y, phi, L1, L2, L3);
    setTarget({ x, y, phi });
    if (!sols) { setReachable(false); return; }
    setReachable(true);
    animateTo([sols[0].t1, sols[0].t2, sols[0].t3]);
  }, [L1, L2, L3, target.phi, animateTo]);

  const handlePhiChange = useCallback((newPhi) => {
    applyTarget(target.x, target.y, newPhi);
  }, [applyTarget, target.x, target.y]);

  const [txInput, setTxInput] = useState(String(DEFAULT.x));
  const [tyInput, setTyInput] = useState(String(DEFAULT.y));
  const [phiInput, setPhiInput] = useState('25.0');

  const handleApply = () => {
    const x = parseFloat(txInput);
    const y = parseFloat(tyInput);
    const phi = toRad(parseFloat(phiInput));
    if (!isNaN(x) && !isNaN(y) && !isNaN(phi)) applyTarget(x, y, phi);
  };

  const r = Math.hypot(target.x, target.y);
  const rMax = L1 + L2 + L3;
  const rPct = Math.min(100, (r / rMax) * 100);

  return (
    <div className={styles.section}>
      <div className={styles.sectionHeader}>
        <div>
          <div className={styles.eyebrow}>SECTION 02 · PLANAR INVERSE KINEMATICS</div>
          <div className={styles.title}>
            3-DOF Three-Link Manipulator
            <span className={styles.titleSub}>· redundant orientation</span>
          </div>
        </div>
        <div className={styles.metaRight}>
          <span>SOLVER · <span style={{ color: 'var(--teal)' }}>GEOMETRIC</span></span>
          <span>ITER · <span style={{ color: 'var(--teal)' }}>1</span></span>
          <span>RESID · <span style={{ color: 'var(--teal)' }}>{'< 0.001 mm'}</span></span>
        </div>
      </div>

      <div className={styles.statStrip}>
        <StatCard label="JOINT" id="θ₁" value={fmtDeg(animAngles.t1)} unit="deg" sub="Range: ±180°"/>
        <StatCard label="JOINT" id="θ₂" value={fmtDeg(animAngles.t2)} unit="deg" sub="Range: ±150°"/>
        <StatCard label="JOINT" id="θ₃" value={fmtDeg(animAngles.t3)} unit="deg" sub="Range: ±180°"/>
        <StatCard label="POSITION" id="(X, Y)" value={`${fmtMM(target.x)}, ${fmtMM(target.y)}`} unit="mm"
          sub={`r = ${fmtMM(r)} mm`}/>
        <StatCard label="ORIENTATION" id="φ" value={fmtDeg(target.phi)} unit="deg" sub="EE pointing"/>
        <StatCard label="WORKSPACE" id="REACH" progress={rPct}>
          <div style={{ marginTop: 4 }}><ReachBadge ok={reachable} /></div>
          <div className={styles.subText} style={{ marginTop: 6 }}>r/Rmax = {(rPct / 100).toFixed(3)}</div>
        </StatCard>
      </div>

      <div className={styles.mainArea}>
        <div className={styles.canvasCol}>
          <CanvasFrame
            meta="VIEWPORT · 2D PLANAR · DRAG φ HANDLE"
            metaR={
              <>
                <span>CURSOR</span>
                <span className="amber">TARGET <b>{fmtMM(target.x)}, {fmtMM(target.y)}</b></span>
                <span>φ <b>{fmtDeg(target.phi)}°</b></span>
              </>
            }
            footL="GRID · 30 mm"
            footR="CLICK = MOVE TARGET · DRAG φ HANDLE"
            style={{ flex: 1 }}
          >
            <div ref={canvasRef} style={{ position: 'absolute', inset: 0, display: 'flex', alignItems: 'center', justifyContent: 'center' }}>
              <ThreeDofCanvas
                width={cw} height={ch}
                L1={L1} L2={L2} L3={L3}
                angles={animAngles}
                target={target}
                reachable={reachable}
                onTargetClick={(x, y) => applyTarget(x, y)}
                onPhiChange={handlePhiChange}
              />
            </div>
          </CanvasFrame>

          <div className={styles.controls}>
            <ArmSlider label="L₁" value={L1} min={30} max={200} unit="mm"
              onChange={v => { setL1(v); applyTarget(target.x, target.y, target.phi); }}/>
            <ArmSlider label="L₂" value={L2} min={30} max={200} unit="mm"
              onChange={v => { setL2(v); applyTarget(target.x, target.y, target.phi); }}/>
            <ArmSlider label="L₃" value={L3} min={20} max={150} unit="mm"
              onChange={v => { setL3(v); applyTarget(target.x, target.y, target.phi); }}/>
            <div className={styles.divider}/>
            <CoordIn label="X" value={txInput} onChange={setTxInput}/>
            <CoordIn label="Y" value={tyInput} onChange={setTyInput}/>
            <CoordIn label="φ" value={phiInput} onChange={setPhiInput} unit="°"/>
            <button className={styles.applyBtn} onClick={handleApply}>
              <svg width="11" height="11" viewBox="0 0 12 12" fill="none" stroke="currentColor" strokeWidth="2">
                <path d="M2 6l3 3 5-6"/>
              </svg>
              APPLY
            </button>
          </div>
        </div>

        <CalcPanel3
          L1={L1} L2={L2} L3={L3}
          target={target}
          angles={animAngles}
          reachable={reachable}
        />
      </div>
    </div>
  );
}

function ArmSlider({ label, value, min, max, unit, onChange }) {
  const pct = ((value - min) / (max - min)) * 100;
  return (
    <div className={styles.slider}>
      <span className={styles.sliderLbl}>{label}</span>
      <div className={styles.sliderTrack}>
        <div className={styles.sliderFill} style={{ width: `${pct}%` }}/>
        <div className={styles.sliderThumb} style={{ left: `${pct}%` }}/>
        <input type="range" min={min} max={max} value={value}
          onChange={e => onChange(Number(e.target.value))}
          className={styles.sliderInput}/>
      </div>
      <span className={styles.sliderVal}>{value}<span className={styles.sliderUnit}> {unit}</span></span>
    </div>
  );
}

function CoordIn({ label, value, onChange, unit = 'mm' }) {
  return (
    <div className={styles.coordInput}>
      <span className={styles.coordLbl}>{label}</span>
      <input type="number" value={value} onChange={e => onChange(e.target.value)}
        className={styles.coordField} step="0.1"/>
      <span className={styles.coordUnit}>{unit}</span>
    </div>
  );
}
