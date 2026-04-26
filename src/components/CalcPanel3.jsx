/**
 * CalcPanel3.jsx — 3-DOF Inverse Kinematics Calculation Panel
 *
 * Shows geometric decomposition steps with live substituted values.
 * The key insight is the wrist back-computation: instead of solving a
 * 3-link system directly, we remove the end-link first and solve the
 * resulting 2-link problem.
 *
 * Props:
 *   L1, L2, L3  — link lengths (mm)
 *   target       — { x, y, phi } in mm and radians
 *   angles       — { t1, t2, t3 } current animated angles (radians)
 *   reachable    — boolean
 */

import styles from './CalcPanel.module.css';
import { fmtMM, fmtDeg, fmtRatio, fmtError } from '../utils/format.js';
import { solveFK3 } from '../math/ik3.js';

export function CalcPanel3({ L1 = 90, L2 = 75, L3 = 55, target = { x: 130, y: 70, phi: 0 }, angles = { t1: 0, t2: 0, t3: 0 }, reachable = true }) {
  const { t1, t2, t3 } = angles;
  const { x, y, phi } = target;

  // Wrist position
  const wx = x - L3 * Math.cos(phi);
  const wy = y - L3 * Math.sin(phi);
  const rw = Math.hypot(wx, wy);

  // 2-link sub-problem intermediate values
  const cos2 = Math.max(-1, Math.min(1, (rw * rw - L1 * L1 - L2 * L2) / (2 * L1 * L2)));
  const sin2 = Math.sqrt(1 - cos2 * cos2);
  const k1 = L1 + L2 * cos2;
  const k2 = L2 * sin2;

  // FK verification
  const { tip: fkTip } = solveFK3(t1, t2, t3, L1, L2, L3);
  const err = Math.hypot(fkTip.x - x, fkTip.y - y);

  return (
    <div className={styles.panel}>
      <div className={styles.head}>
        <h2>IK · CALCULATION</h2>
        <span className={styles.sub}>3-DOF · GEOMETRIC DECOMP · CLOSED-FORM</span>
      </div>

      <div className={styles.subpanel}>
        <div className={styles.subhead}>
          <span>STEP-BY-STEP DERIVATION</span>
          <span className={styles.pill}>STATIC</span>
        </div>
        <div className={styles.derivation}>
          <Step n="01">Choose end-effector orientation φ</Step>
          <Step n="02">Wrist xw = x − L₃·cos φ</Step>
          <Step n="03">Wrist yw = y − L₃·sin φ</Step>
          <Step n="04">Solve 2-link IK to (xw, yw) → θ₁, θ₂</Step>
          <Step n="05">θ₃ = φ − θ₁ − θ₂</Step>
        </div>
      </div>

      <div className={`${styles.subpanel} ${styles.flex1}`}>
        <div className={styles.subhead}>
          <span>LIVE EQUATIONS · SUBSTITUTED</span>
          <span className={`${styles.pill} ${styles.live}`}>▶ LIVE · 60 fps</span>
        </div>
        <div className={styles.live}>
          <Row lbl="x, y, φ" val={`${fmtMM(x)}, ${fmtMM(y)}, ${fmtDeg(phi)}°`}/>
          <Row lbl="L₁ L₂ L₃" val={`${L1}, ${L2}, ${L3}`} unit="mm"/>
          <div className={styles.divider}/>
          <Row lbl="xw" val={`${fmtMM(x)} − ${L3}·cos ${fmtDeg(phi)}° = ${fmtMM(wx)}`}/>
          <Row lbl="yw" val={`${fmtMM(y)} − ${L3}·sin ${fmtDeg(phi)}° = ${fmtMM(wy)}`}/>
          <Row lbl="rw" val={`√(xw² + yw²) = ${fmtMM(rw)}`}/>
          <div className={styles.divider}/>
          <Row lbl="cos θ₂" val={`(rw² − L₁² − L₂²) / (2·L₁·L₂)`}/>
          <Row lbl="" val={fmtRatio(cos2)}/>
          <Row lbl="sin θ₂" val={`+√(1 − ${fmtRatio(cos2)}²) = ${fmtRatio(sin2)}`}/>
          <Row lbl="θ₂" val={`${fmtDeg(t2)}°`}/>
          <div className={styles.divider}/>
          <Row lbl="θ₁" val={`atan2(${fmtMM(wy)}, ${fmtMM(wx)}) − atan2(k₂, k₁)`}/>
          <Row lbl="" val={`= ${fmtDeg(t1)}°`}/>
          <Row lbl="θ₃" val={`φ − θ₁ − θ₂ = ${fmtDeg(phi)} − (${fmtDeg(t1)}) − ${fmtDeg(t2)}`}/>
          <Row lbl="" val={`= ${fmtDeg(t3)}°`}/>
          <div className={styles.divider}/>
          <Row lbl="FK CHECK" header/>
          <Row lbl="x_fk" val={fmtMM(fkTip.x)} unit="mm"/>
          <Row lbl="y_fk" val={fmtMM(fkTip.y)} unit="mm"/>
          <Row lbl="|err|" val={fmtError(err)} unit="mm"/>
        </div>
      </div>
    </div>
  );
}

function Step({ n, children }) {
  return (
    <div style={{ display: 'flex', gap: 10, padding: '4px 0', fontSize: 11.5, lineHeight: 1.5 }}>
      <span style={{ color: 'var(--teal-dim)', fontFamily: 'var(--font-mono)', fontSize: 9, minWidth: 18, paddingTop: 2 }}>{n}</span>
      <span style={{ color: 'var(--text-secondary)', fontFamily: 'var(--font-mono)' }}>{children}</span>
    </div>
  );
}

function Row({ lbl, val, unit, header }) {
  return (
    <div className={styles.eqRow}>
      <span className={`${styles.eqLbl} ${header ? styles.header : ''}`}>{lbl}</span>
      {val !== undefined && <span className={styles.val}>{val}</span>}
      {unit && <span className={styles.unit}> {unit}</span>}
    </div>
  );
}
