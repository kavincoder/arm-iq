/**
 * CalcPanel2.jsx — 2-DOF Inverse Kinematics Calculation Panel
 *
 * Shows two sub-panels:
 *   1. STEP-BY-STEP DERIVATION — static walkthrough of the law of cosines method
 *   2. LIVE EQUATIONS — numeric values substituted and updated every frame
 *
 * The live section updates at 60fps as the user clicks targets and the
 * arm animates. Font-variant-numeric tabular-nums prevents layout jitter.
 *
 * Props (all derived from current IK state):
 *   L1, L2      — link lengths (mm)
 *   target      — { x, y } target in mm
 *   angles      — { t1, t2 } current animated angles (radians)
 *   reachable   — boolean
 */

import styles from './CalcPanel.module.css';
import { fmtMM, fmtDeg, fmtRatio, fmtError } from '../utils/format.js';
import { solveFK2 } from '../math/ik2.js';

export function CalcPanel2({ L1 = 100, L2 = 80, target = { x: 110, y: 60 }, angles = { t1: 0.42, t2: 0.72 }, reachable = true }) {
  const { t1, t2 } = angles;

  // Derived values for display
  const r2 = target.x ** 2 + target.y ** 2;
  const r  = Math.sqrt(r2);
  const cos2 = (r2 - L1 ** 2 - L2 ** 2) / (2 * L1 * L2);
  const cos2c = Math.max(-1, Math.min(1, cos2));
  const sin2 = Math.sqrt(1 - cos2c ** 2);
  const k1 = L1 + L2 * cos2c;
  const k2 = L2 * sin2;

  // FK verification
  const fk = solveFK2(t1, t2, L1, L2);
  const errX = fk.x - target.x;
  const errY = fk.y - target.y;
  const err  = Math.hypot(errX, errY);

  return (
    <div className={styles.panel}>
      {/* ── Header ── */}
      <div className={styles.head}>
        <h2>IK · CALCULATION</h2>
        <span className={styles.sub}>2-DOF · LAW OF COSINES · CLOSED-FORM</span>
      </div>

      {/* ── Step-by-step derivation ── */}
      <div className={styles.subpanel}>
        <div className={styles.subhead}>
          <span>STEP-BY-STEP DERIVATION</span>
          <span className={`${styles.pill}`}>STATIC</span>
        </div>
        <div className={styles.derivation}>
          <Step n="01">Given target <em>(x, y)</em> and links <em>L₁, L₂</em></Step>
          <Step n="02">r² = x² + y²</Step>
          <Step n="03">cos θ₂ = (r² − L₁² − L₂²) / (2·L₁·L₂)</Step>
          <Step n="04">sin θ₂ = ±√(1 − cos²θ₂) <span className={styles.hint}>  (± = elbow up/down)</span></Step>
          <Step n="05">θ₂ = atan2(sin θ₂, cos θ₂)</Step>
          <Step n="06">k₁ = L₁ + L₂·cos θ₂ &nbsp; k₂ = L₂·sin θ₂</Step>
          <Step n="07">θ₁ = atan2(y, x) − atan2(k₂, k₁)</Step>
        </div>
      </div>

      {/* ── Live equations ── */}
      <div className={`${styles.subpanel} ${styles.flex1}`}>
        <div className={styles.subhead}>
          <span>LIVE EQUATIONS · SUBSTITUTED</span>
          <span className={`${styles.pill} ${styles.live}`}>▶ LIVE · 60 fps</span>
        </div>
        <div className={styles.live}>
          <Row lbl="r²"    val={`${fmtMM(target.x)}² + ${fmtMM(target.y)}² = ${fmtMM(r2)}`}/>
          <Row lbl="r"     val={fmtMM(r)} unit="mm"/>
          <Row lbl="cos θ₂" val={`(${fmtMM(r2)} − ${L1*L1} − ${L2*L2}) / ${2*L1*L2}`}/>
          <Row lbl=""      val={fmtRatio(cos2c)}/>
          <Row lbl="sin θ₂" val={`+√(1 − ${fmtRatio(cos2c)}²) = ${fmtRatio(sin2)}`}/>
          <Row lbl="θ₂"   val={`atan2(${fmtRatio(sin2)}, ${fmtRatio(cos2c)}) = ${fmtDeg(t2)}°`}/>
          <Row lbl="k₁"   val={fmtMM(k1)} unit="mm" inline>
            <span className={styles.inlineLbl}>k₂</span>
            <span className={styles.val}>{fmtMM(k2)}</span>
          </Row>
          <Row lbl="θ₁" val={`atan2(${fmtMM(target.y)}, ${fmtMM(target.x)}) − atan2(k₂, k₁)`}/>
          <Row lbl=""   val={`= ${fmtDeg(t1)}°`}/>
          <div className={styles.divider}/>
          <Row lbl="FK CHECK" header/>
          <Row lbl="x_fk"  val={`L₁·cos θ₁ + L₂·cos(θ₁+θ₂) = ${fmtMM(fk.x)}`}/>
          <Row lbl="y_fk"  val={`L₁·sin θ₁ + L₂·sin(θ₁+θ₂) = ${fmtMM(fk.y)}`}/>
          <Row lbl="|err|" val={fmtError(err)} unit="mm"/>
        </div>
      </div>
    </div>
  );
}

// ── Local helpers ──────────────────────────────────────────────────────────────

function Step({ n, children }) {
  return (
    <div style={{ display: 'flex', gap: 10, padding: '4px 0', fontSize: 11.5, lineHeight: 1.5 }}>
      <span style={{ color: 'var(--teal-dim)', fontFamily: 'var(--font-mono)', fontSize: 9, minWidth: 18, paddingTop: 2 }}>{n}</span>
      <span style={{ color: 'var(--text-secondary)', fontFamily: 'var(--font-mono)' }}>{children}</span>
    </div>
  );
}

function Row({ lbl, val, unit, header, children, inline }) {
  return (
    <div className={styles.eqRow}>
      <span className={`${styles.eqLbl} ${header ? styles.header : ''}`}>{lbl}</span>
      {children ?? (
        <>
          {val !== undefined && <span className={styles.val}>{val}</span>}
          {unit && <span className={styles.unit}>{unit}</span>}
        </>
      )}
    </div>
  );
}
