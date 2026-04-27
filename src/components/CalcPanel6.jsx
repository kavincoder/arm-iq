/**
 * CalcPanel6.jsx — 6-DOF Calculation Panel (DH table + Jacobian)
 *
 * Three sub-sections:
 *   1. DH TABLE — live θᵢ values + static a, d, α parameters
 *   2. FK DERIVATION — step-by-step T₀⁶ composition (static)
 *   3. JACOBIAN — 6×6 numeric matrix with condition/determinant/manipulability
 *
 * The Jacobian is computed numerically but displayed for educational value;
 * it's NOT used by the actual IK solver (which is closed-form analytical).
 *
 * Props:
 *   dhParams    — DH parameter table for active preset
 *   angles      — [θ1…θ6] current animated angles (radians)
 *   jacobian    — number[][] 6×6 Jacobian matrix (computed in section)
 *   metrics     — { cond, det, manipulability }
 *   presetName  — string: display name of active preset
 */

import styles from './CalcPanel.module.css';
import { fmtDeg, fmtMM, fmtRatio } from '../utils/format.js';

export function CalcPanel6({ dhParams = [], angles = [0,0,0,0,0,0], jacobian = null, metrics = null, presetName = 'UR5', ikSteps = null }) {
  const ROW_LABELS = ['x', 'y', 'z', 'ωx', 'ωy', 'ωz'];
  const ROW_COLORS = ['#ef4444', '#22c55e', '#3b82f6', '#8b949e', '#8b949e', '#8b949e'];

  return (
    <div className={styles.panel}>
      <div className={styles.head}>
        <h2>FK · DH · JACOBIAN</h2>
        <span className={styles.sub}>{presetName} · 6×6 · ANALYTICAL IK</span>
      </div>

      {/* ── DH Table ── */}
      <div className={styles.subpanel}>
        <div className={styles.subhead}>
          <span>DENAVIT–HARTENBERG TABLE</span>
          <span className={`${styles.pill} ${styles.live}`}>▶ LIVE</span>
        </div>
        <table className={styles.dhTable}>
          <thead>
            <tr>
              <th style={{ textAlign: 'left' }}>i</th>
              <th>θᵢ <span style={{ color: 'var(--teal-dim)' }}>(°)</span></th>
              <th>dᵢ <span style={{ color: 'var(--text-dim)' }}>(mm)</span></th>
              <th>aᵢ <span style={{ color: 'var(--text-dim)' }}>(mm)</span></th>
              <th>αᵢ <span style={{ color: 'var(--text-dim)' }}>(°)</span></th>
            </tr>
          </thead>
          <tbody>
            {dhParams.map((row, i) => (
              <tr key={i}>
                <td style={{ color: 'var(--text-secondary)' }}>{row.index ?? i + 1}</td>
                <td className={styles.theta}>{fmtDeg(angles[i] + (row.thetaOffset ?? 0))}</td>
                <td>{row.d?.toFixed(1)}</td>
                <td>{row.a?.toFixed(1)}</td>
                <td>{(row.alpha * 180 / Math.PI).toFixed(0)}</td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>

      {/* ── FK Derivation ── */}
      <div className={styles.subpanel}>
        <div className={styles.subhead}>
          <span>FORWARD KINEMATICS · T₀⁶</span>
          <span className={styles.pill}>DERIVATION</span>
        </div>
        <div className={styles.derivation}>
          <Step n="01">Tᵢ⁻¹ᵢ = Rotz(θᵢ)·Tz(dᵢ)·Tx(aᵢ)·Rotx(αᵢ)</Step>
          <Step n="02">T₀⁶ = T₀¹·T₁²·T₂³·T₃⁴·T₄⁵·T₅⁶</Step>
          <Step n="03">p = T₀⁶[0:3, 3] &nbsp; R = T₀⁶[0:3, 0:3]</Step>
        </div>
      </div>

      {/* ── IK Breakdown ── */}
      <div className={styles.subpanel}>
        <div className={styles.subhead}>
          <span>IK SOLUTION · STEP BY STEP</span>
          {ikSteps
            ? <span className={`${styles.pill} ${styles.live}`}>▶ PIEPER</span>
            : <span className={styles.pill}>SOLVE IK FIRST</span>}
        </div>

        {ikSteps ? (
          <div className={styles.ikBreakdown}>

            {/* Step 1 — Wrist Centre */}
            <IKGroup n="01" label="WRIST CENTRE  Pc = Pe − d6·R·ẑ">
              <IKRow lbl="Pc.x" val={fmtMM(ikSteps.wristCentre.x)} unit="mm"/>
              <IKRow lbl="Pc.y" val={fmtMM(ikSteps.wristCentre.y)} unit="mm"/>
              <IKRow lbl="Pc.z" val={fmtMM(ikSteps.wristCentre.z)} unit="mm"/>
            </IKGroup>

            {/* Step 2 — θ₁ */}
            <IKGroup n="02" label="SHOULDER  θ₁ = atan2(wcy, wcx)">
              <IKRow lbl="θ₁" val={fmtDeg(ikSteps.theta1)} unit="°"/>
            </IKGroup>

            {/* Step 3 — Planar geometry */}
            <IKGroup n="03" label="PLANAR GEOMETRY  law of cosines">
              <IKRow lbl="r"      val={fmtMM(ikSteps.r)}   unit="mm" tip="radial dist"/>
              <IKRow lbl="s"      val={fmtMM(ikSteps.s)}   unit="mm" tip="axial height"/>
              <IKRow lbl="‖Pc‖"  val={fmtMM(ikSteps.r2)}  unit="mm" tip="shoulder→wrist"/>
              <IKRow lbl="L₁"    val={fmtMM(ikSteps.L1)}  unit="mm" tip="|a₂|"/>
              <IKRow lbl="L₂"    val={fmtMM(ikSteps.L2)}  unit="mm" tip="√(a₃²+d₄²)"/>
              <IKRow lbl="cos θ₃" val={fmtRatio(ikSteps.cosT3)} unit="" tip="law of cosines"/>
            </IKGroup>

            {/* Step 4 — Elbow angles */}
            <IKGroup n="04" label="ELBOW ANGLES">
              <IKRow lbl="γ"  val={fmtDeg(ikSteps.gamma)}  unit="°" tip="atan2(d₄,a₃)"/>
              <IKRow lbl="θ₃" val={fmtDeg(ikSteps.theta3)} unit="°"/>
              <IKRow lbl="θ₂" val={fmtDeg(ikSteps.theta2)} unit="°"/>
            </IKGroup>

            {/* Step 5 — Wrist orientation */}
            <IKGroup n="05" label="WRIST  R₃⁶ = R₀³ᵀ · R₀⁶">
              <IKRow lbl="θ₄" val={fmtDeg(ikSteps.theta4)} unit="°"/>
              <IKRow lbl="θ₅" val={fmtDeg(ikSteps.theta5)} unit="°" tip="sin≠0 → ZYZ"/>
              <IKRow lbl="θ₆" val={fmtDeg(ikSteps.theta6)} unit="°"/>
            </IKGroup>

          </div>
        ) : (
          <div style={{ color: 'var(--text-dim)', fontFamily: 'var(--font-mono)', fontSize: 10, padding: '6px 0' }}>
            Press SOLVE IK to see Pieper's method step by step
          </div>
        )}
      </div>

      {/* ── Jacobian ── */}
      <div className={`${styles.subpanel} ${styles.flex1}`}>
        <div className={styles.subhead}>
          <span>JACOBIAN J(q) · 6 × 6</span>
          <span className={`${styles.pill} ${styles.live}`}>▶ LIVE</span>
        </div>

        {jacobian ? (
          <>
            <table className={styles.jacTable}>
              <thead>
                <tr>
                  <th></th>
                  {[1,2,3,4,5,6].map(q => <th key={q}>q{q}</th>)}
                </tr>
              </thead>
              <tbody>
                {jacobian.map((row, i) => (
                  <tr key={i}>
                    <td style={{ color: ROW_COLORS[i], minWidth: 20 }}>{ROW_LABELS[i]}</td>
                    {row.map((cell, j) => (
                      <td key={j} className={Math.abs(cell) < 1e-6 ? styles.zero : ''}>
                        {cell.toFixed(2)}
                      </td>
                    ))}
                  </tr>
                ))}
              </tbody>
            </table>

            {metrics && (
              <div style={{ marginTop: 10, fontFamily: 'var(--font-mono)', fontSize: 10, color: 'var(--text-dim)', lineHeight: '18px' }}>
                <div>
                  cond(J) = <span style={{ color: 'var(--teal)' }}>{metrics.cond.toFixed(2)}</span>
                  &nbsp; det(J) = <span style={{ color: 'var(--teal)' }}>{metrics.det.toExponential(2)}</span>
                </div>
                <div>
                  singular? <span style={{ color: metrics.cond > 100 ? 'var(--red)' : 'var(--green)' }}>
                    {metrics.cond > 100 ? 'YES' : 'NO'}
                  </span>
                  &nbsp; manip = <span style={{ color: 'var(--teal)' }}>{metrics.manipulability.toExponential(2)}</span>
                </div>
              </div>
            )}
          </>
        ) : (
          <div style={{ color: 'var(--text-dim)', fontFamily: 'var(--font-mono)', fontSize: 10, padding: '8px 0' }}>
            Click canvas to solve IK and compute Jacobian
          </div>
        )}
      </div>
    </div>
  );
}

function Step({ n, children }) {
  return (
    <div style={{ display: 'flex', gap: 10, padding: '4px 0', fontSize: 11, lineHeight: 1.5 }}>
      <span style={{ color: 'var(--teal-dim)', fontFamily: 'var(--font-mono)', fontSize: 9, minWidth: 18, paddingTop: 2 }}>{n}</span>
      <span style={{ color: 'var(--text-secondary)', fontFamily: 'var(--font-mono)' }}>{children}</span>
    </div>
  );
}

// ── IK Breakdown sub-components ───────────────────────────────────────────────

function IKGroup({ n, label, children }) {
  return (
    <div style={{ marginBottom: 8 }}>
      <div style={{
        display: 'flex', alignItems: 'center', gap: 6, marginBottom: 3,
      }}>
        <span style={{
          fontFamily: 'var(--font-mono)', fontSize: 8, color: 'var(--teal)',
          background: 'rgba(0,212,200,0.1)', border: '1px solid rgba(0,212,200,0.2)',
          borderRadius: 2, padding: '1px 5px', flexShrink: 0,
        }}>{n}</span>
        <span style={{
          fontFamily: 'var(--font-mono)', fontSize: 8,
          color: 'var(--text-dim)', letterSpacing: '0.08em',
        }}>{label}</span>
      </div>
      <div style={{ paddingLeft: 6, borderLeft: '1px solid var(--border)' }}>
        {children}
      </div>
    </div>
  );
}

function IKRow({ lbl, val, unit, tip }) {
  return (
    <div style={{
      display: 'flex', alignItems: 'baseline', gap: 4,
      fontFamily: 'var(--font-mono)', fontSize: 10, lineHeight: '17px',
    }}>
      <span style={{ minWidth: 46, color: 'var(--text-secondary)', fontSize: 9 }}>{lbl}</span>
      <span style={{ color: 'var(--teal)', fontVariantNumeric: 'tabular-nums' }}>{val}</span>
      {unit && <span style={{ color: 'var(--text-dim)', fontSize: 8 }}>{unit}</span>}
      {tip  && <span style={{ color: 'var(--text-dim)', fontSize: 8, marginLeft: 4 }}>// {tip}</span>}
    </div>
  );
}
