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
import { fmtDeg, fmtMM } from '../utils/format.js';

export function CalcPanel6({ dhParams = [], angles = [0,0,0,0,0,0], jacobian = null, metrics = null, presetName = 'UR5' }) {
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
