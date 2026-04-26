/**
 * StatCard.jsx — Horizontal stat strip card
 *
 * Each card shows one live value with a label, value, unit, and optional
 * sub-text or progress bar. The left teal border stripe is the signature
 * "scientific instrument readout" detail from the design spec.
 *
 * Props:
 *   label    — string: top-left label (e.g. "JOINT")
 *   id       — string: subscript identifier (e.g. "θ₁")
 *   value    — string | number: primary displayed value
 *   unit     — string: unit label (e.g. "deg", "mm")
 *   sub      — string: secondary small text below the value
 *   progress — number 0-100: optional progress bar percentage
 *   children — override slot (replaces default value/unit/sub layout)
 */

import styles from './StatCard.module.css';

export function StatCard({ label, id, value, unit, sub, progress, children }) {
  return (
    <div className={styles.card}>
      {/* Label row */}
      <div className={styles.label}>
        <span>{label}</span>
        {id && <span className={styles.id}>{id}</span>}
      </div>

      {/* Value area — use children prop to completely override */}
      {children ?? (
        <>
          <div className={styles.valueRow}>
            <span className={styles.value}>{value}</span>
            {unit && <span className={styles.unit}>{unit}</span>}
          </div>
          {sub && <div className={styles.sub}>{sub}</div>}
        </>
      )}

      {/* Progress bar (optional — used for workspace % coverage) */}
      {progress != null && (
        <div className={styles.progress}>
          <div className={styles.progressFill} style={{ width: `${progress}%` }} />
        </div>
      )}
    </div>
  );
}

/**
 * ReachBadge — small pill showing reachability status.
 * Goes inside a StatCard's children slot.
 */
export function ReachBadge({ ok = true, label }) {
  return (
    <span className={`${styles.badge} ${ok ? styles.ok : styles.bad}`}>
      <span className={styles.badgeDot} />
      {label ?? (ok ? 'REACHABLE' : 'OUT OF REACH')}
    </span>
  );
}
