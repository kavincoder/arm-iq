/**
 * CanvasFrame.jsx — Scientific instrument-style canvas container
 *
 * Wraps the SVG / Three.js viewport with:
 *   - Corner tick marks (CAD / oscilloscope aesthetic)
 *   - Top-left metadata label
 *   - Top-right cursor / target readout
 *   - Bottom footer strips (grid info, zoom level)
 *
 * The frame never clips its children — the inner content fills the
 * available space and the tick marks are positioned absolutely on top.
 *
 * Props:
 *   meta      — string: top-left label (viewport mode description)
 *   metaR     — ReactNode: top-right readout (cursor coords, target)
 *   footL     — string: bottom-left footer (grid scale)
 *   footR     — string: bottom-right footer (zoom level / origin info)
 *   style     — CSSProperties: applied to the outer wrapper
 *   className — extra class names
 *   children  — the canvas/SVG content
 */

import styles from './CanvasFrame.module.css';

export function CanvasFrame({ children, meta, metaR, footL, footR, style, className = '' }) {
  return (
    <div className={`${styles.frame} ${className}`} style={style}>
      {/* Metadata labels */}
      {meta  && <div className={styles.metaL}>{meta}</div>}
      {metaR && <div className={styles.metaR}>{metaR}</div>}

      {/* Corner tick marks — 4 corners × 2 lines (H + V) each */}
      <span className={`${styles.tick} ${styles.tlH}`} />
      <span className={`${styles.tick} ${styles.tlV}`} />
      <span className={`${styles.tick} ${styles.trH}`} />
      <span className={`${styles.tick} ${styles.trV}`} />
      <span className={`${styles.tick} ${styles.blH}`} />
      <span className={`${styles.tick} ${styles.blV}`} />
      <span className={`${styles.tick} ${styles.brH}`} />
      <span className={`${styles.tick} ${styles.brV}`} />

      {/* Canvas content */}
      <div className={styles.inner}>
        {children}
      </div>

      {/* Footer strips */}
      {footL && <div className={styles.footL}>{footL}</div>}
      {footR && <div className={styles.footR}>{footR}</div>}
    </div>
  );
}
