/**
 * Navbar.jsx — Top navigation bar
 *
 * Shows the ArmIQ logo, section tabs (2-DOF / 3-DOF / 6-DOF),
 * solver status indicator, FPS counter, and utility icon buttons.
 *
 * Props:
 *   activeTab  — '2dof' | '3dof' | '6dof'
 *   onTabChange — (tab: string) => void
 *   fps         — number (live frames per second)
 *   solverStatus — 'NOMINAL' | 'SOLVING' | 'UNREACHABLE'
 */

import styles from './Navbar.module.css';

const TABS = [
  { id: '2dof', num: '01', label: '2-DOF', desc: 'Planar 2D' },
  { id: '3dof', num: '02', label: '3-DOF', desc: 'Planar 2D' },
  { id: '6dof', num: '03', label: '6-DOF', desc: 'Spatial 3D' },
];

export function Navbar({ activeTab, onTabChange, fps = 60.0, solverStatus = 'NOMINAL' }) {
  return (
    <nav className={styles.nav}>
      {/* ── Logo ── */}
      <div className={styles.logo}>
        <LogoIcon />
        <span className={styles.logoText}>ArmIQ</span>
        <span className={styles.ver}>v1.0 · IK ENGINE</span>
      </div>

      {/* ── Section tabs ── */}
      <div className={styles.tabs}>
        {TABS.map(tab => (
          <button
            key={tab.id}
            className={`${styles.tab} ${activeTab === tab.id ? styles.active : ''}`}
            onClick={() => onTabChange(tab.id)}
          >
            <span className={styles.tabNum}>{tab.num}</span>
            {tab.label}
          </button>
        ))}
      </div>

      {/* ── Right: status + icons ── */}
      <div className={styles.right}>
        <div className={styles.status}>
          <span className={`${styles.dot} ${styles[solverStatus.toLowerCase()]}`} />
          <span>SOLVER · {solverStatus}</span>
        </div>

        <div className={`${styles.status} ${styles.fpsBlock}`}>
          <span className={styles.fpsValue}>{fps.toFixed(1)}</span>
          <span>FPS</span>
        </div>

        <a
          href="https://github.com/kavincoder/arm-iq"
          target="_blank"
          rel="noopener noreferrer"
          className={styles.iconBtn}
          title="GitHub"
        >
          {/* GitHub icon */}
          <svg width="14" height="14" viewBox="0 0 16 16" fill="currentColor">
            <path d="M8 0C3.58 0 0 3.58 0 8c0 3.54 2.29 6.53 5.47 7.59.4.07.55-.17.55-.38 0-.19-.01-.82-.01-1.49-2 .37-2.53-.49-2.69-.94-.09-.23-.48-.94-.82-1.13-.28-.15-.68-.52-.01-.53.63-.01 1.08.58 1.23.82.72 1.21 1.87.87 2.33.66.07-.52.28-.87.51-1.07-1.78-.2-3.64-.89-3.64-3.95 0-.87.31-1.59.82-2.15-.08-.2-.36-1.02.08-2.12 0 0 .67-.21 2.2.82.64-.18 1.32-.27 2-.27.68 0 1.36.09 2 .27 1.53-1.04 2.2-.82 2.2-.82.44 1.1.16 1.92.08 2.12.51.56.82 1.27.82 2.15 0 3.07-1.87 3.75-3.65 3.95.29.25.54.73.54 1.48 0 1.07-.01 1.93-.01 2.2 0 .21.15.46.55.38A8.013 8.013 0 0 0 16 8c0-4.42-3.58-8-8-8z"/>
          </svg>
        </a>
      </div>
    </nav>
  );
}

/** ArmIQ logo — stylised two-link arm in SVG */
function LogoIcon() {
  return (
    <svg width="20" height="20" viewBox="0 0 20 20" fill="none"
      stroke="#00d4c8" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round">
      <circle cx="3.5"  cy="16.5" r="1.5" fill="#0d1117"/>
      <line x1="3.5" y1="16.5" x2="9" y2="9" />
      <circle cx="9"    cy="9"    r="1.2" fill="#161b22"/>
      <line x1="9" y1="9" x2="15.5" y2="5.5" />
      <circle cx="15.5" cy="5.5"  r="1.6" fill="#0d1117" stroke="#00d4c8"/>
      <line x1="0.5" y1="18.5" x2="19.5" y2="18.5" />
    </svg>
  );
}
