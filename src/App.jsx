/**
 * App.jsx — Root application component
 *
 * Manages tab state (2-DOF / 3-DOF / 6-DOF) and renders the layout:
 *   [Navbar] → [active section (fills remaining height)]
 *
 * FPS counter is tracked via requestAnimationFrame in this component
 * and passed down to Navbar for the live readout.
 */

import { useState, useEffect, useRef } from 'react';
import { Navbar } from './components/Navbar.jsx';
import { TwoDOF } from './sections/TwoDOF.jsx';
import { ThreeDOF } from './sections/ThreeDOF.jsx';
import { SixDOF } from './sections/SixDOF.jsx';
import styles from './App.module.css';

export function App() {
  const [activeTab, setActiveTab] = useState('2dof');
  const [fps, setFps] = useState(60);

  // FPS counter: sample frame times over a rolling 500ms window
  const fpsRef    = useRef({ last: performance.now(), count: 0 });
  const rafRef    = useRef(null);

  useEffect(() => {
    const tick = (now) => {
      fpsRef.current.count++;
      const elapsed = now - fpsRef.current.last;
      if (elapsed >= 500) {
        setFps((fpsRef.current.count / elapsed) * 1000);
        fpsRef.current = { last: now, count: 0 };
      }
      rafRef.current = requestAnimationFrame(tick);
    };
    rafRef.current = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(rafRef.current);
  }, []);

  return (
    <div className={styles.app}>
      <Navbar
        activeTab={activeTab}
        onTabChange={setActiveTab}
        fps={fps}
      />
      <div className={styles.content}>
        {activeTab === '2dof' && <TwoDOF />}
        {activeTab === '3dof' && <ThreeDOF />}
        {activeTab === '6dof' && <SixDOF />}
      </div>
    </div>
  );
}

export default App;
