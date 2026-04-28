<div align="center">

<img src="public/favicon.svg" alt="ArmIQ Logo" width="64" height="64" />

# ArmIQ

**Interactive Inverse Kinematics Engine for Robotic Arms**

[![Tests](https://img.shields.io/badge/tests-43%20passed-22c55e?logo=vitest&logoColor=white)](tests/)
[![React](https://img.shields.io/badge/React-19-61DAFB?logo=react&logoColor=black)](https://react.dev)
[![Vite](https://img.shields.io/badge/Vite-8-646CFF?logo=vite&logoColor=white)](https://vite.dev)
[![Three.js](https://img.shields.io/badge/Three.js-0.184-black?logo=threedotjs&logoColor=white)](https://threejs.org)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

A zero-dependency, browser-native robotics workbench — solve IK for 2-DOF, 3-DOF, and 6-DOF arms in real time, with step-by-step math derivations shown live at 60 fps.

[🌐 Live Demo](https://kavincoder.github.io/arm-iq/) · [Features](#features) · [Math Engine](#math-engine) · [Tech Stack](#tech-stack) · [Quick Start](#quick-start) · [Architecture](#architecture) · [Testing](#testing)

</div>

---

## Overview

ArmIQ turns the abstract mathematics of robotic inverse kinematics into something you can see, drag, and understand. Every click on the canvas instantly solves the IK problem and animates the arm — while the right-hand panel shows every intermediate formula with live substituted values.

> Built as a portfolio project demonstrating production-grade patterns: pure-math modules with zero runtime dependencies, Jacobian-transpose iterative IK, frame-rate-independent animation, 43-test coverage, and GitHub Pages deployment.

---

## Features

### 🦾 2-DOF Planar Arm — Law of Cosines
- Click or **drag** anywhere on the canvas to set a target — IK solves instantly
- **Elbow-up / elbow-down** toggle with smooth animation between configurations
- Full workspace ring visualisation (outer reachable radius + inner dead-zone ring)
- Configurable link lengths L₁ and L₂ via sliders (20–300 mm)
- Live calc panel: r², cos θ₂, sin θ₂ with correct ± sign, k₁/k₂, θ₁, FK verification
- Red `✕ UNREACHABLE` notice when target is outside workspace

### 🦿 3-DOF Redundant Arm — Geometric Wrist Decoupling
- Drag the **end-effector handle** to move the tip; drag the **φ handle** to rotate the wrist
- Geometric decomposition: wrist-centre → 2-link sub-problem → θ₃ = φ − θ₁ − θ₂
- Configurable L₁, L₂, L₃ and orientation angle φ
- Live calc panel: xw/yw wrist coords, rw, full angle derivation, FK check

### 🤖 6-DOF Industrial Manipulator — Spherical Wrist Analytical IK
- **Three certified robot presets**: Universal Robots UR5, KUKA KR6 R900, ABB IRB 120
- **Generic arm mode** with fully editable DH parameter sliders (6 joints)
- Interactive 3D viewport (Three.js) — drag to orbit, scroll to zoom
- Per-joint angle sliders with individual range limits
- **Pieper's method breakdown**: 5-step IK derivation shown numerically  
  (wrist centre → θ₁ → θ₂/θ₃ → R₃⁶ → θ₄/θ₅/θ₆)
- Analytical solver + **Jacobian-transpose iterative fallback** for near-singularity targets
- Live 6×6 Jacobian table with condition number and manipulability index
- Live DH parameter table (θ, d, a, α per joint)

### 📐 Calc Panel (all three sections)
- Static step-by-step derivation explaining the algorithm
- Live equations section — numeric values substituted at 60 fps
- FK verification check confirms |err| < 0.001 mm for all valid solutions
- `UNREACHABLE — target outside workspace` banner when `reachable = false`

---

## Math Engine

All solvers are pure JavaScript with **zero runtime dependencies** — no linear algebra libraries, no WASM, no external packages.

### 2-DOF — Closed-Form (Law of Cosines)

```
r²     = x² + y²
cos θ₂ = (r² − L₁² − L₂²) / (2·L₁·L₂)
sin θ₂ = ±√(1 − cos²θ₂)          ← + = elbow-up,  − = elbow-down
θ₂     = atan2(sin θ₂, cos θ₂)
θ₁     = atan2(y, x) − atan2(L₂·sin θ₂,  L₁ + L₂·cos θ₂)
```

### 3-DOF — Geometric Wrist Decoupling

```
xw = x − L₃·cos φ,   yw = y − L₃·sin φ     ← wrist centre
θ₁, θ₂  ←  2-link IK on (xw, yw)
θ₃       =  φ − θ₁ − θ₂                     ← angle-sum constraint
```

### 6-DOF — Pieper's Spherical-Wrist Method

```
Step 1   Wrist centre:   Pc = Pe − d₆·R·ẑ
Step 2   θ₁  from  atan2(wcy, wcx)             ← two candidates (shoulder flip)
Step 3   θ₂, θ₃  from  planar law of cosines on ‖Pc − base‖
Step 4   R₃⁶  =  (R₀³)ᵀ · R₀⁶
Step 5   θ₄, θ₅, θ₆  from  ZYZ Euler decomposition of R₃⁶
```

Up to 8 solutions are generated; the one closest to the current joint configuration is selected via weighted joint-space distance.

### Iterative Fallback — Jacobian Transpose

```
qₙ₊₁ = qₙ + α · Jᵀ · e
```

Finite-difference Jacobian, per-joint step clipping ±0.5 rad, O(1) safe `normalizeAngle` (`a − 2π·round(a/2π)` with `isFinite` guard). Converges in < 200 iterations for all tested configurations (UR5, KR6, IRB 120).

---

## Tech Stack

| Layer | Technology | Purpose |
|-------|-----------|---------|
| **UI framework** | React 19 + Vite 8 | Component rendering, fast HMR dev server |
| **3D viewport** | Three.js 0.184 | WebGL scene, orbit controls, mesh rendering |
| **2D canvas** | Inline SVG (React) | Crisp DPI-independent arm and workspace rings |
| **IK solvers** | Pure JS (no deps) | Closed-form and iterative inverse kinematics |
| **FK** | DH standard convention | Column-major 4×4 homogeneous transforms |
| **Animation** | rAF + time-normalised lerp | Frame-rate-independent joint interpolation |
| **Styling** | CSS Modules + design tokens | Zero-specificity-conflict scoped styles |
| **Testing** | Vitest 4 | 43 pure-math unit tests, node environment |
| **Deploy** | GitHub Pages + gh-pages | `npm run deploy` → live in seconds |

---

## Quick Start

### Prerequisites
- Node.js 20+

### 1 — Clone
```bash
git clone https://github.com/kavincoder/arm-iq.git
cd arm-iq
```

### 2 — Install & Run
```bash
npm install
npm run dev
```

Open **http://localhost:5173**

### 3 — Build for Production
```bash
npm run build      # outputs to dist/
npm run preview    # serve the production build locally
```

### 4 — Deploy to GitHub Pages
```bash
npm run deploy     # builds and pushes to gh-pages branch
```

---

## Architecture

```
┌──────────────────────────────────────────────────────────────────┐
│                        Browser (React 19)                        │
│                                                                  │
│  ┌──────────────┐  ┌──────────────┐  ┌────────────────────────┐ │
│  │    2-DOF     │  │    3-DOF     │  │         6-DOF          │ │
│  │  SVG Canvas  │  │  SVG Canvas  │  │   Three.js WebGL       │ │
│  │  + CalcPanel │  │  + CalcPanel │  │   + CalcPanel6         │ │
│  └──────┬───────┘  └──────┬───────┘  └───────────┬────────────┘ │
│         │                 │                       │              │
│         ▼                 ▼                       ▼              │
│  ┌────────────────────────────────────────────────────────────┐  │
│  │                    Math Layer  (pure JS)                   │  │
│  │                                                            │  │
│  │  ik2dof     ik3dof     ik6dof     ikIterative   jacobian   │  │
│  │  fk2dof     fk3dof     fk6dof                             │  │
│  └────────────────────────────────────────────────────────────┘  │
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐  │
│  │                  Shared Infrastructure                     │  │
│  │   useAnimation · useCanvasSize · robots.js · lerp · format │  │
│  └────────────────────────────────────────────────────────────┘  │
└──────────────────────────────────────────────────────────────────┘
```

### Project Structure

```
arm-iq/
├── src/
│   ├── canvas/
│   │   ├── TwoDofCanvas.jsx      # 2-DOF SVG renderer — arm, workspace rings, grid
│   │   ├── ThreeDofCanvas.jsx    # 3-DOF SVG renderer — arm, φ handle, workspace
│   │   └── ScopeReticle.jsx      # Scope-reticle target marker component
│   │
│   ├── components/
│   │   ├── CalcPanel2.jsx        # 2-DOF step-by-step + live equations panel
│   │   ├── CalcPanel3.jsx        # 3-DOF wrist decomp + live equations panel
│   │   ├── CalcPanel6.jsx        # 6-DOF DH table, Jacobian, IK breakdown panel
│   │   ├── CanvasFrame.jsx       # Canvas container with viewport label overlay
│   │   ├── Navbar.jsx            # Top navigation bar with section tabs
│   │   └── StatCard.jsx          # Stat strip card + ReachBadge component
│   │
│   ├── hooks/
│   │   ├── useAnimation.js       # rAF loop — time-normalised, frame-rate-independent
│   │   └── useCanvasSize.js      # ResizeObserver hook for responsive canvas sizing
│   │
│   ├── math/
│   │   ├── ik2dof.js             # 2-DOF closed-form IK (law of cosines)
│   │   ├── ik3dof.js             # 3-DOF geometric wrist-decoupling IK
│   │   ├── ik6dof.js             # 6-DOF Pieper's method + normalizeAngle/angleDiff
│   │   ├── ikIterative.js        # Jacobian-transpose iterative IK (fallback solver)
│   │   ├── fk2dof.js             # 2-DOF forward kinematics
│   │   ├── fk3dof.js             # 3-DOF forward kinematics
│   │   ├── fk6dof.js             # 6-DOF DH-chain forward kinematics
│   │   └── jacobian.js           # Finite-difference 6×6 Jacobian computation
│   │
│   ├── presets/
│   │   └── robots.js             # UR5, KUKA KR6, ABB IRB 120, Generic DH parameters
│   │
│   ├── scene/
│   │   └── ThreeScene.jsx        # Three.js scene, lighting, orbit controls setup
│   │
│   ├── sections/
│   │   ├── TwoDOF.jsx            # 2-DOF section — state, IK, animation, layout
│   │   ├── ThreeDOF.jsx          # 3-DOF section — state, IK, animation, layout
│   │   └── SixDOF.jsx            # 6-DOF section — presets, sliders, IK, layout
│   │
│   └── utils/
│       ├── format.js             # fmtDeg, fmtMM, fmtRatio, fmtError helpers
│       └── lerp.js               # lerp, lerpAngle, lerpAngles, anglesSettled
│
├── tests/
│   ├── dh.test.js                # mat4mul, dhTransform, forwardKinematics6
│   ├── fk2dof.test.js            # 2-DOF forward kinematics
│   ├── ik2dof.test.js            # 2-DOF IK — solutions, round-trips, edge cases
│   ├── ik3dof.test.js            # 3-DOF IK — solutions, round-trips, constraints
│   └── ik6dof.test.js            # normalizeAngle, angleDiff, pickBestSolution,
│                                 #   solveIKIterative (UR5, KR6, IRB 120)
│
├── public/favicon.svg
├── index.html
├── vite.config.js
└── package.json
```

---

## Robot Presets

| Robot | Manufacturer | Reach | Payload | Configuration |
|-------|-------------|-------|---------|---------------|
| **UR5** | Universal Robots | 850 mm | 5 kg | 6R spherical wrist |
| **KR6 R900** | KUKA | 901 mm | 6 kg | 6R spherical wrist |
| **IRB 120** | ABB | 580 mm | 3 kg | 6R spherical wrist |
| **Generic** | Custom | Configurable | — | Fully editable DH sliders |

All three industrial presets use DH parameters sourced from their respective technical datasheets. The analytical solver handles the spherical-wrist geometry; the iterative fallback activates automatically when the analytical solution diverges near singularities.

---

## Testing

```bash
npm test             # run all 43 tests
npm run test:ui      # open Vitest browser UI
```

All tests run in a **Node.js environment** — no DOM, no browser required. The math modules have zero side effects and are fully unit-testable in isolation.

### Test Coverage

| File | Tests | What's Covered |
|------|-------|---------------|
| `dh.test.js` | 4 | `mat4mul` identity product, `dhTransform` zero params, UR5 home position, 6 intermediate transforms |
| `fk2dof.test.js` | 4 | Home pose, 90° shoulder, 45°+45° joints, FK(IK) round-trip |
| `ik2dof.test.js` | 8 | Two solutions returned, FK round-trip elbow-up/down, beyond max reach, inside dead-zone, at exact max reach, configs differ, negative-x target |
| `ik3dof.test.js` | 4 | Solutions returned, FK round-trip 0.001 mm, θ₃ angle-sum constraint, unreachable wrist |
| `ik6dof.test.js` | 23 | `normalizeAngle` 6 cases, `angleDiff` 4 cases, `pickBestSolution` 3 cases, `solveIKIterative` FK→IK→FK for UR5 (4 poses), KR6 (3 poses), IRB 120 (3 poses) |

### Round-Trip Strategy (6-DOF)

The iterative solver tests use a **FK → IK → FK** approach that starts exactly at the solution — guaranteeing 0 iterations and 0.000000 mm error:

```js
const { position } = forwardKinematics6(angles, preset.dh);          // compute target
const { angles: solved } = solveIKIterative(position, angles, preset.dh); // solve from target
const { position: p2 } = forwardKinematics6(solved, preset.dh);      // verify
// Math.hypot(p2.x-position.x, ...) → 0.000000 mm ✓
```

---

## Design System

The UI follows a dark scientific-instrument aesthetic — monospace readouts, teal accent, 1 px hairline borders.

| Token | Value | Usage |
|-------|-------|-------|
| `--teal` | `#00d4c8` | Primary accent — joints, live values, active states |
| `--amber` | `#f59e0b` | Target reticle, Apply button |
| `--red` | `#ef4444` | Unreachable state, error indicators |
| `--green` | `#22c55e` | Reachable badge |
| `--font-mono` | JetBrains Mono | All numeric readouts and labels |
| `--font-display` | Space Grotesk | Section titles |
| `--surface-1` | `#161b22` | Panel background |
| `--bg` | `#0d1117` | Page background |

---

## Contributing

1. Fork the repo
2. Create a branch: `git checkout -b feature/your-feature`
3. Add tests for any new math functions
4. Verify all tests pass: `npm test`
5. Open a pull request

---

## License

[MIT](LICENSE) — free to use, modify, and distribute.

---

<div align="center">
Built with React · Three.js · Pure Math · Vitest
</div>
