/**
 * ThreeScene.jsx — Three.js 6-DOF 3D arm renderer
 *
 * Mounts a Three.js scene into a div via useEffect (raw Three.js,
 * NOT React Three Fiber — keeps the bundle smaller and gives full control).
 *
 * The scene includes:
 *   - Orthographic camera (isometric view)
 *   - Grid floor plane
 *   - 6-link arm with cylindrical links and spherical joints
 *   - End-effector glow (PointLight)
 *   - Target marker (WireframeGeometry sphere)
 *   - Material toggle (matte generic vs. metallic industrial)
 *   - OrbitControls-style drag rotation (manual implementation)
 *
 * Scale: MM_TO_SCENE = 0.3 → 300mm arm ≈ 90 scene units (fills viewport well)
 *
 * Props:
 *   width, height   — canvas dimensions
 *   angles          — [θ1…θ6] current animated angles (radians)
 *   dhParams        — DH parameter table for active preset
 *   target          — { x, y, z } in mm (or null)
 *   reachable       — boolean
 *   metallic        — boolean: material toggle
 *   presetColor     — hex string: preset accent color (used for joints)
 */

import { useEffect, useRef } from 'react';
import * as THREE from 'three';
import { forwardKinematics6 } from '../math/dh.js';

const MM = 0.3;  // mm → scene units scale factor

export function ThreeScene({ width, height, angles = [0,0,0,0,0,0], dhParams = [], target = null, reachable = true, metallic = false, presetColor = '#00d4c8' }) {
  const mountRef   = useRef(null);
  const sceneRef   = useRef(null);
  const rendererRef = useRef(null);
  const armRef     = useRef(null);

  // ── Initial Three.js setup ── (runs once on mount)
  useEffect(() => {
    const el = mountRef.current;
    if (!el) return;

    // ── Renderer ──
    const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    renderer.setPixelRatio(window.devicePixelRatio);
    renderer.setSize(width, height);
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFShadowMap;  // PCFSoft is deprecated in r184+
    renderer.setClearColor(0x080c10, 1);
    el.appendChild(renderer.domElement);
    rendererRef.current = renderer;

    // ── Scene ──
    const scene = new THREE.Scene();
    scene.fog = new THREE.FogExp2(0x080c10, 0.003);
    sceneRef.current = scene;

    // ── Camera (orthographic isometric view) ──
    const aspect = width / height;
    const frustum = 280;
    const camera = new THREE.OrthographicCamera(
      -frustum * aspect, frustum * aspect,
      frustum, -frustum,
      -2000, 2000
    );
    // Isometric angle
    camera.position.set(200, 180, 200);
    camera.lookAt(0, 60, 0);

    // ── Lights ──
    scene.add(new THREE.AmbientLight(0x202840, 2.5));

    const dirLight = new THREE.DirectionalLight(0xffffff, 1.5);
    dirLight.position.set(200, 400, 100);
    dirLight.castShadow = true;
    dirLight.shadow.mapSize.set(1024, 1024);
    scene.add(dirLight);

    const rimLight = new THREE.DirectionalLight(0x00d4c8, 0.4);
    rimLight.position.set(-200, 100, -100);
    scene.add(rimLight);

    // ── Grid floor ──
    const gridHelper = new THREE.GridHelper(600, 20, 0x21262d, 0x161b22);
    gridHelper.position.y = 0;
    scene.add(gridHelper);

    // Floor plane (subtle glow at base)
    const floorGeo  = new THREE.CircleGeometry(200, 64);
    const floorMat  = new THREE.MeshBasicMaterial({ color: 0x00d4c8, transparent: true, opacity: 0.02, side: THREE.DoubleSide });
    const floor     = new THREE.Mesh(floorGeo, floorMat);
    floor.rotation.x = -Math.PI / 2;
    scene.add(floor);

    // ── Base platform ──
    const baseGeo  = new THREE.CylinderGeometry(18, 22, 14, 32);
    const baseMat  = new THREE.MeshStandardMaterial({ color: 0x1c2330, metalness: 0.7, roughness: 0.4 });
    const base     = new THREE.Mesh(baseGeo, baseMat);
    base.position.y = 7;
    base.castShadow = true;
    scene.add(base);

    // ── Axis markers at origin ──
    scene.add(buildAxisMarkers());

    // ── Compass indicator ──
    // (shown in bottom-right corner — just axis lines at fixed position)

    // ── End-effector glow light ──
    const eeLight = new THREE.PointLight(0x00d4c8, 1.5, 80);
    scene.add(eeLight);

    // Store eeLight ref for updates
    scene.userData.eeLight = eeLight;

    // ── Target marker ──
    const targetMarker = buildTargetMarker();
    scene.add(targetMarker);
    scene.userData.targetMarker = targetMarker;

    // ── Arm group (rebuilt when DH params change) ──
    const armGroup = new THREE.Group();
    scene.add(armGroup);
    armRef.current = armGroup;

    // ── Manual orbit: rotate (left drag), pan (right drag), zoom (wheel) ──
    let isDown = false, isPan = false, lastX = 0, lastY = 0;
    let azimuth = Math.PI / 4, polar = Math.PI / 4;
    let frustum = 280;   // orthographic frustum half-height (zoom proxy)

    const camRadius = Math.sqrt(200**2 + 180**2 + 200**2);
    const camTarget = new THREE.Vector3(0, 60, 0);

    // Apply current orbit state to the camera
    const updateCamera = () => {
      camera.position.set(
        camTarget.x + camRadius * Math.sin(polar) * Math.sin(azimuth),
        camTarget.y + camRadius * Math.cos(polar),
        camTarget.z + camRadius * Math.sin(polar) * Math.cos(azimuth),
      );
      camera.lookAt(camTarget);

      // Update orthographic projection for zoom
      const aspect = camera.right / camera.top;  // current aspect
      camera.top    =  frustum;
      camera.bottom = -frustum;
      camera.left   = -frustum * aspect;
      camera.right  =  frustum * aspect;
      camera.updateProjectionMatrix();
    };

    // ── Left-button drag → rotate ──
    el.addEventListener('mousedown', (e) => {
      if (e.button === 0) { isDown = true; lastX = e.clientX; lastY = e.clientY; }
      if (e.button === 2) { isPan  = true; lastX = e.clientX; lastY = e.clientY; }
    });

    el.addEventListener('mousemove', (e) => {
      const dx = e.clientX - lastX;
      const dy = e.clientY - lastY;

      if (isDown) {
        // Orbit rotate
        azimuth -= dx * 0.006;
        polar    = Math.max(0.1, Math.min(Math.PI - 0.1, polar + dy * 0.006));
        updateCamera();
      }

      if (isPan) {
        // Pan: move camTarget in the camera's local right/up plane
        // Right vector = cross(camDir, worldUp)
        const camDir = new THREE.Vector3()
          .subVectors(camTarget, camera.position).normalize();
        const worldUp = new THREE.Vector3(0, 1, 0);
        const right   = new THREE.Vector3().crossVectors(camDir, worldUp).normalize();
        const up      = new THREE.Vector3().crossVectors(right, camDir).normalize();

        // Scale pan speed with frustum size so it feels consistent at any zoom
        const panSpeed = frustum * 0.002;
        camTarget.addScaledVector(right, -dx * panSpeed);
        camTarget.addScaledVector(up,     dy * panSpeed);
        updateCamera();
      }

      lastX = e.clientX; lastY = e.clientY;
    });

    el.addEventListener('mouseup',   (e) => { if (e.button === 0) isDown = false; if (e.button === 2) isPan = false; });
    el.addEventListener('mouseleave', ()  => { isDown = false; isPan = false; });

    // ── Prevent context menu on right-click ──
    el.addEventListener('contextmenu', (e) => e.preventDefault());

    // ── Wheel → zoom (change frustum size) ──
    el.addEventListener('wheel', (e) => {
      e.preventDefault();
      const zoomFactor = e.deltaY > 0 ? 1.1 : 0.9;  // scroll down = zoom out
      frustum = Math.max(40, Math.min(800, frustum * zoomFactor));
      updateCamera();
    }, { passive: false });

    // ── Render loop ──
    let rafId;
    const render = () => {
      rafId = requestAnimationFrame(render);
      renderer.render(scene, camera);
    };
    render();

    // Store camera for resize
    scene.userData.camera = camera;

    return () => {
      cancelAnimationFrame(rafId);
      renderer.dispose();
      if (el.contains(renderer.domElement)) el.removeChild(renderer.domElement);
    };
  }, []); // eslint-disable-line react-hooks/exhaustive-deps

  // ── Update arm pose every time angles or dhParams change ──
  useEffect(() => {
    const scene = sceneRef.current;
    const armGroup = armRef.current;
    if (!scene || !armGroup || dhParams.length === 0) return;

    // Rebuild arm geometry
    while (armGroup.children.length) armGroup.remove(armGroup.children[0]);
    buildArm(armGroup, angles, dhParams, metallic, presetColor);

    // Move end-effector glow to TCP position
    const { Ts } = forwardKinematics6(angles, dhParams);
    if (Ts && Ts.length === 6) {
      const T = Ts[5];
      const eePos = new THREE.Vector3(T[12] * MM, T[14] * MM, -T[13] * MM);
      const eeLight = scene.userData.eeLight;
      if (eeLight) eeLight.position.copy(eePos);
    }
  }, [angles, dhParams, metallic, presetColor]);

  // ── Update target marker ──
  useEffect(() => {
    const scene = sceneRef.current;
    if (!scene) return;
    const marker = scene.userData.targetMarker;
    if (!marker) return;

    if (target) {
      marker.position.set(target.x * MM, target.z * MM, -target.y * MM);
      marker.visible = true;
      marker.children.forEach(c => {
        if (c.material) c.material.color.set(reachable ? 0xf59e0b : 0xef4444);
      });
    } else {
      marker.visible = false;
    }
  }, [target, reachable]);

  // ── Resize renderer when dimensions change ──
  useEffect(() => {
    const renderer = rendererRef.current;
    const scene = sceneRef.current;
    if (!renderer || !scene) return;
    renderer.setSize(width, height);
    const camera = scene.userData.camera;
    if (camera) {
      const aspect = width / height;
      const frustum = 280;
      camera.left   = -frustum * aspect;
      camera.right  =  frustum * aspect;
      camera.updateProjectionMatrix();
    }
  }, [width, height]);

  return (
    <div ref={mountRef} style={{ width, height, display: 'block', cursor: 'grab' }} />
  );
}

// ── Scene builders ────────────────────────────────────────────────────────────

/** Build 6-link arm in the Three.js scene using DH transforms. */
function buildArm(group, angles, dhParams, metallic, presetColor) {
  const { Ts } = forwardKinematics6(angles, dhParams);
  if (!Ts || Ts.length === 0) return;

  // Joint positions in scene space: apply MM scale + Y/Z swap (Z-up → Y-up)
  const prevPos = new THREE.Vector3(0, 14, 0);   // start at top of base

  const linkWidths = [18, 16, 14, 12, 9, 7];
  const linkColors = [0x7d8a9a, 0x6a7585, 0x5a6575, 0x4a5565, 0x3a4555, 0x2d3845];
  const jointColors = [0x1c2330, 0x1c2330, 0x1c2330, 0x1c2330, 0x1c2330, 0x1c2330];

  for (let i = 0; i < Ts.length; i++) {
    const T = Ts[i];
    // Convert from DH column-major (Z-up) to Three.js (Y-up)
    const pos = new THREE.Vector3(T[12] * MM, T[14] * MM, -T[13] * MM);

    // ── Link cylinder from prevPos to pos ──
    const dir = new THREE.Vector3().subVectors(pos, prevPos);
    const len = dir.length();

    if (len > 0.5) {
      const linkMat = metallic
        ? new THREE.MeshStandardMaterial({ color: linkColors[i], metalness: 0.9, roughness: 0.15 })
        : new THREE.MeshStandardMaterial({ color: linkColors[i], metalness: 0.3, roughness: 0.6 });

      const r = linkWidths[i] * 0.4;   // scale radius
      const linkGeo = new THREE.CylinderGeometry(r, r * 1.1, len, 16);
      const linkMesh = new THREE.Mesh(linkGeo, linkMat);
      linkMesh.castShadow = true;

      // Position & orient link along dir
      linkMesh.position.copy(prevPos).lerp(pos, 0.5);
      linkMesh.quaternion.setFromUnitVectors(
        new THREE.Vector3(0, 1, 0),
        dir.clone().normalize()
      );

      // Teal highlight edge (thin cylinder)
      const hlGeo = new THREE.CylinderGeometry(r + 0.5, r * 1.1 + 0.5, len, 16);
      const hlMat = new THREE.MeshBasicMaterial({ color: 0x00d4c8, transparent: true, opacity: 0.12, wireframe: false });
      const hl = new THREE.Mesh(hlGeo, hlMat);
      hl.quaternion.copy(linkMesh.quaternion);
      hl.position.copy(linkMesh.position);

      group.add(linkMesh, hl);
    }

    // ── Joint sphere at this position ──
    const jointRadius = i === 5 ? 7 : (i < 3 ? 12 : 9);
    const jointGeo    = new THREE.SphereGeometry(jointRadius * 0.4, 16, 16);
    const jointMat    = new THREE.MeshStandardMaterial({
      color: 0x1c2330,
      metalness: 0.7,
      roughness: 0.3,
      emissive: i === 5 ? new THREE.Color(presetColor) : 0x000000,
      emissiveIntensity: i === 5 ? 0.15 : 0,
    });
    const jointMesh   = new THREE.Mesh(jointGeo, jointMat);
    jointMesh.position.copy(pos);
    jointMesh.castShadow = true;

    // Joint ring (teal accent ring around each joint)
    const ringGeo = new THREE.TorusGeometry(jointRadius * 0.4 + 1.2, 0.8, 8, 32);
    const ringMat = new THREE.MeshBasicMaterial({ color: 0x00a89e, transparent: true, opacity: 0.6 });
    const ring    = new THREE.Mesh(ringGeo, ringMat);
    ring.position.copy(pos);

    group.add(jointMesh, ring);

    // ── End-effector decoration ──
    if (i === 5) {
      // Bright end-effector sphere
      const eeGeo = new THREE.SphereGeometry(6, 16, 16);
      const eeMat = new THREE.MeshStandardMaterial({
        color: 0xffffff,
        emissive: new THREE.Color(presetColor),
        emissiveIntensity: 0.5,
        metalness: 0.8,
        roughness: 0.1,
      });
      const ee = new THREE.Mesh(eeGeo, eeMat);
      ee.position.copy(pos);
      group.add(ee);

      // TCP coordinate frame arrows (tiny axes)
      const axLen = 20;
      const addAxis = (dir3, color) => {
        const geo = new THREE.CylinderGeometry(0.6, 0.6, axLen, 8);
        const mat = new THREE.MeshBasicMaterial({ color });
        const mesh = new THREE.Mesh(geo, mat);
        mesh.position.copy(pos.clone().add(dir3.clone().multiplyScalar(axLen / 2)));
        mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0,1,0), dir3);
        group.add(mesh);
      };
      addAxis(new THREE.Vector3(1,0,0), 0xef4444);
      addAxis(new THREE.Vector3(0,1,0), 0x22c55e);
      addAxis(new THREE.Vector3(0,0,1), 0x3b82f6);
    }

    prevPos.copy(pos);
  }
}

/** Target marker: wire sphere + crosshair lines. */
function buildTargetMarker() {
  const group = new THREE.Group();

  // Wire sphere
  const geo = new THREE.SphereGeometry(8, 12, 8);
  const mat = new THREE.MeshBasicMaterial({ color: 0xf59e0b, wireframe: true, transparent: true, opacity: 0.5 });
  group.add(new THREE.Mesh(geo, mat));

  // Center dot
  const dotGeo = new THREE.SphereGeometry(2, 8, 8);
  const dotMat = new THREE.MeshBasicMaterial({ color: 0xf59e0b });
  group.add(new THREE.Mesh(dotGeo, dotMat));

  group.visible = false;
  return group;
}

/** XYZ axis markers at world origin (small arrows). */
function buildAxisMarkers() {
  const group = new THREE.Group();
  const len = 50;
  const r   = 0.8;

  const addAxis = (dir, color) => {
    const geo = new THREE.CylinderGeometry(r, r, len, 8);
    const mat = new THREE.MeshBasicMaterial({ color });
    const mesh = new THREE.Mesh(geo, mat);
    mesh.position.copy(dir.clone().multiplyScalar(len / 2));
    mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0,1,0), dir);
    group.add(mesh);
  };

  addAxis(new THREE.Vector3(1,0,0), 0xef4444);
  addAxis(new THREE.Vector3(0,1,0), 0x22c55e);
  addAxis(new THREE.Vector3(0,0,1), 0x3b82f6);
  group.position.y = 0;
  return group;
}
