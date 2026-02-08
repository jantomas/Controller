import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

// ── State ──────────────────────────────────────────────────────────────
let config = null;       // { Body, Rendering, Legs }
let legConfigs = [];
let selectedLeg = 0;
let mode = 'ik';

const legVisuals = [];
let bodyMesh = null;
let targetSphere = null;
const labelSprites = [];  // all 3D text labels

// ── Three.js Setup ─────────────────────────────────────────────────────
const container = document.getElementById('canvas-container');
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.setClearColor(0x0a0a1a);
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
container.appendChild(renderer.domElement);

const scene = new THREE.Scene();
scene.fog = new THREE.FogExp2(0x0a0a1a, 0.0006);

const camera = new THREE.PerspectiveCamera(50, window.innerWidth / window.innerHeight, 1, 5000);
camera.position.set(350, 300, 400);

const orbitControls = new OrbitControls(camera, renderer.domElement);
orbitControls.enableDamping = true;
orbitControls.dampingFactor = 0.08;
orbitControls.target.set(0, -20, 0);

// ── Lights ─────────────────────────────────────────────────────────────
scene.add(new THREE.AmbientLight(0x667788, 1.0));

const dirLight = new THREE.DirectionalLight(0xffffff, 1.2);
dirLight.position.set(250, 400, 200);
dirLight.castShadow = true;
dirLight.shadow.mapSize.set(2048, 2048);
const sc = dirLight.shadow.camera;
sc.near = 10; sc.far = 1200; sc.left = -400; sc.right = 400; sc.top = 400; sc.bottom = -400;
scene.add(dirLight);

const fillLight = new THREE.DirectionalLight(0x8899bb, 0.5);
fillLight.position.set(-200, 100, -150);
scene.add(fillLight);

scene.add(new THREE.HemisphereLight(0x88aaff, 0x443322, 0.4));

// ── Ground ─────────────────────────────────────────────────────────────
scene.add(new THREE.GridHelper(600, 30, 0x334455, 0x1a2233));

const groundMesh = new THREE.Mesh(
    new THREE.PlaneGeometry(800, 800),
    new THREE.ShadowMaterial({ opacity: 0.15 })
);
groundMesh.rotation.x = -Math.PI / 2;
groundMesh.position.y = -0.5;
groundMesh.receiveShadow = true;
scene.add(groundMesh);

// ── Coordinate mapping: robot(X,Y,Z-up) → Three.js(X,Y-up,Z) ─────────
function R(rx, ry, rz) {
    return new THREE.Vector3(rx, rz, -ry);
}

// ── Axes (right-hand rule: X forward, Y left, Z up) ───────────────────
{
    const len = 80;
    const addAxis = (dir, color) => {
        const mat = new THREE.LineBasicMaterial({ color });
        const pts = [new THREE.Vector3(), dir.clone().multiplyScalar(len)];
        scene.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), mat));
    };
    const xDir = R(1, 0, 0); // robot +X  → red
    const yDir = R(0, 1, 0); // robot +Y  → green
    const zDir = R(0, 0, 1); // robot +Z  → blue
    addAxis(xDir, 0xff0000);
    addAxis(yDir, 0x00ff00);
    addAxis(zDir, 0x0000ff);
    makeTextSprite('X', xDir.clone().multiplyScalar(len + 10), '#ff6666', 48, 18);
    makeTextSprite('Y', yDir.clone().multiplyScalar(len + 10), '#66ff66', 48, 18);
    makeTextSprite('Z', zDir.clone().multiplyScalar(len + 10), '#6666ff', 48, 18);
}

// ── Colors ─────────────────────────────────────────────────────────────
const COL = {
    body:  0x3a4a5c,
    coxa:  0x44aa88,
    femur: 0x4488cc,
    tibia: 0x9966dd,
    joint: 0xddcc33,
    foot:  0xff6644,
    mount: 0x99aa66,
    target: 0x44ff88,
};

// ── Materials ──────────────────────────────────────────────────────────
const M = {
    body:   new THREE.MeshStandardMaterial({ color: COL.body,  roughness: 0.45, metalness: 0.3 }),
    coxa:   new THREE.MeshStandardMaterial({ color: COL.coxa,  roughness: 0.35, metalness: 0.2 }),
    femur:  new THREE.MeshStandardMaterial({ color: COL.femur, roughness: 0.35, metalness: 0.2 }),
    tibia:  new THREE.MeshStandardMaterial({ color: COL.tibia, roughness: 0.35, metalness: 0.2 }),
    joint:  new THREE.MeshStandardMaterial({ color: COL.joint, roughness: 0.3,  metalness: 0.5 }),
    foot:   new THREE.MeshStandardMaterial({ color: COL.foot,  roughness: 0.5,  metalness: 0.1 }),
    target: new THREE.MeshStandardMaterial({ color: COL.target, roughness: 0.3, transparent: true, opacity: 0.7 }),
    mount:  new THREE.MeshStandardMaterial({ color: COL.mount, roughness: 0.4,  metalness: 0.4 }),
    dim:    new THREE.MeshStandardMaterial({ color: 0x445566, roughness: 0.7, transparent: true, opacity: 0.25 }),
    dimJ:   new THREE.MeshStandardMaterial({ color: 0x888855, roughness: 0.7, transparent: true, opacity: 0.25 }),
};

// ── Build Scene ────────────────────────────────────────────────────────
function buildScene(cfg) {
    config = cfg;
    legConfigs = cfg.Legs;

    const br = cfg.Body.RadiusMm;
    const bt = cfg.Body.ThicknessMm;

    // — Body hexagonal plate —
    // The hexagon shape is defined by the 6 leg mount points at BodyRadius
    const shape = new THREE.Shape();
    for (let i = 0; i < 6; i++) {
        const a = legConfigs[i].MountAngleDeg * Math.PI / 180;
        const px = br * Math.cos(a), py = br * Math.sin(a);
        i === 0 ? shape.moveTo(px, py) : shape.lineTo(px, py);
    }
    shape.closePath();

    const bodyGeom = new THREE.ExtrudeGeometry(shape, {
        depth: bt, bevelEnabled: true, bevelThickness: 1.5, bevelSize: 1.5, bevelSegments: 3
    });
    bodyMesh = new THREE.Mesh(bodyGeom, M.body);
    // ExtrudeGeometry builds XY and extrudes along Z.
    // Rotate so robot XY plane becomes Three.js XZ (horizontal), robot Z → Three.js Y (up)
    bodyMesh.rotation.x = -Math.PI / 2;
    bodyMesh.position.y = -bt / 2;
    bodyMesh.castShadow = true;
    bodyMesh.receiveShadow = true;
    scene.add(bodyMesh);

    // "BODY" label at center
    makeTextSprite('BODY', R(0, 0, bt + 5), '#8899aa', 28, 14);

    // — Mount point markers + leg name labels —
    const mountR = cfg.Rendering.JointDiameterMm / 2;
    const mountGeom = new THREE.SphereGeometry(mountR, 14, 14);
    const mountMeshes = [];
    for (let li = 0; li < legConfigs.length; li++) {
        const leg = legConfigs[li];
        const a = leg.MountAngleDeg * Math.PI / 180;
        const mx = br * Math.cos(a), my = br * Math.sin(a);
        const m = new THREE.Mesh(mountGeom, M.mount.clone());
        m.position.copy(R(mx, my, 0));
        m.castShadow = true;
        scene.add(m);
        mountMeshes.push(m);

        // Leg name label offset outward from mount
        const labelDist = br + 25;
        makeTextSprite(
            leg.Name.replace('Front', 'F').replace('Middle', 'M').replace('Rear', 'R').replace('Right', 'R').replace('Left', 'L'),
            R(labelDist * Math.cos(a), labelDist * Math.sin(a), 15),
            '#aabbcc', 30, 12
        );
    }

    // — Selection ring (highlights the selected leg's mount) —
    const ringGeom = new THREE.RingGeometry(mountR + 2, mountR + 5, 24);
    const ringMat = new THREE.MeshBasicMaterial({ color: 0x44ff88, side: THREE.DoubleSide, transparent: true, opacity: 0.8 });
    const selectionRing = new THREE.Mesh(ringGeom, ringMat);
    selectionRing.rotation.x = -Math.PI / 2;
    scene.add(selectionRing);

    // Store for later highlighting
    window._mountMeshes = mountMeshes;
    window._selectionRing = selectionRing;
    updateSelectionHighlight();

    // — Legs —
    legVisuals.length = 0;
    for (let i = 0; i < legConfigs.length; i++) {
        legVisuals.push(createLeg(cfg.Rendering, i));
    }

    // — Target sphere —
    targetSphere = new THREE.Mesh(
        new THREE.SphereGeometry(cfg.Rendering.FootDiameterMm / 2 + 2, 20, 20),
        M.target
    );
    targetSphere.visible = mode === 'ik';
    scene.add(targetSphere);

    // Init
    initAllLegs();
}

// ── Create one leg's 3D objects ────────────────────────────────────────
function createLeg(r, legIdx) {
    const cR = r.CoxaDiameterMm / 2;
    const fR = r.FemurDiameterMm / 2;
    const tR = r.TibiaDiameterMm / 2;
    const jR = r.JointDiameterMm / 2;
    const footR = r.FootDiameterMm / 2;

    // Joint spheres
    const coxaJoint  = makeSphere(jR, M.joint);
    const femurJoint = makeSphere(jR, M.joint);
    const tibiaJoint = makeSphere(jR * 0.85, M.joint);
    const foot       = makeSphere(footR, M.foot);

    // Segment cylinders (initial placeholder length=1)
    const coxaSeg  = makeCyl(cR, 1, M.coxa);
    const femurSeg = makeCyl(fR, 1, M.femur);
    const tibiaSeg = makeCyl(tR, 1, M.tibia);

    scene.add(coxaJoint, femurJoint, tibiaJoint, foot, coxaSeg, femurSeg, tibiaSeg);

    // Floating labels for active leg segments (only shown for selected leg)
    const lblCoxa  = makeTextSprite('Coxa',  new THREE.Vector3(), '#44aa88', 22, 9);
    const lblFemur = makeTextSprite('Femur', new THREE.Vector3(), '#4488cc', 22, 9);
    const lblTibia = makeTextSprite('Tibia', new THREE.Vector3(), '#9966dd', 22, 9);
    const lblFoot  = makeTextSprite('Foot',  new THREE.Vector3(), '#ff6644', 22, 9);

    return {
        coxaJoint, femurJoint, tibiaJoint, foot,
        coxaSeg, femurSeg, tibiaSeg,
        lblCoxa, lblFemur, lblTibia, lblFoot,
        cR, fR, tR,
        _lastJoints: null
    };
}

function makeSphere(r, mat) {
    const m = new THREE.Mesh(new THREE.SphereGeometry(r, 14, 14), mat.clone());
    m.castShadow = true;
    return m;
}

function makeCyl(r, len, mat) {
    const m = new THREE.Mesh(new THREE.CylinderGeometry(r, r, len, 10), mat.clone());
    m.castShadow = true;
    return m;
}

function placeCyl(mesh, from, to, radius) {
    const dir = new THREE.Vector3().subVectors(to, from);
    const len = dir.length();
    if (len < 0.01) { mesh.visible = false; return; }
    mesh.visible = true;
    mesh.geometry.dispose();
    mesh.geometry = new THREE.CylinderGeometry(radius, radius, len, 10);
    mesh.position.lerpVectors(from, to, 0.5);
    mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir.normalize());
}

// ── Selection highlight ────────────────────────────────────────────────
function updateSelectionHighlight() {
    if (!window._selectionRing || !window._mountMeshes) return;
    // Position ring at selected leg's mount point
    const mp = window._mountMeshes[selectedLeg];
    if (mp) {
        window._selectionRing.position.copy(mp.position);
        window._selectionRing.position.y += 1; // slightly above body surface
    }
    // Highlight mount sphere: selected = bright, others = normal
    for (let i = 0; i < window._mountMeshes.length; i++) {
        const active = i === selectedLeg;
        window._mountMeshes[i].material.color.setHex(active ? 0x44ff88 : COL.mount);
        window._mountMeshes[i].material.emissive = new THREE.Color(active ? 0x226644 : 0x000000);
    }
}

// ── Update one leg's visuals from joint positions ──────────────────────
function updateLeg(idx, joints) {
    const v = legVisuals[idx];
    if (!v || !joints) return;
    v._lastJoints = joints;

    const active = idx === selectedLeg;

    // Convert robot coords → Three.js
    const pC  = R(joints.CoxaJoint.X,  joints.CoxaJoint.Y,  joints.CoxaJoint.Z);
    const pF  = R(joints.FemurJoint.X, joints.FemurJoint.Y, joints.FemurJoint.Z);
    const pT  = R(joints.TibiaJoint.X, joints.TibiaJoint.Y, joints.TibiaJoint.Z);
    const pFt = R(joints.Foot.X,       joints.Foot.Y,       joints.Foot.Z);

    // Position joints
    v.coxaJoint.position.copy(pC);
    v.femurJoint.position.copy(pF);
    v.tibiaJoint.position.copy(pT);
    v.foot.position.copy(pFt);

    // Position segments
    placeCyl(v.coxaSeg,  pC, pF,  v.cR);
    placeCyl(v.femurSeg, pF, pT,  v.fR);
    placeCyl(v.tibiaSeg, pT, pFt, v.tR);

    // Materials: active vs dimmed
    v.coxaJoint.material  = active ? M.joint : M.dimJ;
    v.femurJoint.material = active ? M.joint : M.dimJ;
    v.tibiaJoint.material = active ? M.joint : M.dimJ;
    v.foot.material       = active ? M.foot  : M.dimJ;
    v.coxaSeg.material    = active ? M.coxa  : M.dim;
    v.femurSeg.material   = active ? M.femur : M.dim;
    v.tibiaSeg.material   = active ? M.tibia : M.dim;

    // Segment labels — visible only on the active leg
    const labelOffset = new THREE.Vector3(0, 8, 0);
    v.lblCoxa.position.lerpVectors(pC, pF, 0.5).add(labelOffset);
    v.lblFemur.position.lerpVectors(pF, pT, 0.5).add(labelOffset);
    v.lblTibia.position.lerpVectors(pT, pFt, 0.5).add(labelOffset);
    v.lblFoot.position.copy(pFt).add(new THREE.Vector3(0, -10, 0));

    v.lblCoxa.visible  = active;
    v.lblFemur.visible = active;
    v.lblTibia.visible = active;
    v.lblFoot.visible  = active;
}

// ── Init all legs with default standing pose via FK ────────────────────
async function initAllLegs() {
    // Use FK with a natural standing pose: coxa=0°, femur=0°, tibia=90°
    const defaultCoxa = 0, defaultFemur = 0, defaultTibia = 90;
    for (let i = 0; i < legConfigs.length; i++) {
        try {
            const d = await callFK(i, defaultCoxa, defaultFemur, defaultTibia);
            if (d.Joints) updateLeg(i, d.Joints);
        } catch (e) { console.warn(`Leg ${i}:`, e); }
    }
    setDefaultInputs(selectedLeg);
}

async function setDefaultInputs(idx) {
    // Use FK to get the default foot position for this leg
    const defaultCoxa = 0, defaultFemur = 0, defaultTibia = 90;
    try {
        const d = await callFK(idx, defaultCoxa, defaultFemur, defaultTibia);
        if (d.FootMm) {
            document.getElementById('target-x').value = Math.round(d.FootMm.X);
            document.getElementById('target-y').value = Math.round(d.FootMm.Y);
            document.getElementById('target-z').value = Math.round(d.FootMm.Z);
        }
    } catch (e) {
        console.warn('setDefaultInputs FK failed:', e);
    }
    syncSliders({ CoxaDeg: defaultCoxa, FemurDeg: defaultFemur, TibiaDeg: defaultTibia });
    updateTargetSphere();
}

// ── API ────────────────────────────────────────────────────────────────
async function callIK(id, x, y, z) {
    return (await fetch('/api/ik', { method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ LegId: id, X: x, Y: y, Z: z }) })).json();
}
async function callFK(id, c, f, t) {
    return (await fetch('/api/fk', { method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ LegId: id, CoxaDeg: c, FemurDeg: f, TibiaDeg: t }) })).json();
}

// ── Solve IK ───────────────────────────────────────────────────────────
window.solveIK = async function () {
    const x = +document.getElementById('target-x').value;
    const y = +document.getElementById('target-y').value;
    const z = +document.getElementById('target-z').value;
    updateTargetSphere();
    const el = document.getElementById('status');
    try {
        const d = await callIK(selectedLeg, x, y, z);
        if (d.Reachable) {
            updateLeg(selectedLeg, d.Joints);
            el.className = 'status-ok';
            el.textContent = `✓ Reachable\nCoxa: ${d.Angles.CoxaDeg.toFixed(1)}°  Femur: ${d.Angles.FemurDeg.toFixed(1)}°  Tibia: ${d.Angles.TibiaDeg.toFixed(1)}°`;
            syncSliders(d.Angles);
        } else {
            el.className = 'status-error';
            el.textContent = `✗ ${d.Error || 'Unreachable'}`;
        }
    } catch (e) { el.className = 'status-error'; el.textContent = `✗ ${e.message}`; }
};

// ── Solve FK ───────────────────────────────────────────────────────────
window.solveFK = async function () {
    const c = +document.getElementById('angle-coxa').value;
    const f = +document.getElementById('angle-femur').value;
    const t = +document.getElementById('angle-tibia').value;
    const el = document.getElementById('status');
    try {
        const d = await callFK(selectedLeg, c, f, t);
        if (d.Joints) {
            updateLeg(selectedLeg, d.Joints);
            el.className = 'status-ok';
            el.textContent = `✓ Foot\nX: ${d.FootMm.X.toFixed(1)}  Y: ${d.FootMm.Y.toFixed(1)}  Z: ${d.FootMm.Z.toFixed(1)} mm`;
            document.getElementById('target-x').value = Math.round(d.FootMm.X);
            document.getElementById('target-y').value = Math.round(d.FootMm.Y);
            document.getElementById('target-z').value = Math.round(d.FootMm.Z);
            updateTargetSphere();
        }
    } catch (e) { el.className = 'status-error'; el.textContent = `✗ ${e.message}`; }
};

function syncSliders(a) {
    document.getElementById('angle-coxa').value = a.CoxaDeg;
    document.getElementById('angle-femur').value = a.FemurDeg;
    document.getElementById('angle-tibia').value = a.TibiaDeg;
    document.getElementById('val-coxa').textContent = a.CoxaDeg.toFixed(1) + '°';
    document.getElementById('val-femur').textContent = a.FemurDeg.toFixed(1) + '°';
    document.getElementById('val-tibia').textContent = a.TibiaDeg.toFixed(1) + '°';
}

window.updateFk = function () {
    document.getElementById('val-coxa').textContent = (+document.getElementById('angle-coxa').value).toFixed(1) + '°';
    document.getElementById('val-femur').textContent = (+document.getElementById('angle-femur').value).toFixed(1) + '°';
    document.getElementById('val-tibia').textContent = (+document.getElementById('angle-tibia').value).toFixed(1) + '°';
};

window.setMode = function (m) {
    mode = m;
    document.querySelectorAll('.mode-tab').forEach(t => t.classList.toggle('active', t.dataset.mode === m));
    document.getElementById('ik-controls').style.display = m === 'ik' ? 'block' : 'none';
    document.getElementById('fk-controls').style.display = m === 'fk' ? 'block' : 'none';
    if (targetSphere) targetSphere.visible = m === 'ik';
};

function updateTargetSphere() {
    if (!targetSphere) return;
    targetSphere.position.copy(R(
        +document.getElementById('target-x').value || 0,
        +document.getElementById('target-y').value || 0,
        +document.getElementById('target-z').value || 0
    ));
}

window.resetView = function () {
    camera.position.set(350, 300, 400);
    orbitControls.target.set(0, -20, 0);
    orbitControls.update();
};

// ── 3D text sprite helper ──────────────────────────────────────────────
function makeTextSprite(text, pos, color, fontSize, scale) {
    const canvas = document.createElement('canvas');
    const ctx = canvas.getContext('2d');
    const font = `bold ${fontSize || 32}px 'Segoe UI', sans-serif`;
    ctx.font = font;
    const textWidth = ctx.measureText(text).width;
    canvas.width = Math.ceil(textWidth) + 12;
    canvas.height = (fontSize || 32) + 12;
    ctx.font = font;  // re-set after resize
    ctx.fillStyle = color || '#ffffff';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(text, canvas.width / 2, canvas.height / 2);

    const tex = new THREE.CanvasTexture(canvas);
    tex.minFilter = THREE.LinearFilter;
    const mat = new THREE.SpriteMaterial({ map: tex, depthTest: false, transparent: true });
    const sprite = new THREE.Sprite(mat);
    if (pos) sprite.position.copy(pos);
    const s = scale || 14;
    sprite.scale.set(s * (canvas.width / canvas.height), s, 1);
    scene.add(sprite);
    labelSprites.push(sprite);
    return sprite;
}

// ── Leg selector ───────────────────────────────────────────────────────
function populateLegSelect() {
    const sel = document.getElementById('leg-select');
    sel.innerHTML = '';
    legConfigs.forEach((leg, i) => {
        const o = document.createElement('option');
        o.value = i;
        o.textContent = `${i}: ${leg.Name}`;
        sel.appendChild(o);
    });
    sel.addEventListener('change', async (e) => {
        selectedLeg = parseInt(e.target.value);
        // Re-apply materials for all legs (active vs dimmed)
        for (let i = 0; i < legConfigs.length; i++) {
            if (legVisuals[i]?._lastJoints) updateLeg(i, legVisuals[i]._lastJoints);
        }
        updateSelectionHighlight();
        setDefaultInputs(selectedLeg);
    });
}

// ── Events ─────────────────────────────────────────────────────────────
document.addEventListener('keydown', (e) => { if (e.key === 'Enter' && mode === 'ik') solveIK(); });

window.addEventListener('resize', () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
});

// ── Render ─────────────────────────────────────────────────────────────
(function animate() {
    requestAnimationFrame(animate);
    orbitControls.update();
    renderer.render(scene, camera);
})();

// ── Init ───────────────────────────────────────────────────────────────
(async function init() {
    try {
        const cfg = await (await fetch('/api/config')).json();
        config = cfg;
        legConfigs = cfg.Legs;
        populateLegSelect();
        buildScene(cfg);

        // FK slider limits from config
        const lim = legConfigs[0];
        document.getElementById('angle-coxa').min  = lim.CoxaLimits.MinDeg;
        document.getElementById('angle-coxa').max  = lim.CoxaLimits.MaxDeg;
        document.getElementById('angle-femur').min = lim.FemurLimits.MinDeg;
        document.getElementById('angle-femur').max = lim.FemurLimits.MaxDeg;
        document.getElementById('angle-tibia').min = lim.TibiaLimits.MinDeg;
        document.getElementById('angle-tibia').max = lim.TibiaLimits.MaxDeg;

        document.getElementById('status').className = 'status-ok';
        document.getElementById('status').textContent =
            `Body: ⌀${cfg.Body.RadiusMm * 2}mm  h=${cfg.Body.ThicknessMm}mm\n` +
            `Coxa: ${lim.CoxaLengthMm}mm  Femur: ${lim.FemurLengthMm}mm  Tibia: ${lim.TibiaLengthMm}mm`;
    } catch (e) {
        document.getElementById('status').className = 'status-error';
        document.getElementById('status').textContent = `Failed: ${e.message}`;
    }
})();
