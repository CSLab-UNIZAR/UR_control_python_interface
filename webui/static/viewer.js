// 3D view of the Campero + UR10, built from robot/model.json (the URDF used on
// the robot, with the arm calibration) and robot/meshes.glb. Everything is
// served locally, so it works without internet. It renders only on changes.
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { GLTFLoader } from "three/addons/loaders/GLTFLoader.js";
import { CSS2DObject, CSS2DRenderer } from "three/addons/renderers/CSS2DRenderer.js";

const AXIS_COLORS = [0xe5484d, 0x30a46c, 0x3e63dd];
const ARM_PREFIXES = ["campero_ur10_", "campero_robotiq_", "campero_sensor_", "tool0_gripper", "gripper_aruco"];
const VIEWS = {   // camera offsets from the target, in the Campero frame (x forward, z up)
  iso: [-1.7, -2.6, 1.5],
  top: [0, -0.01, 3.6],
  side: [0, -3.4, 0.4],
  rear: [-3.4, 0, 0.8],
};

function cssColor(name) {
  return getComputedStyle(document.documentElement).getPropertyValue(name).trim() || "#888";
}

function setOrigin(obj, o) {   // o = [x, y, z, qx, qy, qz, qw]
  obj.position.set(o[0], o[1], o[2]);
  obj.quaternion.set(o[3], o[4], o[5], o[6]);
}

function isArmPart(linkName) {
  return ARM_PREFIXES.some((p) => linkName.startsWith(p));
}

export class RobotViewer {
  constructor(container, cfg) {
    this.container = container;
    this.cfg = cfg;
    this.links = {};
    this.joints = {};
    this.options = { campero: true, workspace: true, labels: true, jointFrames: false, linkFrames: false };
    this.dirty = true;
    this.lastKey = "";

    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    container.append(this.renderer.domElement);
    this.labels = new CSS2DRenderer();
    this.labels.domElement.className = "viewer-labels";
    container.append(this.labels.domElement);

    this.scene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(40, 1, 0.02, 60);
    this.camera.up.set(0, 0, 1);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.addEventListener("change", () => this.requestRender());

    this.scene.add(new THREE.HemisphereLight(0xffffff, 0x404040, 1.6));
    const key = new THREE.DirectionalLight(0xffffff, 1.8);
    key.position.set(2, -3, 5);
    const fill = new THREE.DirectionalLight(0xffffff, 0.6);
    fill.position.set(-3, 2, 2);
    this.scene.add(key, fill);
    this.grid = new THREE.GridHelper(8, 32);
    this.grid.rotation.x = Math.PI / 2;   // three's grid lies in XZ; ROS is Z-up
    this.scene.add(this.grid);

    new ResizeObserver(() => this.resize()).observe(container);
    window.matchMedia("(prefers-color-scheme: dark)").addEventListener("change", () => this.applyTheme());
    this.applyTheme();
    const loop = () => {
      if (this.dirty) {
        this.dirty = false;
        this.renderer.render(this.scene, this.camera);
        this.labels.render(this.scene, this.camera);
      }
      requestAnimationFrame(loop);
    };
    loop();
  }

  requestRender() {
    this.dirty = true;
  }

  applyTheme() {
    this.scene.background = new THREE.Color(cssColor("--viewer-bg"));
    const grid = new THREE.Color(cssColor("--viewer-grid"));
    this.grid.material.color = grid;
    this.grid.material.vertexColors = false;
    this.grid.material.needsUpdate = true;
    this.requestRender();
  }

  resize() {
    const w = this.container.clientWidth, h = this.container.clientHeight;
    if (!w || !h) return;
    this.renderer.setSize(w, h);
    this.labels.setSize(w, h);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
    this.requestRender();
  }

  async load() {
    const [model, gltf] = await Promise.all([
      fetch("robot/model.json").then((r) => r.json()),
      new GLTFLoader().loadAsync("robot/meshes.glb"),
    ]);
    this.model = model;
    this.buildTree(model);
    this.addVisuals(model, gltf.scene);
    this.addFrames();
    this.addWorkspace();
    this.arrow = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(), 0.22,
      new THREE.Color(cssColor("--accent")), 0.06, 0.035);
    this.arrow.visible = false;
    this.scene.add(this.arrow);
    this.ghost = this.makeGhost();
    const mimicked = new Set(model.joints.filter((j) => j.mimic).map((j) => j.mimic.joint));
    this.gripperJoint = model.joints.find((j) => mimicked.has(j.name));
    this.setView("iso");
    this.setOptions(this.options);
  }

  buildTree(model) {
    for (const name of Object.keys(model.links)) {
      const link = new THREE.Group();
      link.name = name;
      this.links[name] = link;
    }
    for (const j of model.joints) {
      const origin = new THREE.Group();
      setOrigin(origin, j.origin);
      const motion = new THREE.Group();
      motion.name = "joint:" + j.name;
      this.links[j.parent].add(origin);
      origin.add(motion);
      motion.add(this.links[j.child]);
      this.joints[j.name] = { def: j, motion, axis: new THREE.Vector3(...j.axis).normalize() };
    }
    this.root = this.links[model.root];
    this.scene.add(this.root);
  }

  addVisuals(model, meshScene) {
    const fallback = new THREE.Color(0.7, 0.7, 0.7);
    for (const [name, link] of Object.entries(model.links)) {
      for (const v of link.visuals) {
        const group = new THREE.Group();
        setOrigin(group, v.origin);
        group.userData.visual = isArmPart(name) ? "arm" : "campero";
        const color = v.color ? new THREE.Color(v.color[0], v.color[1], v.color[2]) : fallback;
        const plain = new THREE.MeshStandardMaterial({ color, roughness: 0.6, metalness: 0.1 });
        if (v.nodes) {
          for (const nodeName of v.nodes) {
            const node = meshScene.getObjectByName(nodeName);
            if (!node) continue;
            node.traverse((obj) => {
              if (!obj.isMesh) return;
              if (!obj.geometry.attributes.normal) obj.geometry.computeVertexNormals();
              if (!v.own_material) obj.material = plain;
              else {
                obj.material.metalness = Math.min(obj.material.metalness ?? 0, 0.2);
                obj.material.roughness = Math.max(obj.material.roughness ?? 0.6, 0.45);
              }
            });
            group.add(node);
          }
        } else {
          let geometry;
          if (v.box) geometry = new THREE.BoxGeometry(...v.box);
          else if (v.sphere) geometry = new THREE.SphereGeometry(v.sphere, 20, 14);
          else if (v.cylinder) geometry = new THREE.CylinderGeometry(v.cylinder[0], v.cylinder[0], v.cylinder[1], 24)
            .rotateX(Math.PI / 2);   // URDF cylinders are along Z
          if (geometry) group.add(new THREE.Mesh(geometry, plain));
        }
        this.links[name].add(group);
      }
    }
  }

  makeAxes(size, label) {
    const group = new THREE.Group();
    group.userData.ghostHide = true;
    const radius = Math.max(size * 0.03, 0.003);
    AXIS_COLORS.forEach((color, i) => {
      const shaft = new THREE.Mesh(
        new THREE.CylinderGeometry(radius, radius, size, 10).translate(0, size / 2, 0),
        new THREE.MeshBasicMaterial({ color, depthTest: false, transparent: true }));
      if (i === 0) shaft.rotation.z = -Math.PI / 2;
      if (i === 2) shaft.rotation.x = Math.PI / 2;
      shaft.renderOrder = 10;
      group.add(shaft);
    });
    if (label) {
      const div = document.createElement("div");
      div.className = "frame-label";
      div.textContent = label;
      const tag = new CSS2DObject(div);
      tag.position.set(0, 0, size * 0.25);
      tag.userData.frameLabel = true;
      group.add(tag);
    }
    return group;
  }

  addFrames() {
    const cfg = this.cfg;
    this.fixedFrames = [
      [this.links[cfg.base_frame], this.makeAxes(0.25, "UR base (pendant Base)")],
      [this.links[cfg.flange_frame], this.makeAxes(0.09, "flange")],
    ];
    const campero = this.links["campero_base_link"];
    if (campero) this.fixedFrames.push([campero, this.makeAxes(0.35, "Campero base_link")]);
    for (const [parent, axes] of this.fixedFrames) parent.add(axes);
    this.camperoFrame = campero ? this.fixedFrames[this.fixedFrames.length - 1][1] : null;

    const [x, y, z, rx, ry, rz] = cfg.tcp;
    this.tcp = new THREE.Group();
    this.tcp.position.set(x, y, z);
    const rot = new THREE.Vector3(rx, ry, rz);
    if (rot.length() > 0) this.tcp.quaternion.setFromAxisAngle(rot.clone().normalize(), rot.length());
    this.tcp.add(this.makeAxes(0.12, "TCP"));
    this.links[cfg.flange_frame].add(this.tcp);

    this.jointFrames = cfg.arm_joints.map((name, i) => {
      const axes = this.makeAxes(0.13, `J${i + 1}`);
      this.links[this.joints[name].def.child].add(axes);
      return axes;
    });
    this.linkFrames = Object.values(this.links).map((link) => {
      const axes = this.makeAxes(0.06);
      link.add(axes);
      return axes;
    });
  }

  addWorkspace() {
    const w = this.cfg.workspace;
    const size = ["x", "y", "z"].map((a) => w[a][1] - w[a][0]);
    const center = ["x", "y", "z"].map((a) => (w[a][0] + w[a][1]) / 2);
    const box = new THREE.BoxGeometry(...size);
    this.workspaceFill = new THREE.MeshBasicMaterial({ transparent: true, opacity: 0.06, depthWrite: false });
    this.workspaceEdge = new THREE.LineBasicMaterial();
    this.workspace = new THREE.Group();
    this.workspace.add(new THREE.Mesh(box, this.workspaceFill), new THREE.LineSegments(new THREE.EdgesGeometry(box), this.workspaceEdge));
    this.workspace.position.set(...center);
    this.workspace.userData.ghostHide = true;
    this.links[this.cfg.base_frame].add(this.workspace);
    this.setWorkspaceState(true);
  }

  setWorkspaceState(inside) {
    const color = new THREE.Color(cssColor(inside ? "--accent" : "--warn"));
    this.workspaceFill.color = color;
    this.workspaceEdge.color = color;
  }

  makeGhost() {
    const material = new THREE.MeshBasicMaterial({
      color: new THREE.Color(cssColor("--accent")), transparent: true, opacity: 0.22, depthWrite: false });
    const ghost = this.root.clone(true);
    ghost.traverse((obj) => {
      if (obj.isMesh) obj.material = material;
      if (obj.userData.visual === "campero" || obj.userData.ghostHide) obj.visible = false;
    });
    ghost.visible = false;
    this.scene.add(ghost);
    return ghost;
  }

  setJoints(root, values) {
    for (const { def, axis } of Object.values(this.joints)) {
      if (def.type === "fixed") continue;
      let value = values[def.name];
      if (def.mimic && values[def.mimic.joint] !== undefined) {
        value = def.mimic.multiplier * values[def.mimic.joint] + def.mimic.offset;
      }
      if (value === undefined) continue;
      const motion = root === this.root ? this.joints[def.name].motion : root.getObjectByName("joint:" + def.name);
      if (def.type === "prismatic") motion.position.copy(axis).multiplyScalar(value);
      else motion.quaternion.setFromAxisAngle(axis, value);
    }
  }

  jointValues(q, closed) {
    const values = {};
    this.cfg.arm_joints.forEach((name, i) => { values[name] = q[i]; });
    if (this.gripperJoint && closed !== null && closed !== undefined) {
      values[this.gripperJoint.name] = closed * (this.gripperJoint.limit ? this.gripperJoint.limit[1] : 0.8);
    }
    return values;
  }

  // q: J1..J6 [rad]; gripperClosed: 0..1 or null; jog: {axes, space, frame}; inside: TCP within the box
  update({ q, gripperClosed, jog, inside }) {
    if (!this.root || !q) return;
    const key = JSON.stringify([q.map((v) => v.toFixed(4)), gripperClosed, jog, inside]);
    if (key === this.lastKey) return;
    this.lastKey = key;
    this.setJoints(this.root, this.jointValues(q, gripperClosed));
    this.setWorkspaceState(inside);
    this.updateArrow(jog);
    this.requestRender();
  }

  updateArrow(jog) {
    const axes = jog && jog.space === "cartesian" ? jog.axes : null;
    const linear = axes ? new THREE.Vector3(axes[0], axes[1], axes[2]) : new THREE.Vector3();
    const angular = axes ? new THREE.Vector3(axes[3], axes[4], axes[5]) : new THREE.Vector3();
    const dir = linear.lengthSq() > 0 ? linear : angular;
    if (dir.lengthSq() === 0) { this.arrow.visible = false; return; }
    this.scene.updateMatrixWorld(true);
    const frame = jog.frame === "tool" ? this.tcp : this.links[this.cfg.base_frame];
    dir.normalize().applyQuaternion(frame.getWorldQuaternion(new THREE.Quaternion()));
    this.arrow.position.copy(this.tcp.getWorldPosition(new THREE.Vector3()));
    this.arrow.setDirection(dir);
    this.arrow.setColor(new THREE.Color(cssColor(linear.lengthSq() > 0 ? "--accent" : "--warn")));
    this.arrow.visible = true;
  }

  // Translucent copy of the arm at a move target (q in rad), or null to hide it
  setGhost(q) {
    if (!this.ghost) return;
    this.ghost.visible = Boolean(q);
    if (q) this.setJoints(this.ghost, this.jointValues(q, null));
    this.requestRender();
  }

  setOptions(options) {
    Object.assign(this.options, options);
    const o = this.options;
    this.root.traverse((obj) => {
      if (obj.userData.visual === "campero") obj.visible = o.campero;
    });
    if (this.camperoFrame) this.camperoFrame.visible = o.campero;
    this.workspace.visible = o.workspace;
    this.jointFrames.forEach((f) => { f.visible = o.jointFrames; });
    this.linkFrames.forEach((f) => { f.visible = o.linkFrames; });
    this.scene.traverse((obj) => {
      if (obj.userData.frameLabel) obj.element.style.display = o.labels ? "" : "none";
    });
    this.requestRender();
  }

  setView(name) {
    this.scene.updateMatrixWorld(true);
    const w = this.cfg.workspace;
    const center = new THREE.Vector3((w.x[0] + w.x[1]) / 2, (w.y[0] + w.y[1]) / 2, (w.z[0] + w.z[1]) / 2);
    const base = this.links[this.cfg.base_frame];
    const target = base.localToWorld(center.multiplyScalar(0.5));   // between the arm base and the workspace
    this.controls.target.copy(target);
    this.camera.position.copy(target).add(new THREE.Vector3(...VIEWS[name]));
    this.controls.update();
    this.requestRender();
  }
}
