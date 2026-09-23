"""Build the 3D model of the web panel from the Campero catkin workspace.

Expands the robot xacro (as campero_ur10_bringup.launch does, including the
arm calibration), then writes for the browser:

  webui/static/robot/model.json   kinematic tree: links, visuals, joints
  webui/static/robot/meshes.glb   every visual mesh, one node per mesh

Only needed when the robot description changes. Requires (not part of
requirements.txt):  pip install xacro==2.1.1 trimesh pycollada

Usage (from the repository root):
  python tools/build_robot_model.py --src <catkin_ws>/src
"""

import argparse
import io
import json
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import trimesh
import xacro
import xacro.substitution_args
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_XACRO = "campero_robot_real/campero_robot_real_description/urdf/campero_DLO.urdf.xacro"
PACKAGE_URI = re.compile(r"package://([^/]+)/(.+)")


def find_packages(src):
    """Map ROS package name -> directory, from the package.xml files under src."""
    packages = {}
    for manifest in Path(src).resolve().rglob("package.xml"):
        name = ET.parse(manifest).getroot().findtext("name")
        if name and "CATKIN_IGNORE" not in {p.name for p in manifest.parent.iterdir()}:
            packages.setdefault(name.strip(), manifest.parent)
    return packages


def expand_xacro(path, packages, mappings):
    def find(pkg):
        if pkg not in packages:
            raise SystemExit(f"xacro needs package '{pkg}', which is not in the source tree")
        return packages[pkg].as_posix()   # forward slashes: the path ends up inside Python expressions

    original_parse = xacro.parse

    def parse(inp, filename=None):
        # fmauch's ur_description names a property '__kinematics', which ROS 1
        # xacro accepted and xacro >= 2 rejects: rename it while reading.
        if inp is None and filename and Path(xacro.root_dir, filename).is_file():
            inp = Path(xacro.root_dir, filename).read_text(encoding="utf-8")
            inp = inp.replace("__kinematics", "ur_kinematics")
        return original_parse(inp, filename)

    xacro.substitution_args._eval_find = find
    xacro.parse = parse
    return xacro.process_file(str(path), mappings=mappings).toxml()


def origin(elem):
    """URDF <origin> -> [x, y, z, qx, qy, qz, qw]."""
    xyz, rpy = [0.0] * 3, [0.0] * 3
    if elem is not None:
        xyz = [float(v) for v in elem.get("xyz", "0 0 0").split()]
        rpy = [float(v) for v in elem.get("rpy", "0 0 0").split()]
    return [round(v, 9) for v in xyz + list(Rotation.from_euler("xyz", rpy).as_quat())]


def color_of(material, named):
    if material is None:
        return None
    rgba = material.find("color")
    if rgba is not None:
        return [float(v) for v in rgba.get("rgba").split()]
    return named.get(material.get("name"))


def load_mesh(path, scale):
    """All geometries of a mesh file, in the file's own frame and in metres."""
    unit = 1.0
    if path.suffix.lower() == ".dae":
        data = path.read_bytes()
        # Some exported COLLADA files carry '<' / '>' inside attribute values
        # (e.g. id="<STL_BINARY>"): invalid XML that RViz tolerates.
        data = re.sub(rb'="([^"]*)"', lambda m: b'="' + re.sub(rb"[<>]", b"", m.group(1)) + b'"', data)
        # trimesh does not apply the COLLADA <unit>
        match = re.search(rb'<unit[^>]*meter="([0-9.eE+-]+)"', data[:4000])
        unit = float(match.group(1)) if match else 1.0
        scene = trimesh.load(io.BytesIO(data), file_type="dae", force="scene", process=False,
                             resolver=trimesh.resolvers.FilePathResolver(str(path)))
    else:
        scene = trimesh.load(str(path), force="scene", process=False)
    meshes = []
    for node in scene.graph.nodes_geometry:
        transform, name = scene.graph[node]
        mesh = scene.geometry[name].copy()
        mesh.apply_transform(transform)
        mesh.apply_scale(np.asarray(scale) * unit)
        meshes.append(mesh)
    return meshes


def build(src, xacro_file, out_dir):
    packages = find_packages(src)
    urdf = ET.fromstring(expand_xacro(Path(src) / xacro_file, packages,
                                      {"transmission_hw_interface": "hardware_interface/PositionJointInterface"}))
    named = {m.get("name"): color_of(m, {}) for m in urdf.findall("material") if m.find("color") is not None}

    glb = trimesh.Scene()
    cache, links, counter = {}, {}, 0
    for link in urdf.findall("link"):
        visuals = []
        for visual in link.findall("visual"):
            geom = visual.find("geometry")[0]
            entry = {"origin": origin(visual.find("origin")), "color": color_of(visual.find("material"), named)}
            if geom.tag == "mesh":
                pkg, rel = PACKAGE_URI.match(geom.get("filename")).groups()
                scale = tuple(float(v) for v in geom.get("scale", "1 1 1").split())
                key = (pkg, rel, scale)
                if key not in cache:
                    try:
                        cache[key] = load_mesh(packages[pkg] / rel, scale)
                    except Exception as exc:   # e.g. malformed COLLADA that RViz tolerates
                        print(f"warning: skipping {pkg}/{rel} ({type(exc).__name__}: {exc})")
                        cache[key] = []
                entry["nodes"] = []
                entry["own_material"] = rel.lower().endswith(".dae")   # STL: painted with the URDF color
                for mesh in cache[key]:
                    name = f"m{counter}"
                    counter += 1
                    glb.add_geometry(mesh, node_name=name, geom_name=name)
                    entry["nodes"].append(name)
            elif geom.tag == "box":
                entry["box"] = [float(v) for v in geom.get("size").split()]
            elif geom.tag == "cylinder":
                entry["cylinder"] = [float(geom.get("radius")), float(geom.get("length"))]
            elif geom.tag == "sphere":
                entry["sphere"] = float(geom.get("radius"))
            visuals.append(entry)
        links[link.get("name")] = {"visuals": visuals}

    joints = []
    for joint in urdf.findall("joint"):
        axis = joint.find("axis")
        mimic = joint.find("mimic")
        limit = joint.find("limit")
        joints.append({
            "name": joint.get("name"),
            "type": joint.get("type"),
            "parent": joint.find("parent").get("link"),
            "child": joint.find("child").get("link"),
            "origin": origin(joint.find("origin")),
            "axis": [float(v) for v in (axis.get("xyz") if axis is not None else "1 0 0").split()],
            "mimic": None if mimic is None else {
                "joint": mimic.get("joint"),
                "multiplier": float(mimic.get("multiplier", 1)),
                "offset": float(mimic.get("offset", 0)),
            },
            "limit": None if limit is None else [float(limit.get("lower", 0)), float(limit.get("upper", 0))],
        })
    children = {j["child"] for j in joints}
    roots = [name for name in links if name not in children]

    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "meshes.glb").write_bytes(glb.export(file_type="glb"))
    model = {"source": xacro_file, "root": roots[0], "links": links, "joints": joints}
    (out_dir / "model.json").write_text(json.dumps(model, separators=(",", ":")), encoding="utf-8")
    faces = sum(len(g.faces) for g in glb.geometry.values())
    print(f"root link {roots[0]}; {len(links)} links, {len(joints)} joints, {counter} meshes, {faces} triangles")
    print(f"wrote {out_dir / 'model.json'} and {out_dir / 'meshes.glb'} "
          f"({(out_dir / 'meshes.glb').stat().st_size / 1e6:.1f} MB)")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--src", required=True, help="catkin workspace src folder")
    parser.add_argument("--xacro", default=DEFAULT_XACRO, help="robot xacro, relative to --src")
    parser.add_argument("--out", default=str(ROOT / "webui" / "static" / "robot"), help="output folder")
    args = parser.parse_args()
    build(args.src, args.xacro, Path(args.out))


if __name__ == "__main__":
    main()
