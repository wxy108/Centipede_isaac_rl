#!/usr/bin/env python3
"""
split_compound_joints.py — Make every MJCF joint single-DOF
==============================================================
Why: Isaac Sim's MJCF importer collapses any MJCF "compound" joint
(multiple <joint> elements sharing one body pair) into a single
multi-DOF USD `PhysicsJoint`. Our diagnostic showed PhysX's
articulation solver locks at least one DOF of one of those compounds
(joint_leg_9_R_0), producing the visible "leg 10" thrash. The Isaac
Sim importer has NO config flag to change this behavior — so we fix
it at the MJCF source by splitting each compound into a chain of
single-joint bodies separated by massless dummies.

What it does:
    For every <body> with N > 1 direct <joint> children, restructure
    so the body becomes a chain of N bodies:
        original_parent
          └── DUMMY_0 (with 1st joint, at original pos, m≈0)
              └── DUMMY_1 (with 2nd joint, at pos=0, m≈0)
                  └── ...
                      └── DUMMY_{N-2}
                          └── ORIGINAL_BODY (with Nth joint,
                                             at pos=0, all original
                                             geoms/inertial/children
                                             intact)

    All joint NAMES are preserved exactly — the IsaacLab kinematics
    `find_joints("joint_pitch_body_10")` etc still resolve to a real
    revolute joint, just one that's now alone on its body. The
    importer therefore produces a `PhysicsRevoluteJoint` for every
    one instead of a multi-DOF `PhysicsJoint`.

Usage:
    python scripts/split_compound_joints.py
    python scripts/split_compound_joints.py \\
        --input  ../Centipede_MUJOCO-main/models/farms/centipede.xml \\
        --output models/farms/centipede_split.xml

Then re-run convert_mjcf.py on the output:
    python scripts/convert_mjcf.py \\
        --input models/farms/centipede_split.xml \\
        --output assets/centipede.usd
"""
import argparse
import os
import shutil
import sys

try:
    from lxml import etree
except ImportError:
    print("ERROR: lxml is required. Install with: pip install lxml")
    sys.exit(1)

SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.normpath(os.path.join(SCRIPT_DIR, ".."))

DEFAULT_INPUT  = os.path.normpath(os.path.join(
    PROJECT_ROOT, "..", "Centipede_MUJOCO-main",
    "models", "farms", "centipede.xml"))
DEFAULT_OUTPUT = os.path.join(
    PROJECT_ROOT, "models", "farms", "centipede_split.xml")


def make_dummy_inertial():
    """A negligible-mass / negligible-inertia placeholder for dummy bodies.

    Matches the existing placeholder pattern already used in the
    centipede MJCF (mass=1e-9, diaginertia=1e-15) so the MuJoCo
    compiler's balanceinertia step doesn't complain.
    """
    e = etree.Element("inertial")
    e.set("pos", "0 0 0")
    e.set("mass", "1e-9")
    e.set("diaginertia", "1e-15 1e-15 1e-15")
    return e


def split_one_body(body):
    """If `body` has N > 1 direct <joint> children, restructure it
    into a chain of N bodies (N-1 dummy ancestors + the original)."""
    direct_joints = [c for c in body if c.tag == "joint"]
    n = len(direct_joints)
    if n <= 1:
        return 0

    parent = body.getparent()
    if parent is None:
        return 0
    body_index = list(parent).index(body)

    body_name = body.get("name", "<unnamed>")
    body_pos   = body.get("pos",   "0 0 0")
    body_euler = body.get("euler",  None)
    body_quat  = body.get("quat",   None)

    # Detach original from its parent — we re-insert at the bottom of
    # the new chain.
    parent.remove(body)

    # 1. Strip all joints except the last from `body`. The last
    #    joint stays where it was. The original body now reports
    #    pos=(0,0,0) because the outermost dummy will carry the offset.
    moved_joints = direct_joints[:-1]
    for j in moved_joints:
        body.remove(j)
    body.set("pos", "0 0 0")
    for k in ("euler", "quat"):
        if k in body.attrib:
            del body.attrib[k]

    # 2. Build the chain of N-1 dummy bodies, deepest-first.
    current = body
    n_dummies = n - 1
    for i in range(n_dummies - 1, -1, -1):
        dummy = etree.Element("body")
        dummy.set("name", f"{body_name}__split{i}")
        if i == 0:
            # Outermost dummy carries the original body's offset.
            dummy.set("pos", body_pos)
            if body_euler is not None:
                dummy.set("euler", body_euler)
            if body_quat is not None:
                dummy.set("quat", body_quat)
        else:
            dummy.set("pos", "0 0 0")
        dummy.append(moved_joints[i])         # this joint goes here
        dummy.append(make_dummy_inertial())   # tiny inertia placeholder
        dummy.append(current)
        current = dummy

    # 3. Re-insert the outermost dummy at the body's old slot.
    parent.insert(body_index, current)
    return n - 1   # number of dummies created


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--input",  default=DEFAULT_INPUT,
                   help=f"Source MJCF (default: {DEFAULT_INPUT})")
    p.add_argument("--output", default=DEFAULT_OUTPUT,
                   help=f"Output MJCF (default: {DEFAULT_OUTPUT})")
    p.add_argument("--copy-meshes", action="store_true",
                   help="If set, also copy the meshes/ subdir from the "
                        "input directory to the output's directory so "
                        "the MJCF's relative meshdir reference still "
                        "resolves. Useful if input is in another repo.")
    args = p.parse_args()

    if not os.path.isfile(args.input):
        print(f"ERROR: input file not found: {args.input}")
        sys.exit(1)
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)

    print(f"[split] reading {args.input}")
    parser = etree.XMLParser(remove_blank_text=False, remove_comments=False)
    tree = etree.parse(args.input, parser)
    root = tree.getroot()

    # First, count: how many bodies have multiple joints, and how many
    # total joints will be split out into dummies?
    bodies_to_split = []
    for body in root.iter("body"):
        n = len([c for c in body if c.tag == "joint"])
        if n > 1:
            bodies_to_split.append((body, n))
    n_total_joints_before = len(list(root.iter("joint")))
    n_total_bodies_before = len(list(root.iter("body")))
    print(f"[split] bodies before        : {n_total_bodies_before}")
    print(f"[split] joints before        : {n_total_joints_before}")
    print(f"[split] bodies w/ multi-joint: {len(bodies_to_split)}")
    if bodies_to_split:
        joint_count_distribution = {}
        for _, n in bodies_to_split:
            joint_count_distribution[n] = joint_count_distribution.get(n, 0) + 1
        for k, v in sorted(joint_count_distribution.items()):
            print(f"          {v} bodies with {k} joints each")

    # Run the splits. Order matters: process from leaves toward root so
    # that splitting a body doesn't invalidate the parent reference of
    # an unprocessed sibling. Easiest way is to materialize the list
    # first (we already did), then iterate in reverse depth.
    # In practice, since we never split the body's PARENT chain (only
    # the body itself), order is mostly safe. But safest is to process
    # deepest-first via `list.reverse()` of the iteration order.
    bodies_to_split.reverse()
    n_dummies_added = 0
    for body, _ in bodies_to_split:
        n_dummies_added += split_one_body(body)
    print(f"[split] dummy bodies added   : {n_dummies_added}")

    # Sanity check the result
    n_total_joints_after = len(list(root.iter("joint")))
    n_total_bodies_after = len(list(root.iter("body")))
    bodies_w_multi_after = sum(1 for body in root.iter("body")
                               if len([c for c in body if c.tag == "joint"]) > 1)
    print(f"[split] bodies after         : {n_total_bodies_after} "
          f"(+{n_total_bodies_after - n_total_bodies_before})")
    print(f"[split] joints after         : {n_total_joints_after} "
          f"({'OK' if n_total_joints_after == n_total_joints_before else 'CHANGED — bug!'})")
    print(f"[split] bodies w/ multi-joint after: {bodies_w_multi_after} "
          f"({'OK' if bodies_w_multi_after == 0 else 'BUG — some compounds left'})")

    # Write
    tree.write(args.output, pretty_print=True,
               xml_declaration=False, encoding="utf-8")
    print(f"[split] wrote {args.output}")

    # Optional mesh copy
    if args.copy_meshes:
        src_meshes = os.path.join(os.path.dirname(args.input), "meshes")
        dst_meshes = os.path.join(os.path.dirname(args.output), "meshes")
        if os.path.isdir(src_meshes):
            if os.path.isdir(dst_meshes):
                print(f"[split] meshes already at {dst_meshes} — skipping copy")
            else:
                shutil.copytree(src_meshes, dst_meshes)
                print(f"[split] copied meshes/ → {dst_meshes}")

    print(f"\n[split] next step:")
    print(f"  python scripts/convert_mjcf.py \\")
    print(f"      --input  {args.output} \\")
    print(f"      --output assets/centipede.usd")


if __name__ == "__main__":
    main()
