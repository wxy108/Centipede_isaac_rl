#!/usr/bin/env python3
"""
simplify_mjcf.py — Remove compliance joints (body pitch, body roll, leg roll)
==============================================================================
Why: MJCF compound joints (multiple <joint> on one body, sharing a body
pair) get imported as multi-DOF USD `PhysicsJoint` prims, and PhysX's
articulation solver locks one DOF of one of those compounds at body 10
of the centipede. Removing the *passive compliance* joints — body pitch,
body roll, and leg roll — eliminates EVERY multi-DOF compound in the
model and leaves only single-DOF `PhysicsRevoluteJoint`s, which PhysX
handles cleanly.

What it does:
    1. For every <body name="link_pitch_body_X">:
         - Sum its `pos` into its single child <body>'s `pos`
         - Promote that child to be a direct child of link_pitch_body_X's
           parent (= link_body_X)
         - Discard link_pitch_body_X and its two joints
           (joint_pitch_body_X, joint_roll_body_X)
    2. For every <body name="link_leg_*_*_0">:
         - Delete the <joint name="joint_roll_leg_*_*"> child (leg roll)
         - Keep <joint name="joint_leg_*_*_0"> (hip yaw)
    3. Remove the corresponding <general> actuators and any
       <jointpos>/<jointvel> sensors for the deleted joints.

Result for the centipede:
    Before: 231 bodies, 287 joints (58 multi-DOF compounds)
    After:  211 bodies, 209 joints (0  compounds)

What's preserved:
    * Freejoint at link_body_0 (root)
    * 19 body-yaw joints (the body wave)
    * 19 × 2 × 4 = 152 leg joints (hip yaw + hip pitch + tibia + tarsus)
    * 38 foot joints
    * All masses, inertias, geom shapes, collision setup unchanged

Usage:
    python scripts/simplify_mjcf.py
    python scripts/simplify_mjcf.py \\
        --input  ../Centipede_MUJOCO-main/models/farms/centipede.xml \\
        --output models/farms/centipede_simple.xml
    python scripts/convert_mjcf.py \\
        --input  models/farms/centipede_simple.xml \\
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
    PROJECT_ROOT, "models", "farms", "centipede_simple.xml")


def _parse_pos(s):
    return [float(x) for x in s.split()]


def _format_pos(arr):
    return " ".join(f"{x:.10g}" for x in arr)


def collapse_pitch_body_intermediate(root):
    """Remove every link_pitch_body_X intermediate body, sum its pos
    into its child link_body_X+1, attach that child to the original
    grandparent. Discard joint_pitch_body_X and joint_roll_body_X."""
    pitch_intermediates = [
        b for b in root.iter("body")
        if b.get("name", "").startswith("link_pitch_body_")
    ]
    n_collapsed = 0
    for pb in pitch_intermediates:
        parent = pb.getparent()
        if parent is None:
            continue
        index_in_parent = list(parent).index(pb)

        pb_pos = _parse_pos(pb.get("pos", "0 0 0"))
        pb_euler = pb.get("euler", None)
        pb_quat  = pb.get("quat",  None)
        if pb_euler is not None or pb_quat is not None:
            print(f"  [warn] {pb.get('name')} has euler/quat — not "
                  f"handled, may produce incorrect orientation.")

        # The pitch_body's CHILD <body> is the next-segment link.
        # There may also be inertial/geom siblings — those came with
        # the (now-deleted) intermediate body and represent its mass
        # (which is 1e-9 anyway), so we drop them too.
        child_bodies = pb.findall("body")
        if len(child_bodies) != 1:
            print(f"  [warn] {pb.get('name')} has {len(child_bodies)} "
                  f"child bodies, expected 1 — skipping.")
            continue
        child = child_bodies[0]

        # Sum positions: child stays at its old relative pos PLUS the
        # offset that pb provided.
        child_pos = _parse_pos(child.get("pos", "0 0 0"))
        new_pos = [pb_pos[i] + child_pos[i] for i in range(3)]
        child.set("pos", _format_pos(new_pos))

        # Detach pb, insert child at pb's slot.
        parent.remove(pb)
        parent.insert(index_in_parent, child)
        n_collapsed += 1
    return n_collapsed


def keep_body_pitch_remove_only_roll(root):
    """Less aggressive variant: keep every link_pitch_body_X and its
    joint_pitch_body_X joint (so body pitch compliance is preserved).
    Just delete joint_roll_body_X from each, so the remaining body
    has exactly one joint (single-DOF revolute, no PhysX compound)."""
    n_removed = 0
    for pb in root.iter("body"):
        if not pb.get("name", "").startswith("link_pitch_body_"):
            continue
        for j in list(pb):
            if j.tag == "joint" and j.get("name", "").startswith("joint_roll_body_"):
                pb.remove(j)
                n_removed += 1
    return n_removed


def remove_leg_roll_joints(root):
    """Delete every <joint name="joint_roll_leg_*"> child element so
    each leg-base body has only hip yaw."""
    n_removed = 0
    for j in list(root.iter("joint")):
        name = j.get("name", "")
        if name.startswith("joint_roll_leg_"):
            parent = j.getparent()
            if parent is not None:
                parent.remove(j)
                n_removed += 1
    return n_removed


def remove_orphan_actuators_and_sensors(root, removed_joint_prefixes):
    """Remove <general>/<motor> actuators and joint-related sensors
    that referenced any of the deleted joints."""
    n_act = 0
    n_sens = 0
    for actuator_tag in ("general", "motor", "position", "velocity"):
        for a in list(root.iter(actuator_tag)):
            jname = a.get("joint", "")
            if any(jname.startswith(pfx) for pfx in removed_joint_prefixes):
                a.getparent().remove(a)
                n_act += 1
    for sensor_tag in ("jointpos", "jointvel", "jointlimitpos",
                       "jointlimitvel", "jointlimitfrc"):
        for s in list(root.iter(sensor_tag)):
            jname = s.get("joint", "")
            if any(jname.startswith(pfx) for pfx in removed_joint_prefixes):
                s.getparent().remove(s)
                n_sens += 1
    return n_act, n_sens


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--input",  default=DEFAULT_INPUT)
    p.add_argument("--output", default=DEFAULT_OUTPUT)
    p.add_argument("--copy-meshes", action="store_true",
                   help="Also copy meshes/ next to the output.")
    # Body-pitch retention: with --keep-body-pitch, we KEEP each
    # link_pitch_body_X intermediate body and its joint_pitch_body_X
    # joint, but still remove joint_roll_body_X from it (so the body
    # has exactly one joint — no compound). That preserves vertical
    # body compliance for terrain. Default is to keep pitch.
    p.add_argument("--keep-body-pitch", dest="keep_body_pitch",
                   action="store_true", default=True,
                   help="Keep body pitch joints (default). Removes only "
                        "the body roll joint from each compound, so each "
                        "link_pitch_body_X becomes single-DOF.")
    p.add_argument("--no-keep-body-pitch", dest="keep_body_pitch",
                   action="store_false",
                   help="Aggressive: also remove body pitch joints "
                        "(collapse link_pitch_body_X intermediates).")
    args = p.parse_args()

    if not os.path.isfile(args.input):
        print(f"ERROR: input file not found: {args.input}")
        sys.exit(1)
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)

    print(f"[simplify] reading {args.input}")
    parser = etree.XMLParser(remove_blank_text=False, remove_comments=False)
    tree = etree.parse(args.input, parser)
    root = tree.getroot()

    n_bodies_before = len(list(root.iter("body")))
    n_joints_before = len(list(root.iter("joint")))
    print(f"[simplify] before: {n_bodies_before} bodies, {n_joints_before} joints")

    # 1. Body-pitch handling — two modes:
    #     keep-body-pitch (default): keep link_pitch_body_X but strip
    #         the joint_roll_body_X from each (each remaining body has
    #         exactly one joint — single-DOF, no compound). Preserves
    #         vertical body compliance for terrain.
    #     not keep-body-pitch: collapse link_pitch_body_X entirely
    #         (removes both pitch and roll body joints).
    if args.keep_body_pitch:
        n_roll_only = keep_body_pitch_remove_only_roll(root)
        print(f"[simplify] kept {n_roll_only} link_pitch_body_X "
              f"intermediates with body-pitch joint (removed only "
              f"joint_roll_body_X)")
    else:
        n_col = collapse_pitch_body_intermediate(root)
        print(f"[simplify] collapsed {n_col} link_pitch_body_X "
              f"intermediate bodies (removed 2*{n_col} = {2*n_col} "
              f"pitch+roll body joints)")

    # 2. Remove leg-roll joints from leg-base bodies.
    n_lr = remove_leg_roll_joints(root)
    print(f"[simplify] removed {n_lr} joint_roll_leg_* joints")

    # 3. Clean up actuators and sensors referencing the deleted joints.
    # Build the list of removed-joint prefixes based on what we actually
    # removed in the previous steps. If --keep-body-pitch is on, the
    # joint_pitch_body_* actuators / sensors must STAY.
    removed_prefixes = ["joint_roll_body_", "joint_roll_leg_"]
    if not args.keep_body_pitch:
        removed_prefixes.append("joint_pitch_body_")
    n_act, n_sens = remove_orphan_actuators_and_sensors(
        root,
        removed_joint_prefixes=tuple(removed_prefixes),
    )
    print(f"[simplify] removed {n_act} orphan actuators, {n_sens} "
          f"orphan sensors (prefixes: {removed_prefixes})")

    n_bodies_after = len(list(root.iter("body")))
    n_joints_after = len(list(root.iter("joint")))
    print(f"[simplify] after:  {n_bodies_after} bodies "
          f"({n_bodies_after - n_bodies_before:+d}), "
          f"{n_joints_after} joints "
          f"({n_joints_after - n_joints_before:+d})")

    # Sanity: every remaining <body> should have ≤ 1 <joint> child.
    multi_joint_bodies = [
        b for b in root.iter("body")
        if len([c for c in b if c.tag == "joint"]) > 1
    ]
    if multi_joint_bodies:
        print(f"[simplify] WARNING: {len(multi_joint_bodies)} bodies still "
              f"have >1 joint:")
        for b in multi_joint_bodies[:5]:
            print(f"    {b.get('name')}")
    else:
        print(f"[simplify] ✓ no remaining compound joints — every body has ≤1 joint")

    tree.write(args.output, pretty_print=True,
               xml_declaration=False, encoding="utf-8")
    print(f"[simplify] wrote {args.output}")

    if args.copy_meshes:
        src_meshes = os.path.join(os.path.dirname(args.input), "meshes")
        dst_meshes = os.path.join(os.path.dirname(args.output), "meshes")
        if os.path.isdir(src_meshes) and not os.path.isdir(dst_meshes):
            shutil.copytree(src_meshes, dst_meshes)
            print(f"[simplify] copied meshes/ → {dst_meshes}")

    print(f"\n[simplify] next:")
    print(f"  python scripts/convert_mjcf.py \\")
    print(f"      --input  {args.output} \\")
    print(f"      --output assets/centipede.usd")


if __name__ == "__main__":
    main()
