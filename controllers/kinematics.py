"""
kinematics.py — Joint naming and index resolution for Isaac Lab
================================================================
Port of the MuJoCo FARMSModelIndex to Isaac Lab's Articulation API.

Isaac Lab joint names (verified from test_articulation.py output):
    Body yaw:   joint_body_1 .. joint_body_19     (19 joints)
    Pitch:      joint_pitch_body_0 .. _19          (20 joints)
    Roll:       joint_roll_body_0 .. _19           (20 joints)
    Leg DOF0:   joint_leg_N_L/R_0                  (38 joints)
    Leg roll:   joint_roll_leg_N_L/R               (38 joints, passive)
    Leg DOF1-3: joint_leg_N_L/R_1/2/3             (114 joints)
    Foot:       joint_foot_N_0/1                   (38 joints, passive)
    Total: 287 joints

Isaac Lab find_joints() returns: (list_of_indices, list_of_names)
"""

import numpy as np

# ═══════════════════════════════════════════════════════════
# Model constants
# ═══════════════════════════════════════════════════════════

N_BODY_SEGS   = 21    # link_body_0 .. link_body_20
N_BODY_JOINTS = 19    # joint_body_1 .. joint_body_19
N_PITCH       = 20    # joint_pitch_body_0 .. joint_pitch_body_19
N_ROLL        = 20    # joint_roll_body_0  .. joint_roll_body_19
N_LEGS        = 19    # leg index n = 0..18 per side
N_LEG_DOF     = 4     # LEGACY DEFAULT (deployed / renamed-4DOF): DOF 0 (hip yaw),
                      # 1 (hip pitch), 2 (tibia/legbase_roll), 3 (tarsus/knee).
                      # Block A (2026-07-07): this module constant stays the legacy
                      # default; IsaacLabModelIndex now detects a PER-INSTANCE
                      # `n_leg_dof` (4 for the 4-DOF models, 2 for the leaner rebuild)
                      # from the joint names — the per-instance value wins downstream.
ACTIVE_DOFS   = (0, 1)
LINK_LENGTH   = 0.005  # metres


# ═══════════════════════════════════════════════════════════
# Naming functions
# ═══════════════════════════════════════════════════════════

def body_joint_name(i):
    """Body yaw joint between segment i-1 and i (i=1..19)."""
    return f"joint_body_{i}"

def pitch_joint_name(i):
    """Body pitch joint at segment i (i=0..19)."""
    return f"joint_pitch_body_{i}"

def roll_joint_name(i):
    """Body roll joint at segment i (i=0..19)."""
    return f"joint_roll_body_{i}"

def leg_joint_name(n, side, dof):
    """Leg joint: n=0..18, side='L'/'R', dof=0..3."""
    return f"joint_leg_{n}_{side}_{dof}"

def body_link_name(i):
    """Body link (rigid body) name."""
    return f"link_body_{i}"


# ═══════════════════════════════════════════════════════════
# Helper: unpack find_joints / find_bodies return value
# ═══════════════════════════════════════════════════════════

def _unpack_joint_id(result, name):
    """
    Isaac Lab find_joints() returns (indices_list, names_list).
    Extract the single integer index, or raise if not found.
    """
    if result is None:
        raise ValueError(f"Joint not found: {name!r}")
    # Handle tuple return: (indices, names)
    if isinstance(result, tuple):
        indices = result[0]
    else:
        indices = result
    if not indices or len(indices) == 0:
        raise ValueError(f"Joint not found: {name!r}")
    return int(indices[0])


def _unpack_body_id(result, name):
    """Same as _unpack_joint_id but for find_bodies()."""
    if result is None:
        raise ValueError(f"Body not found: {name!r}")
    if isinstance(result, tuple):
        indices = result[0]
    else:
        indices = result
    if not indices or len(indices) == 0:
        raise ValueError(f"Body not found: {name!r}")
    return int(indices[0])


# ═══════════════════════════════════════════════════════════
# Isaac Lab index resolver
# ═══════════════════════════════════════════════════════════

class IsaacLabModelIndex:
    """
    Resolves all joint/body names to integer indices in the Isaac Lab
    Articulation tensor. Drop-in replacement for FARMSModelIndex.

    IMPORTANT — exact-name lookup, not regex:
        Earlier versions used `articulation.find_joints(name)`, which
        is a regex-based matcher. For names like `joint_body_1`, the
        regex `joint_body_1` *also* matches `joint_body_10`, `_11`,
        ..., `_19`. The unpacker took match[0], which silently
        returned a wrong joint index — producing exactly the
        "specific-joint-wildly-swings-while-the-rest-look-fine"
        symptom (e.g. body 2 / leg 10).

        We now build a single name→index dict from
        `articulation.joint_names` / `body_names` and look up via
        plain Python string equality. No regex anywhere.
    """

    def __init__(self, articulation):
        self.articulation = articulation

        # ── Build exact-name lookup dicts ONCE from articulation ──
        joint_name_to_id = {n: i for i, n in enumerate(articulation.joint_names)}
        body_name_to_id  = {n: i for i, n in enumerate(articulation.body_names)}

        def jid(name):
            if name not in joint_name_to_id:
                raise ValueError(f"Joint not in articulation: {name!r}")
            return joint_name_to_id[name]

        def bid(name):
            if name not in body_name_to_id:
                raise ValueError(f"Body not in articulation: {name!r}")
            return body_name_to_id[name]

        # ── P3 rename (2026-06-17): AUTO-DETECT naming scheme ──
        # The deployed/rebuild models use joint_body_*/joint_pitch_body_*/
        # joint_roll_body_*/joint_leg_*; the FUNCTION-RENAMED rebuild uses
        # body_yaw_*/body_pitch_*/body_roll_*/legbase_*/knee_*. Detect which by
        # presence and bind the matching name builders, so BOTH resolve to the
        # SAME indices (joint order is preserved by the rename) — deployed stays
        # byte-identical; the renamed variant resolves correctly.
        _new_scheme = ("body_yaw_1" in joint_name_to_id) or ("body_pitch_0" in joint_name_to_id)
        if _new_scheme:
            _LEG_NEW = ("legbase_yaw", "legbase_pitch", "legbase_roll", "knee")
            _bjn = lambda i: f"body_yaw_{i}"
            _pjn = lambda i: f"body_pitch_{i}"
            _rjn = lambda i: f"body_roll_{i}"
            _ljn = lambda n, s, d: f"{_LEG_NEW[d]}_{n}_{s}"
            _leg_full_names = _LEG_NEW
        else:
            _bjn, _pjn, _rjn, _ljn = (body_joint_name, pitch_joint_name,
                                      roll_joint_name, leg_joint_name)
            _leg_full_names = ("dof0", "dof1", "dof2", "dof3")
        self.naming_scheme = "function_renamed" if _new_scheme else "legacy_joint_prefix"

        # ── Block A (2026-07-07): PER-INSTANCE leg-DOF count (Option B) ──
        # The leaner rebuild drops legbase_roll (DOF-2) + knee (DOF-3) → 2 active
        # leg DOFs (legbase_yaw, legbase_pitch) + a distal PASSIVE tarsus_yaw; the
        # deployed / renamed-4DOF rebuild keeps all 4 leg DOFs. Detect the count
        # from the joint names actually present (probe leg 0 side L for DOF-2/3 via
        # the bound _ljn) so N_LEG_DOF becomes per-instance. The module-level
        # N_LEG_DOF (=4) stays the legacy default; this per-instance value wins
        # everywhere downstream. For a 4-DOF model both DOF-2 and DOF-3 names
        # resolve ⇒ n_leg_dof == 4 == the pre-Block-A constant, so the 4-DOF leg
        # resolution is BYTE-IDENTICAL to before this change.
        _has_dof2 = _ljn(0, 'L', 2) in joint_name_to_id
        _has_dof3 = _ljn(0, 'L', 3) in joint_name_to_id
        self.n_leg_dof = 4 if (_has_dof2 and _has_dof3) else 2
        self.leg_dof_names = tuple(_leg_full_names[d] for d in range(self.n_leg_dof))

        # ── Block A: distal PASSIVE tarsus_yaw joints (leaner model only) ──
        # Resolved best-effort (like pitch/roll): present in the leaner rebuild
        # (tarsus_yaw_{n}_{s}, 38 joints), absent in the 4-DOF models. NOT driven
        # by the controller (no torque is written to these indices); handled
        # passively by the sim / a dedicated passive actuator group. has_tarsus
        # reflects presence so callers (e.g. the passive-spring harness) can skip
        # them when absent.
        tarsus_ids = []
        for n in range(N_LEGS):
            for side in ('L', 'R'):
                name = f"tarsus_yaw_{n}_{side}"
                if name in joint_name_to_id:
                    tarsus_ids.append(joint_name_to_id[name])
        self.tarsus_joint_ids = np.asarray(tarsus_ids, dtype=int)
        self.has_tarsus = len(self.tarsus_joint_ids) > 0

        # ── body yaw joint indices (19 joints) ──
        self.body_joint_ids = np.array(
            [jid(_bjn(i + 1)) for i in range(N_BODY_JOINTS)],
            dtype=int,
        )

        # ── pitch joint indices (0 or N_PITCH joints) ──
        # The simplified MJCF (centipede_simple.xml) removes all
        # joint_pitch_body_* / joint_roll_body_* joints to eliminate
        # multi-DOF compound joints. Lookup is therefore best-effort:
        # if a name isn't present in the articulation, we just skip
        # it (rather than raising) and `has_pitch`/`has_roll` reflect
        # what's actually there.
        pitch_ids = []
        for i in range(N_PITCH):
            name = _pjn(i)
            if name in joint_name_to_id:
                pitch_ids.append(joint_name_to_id[name])
        self.pitch_joint_ids = np.asarray(pitch_ids, dtype=int)
        self.has_pitch = len(self.pitch_joint_ids) > 0

        # ── roll joint indices (0 or N_ROLL joints) ──
        roll_ids = []
        for i in range(N_ROLL):
            name = _rjn(i)
            if name in joint_name_to_id:
                roll_ids.append(joint_name_to_id[name])
        self.roll_joint_ids = np.asarray(roll_ids, dtype=int)
        self.has_roll = len(self.roll_joint_ids) > 0

        # ── leg joint indices [n, side, dof] ──
        # Shape (N_LEGS, 2, self.n_leg_dof): 4-DOF for the deployed/renamed-4DOF
        # models (byte-identical to the pre-Block-A (N_LEGS, 2, 4)), 2-DOF for the
        # leaner rebuild (legbase_yaw, legbase_pitch only).
        self.leg_joint_ids = np.zeros((N_LEGS, 2, self.n_leg_dof), dtype=int)
        for n in range(N_LEGS):
            for si, side in enumerate(('L', 'R')):
                for dof in range(self.n_leg_dof):
                    self.leg_joint_ids[n, si, dof] = jid(
                        _ljn(n, side, dof))

        # ── head body index (for heading quaternion) ──
        self.head_body_id = bid("link_body_0")

        # ── Sanity: every resolved joint index must be unique. If two
        #     of our 287 named joints somehow point to the same
        #     articulation index, log it loudly so we notice.
        all_ids = list(self.body_joint_ids) \
                + list(self.pitch_joint_ids) \
                + list(self.roll_joint_ids) \
                + list(self.leg_joint_ids.reshape(-1))
        if len(set(all_ids)) != len(all_ids):
            from collections import Counter
            dupes = [(idx, cnt) for idx, cnt in Counter(all_ids).items() if cnt > 1]
            print(f"[IsaacLabModelIndex] WARNING: duplicate joint indices "
                  f"after lookup: {dupes[:10]} (showing first 10)")

        n_pitch = len(self.pitch_joint_ids)
        n_roll  = len(self.roll_joint_ids)
        print(f"[IsaacLabModelIndex] Loaded (exact-name lookup): "
              f"{N_BODY_JOINTS} body yaw, "
              f"{N_LEGS}x2x{self.n_leg_dof} leg, "
              f"{n_pitch} pitch, {n_roll} roll joints "
              f"(n_leg_dof={self.n_leg_dof}, has_tarsus={self.has_tarsus}, "
              f"n_tarsus={len(self.tarsus_joint_ids)}, dofs={self.leg_dof_names})")
        print(f"[IsaacLabModelIndex] Total articulation joints: "
              f"{articulation.num_joints}, bodies: {articulation.num_bodies}")
