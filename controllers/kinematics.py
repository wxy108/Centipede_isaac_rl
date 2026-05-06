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
N_LEG_DOF     = 4     # DOF 0 (hip yaw), 1 (hip pitch), 2 (tibia), 3 (tarsus)
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

        # ── body yaw joint indices (19 joints: joint_body_1..19) ──
        self.body_joint_ids = np.array(
            [jid(body_joint_name(i + 1)) for i in range(N_BODY_JOINTS)],
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
            name = pitch_joint_name(i)
            if name in joint_name_to_id:
                pitch_ids.append(joint_name_to_id[name])
        self.pitch_joint_ids = np.asarray(pitch_ids, dtype=int)
        self.has_pitch = len(self.pitch_joint_ids) > 0

        # ── roll joint indices (0 or N_ROLL joints) ──
        roll_ids = []
        for i in range(N_ROLL):
            name = roll_joint_name(i)
            if name in joint_name_to_id:
                roll_ids.append(joint_name_to_id[name])
        self.roll_joint_ids = np.asarray(roll_ids, dtype=int)
        self.has_roll = len(self.roll_joint_ids) > 0

        # ── leg joint indices [n, side, dof] ──
        self.leg_joint_ids = np.zeros((N_LEGS, 2, N_LEG_DOF), dtype=int)
        for n in range(N_LEGS):
            for si, side in enumerate(('L', 'R')):
                for dof in range(N_LEG_DOF):
                    self.leg_joint_ids[n, si, dof] = jid(
                        leg_joint_name(n, side, dof))

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
              f"{N_LEGS}x2x{N_LEG_DOF} leg, "
              f"{n_pitch} pitch, {n_roll} roll joints")
        print(f"[IsaacLabModelIndex] Total articulation joints: "
              f"{articulation.num_joints}, bodies: {articulation.num_bodies}")
