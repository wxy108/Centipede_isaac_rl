"""v3 (GO_STAGE_A_V3) unit tests — parameter-space softness reward (PI#4) + weak legs (PI#5).
Locks: softness in [0,1], monotone (lower kp/kv -> softer), ranks PI-COMPLIANT > RIGID,
and rough_quality rewards softness. Run: python -m pytest tests/test_stage_a_vec_core_v3_2026-06-19.py -q
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), os.pardir, "scripts", "optimization"))
import stage_a_vec_core as core  # noqa: E402

LIM = core.default_limits()
GROUPS = ["body_yaw", "body_pitch", "body_roll", "legbase_yaw", "legbase_pitch", "legbase_roll", "knee"]


def _gains(kp_body_pitch, kp_other=1.0, kv=1.0):
    """Build a cell gain dict with a given body_pitch kp; others uniform."""
    kp = {g: kp_other for g in GROUPS}
    kp["body_pitch"] = kp_body_pitch
    return dict(
        body_kp=kp["body_yaw"], body_kv=kv, pitch_kp=kp["body_pitch"], pitch_kv=kv,
        roll_kp=kp["body_roll"], roll_kv=kv,
        leg_kp=[kp["legbase_yaw"], kp["legbase_pitch"], kp["legbase_roll"], kp["knee"]],
        leg_kv=[kv, kv, kv, kv], effort=[1.0] * 7, body_amp=1.0, leg_yaw_amp=1.0, leg_pitch_amp=1.0)


def test_softness_in_unit_interval():
    softest, _ = core.softness(_gains(0.1, 0.1, 0.05), LIM)   # all at lower bound
    stiffest, _ = core.softness(_gains(20.0, 20.0, 40.0), LIM)  # all at upper bound
    assert 0.99 <= softest <= 1.0
    assert 0.0 <= stiffest <= 0.01
    assert core.softness(None, LIM)[0] == 0.0   # None gains -> 0 (v2 backward-compat)


def test_softness_monotone_in_body_pitch_kp():
    # lower body_pitch kp -> softer (body_pitch is the heaviest-weighted group)
    soft = core.softness(_gains(0.15), LIM)[0]
    rigid = core.softness(_gains(4.0), LIM)[0]
    assert soft > rigid


def test_softness_ranks_pi_compliant_above_rigid():
    # representative compliant (low body_pitch kp) vs rigid (high body_pitch kp), legs equal
    compliant = [core.softness(_gains(bp), LIM)[0] for bp in (0.14, 1.12, 0.15, 0.15)]  # ranks 3,4,5,8
    rigid = [core.softness(_gains(bp), LIM)[0] for bp in (3.10, 3.19, 4.82, 4.18)]       # ranks 0,1,2,9
    assert min(compliant) > max(rigid)


def test_rough_quality_rewards_softness():
    s3 = dict(v_fwd_mps=0.02, gait_corr={"hip_yaw": 1.0}, gait_amp_ratio={"hip_yaw": 1.0},
              body_amp_ratio=0.8, roll_rms_deg=6.0, pd_effort=0.001)
    LIM2 = dict(LIM, norm_v=0.2, norm_pd=0.001)
    q_soft, p_soft = core.rough_quality(s3, LIM2, _gains(0.15))   # soft body_pitch
    q_rigid, p_rigid = core.rough_quality(s3, LIM2, _gains(4.0))  # rigid body_pitch
    assert p_soft["q_softness"] > p_rigid["q_softness"]
    assert q_soft > q_rigid          # softer ranks higher (same locomotion)
