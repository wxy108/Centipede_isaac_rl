"""v2 gate logic tests: roll SOFT + tip-over/net-yaw/backward HARD; rough-quality ranks."""
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts", "optimization"))
import stage_a_vec_core as core

def mk(scn, **o):
    d = dict(scenario=scn, freq_hz=1.0, fwd_signed_m=0.15, v_fwd_mps=0.03, net_yaw_deg=5.0,
             roll_rms_deg=2.0, root_z_p2p_m=0.01, buckled=False, nan=False, bl_per_cycle=0.3,
             peak_leg_torque=[0.003,0.002,1e-5,0.01], pd_effort=0.0007, body_amp_ratio=0.95,
             gait_corr={"hip_yaw":0.9,"hip_pitch":0.85}, gait_amp_ratio={"hip_yaw":0.9,"hip_pitch":0.8},
             refs_degenerate=False, min_upright=0.99)
    d.update(o); return d

def L():
    l = core.default_limits(); l["norm_pd"]=0.0007; l["norm_v"]=0.2; l["leg_torque_limit"]=0.004; return l

def test_all_ok_feasible():
    f,v = core.scenario_feasibility(mk("s1"), mk("s2",freq_hz=5.0,roll_rms_deg=3.0), mk("s3"), L()); assert f, v
def test_spin_infeasible():
    f,v = core.scenario_feasibility(mk("s1"), mk("s2",freq_hz=5.0,roll_rms_deg=3.0), mk("s3",net_yaw_deg=25.0), L()); assert (not f) and "s3_net_yaw" in v
def test_tipover_infeasible():
    f,v = core.scenario_feasibility(mk("s1"), mk("s2",freq_hz=5.0,roll_rms_deg=3.0), mk("s3",min_upright=0.7), L()); assert (not f) and "s3_tipover" in v
def test_backward_infeasible():
    f,v = core.scenario_feasibility(mk("s1"), mk("s2",freq_hz=5.0,roll_rms_deg=3.0), mk("s3",fwd_signed_m=-0.05), L()); assert (not f) and "s3_backward" in v
def test_high_roll_is_SOFT_feasible():
    # v2 key change: 8 deg rough roll alone (upright, non-spin, forward) MUST be feasible
    f,v = core.scenario_feasibility(mk("s1"), mk("s2",freq_hz=5.0,roll_rms_deg=3.0), mk("s3",roll_rms_deg=8.0), L())
    assert f, ("high rough roll should be feasible (soft)", v)
def test_quality_ranks_fwd_and_beats_spin():
    l=L()
    c_fast = core.lexicographic_cost(mk("s1"), mk("s2",freq_hz=5.0,roll_rms_deg=3.0), mk("s3",v_fwd_mps=0.05), l)[0]
    c_slow = core.lexicographic_cost(mk("s1"), mk("s2",freq_hz=5.0,roll_rms_deg=3.0), mk("s3",v_fwd_mps=0.01), l)[0]
    c_spin = core.lexicographic_cost(mk("s1"), mk("s2",freq_hz=5.0,roll_rms_deg=3.0), mk("s3",net_yaw_deg=30.0), l)[0]
    assert c_fast < c_slow, (c_fast, c_slow)
    assert c_fast < c_spin, (c_fast, c_spin)
