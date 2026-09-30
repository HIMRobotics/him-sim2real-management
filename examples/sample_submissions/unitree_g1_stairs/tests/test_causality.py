"""Causality: scripted/pretrained succeeds, neutral fails on the same seed (v4 free-base)."""

from __future__ import annotations

from parkour_env import evaluate_episode
from policy import NeutralPolicy, ScriptedParkourPolicy

SEED = 0
HORIZON = 20.0


def test_causality_scripted_succeeds_neutral_fails():
    scripted = evaluate_episode(ScriptedParkourPolicy(), seed=SEED, horizon_s=HORIZON, record=False)
    neutral = evaluate_episode(NeutralPolicy(), seed=SEED, horizon_s=HORIZON, record=False)

    print("CAUSALITY REPORT")
    print("seed", SEED)
    print("scripted_success", scripted.success)
    print("scripted_call_count", scripted.call_count)
    print("scripted_action_bounds", scripted.action_min, scripted.action_max)
    print("scripted_stair_foot", scripted.contact_stairs)
    print("scripted_distinct_stairs", scripted.distinct_stair_foot_contacts)
    print("scripted_top_feet", scripted.contact_top_landing)
    print("scripted_finish_feet", scripted.contact_finish_pad)
    print("scripted_stood_on_top", scripted.stood_on_top)
    print("scripted_aerial_gap", scripted.aerial_gap)
    print("scripted_knee_range", scripted.joint_knee_range)
    print("scripted_root_writes", scripted.root_qpos_writes_after_reset)
    print("scripted_final_pelvis", scripted.final_pelvis_xyz)
    print("neutral_success", neutral.success)
    print("neutral_call_count", neutral.call_count)
    print("neutral_contacts", neutral.total_platform_contacts)
    print("neutral_final_pelvis", neutral.final_pelvis_xyz)

    assert scripted.success, "pretrained+scripted policy must clear the v4 free-base course"
    assert not neutral.success, "neutral policy must fail"
    assert scripted.call_count > 0
    assert neutral.call_count > 0
    assert scripted.distinct_stair_foot_contacts >= 2, "need foot contacts on >=2 stair treads"
    assert scripted.stood_on_top, "must stand upright on top landing"
    assert scripted.aerial_gap, "must clear aerial gap"
    assert scripted.contact_finish_pad > 0, "need foot contact on finish pad"
    assert scripted.total_platform_contacts > 0
    assert scripted.root_qpos_writes_after_reset == 0, "root qpos must not be written mid-episode"
    # Visible bending: knee range should be nontrivial
    kmin, kmax = scripted.joint_knee_range
    assert (kmax - kmin) > 0.15, f"expected visible knee bending, got range {kmin}..{kmax}"
    assert scripted.max_abs_action <= 3.5
