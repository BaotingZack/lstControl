"""site_config 加载与字段覆盖。"""

from site_config import SITE_CONFIG_FIELDS, load_site_config


def test_load_site_config_includes_v2_fields():
    site = load_site_config()
    assert 'hook_sway_angle_tol_approach_deg' in SITE_CONFIG_FIELDS
    assert 'anti_sway_locked_max_shift' in SITE_CONFIG_FIELDS
    if site:
        assert site.get('enable_anti_sway') is True
        assert site.get('anti_sway_travel_damping') is False
        assert site.get('anti_sway_approach_damping') is True
        assert float(site.get('anti_sway_approach_pos')) == 0.40
        assert float(site.get('anti_sway_approach_vel')) == 0.10
        assert float(site.get('anti_sway_settle_max_shift', 1.0)) == 0.0
        assert float(site['rope_length_sheave_height']) == 4.0
        assert float(site['rope_length_grab_offset']) == 0.0
        assert float(site['rope_length_structure']) == 1.5
