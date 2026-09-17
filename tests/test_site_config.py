"""site_config 加载与字段覆盖。"""

from site_config import SITE_CONFIG_FIELDS, load_site_config


def test_load_site_config_includes_v2_fields():
    site = load_site_config()
    assert 'hook_sway_angle_tol_approach_deg' in SITE_CONFIG_FIELDS
    assert 'anti_sway_locked_max_shift' in SITE_CONFIG_FIELDS
    if site:
        assert site.get('enable_anti_sway') is True
