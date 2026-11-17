"""消摆锚点：真机 Z 用抓钩高度，仿真可覆盖 anchor_z。"""

from crane_model import CraneConfig

from web.server import _parse_settle_sim_anchor


def test_parse_settle_sim_anchor_z_only():
    cfg = CraneConfig()
    default = (1.0, 2.0, 3.0)
    out = _parse_settle_sim_anchor({'anchor_z': 1.25}, cfg, default)
    assert out == (1.0, 2.0, 1.25)


def test_parse_settle_sim_anchor_full():
    cfg = CraneConfig()
    default = (0.0, 0.0, 3.0)
    out = _parse_settle_sim_anchor(
        {'anchor': {'x': 4.0, 'y': 5.0, 'z': 0.8}},
        cfg,
        default,
    )
    assert out == (4.0, 5.0, 0.8)


def test_parse_settle_sim_anchor_invalid_falls_back():
    cfg = CraneConfig()
    default = (1.0, 2.0, 3.0)
    assert _parse_settle_sim_anchor({'anchor_z': 'bad'}, cfg, default) == default