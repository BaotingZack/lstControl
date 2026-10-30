"""Top navigation for 防摇测试平台 sub-pages."""

from __future__ import annotations

# id → (href, label)
PLATFORM_NAV_ITEMS = (
    ('ptp-sim', '/ptp/sim', '点到点·仿真'),
    ('ptp-live', '/ptp/live', '点到点·真机'),
    ('settle-sim', '/settle/sim', '消摆·仿真'),
    ('settle-live', '/settle/live', '消摆·真机'),
)


def platform_nav_markup(current_id: str) -> str:
    """Horizontal sub-nav; ``current_id`` gets aria-current=\"page\"."""
    parts = []
    for nav_id, href, label in PLATFORM_NAV_ITEMS:
        cur = ' aria-current="page"' if nav_id == current_id else ''
        parts.append(
            f'<a class="calibration-link ptp-subnav" data-sub="{nav_id}" '
            f'href="{href}"{cur}>{label}</a>'
        )
    return '\n          '.join(parts)
