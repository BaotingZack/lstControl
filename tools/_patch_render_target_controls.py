from pathlib import Path

path = Path(__file__).resolve().parents[1] / "live_view_shared.py"
text = path.read_text(encoding="utf-8")
old = (
    "    function renderTargetControls() {\\n"
    "      els.targetX.value = payload.target.x.toFixed(2);\\n"
    "      els.targetY.value = payload.target.y.toFixed(2);\\n"
    "      els.targetZ.value = payload.target.z.toFixed(2);\\n"
    "    }\\n"
)
new = (
    "    function renderTargetControls() {\\n"
    "      if (!payload || !payload.target) return;\\n"
    "      if (els.targetCoord) {\\n"
    "        els.targetCoord.value = payload.target.x.toFixed(2) + ', ' + payload.target.y.toFixed(2) + ', ' + payload.target.z.toFixed(2);\\n"
    "      } else if (els.targetX) {\\n"
    "        els.targetX.value = payload.target.x.toFixed(2);\\n"
    "        els.targetY.value = payload.target.y.toFixed(2);\\n"
    "        els.targetZ.value = payload.target.z.toFixed(2);\\n"
    "      }\\n"
    "    }\\n"
)
if old not in text:
    raise SystemExit("renderTargetControls block not found")
path.write_text(text.replace(old, new, 1), encoding="utf-8")
print("patched live_view_shared.py")
