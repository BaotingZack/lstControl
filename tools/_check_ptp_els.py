import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import live_ptp_html as p
from live_view_shared import LIVE_CANVAS_JS

script = p._BOOT + LIVE_CANVAS_JS
body = p._PAGE_BODY

ids = set(re.findall(r'id="([^"]+)"', body))
els_refs = set(re.findall(r"els\.(\w+)", script))
value_refs = set(re.findall(r"els\.(\w+)\.value", script))

missing = []
for name in sorted(value_refs):
    if name not in els_refs:
        continue
    # element must exist in DOM or be optional
    if name in ("targetCoord", "commandMsg"):
        if name not in ids:
            missing.append(f"{name} missing id in body")
    elif name in ("targetX", "targetY", "targetZ", "startX", "startY", "startZ"):
        missing.append(f"{name} still referenced in script")

print("Missing value targets:")
for m in missing:
    print(" ", m)

# show lines with startX targetX in script
for pat in ["startX", "startY", "startZ", "targetX", "targetY", "targetZ"]:
    if pat in script:
        i = script.find(f"els.{pat}.value")
        if i >= 0:
            print(f"FOUND els.{pat}.value at", i, script[i : i + 80].replace("\n", " "))
