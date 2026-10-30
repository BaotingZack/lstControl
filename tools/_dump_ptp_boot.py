import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import live_operation_html as op

t = op._BOOT_TAIL
a = t.find("function animate")
b = t.find("    els.applyTarget.addEventListener")
print("animate at", a, "listener at", b)
print(t[a : a + 900])
