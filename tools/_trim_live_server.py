"""Remove inline HTML from live_server.py; re-export page renderers."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
path = ROOT / "live_server.py"
text = path.read_text(encoding="utf-8")
start = text.index("\ndef render_ptp_html")
end = text.index("\n\nclass _LiveRequestHandler")
replacement = """

from live_operation_html import render_live_html
from live_ptp_html import render_ptp_html

"""
path.write_text(text[:start] + replacement + text[end:], encoding="utf-8")
print("Trimmed live_server.py")
