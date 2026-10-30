from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
lines = (ROOT / "_dev_incl_live_server.py").read_text(encoding="utf-8").splitlines()
chunk = lines[1819:2242]
text = "\n".join(chunk)
text = text.replace(
    "_liveControlState && _liveControlState.sway",
    "_liveSwayState",
)
text = text.replace(
    "      if (!ANTI_SWAY) {\n"
    "        label('Anti-sway disabled / 未启用防摇', box.x + 12, box.y + box.h / 2, '#91a3b3');\n"
    "        return;\n"
    "      }\n",
    "",
)
text += """

    function draw() {
      var rect = canvas.getBoundingClientRect();
      ctx.clearRect(0, 0, rect.width, rect.height);
      ctx.fillStyle = '#060a0e';
      ctx.fillRect(0, 0, rect.width, rect.height);
      var box = { x: 14, y: 62, w: Math.max(200, rect.width - 28), h: Math.max(220, rect.height - 76) };
      drawHookSway(box);
    }
"""
out = ROOT / "live_ptp_sway_canvas.py"
out.write_text(
    '"""PTP left stage: Hook Sway visualization (dev_incl style)."""\n\n'
    "PTP_SWAY_STAGE_JS = "
    + repr(text)
    + "\n",
    encoding="utf-8",
)
print("wrote", out, "chars", len(text))
