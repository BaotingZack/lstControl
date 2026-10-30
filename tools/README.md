# tools/

本目录脚本是**开发维护用**，不是运行时依赖。从仓库根目录执行，例如：

```bash
python tools/_check_ptp_els.py
```

| 脚本 | 用途 |
|------|------|
| `_split_live_pages.py` | 从旧版 monolithic server 拆出 HTML 模块（历史） |
| `_trim_live_server.py` | 去掉 server 内联 HTML |
| `_rebuild_ptp_html.py` / `_fix_ptp_html.py` | 重建 PTP 页 Python 字符串 |
| `_extract_live_views.py` | 提取视图片段 |
| `_gen_ptp_sway_js.py` | 生成 Hook Sway 画布 JS |
| `_patch_render_target_controls.py` | 修补 `view_shared`（路径现为 `web/view_shared.py`） |
| `_dump_ptp_boot.py` / `_check_ptp_els.py` | 调试 PTP 页元素 |

修改 Live 页时优先直接编辑 **`web/`** 下源文件；只有大规模重构时才考虑这些脚本。
