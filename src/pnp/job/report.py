"""The job report (docs/13 §4.7): `report.html` in the job's folder, next to state.json and the photos. Plain HTML,
no scripts, works offline; the photos are linked relatively, so the folder can be zipped and sent."""
from __future__ import annotations

import html
import os
import time
from pathlib import Path


def _rel(folder: Path, p: str) -> str:
    try:
        return os.path.relpath(p, folder)
    except ValueError:
        return p


def write_report(job) -> Path:
    folder = Path(job.folder)
    folder.mkdir(parents=True, exist_ok=True)
    e = html.escape
    s = job.summary()
    rows = []
    for p in job.parts:
        res = p.result or {}
        err = res.get("error")
        photos = {**(res.get("photos") or {})}
        for i, miss in enumerate(res.get("misses", []) or []):
            photos.update({f"miss{i + 1}_{k}": v for k, v in miss.items()})
        links = " ".join(f'<a href="{e(_rel(folder, v))}">{e(k)}</a>' for k, v in photos.items())
        rows.append(f"<tr class='{e(p.state)}'><td>{e(p.ref)}</td><td>{e(p.value)}</td><td>{e(p.package)}</td>"
                    f"<td>{p.tray or ''}</td><td>{e(p.state)}</td><td>{e(p.reason)}</td>"
                    f"<td>{'' if not err else f'{err[0]:+.3f}, {err[1]:+.3f}'}</td>"
                    f"<td>{'' if res.get('angle_error') is None else res['angle_error']}</td><td>{links}</td></tr>")
    picks = ""
    if job.mode == "pick_test":
        miss = [p for p in job.picks if p["verdict"] != "picked"]
        picks = (f"<h2>Pick test</h2><p>{s['picks']['picked']} of {s['picks']['done']} picked"
                 f" ({0 if not s['picks']['done'] else 100 * s['picks']['picked'] / s['picks']['done']:.1f} %).</p>"
                 + "".join(f"<p>Pick {p['n']} (tray {p['tray']}): {e(p['verdict'])} "
                           + " ".join(f'<a href="{e(_rel(folder, v))}">{e(k)}</a>' for k, v in p["photos"].items())
                           + "</p>" for p in miss))
    skipped = "".join(f"<tr><td>{e(x['ref'])}</td><td>{e(x.get('value', ''))}</td><td>{e(x.get('package', ''))}</td>"
                      f"<td>{e(x['reason'])}</td></tr>" for x in job.skipped)
    drift = "".join(f"<li>{time.strftime('%H:%M:%S', time.localtime(h['t']))}: "
                    + (f"{h['mm']} mm ({h['drift'][0]:+.3f}, {h['drift'][1]:+.3f})" if "mm" in h else e(h.get('why', '')))
                    + "</li>" for h in job.home_checks)
    dur = (job.ended or time.time()) - job.started if job.started else 0
    doc = f"""<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width">
<title>Job {e(job.id)}</title><style>
body{{font:14px system-ui,sans-serif;margin:16px;color:#18202a}} table{{border-collapse:collapse}}
td,th{{border:1px solid #ccd;padding:3px 6px;text-align:left}} tr.verified td{{background:#e8f6ee}}
tr.failed td{{background:#fbeaea}} tr.skipped td{{color:#777}} .warn{{color:#a60}}
</style></head><body>
<h1>{e(job.board)}, side {e(job.side)}: {e(job.mode)} job</h1>
<p>Started {e(job.created)} · status <b>{e(job.status)}</b>{(' (' + e(job.pause.get('why', '')) + ')') if job.pause else ''}
 · {round(dur / 60, 1)} min · {", ".join(f"{v} {k}" for k, v in sorted(s['counts'].items()))}</p>
{('<p class="warn">Started with stale calibration (the operator chose to): ' + e(', '.join(job.overrides)) + '</p>') if job.overrides else ''}
<h2>HOME checks</h2><ul>{drift or '<li>none</li>'}</ul>
{picks}
<h2>Parts</h2><table><tr><th>Ref</th><th>Value</th><th>Package</th><th>Tray</th><th>State</th><th>Why</th>
<th>Error X, Y (mm)</th><th>Angle (°)</th><th>Photos</th></tr>{''.join(rows)}</table>
<h2>Not placed by the machine</h2><table><tr><th>Ref</th><th>Value</th><th>Package</th><th>Why</th></tr>{skipped}</table>
<h2>Log</h2><pre>{e(chr(10).join(job.log))}</pre></body></html>"""
    out = folder / "report.html"
    out.write_text(doc)
    return out
