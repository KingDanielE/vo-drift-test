// Draws the page's charts as inline SVG from window.VO_DATA (exported from the evaluation runs).
(function () {
  const D = window.VO_DATA;
  const NS = "http://www.w3.org/2000/svg";

  function el(parent, tag, attrs, text) {
    const n = document.createElementNS(NS, tag);
    for (const k in attrs) n.setAttribute(k, attrs[k]);
    if (text !== undefined) n.textContent = text;
    parent.appendChild(n);
    return n;
  }
  const path = (pts) => pts.map((p, i) => (i ? "L" : "M") + p[0].toFixed(1) + " " + p[1].toFixed(1)).join("");

  function niceTicks(min, max, count) {
    const span = max - min || 1;
    const step0 = span / count;
    const mag = Math.pow(10, Math.floor(Math.log10(step0)));
    const step = [1, 2, 2.5, 5, 10].map((m) => m * mag).find((s) => s >= step0);
    const ticks = [];
    for (let v = Math.ceil(min / step) * step; v <= max + 1e-9; v += step) ticks.push(+v.toFixed(6));
    return ticks;
  }
  const fixed = (v, d) => (Math.round(v * 10 ** d + 1e-9) / 10 ** d).toFixed(d); // 0.605 -> "0.61"
  const fmt = (v) => (Math.abs(v) >= 10 || Number.isInteger(v) ? String(Math.round(v)) : v.toFixed(1));

  // Top-view trajectory with equal axis scaling.
  function trajectory(svg, t) {
    const W = 400, H = 400, pad = 34;
    svg.setAttribute("viewBox", `0 0 ${W} ${H}`);
    const all = t.gt.concat(t.est);
    let [x0, x1] = [Math.min(...all.map((p) => p[0])), Math.max(...all.map((p) => p[0]))];
    let [y0, y1] = [Math.min(...all.map((p) => p[1])), Math.max(...all.map((p) => p[1]))];
    const span = Math.max(x1 - x0, y1 - y0) * 1.08;
    const cx = (x0 + x1) / 2, cy = (y0 + y1) / 2;
    x0 = cx - span / 2; y0 = cy - span / 2;
    const s = (W - 2 * pad) / span;
    const X = (x) => pad + (x - x0) * s;
    const Y = (y) => H - pad - (y - y0) * s;
    for (const v of niceTicks(x0, x0 + span, 5)) {
      el(svg, "line", { x1: X(v), x2: X(v), y1: pad, y2: H - pad, class: "gridline" });
      el(svg, "text", { x: X(v), y: H - pad + 16, "text-anchor": "middle" }, fmt(v));
    }
    for (const v of niceTicks(y0, y0 + span, 5)) {
      el(svg, "line", { x1: pad, x2: W - pad, y1: Y(v), y2: Y(v), class: "gridline" });
      el(svg, "text", { x: pad - 6, y: Y(v) + 4, "text-anchor": "end" }, fmt(v));
    }
    el(svg, "text", { x: W - pad, y: H - 4, "text-anchor": "end" }, "x [m]");
    el(svg, "path", { d: path(t.gt.map((p) => [X(p[0]), Y(p[1])])), class: "gt" });
    el(svg, "path", { d: path(t.est.map((p) => [X(p[0]), Y(p[1])])), class: "vo" });
    el(svg, "circle", { cx: X(t.gt[0][0]), cy: Y(t.gt[0][1]), r: 4.5, class: "start" });
    el(svg, "text", { x: X(t.gt[0][0]) + 8, y: Y(t.gt[0][1]) - 8 }, "start");
  }

  // Line chart. series: [{pts:[[x,y]], cls, dots}], opts: {xLabel, yLabel, xTicks, xFmt, bars, H}
  function lineChart(svg, series, o) {
    const W = o.W || 480, H = o.H || 220, L = 40, R = o.bars ? 22 : 12, T = 14, B = 30;
    svg.setAttribute("viewBox", `0 0 ${W} ${H}`);
    const xs = series.flatMap((s) => s.pts.map((p) => p[0]));
    const ys = series.flatMap((s) => s.pts.map((p) => p[1]));
    const xmin = o.xmin ?? Math.min(...xs), xmax = o.xmax ?? Math.max(...xs);
    const ymax = (o.ymax ?? Math.max(...ys)) * 1.12;
    const X = (x) => L + ((x - xmin) / (xmax - xmin || 1)) * (W - L - R);
    const Y = (y) => H - B - (y / ymax) * (H - T - B);
    for (const v of niceTicks(0, ymax, 4)) {
      el(svg, "line", { x1: L, x2: W - R, y1: Y(v), y2: Y(v), class: "gridline" });
      el(svg, "text", { x: L - 6, y: Y(v) + 4, "text-anchor": "end" }, fmt(v));
    }
    const xt = o.xTicks || niceTicks(xmin, xmax, 6);
    xt.forEach((v, i) => el(svg, "text", { x: X(v), y: H - B + 16, "text-anchor": "middle" }, o.xFmt ? o.xFmt(v, i) : fmt(v)));
    el(svg, "line", { x1: L, x2: W - R, y1: H - B, y2: H - B, class: "axis" });
    if (o.bars) {
      const bmax = Math.max(...o.bars.map((b) => b[1]), 1) * 1.8; // keep bars in the lower half
      const bw = Math.min(22, (W - L - R) / o.bars.length / 2.5);
      for (const [x, v] of o.bars) {
        const h = (v / bmax) * (H - T - B);
        el(svg, "rect", { x: X(x) - bw / 2, y: H - B - h, width: bw, height: h, rx: 2, class: "bar" });
        if (v > 0) el(svg, "text", { x: X(x) + bw / 2 + 3, y: H - B - h + 10, "font-size": 10 }, String(v));
      }
    }
    for (const s of series) {
      el(svg, "path", { d: path(s.pts.map((p) => [X(p[0]), Y(p[1])])), class: s.cls });
      if (s.dots) for (const p of s.pts) el(svg, "circle", { cx: X(p[0]), cy: Y(p[1]), r: 3.2, class: s.dots });
    }
    if (o.yLabel) el(svg, "text", { x: L, y: T - 2, "text-anchor": "start" }, o.yLabel);
    if (o.xLabel) el(svg, "text", { x: W - R, y: H - 4, "text-anchor": "end" }, o.xLabel);
  }

  // Hero: real feature tracks over the camera frame, drawn in once on load.
  const hero = document.getElementById("hero-tracks");
  (window.VO_TRACKS || []).forEach((t, i) => {
    const d = `${((i * 37) % 60) / 100}s`;
    el(hero, "path", { d: path(t), class: "trk", pathLength: 1, style: `--d:${d}` });
    const end = t[t.length - 1];
    el(hero, "circle", { cx: end[0], cy: end[1], r: 3.2, class: "trk-dot", style: `--d:${d}` });
  });

  // Baseline
  trajectory(document.getElementById("chart-traj"), D.euroc);
  lineChart(document.getElementById("chart-err"), [{ pts: D.euroc.err, cls: "vo" }], {
    xLabel: "distance travelled [m]", yLabel: "error [m]", xmin: 0, H: 190,
  });
  const m = D.euroc.metrics;
  const seg = D.segments.map((d) => [d, (m[`drift_${d}m_pct`] * d) / 100]);
  lineChart(document.getElementById("chart-seg"), [{ pts: [[0, 0]].concat(seg), cls: "vo", dots: "dot-vo" }], {
    xLabel: "segment length [m]", yLabel: "mean error [m]", xmin: 0, xTicks: [0].concat(D.segments), H: 170,
  });

  // Stress tests
  const FACTORS = [
    { key: "noise_sigma", title: "Image noise", unit: "Gaussian σ", labels: ["0", "10", "20", "30"] },
    { key: "blur_px", title: "Blur", unit: "kernel px", labels: ["0", "5", "9", "15"] },
    { key: "frame_skip", title: "Dropped frames", unit: "frames used", labels: ["all", "½", "⅓", "¼"] },
    { key: "brightness", title: "Darkness", unit: "brightness", labels: ["100%", "50%", "30%", "15%"] },
  ];
  const base = D.stress.find((r) => r.run === "baseline");
  const holder = document.getElementById("stress-charts");
  for (const f of FACTORS) {
    const rows = [base].concat(D.stress.filter((r) => r.factor === f.key));
    const card = document.createElement("figure");
    card.className = "panel stress-card";
    card.innerHTML = `<h3>${f.title} <span class="unit">(${f.unit})</span></h3>`;
    const svg = document.createElementNS(NS, "svg");
    svg.setAttribute("class", "chart");
    svg.setAttribute("role", "img");
    svg.setAttribute("aria-label", `${f.title}: drift and rejected steps by level`);
    card.appendChild(svg);
    holder.appendChild(card);
    const idx = rows.map((_, i) => i);
    lineChart(svg, [
      { pts: rows.map((r, i) => [i, r.drift_2m_pct]), cls: "short", dots: "dot-short" },
      { pts: rows.map((r, i) => [i, r.drift_10m_pct]), cls: "vo", dots: "dot-vo" },
    ], {
      xTicks: idx, xFmt: (v) => f.labels[v], ymax: 13, yLabel: "drift [%]", W: 420, H: 220,
      bars: rows.map((r, i) => [i, r.failures + r.rejected]),
    });
  }

  // stress table
  const NAMES = {
    baseline: "Clean", brightness: "Brightness", blur_px: "Blur", noise_sigma: "Noise σ", frame_skip: "Every",
  };
  const levelText = (r) => {
    if (r.factor === "brightness") return `${Math.round(r.level * 100)}%`;
    if (r.factor === "blur_px") return `${r.level} px`;
    if (r.factor === "frame_skip") return { 2: "2nd", 3: "3rd", 4: "4th" }[r.level] + " frame";
    return String(r.level);
  };
  const tbody = document.querySelector("#stress-table tbody");
  for (const r of D.stress) {
    const tr = document.createElement("tr");
    if (r.run === "baseline") tr.className = "base";
    const hot = (v, b) => (v >= b * 2 ? ' class="hot"' : "");
    const lost = r.failures + r.rejected;
    tr.innerHTML =
      `<td>${r.run === "baseline" ? NAMES.baseline : NAMES[r.factor] + " " + levelText(r)}</td>` +
      `<td${hot(r.ate_rmse_m, base.ate_rmse_m)}>${fixed(r.ate_rmse_m, 2)} m</td>` +
      `<td${hot(r.drift_2m_pct, base.drift_2m_pct)}>${fixed(r.drift_2m_pct, 1)}%</td>` +
      `<td${hot(r.drift_10m_pct, base.drift_10m_pct)}>${fixed(r.drift_10m_pct, 1)}%</td>` +
      `<td${lost >= 15 ? ' class="hot"' : ""}>${lost}</td>`;
    tbody.appendChild(tr);
  }

  // Flat vs blocks
  trajectory(document.getElementById("chart-blocks"), D.blocks);
  trajectory(document.getElementById("chart-flatc"), D.flat);
  document.getElementById("blocks-tag").textContent = `${fixed(D.blocks.metrics.drift_10m_pct, 1)}% drift per 10 m`;
  document.getElementById("flat-tag").textContent = `${fixed(D.flat.metrics.drift_10m_pct, 1)}% drift per 10 m`;
})();
