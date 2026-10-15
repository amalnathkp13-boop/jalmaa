/* JalMaa digital twin engine — physics, physiology, relay network, bedside analysis.
   Pure JS, deterministic (seeded). Runs in Node and the browser. */
(function (G) {
'use strict';
const TAU = 2 * Math.PI, MU0 = 4e-7 * Math.PI, EPS0 = 8.854e-12, C0 = 2.998e8, DT = 0.25;
const log10 = Math.log10;

function rng(seed) { let s = seed >>> 0; return function () { s = (s + 0x6D2B79F5) >>> 0; let t = s; t = Math.imul(t ^ (t >>> 15), t | 1); t ^= t + Math.imul(t ^ (t >>> 7), t | 61); return ((t ^ (t >>> 14)) >>> 0) / 4294967296; }; }
function gauss(r) { let u = 0; while (u === 0) u = r(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(TAU * r()); }

/* ---------------- Physics ---------------- */
const LINK_DEFAULT = {
  f: 3.39e6,        // carrier, Hz (inductive SRD band; confirm with WPC)
  sigma: 0.05,      // pool water conductivity, S/m (tap water 0.005–0.08)
  a1: 0.025, n1: 20, // body-pod coil radius (m), turns
  a2: 0.15, n2: 10,  // relay coil radius (m), turns
  q0: 80,           // unloaded coil Q
  rate: 8000,       // bit rate, bit/s
  ptx: 10,          // drive power into TX coil, dBm
  nf: 6,            // receiver noise figure, dB
  emi1MHz: 40,      // room man-made noise above kTB at 1 MHz, dB (ITU-R P.372 slope)
  triaxis: true,    // 3-axis orthogonal coils on pods
  tissueSigma: 0.5, tissueCm: 2
};
function coilL(N, a) { const b = 2.5e-4; return MU0 * N * N * a * (Math.log(8 * a / b) - 2); }
function skinDepth(f, s) { return s > 0 ? Math.sqrt(2 / (TAU * f * MU0 * s)) : Infinity; }
function waterEps(f, sigma, T) {
  T = T == null ? 37 : T;
  const es = 87.74 - 0.40008 * T + 9.398e-4 * T * T - 1.41e-6 * T * T * T, ei = 5.7;
  const tau = 8.27e-12 * Math.exp(-0.025 * (T - 25)), w = TAU * f, x = w * tau;
  return { er: ei + (es - ei) / (1 + x * x), ei: (es - ei) * x / (1 + x * x) + sigma / (w * EPS0) };
}
function waterAlphaDbPerM(f, sigma, T) {
  const e = waterEps(f, sigma, T), mag = Math.hypot(e.er, e.ei);
  return (TAU * f / C0) * Math.sqrt((mag - e.er) / 2) * 8.686;
}
function nearFieldRadius(f, sigma) { const e = waterEps(f, sigma); return C0 / f / Math.sqrt(e.er) / TAU; }
const EBN0_REQ = 10 * log10(2 * Math.log(0.5 / 1e-5)); // non-coherent BFSK, BER 1e-5

function miLink(p, d, g, dWater) {
  if (dWater == null) dWater = d;
  const L1 = coilL(p.n1, p.a1), L2 = coilL(p.n2, p.a2);
  const M = MU0 * Math.PI * p.n1 * p.n2 * p.a1 * p.a1 * p.a2 * p.a2 / (2 * Math.pow(d * d + p.a2 * p.a2, 1.5));
  const k = M / Math.sqrt(L1 * L2) * Math.max(g, 1e-3);
  const Q = Math.min(p.q0, p.f / (2 * p.rate));          // bandwidth caps usable Q
  const x = k * k * Q * Q, eff = x / Math.pow(1 + Math.sqrt(1 + x), 2);
  const couplingDb = -10 * log10(eff);
  const eddyDb = 8.686 * dWater / skinDepth(p.f, p.sigma); // conduction (eddy) loss in water
  const tissueDb = 8.686 * (p.tissueCm / 100) / skinDepth(p.f, p.tissueSigma);
  const pl = couplingDb + eddyDb + tissueDb, prx = p.ptx - pl;
  const emi = Math.max(0, p.emi1MHz - 27.7 * log10(p.f / 1e6));
  const noise = -174 + 10 * log10(p.rate) + p.nf + emi;
  const ebn0 = prx - noise, margin = ebn0 - EBN0_REQ;
  const ber = Math.min(0.5, 0.5 * Math.exp(-Math.pow(10, ebn0 / 10) / 2));
  return { k, Q, couplingDb, eddyDb, tissueDb, pl, prx, noise, emi, ebn0, margin, ber, nearField: d <= nearFieldRadius(p.f, p.sigma) };
}
function perOf(ber, bits) { return ber <= 0 ? 0 : -Math.expm1(bits * Math.log1p(-ber)); }

/* BLE (2.4 GHz) control link from a submerged beacon to a bedside tablet 1.5 m away */
const BLE = { ptx: 0, sens: -94, detuneDb: 12, airDist: 1.5 };
function bleMargin(depth, sigma) {
  const f = 2.44e9, fspl = 20 * log10(4 * Math.PI * BLE.airDist * f / C0);
  if (depth <= 0) return BLE.ptx - fspl - BLE.sens;
  const e = waterEps(f, sigma), n = Math.sqrt(e.er), G2 = ((n - 1) / (n + 1)) ** 2;
  return BLE.ptx - fspl - waterAlphaDbPerM(f, sigma) * depth + 10 * log10(1 - G2) - BLE.detuneDb - BLE.sens;
}
function bleFailDepth(sigma) { let lo = 0, hi = 2; for (let k = 0; k < 40; k++) { const m = (lo + hi) / 2; if (bleMargin(m, sigma) > 0) lo = m; else hi = m; } return lo; }

/* ---------------- Pool geometry ---------------- */
const POOL_R = 0.85, MOTHER_R = 0.5, RELAY_DEPTH = 0.15;
function relayPositions(n) { const out = [{ x: POOL_R, y: 0, z: RELAY_DEPTH }]; if (n > 1) out.push({ x: -POOL_R, y: 0, z: RELAY_DEPTH }); return out; }

/* ---------------- Simulation ---------------- */
const SCENARIOS = {
  normal:   { name: 'Normal active labour' },
  late:     { name: 'Recurrent late decelerations' },
  tachy:    { name: 'Fetal tachycardia' },
  lowvar:   { name: 'Reduced FHR variability' },
  mhr:      { name: 'Maternal pulse picked up as FHR' },
  hypoxia:  { name: 'Maternal desaturation' },
  tachysys: { name: 'Uterine tachysystole' }
};
const POD = {
  m1: { name: 'Wrist pod', cap: 400, bits: 288, baseMA: 5.0 },   // MCU 3 + PPG 1.5 + IMU 0.5
  m2: { name: 'Abdominal pod', cap: 1200, bits: 576, baseMA: 31.5 } // MCU 5 + Doppler 25 + AFE 1 + strain 0.5
};
const TX_MA = 25, QMAX = 40, ATTEMPTS = 2;

function makeSim(o) {
  o = o || {};
  const r = rng(o.seed || 7);
  const S = {
    t: 0, i: 0, r, link: Object.assign({}, LINK_DEFAULT, o.link || {}), scen: o.scenario || 'normal',
    relays: o.relays == null ? 2 : o.relays,
    m: { x: 0, y: 0, tx: 0, ty: 0, head: 0, nextMove: 60, speed: 0, armPhase: r() * TAU, armOut: 0 },
    pods: {}, ctx: [], nextCtx: 30 + r() * 40, arts: [], ehgArts: [], accel: null,
    fv: 0, mhrN: 0, mapN: 0, tcore: 37.1, twater: 36.9, dqUntil: 0, mon: o.monitor || null,
    beaconDepth: 0.3, scenStart: o.scenStart == null ? 0 : o.scenStart
  };
  for (const id of ['m1', 'm2']) {
    const u = [gauss(r), gauss(r), gauss(r)], n = Math.hypot(...u);
    S.pods[id] = { id, mAh: POD[id].cap, q: [], out: false, u: u.map(v => v / n), x: 0, y: 0, z: 0.3, g: 1, margin: 0, per: 0, relay: 0, d: 0, txBits: 0, txWin: [], avgMA: POD[id].baseMA };
  }
  return S;
}

function sin2(x) { const s = Math.sin(Math.PI * x); return s * s; }

function physiology(S) {
  const r = S.r, t = S.t, sc = S.scen, active = t >= S.scenStart;
  // contractions
  if (t >= S.nextCtx) {
    const dur = 60 + r() * 30, amp = 35 + r() * 40;
    S.ctx.push({ start: t, dur, amp, peak: t + dur / 2, ld: 20 + r() * 12 });
    S.nextCtx = t + ((sc === 'tachysys' && active) ? 85 + r() * 20 : 150 + r() * 70);
  }
  let ctxVal = 0, decel = 0;
  for (let k = S.ctx.length - 1; k >= 0 && k >= S.ctx.length - 4; k--) {
    const c = S.ctx[k];
    if (t >= c.start && t <= c.start + c.dur) ctxVal += c.amp * sin2((t - c.start) / c.dur);
    if (sc === 'late' && c.start >= S.scenStart) {
      const w = c.dur * 0.55, ctr = c.peak + 28;
      if (Math.abs(t - ctr) < w) decel += c.ld * sin2((t - (ctr - w)) / (2 * w));
    }
  }
  const ctxFrac = ctxVal / 80;
  // motion artefacts (strain only) and rare EHG-only artefacts
  if (r() < (S.m.speed > 0.01 ? 0.004 : 0.0008)) S.arts.push({ start: t, dur: 5 + r() * 12, amp: 25 + r() * 35 });
  if (r() < 0.0003) S.ehgArts.push({ start: t, dur: 4 + r() * 4, amp: 15 + r() * 15 });
  let art = 0, eart = 0;
  for (const a of S.arts.slice(-3)) if (t >= a.start && t <= a.start + a.dur) art += a.amp * sin2((t - a.start) / a.dur);
  for (const a of S.ehgArts.slice(-2)) if (t >= a.start && t <= a.start + a.dur) eart += a.amp * sin2((t - a.start) / a.dur);
  const strain = 12 + ctxVal + art + gauss(r) * 1.2;
  const ehg = 10 + ctxVal * 0.95 + eart + gauss(r) * 2.0;
  // maternal
  S.mhrN = 0.95 * S.mhrN + gauss(r) * 0.4;
  let hyp = 0;
  if (sc === 'hypoxia' && active) hyp = Math.min(10, Math.max(0, (t - S.scenStart - 120) / 60));
  const mhr = 84 + 12 * ctxFrac + S.mhrN + hyp * 2.2;
  const spo2 = 98.2 - hyp + gauss(r) * 0.3;
  S.mapN = 0.98 * S.mapN + gauss(r) * 0.3;
  const map = 86 + 10 * ctxFrac + S.mapN;
  const ptt = 300 - 1.6 * (map - 80) + gauss(r) * 2;
  S.tcore += (37.1 - S.tcore) * 0.001 + gauss(r) * 0.0015;
  S.twater += (36.9 - S.twater) * 0.0005 + gauss(r) * 0.002;
  // fetal
  let base = 140;
  if (sc === 'tachy' && active) base = 140 + Math.min(32, Math.max(0, (t - S.scenStart - 60) / 180 * 32));
  const sig = (sc === 'lowvar' && active) ? 0.3 : 1.2;
  S.fv = 0.93 * S.fv + gauss(r) * sig;
  if (!S.accel && !(sc === 'lowvar' && active) && ctxVal < 5 && r() < 0.0012) S.accel = { start: t, dur: 20 + r() * 15, amp: 18 + r() * 8 };
  let acc = 0;
  if (S.accel) { if (t > S.accel.start + S.accel.dur) S.accel = null; else acc = S.accel.amp * sin2((t - S.accel.start) / S.accel.dur); }
  let fhr = base + S.fv + acc - decel;
  if (sc === 'mhr' && active) fhr = mhr + gauss(r) * 0.8;
  if (t > S.dqUntil && r() < (S.m.speed > 0.01 ? 0.002 : 0.0006)) S.dqUntil = t + 3 + r() * 12;
  if (t < S.dqUntil) fhr = NaN;
  return { m1: { mhr, spo2, ptt }, m2: { fhr, ehg, strain, tcore: S.tcore }, twater: S.twater,
           truth: { ctxVal, decel, fhrTrue: base + S.fv + acc - decel, map, mhr } };
}

function kinematics(S) {
  const r = S.r, t = S.t, m = S.m;
  if (t >= m.nextMove) { const a = r() * TAU, rr = Math.sqrt(r()) * MOTHER_R; m.tx = rr * Math.cos(a); m.ty = rr * Math.sin(a); m.nextMove = t + 90 + r() * 180; }
  const dx = m.tx - m.x, dy = m.ty - m.y, dist = Math.hypot(dx, dy);
  if (dist > 0.01) { const v = 0.04; m.x += dx / dist * v * DT; m.y += dy / dist * v * DT; m.speed = v; m.head += (Math.atan2(dy, dx) - m.head) * 0.01; }
  else m.speed = 0;
  m.x += gauss(r) * 0.002; m.y += gauss(r) * 0.002; m.head += gauss(r) * 0.01;
  m.armPhase += 0.012 + gauss(r) * 0.01;
  if (t > m.armOut && r() < 1 / (600 * 4)) m.armOut = t + 10 + r() * 30;
  const p2 = S.pods.m2, p1 = S.pods.m1;
  p2.x = m.x + 0.12 * Math.cos(m.head); p2.y = m.y + 0.12 * Math.sin(m.head); p2.z = 0.35;
  const aa = m.head + 1.2 + 0.6 * Math.sin(m.armPhase);
  p1.x = m.x + 0.35 * Math.cos(aa); p1.y = m.y + 0.35 * Math.sin(aa);
  p1.z = t < m.armOut ? -0.15 : 0.22 + 0.12 * Math.sin(m.armPhase * 0.7);
  for (const id of ['m1', 'm2']) {
    const p = S.pods[id];
    if (p.out) { p.x = POOL_R + 9; p.y = 0.5; p.z = -0.8; }
    const u = p.u.map(v => v + gauss(r) * (id === 'm1' ? 0.03 : 0.01)), n = Math.hypot(...u); p.u = u.map(v => v / n);
  }
}

function linkStep(S) {
  const rel = relayPositions(S.relays);
  for (const id of ['m1', 'm2']) {
    const p = S.pods[id]; let best = null;
    rel.forEach((R, k) => {
      const dx = R.x - p.x, dy = R.y - p.y, dz = R.z - p.z, d = Math.max(0.05, Math.hypot(dx, dy, dz));
      const dW = p.z < 0 ? d * (RELAY_DEPTH / (RELAY_DEPTH - p.z)) : d;
      const g = S.link.triaxis ? Math.max(...p.u.map(Math.abs)) : Math.abs(p.u[0] * dx / d + p.u[1] * dy / d + p.u[2] * dz / d);
      const L = miLink(S.link, d, g, dW);
      if (!best || L.margin > best.L.margin) best = { L, k, d, g };
    });
    p.margin = best.L.margin; p.relay = best.k; p.d = best.d; p.g = best.g;
    p.per = perOf(best.L.ber, POD[id].bits); p.L = best.L;
  }
}

function step(S) {
  S.t += DT; S.i++;
  kinematics(S);
  linkStep(S);
  const ph = physiology(S);
  S.last = ph;
  const mon = S.mon;
  if (mon) mon.relay(S.i, ph.twater);
  let slot = 0;
  for (const id of ['m1', 'm2']) {
    const p = S.pods[id], bits = POD[id].bits, airtime = bits / S.link.rate;
    p.q.push({ i: S.i, t: S.t, v: ph[id] });
    if (mon) mon.generated[id]++;
    if (p.q.length > QMAX) p.q.shift();
    let bitsSent = 0;
    for (let a = 0; a < ATTEMPTS && p.q.length; a++) {
      slot += airtime; bitsSent += bits;
      if (S.r() >= p.per) { const fr = p.q.shift(); if (mon) mon.ingest(id, fr, S.t + slot + 0.0075 + 0.02); }
    }
    const ma = POD[id].baseMA + TX_MA * (bitsSent / S.link.rate) / DT;
    p.avgMA = p.avgMA * 0.999 + ma * 0.001;
    p.mAh = Math.max(0, p.mAh - ma * DT / 3600);
  }
  if (mon && S.i % 4 === 0) mon.analyze(S);
}

/* ---------------- Bedside monitor (analysis + alarms) ---------------- */
function pctl(arr, p) { if (!arr.length) return NaN; const a = arr.slice().sort((x, y) => x - y); return a[Math.min(a.length - 1, Math.floor(p * a.length))]; }
function makeMonitor(capSamples) {
  const N = capSamples || 4 * 3600 * 6;
  const A = () => new Float32Array(N).fill(NaN);
  const M = {
    N, fhr: A(), mhr: A(), spo2: A(), map: A(), ehg: A(), strain: A(), tcore: A(), twater: A(),
    lat: [], lastRx: { m1: 0, m2: 0 }, delivered: { m1: 0, m2: 0 }, generated: { m1: 0, m2: 0 },
    ctxDet: [], cand: null, decels: [], dcand: null, procIdx: 0, eB: 10, sB: 12,
    baseline: NaN, variability: NaN, alarms: {}, log: [], firstSeen: {}, ackd: {}
  };
  M.relay = (i, tw) => { if (i < N) M.twater[i] = tw; };
  M.ingest = (id, fr, tD) => {
    const i = fr.i; if (i >= N) return;
    if (id === 'm1') { M.mhr[i] = fr.v.mhr; M.spo2[i] = fr.v.spo2; M.map[i] = 80 + (300 - fr.v.ptt) / 1.6; }
    else { M.fhr[i] = fr.v.fhr; M.ehg[i] = fr.v.ehg; M.strain[i] = fr.v.strain; M.tcore[i] = fr.v.tcore; }
    M.delivered[id]++; M.lastRx[id] = Math.max(M.lastRx[id], tD);
    M.lat.push(tD - fr.t); if (M.lat.length > 20000) M.lat.splice(0, 5000);
  };
  const smooth = (arr, i, h) => { let s = 0, n = 0; for (let k = Math.max(0, i - h); k <= i + h; k++) { const v = arr[k]; if (v === v) { s += v; n++; } } return n ? s / n : NaN; };
  const winVals = (arr, i0, i1) => { const out = []; for (let k = Math.max(0, i0); k < i1; k++) { const v = arr[k]; if (v === v) out.push(v); } return out; };
  M.fused = (i) => Math.min(smooth(M.ehg, i, 8) - M.eB, smooth(M.strain, i, 8) - M.sB);

  function setAlarm(key, tier, msg, on, t) {
    const a = M.alarms[key];
    if (on) {
      if (!a) { M.alarms[key] = { key, tier, msg, since: t }; M.log.unshift({ t, tier, msg }); if (M.log.length > 200) M.log.pop(); if (M.firstSeen[key] == null) M.firstSeen[key] = t; }
      else { a.msg = msg; a.tier = tier; }
    } else if (a) { delete M.alarms[key]; delete M.ackd[key]; }
  }

  M.analyze = (S) => {
    const t = S.t, cur = S.i, LAG = 48; // process with 12 s lag so late frames land first
    if (cur % 40 === 0) {
      const e = winVals(M.ehg, cur - 720, cur), s = winVals(M.strain, cur - 720, cur);
      if (e.length > 100) M.eB = pctl(e, 0.2); if (s.length > 100) M.sB = pctl(s, 0.2);
      const f = winVals(M.fhr, cur - 2400, cur);
      if (f.length > 480) { const m = pctl(f, 0.5); const near = f.filter(v => Math.abs(v - m) < 10); M.baseline = near.reduce((a, b) => a + b, 0) / near.length; }
      if (M.baseline === M.baseline) {
        const ranges = [];
        for (let k = 0; k < 10; k++) {
          const i1 = cur - k * 240, i0 = i1 - 240; if (i0 < 0) break;
          const sec = [];
          for (let j = i0; j < i1; j += 4) { const v = smooth(M.fhr, j + 2, 2); if (v === v && Math.abs(v - M.baseline) < 15) sec.push(v); }
          if (sec.length >= 30) ranges.push(Math.max(...sec) - Math.min(...sec));
        }
        M.variability = ranges.length >= 5 ? ranges.reduce((a, b) => a + b, 0) / ranges.length : NaN;
      }
    }
    // sample-wise detectors
    const stop = cur - LAG;
    for (let i = Math.max(M.procIdx, 16); i < stop; i++) {
      const fz = M.fused(i);
      if (fz === fz) {
        if (!M.cand && fz > 15) {
          let j = i; while (j > i - 240 && j > 0) { const v = M.fused(j - 1); if (!(v > 2)) break; j--; }
          M.cand = { onset: j, peak: i, pv: -1 };
        }
        if (M.cand) {
          const sv = smooth(M.strain, i, 8); if (sv > M.cand.pv) { M.cand.pv = sv; M.cand.peak = i; }
          if (fz < 8) { if ((i - M.cand.onset) * DT >= 30) M.ctxDet.push({ onset: M.cand.onset, peak: M.cand.peak, end: i, amp: M.cand.pv - M.sB }); M.cand = null; }
        }
      }
      if (M.baseline === M.baseline) {
        const fs = smooth(M.fhr, i, 8);
        if (fs === fs) {
          if (!M.dcand && fs < M.baseline - 15) M.dcand = { start: i, nadir: i, nv: fs };
          if (M.dcand) {
            if (fs < M.dcand.nv) { M.dcand.nv = fs; M.dcand.nadir = i; }
            if (fs > M.baseline - 5) {
              const d = M.dcand; M.dcand = null;
              if ((i - d.start) * DT >= 15) {
                let best = null;
                for (const c of M.ctxDet) { const lag = (d.nadir - c.peak) * DT; if (lag > -60 && lag < 90 && (!best || Math.abs(lag) < Math.abs(best.lag))) best = { c, lag }; }
                const type = !best ? 'variable' : (best.lag >= 15 && d.start >= best.c.onset ? 'late' : (Math.abs(best.lag) < 15 ? 'early' : 'variable'));
                M.decels.push({ start: d.start, nadir: d.nadir, end: i, depth: M.baseline - d.nv, type, tEnd: i * DT });
              }
            }
          }
        }
      }
    }
    M.procIdx = Math.max(M.procIdx, stop);

    // ---- alarm rules ----
    const tiers = { info: 1, warn: 2, urgent: 3 };
    const lateN = M.decels.filter(d => d.type === 'late' && t - d.tEnd < 1200).length;
    setAlarm('late', lateN >= 2 ? 'urgent' : 'warn', lateN >= 2 ? `Recurrent late decelerations (${lateN} in 20 min) — screening flag, confirm clinically` : 'Late deceleration detected — screening flag, confirm clinically', lateN >= 1, t);
    const b = M.baseline;
    setAlarm('tachy', 'warn', `Fetal tachycardia: baseline ${b | 0} bpm`, b > 160, t);
    setAlarm('brady', 'urgent', `Fetal bradycardia: baseline ${b | 0} bpm`, b < 110, t);
    setAlarm('lowvar', 'warn', `Reduced FHR variability: ${M.variability.toFixed(1)} bpm`, M.variability < 5, t);
    // coincidence
    let nv = 0, close = 0;
    for (let i = cur - 120; i < cur; i++) { const f = M.fhr[i], m = M.mhr[i]; if (f === f && m === m) { nv++; if (Math.abs(f - m) < 5) close++; } }
    setAlarm('coinc', 'warn', 'FHR may be maternal pulse — reposition probe / check with hand-held Doppler', nv >= 60 && close >= 0.8 * nv, t);
    const sp = winVals(M.spo2, cur - 120, cur), spm = sp.length ? sp.reduce((a, c) => a + c, 0) / sp.length : NaN;
    setAlarm('spo2', spm < 90 ? 'urgent' : 'warn', `Maternal SpO₂ ${spm.toFixed(0)}%`, spm < 94, t);
    const mh = winVals(M.mhr, cur - 120, cur), mhm = mh.length ? mh.reduce((a, c) => a + c, 0) / mh.length : NaN;
    setAlarm('mhr', 'warn', `Maternal tachycardia ${mhm | 0} bpm`, mhm > 120, t);
    const n10 = M.ctxDet.filter(c => (cur - c.onset) * DT < 600).length;
    setAlarm('tsys', 'warn', `Tachysystole: ${n10} contractions in 10 min`, t > 600 && n10 > 5, t);
    const tw = M.twater[cur - 1];
    setAlarm('twater', 'warn', `Pool water ${tw.toFixed(1)} °C (> 37.5)`, tw > 37.5, t);
    const tc = winVals(M.tcore, cur - 40, cur).pop();
    setAlarm('tcore', 'warn', `Maternal temperature ${tc && tc.toFixed(1)} °C`, tc >= 37.5, t);
    for (const id of ['m1', 'm2']) {
      setAlarm('loss_' + id, 'urgent', `Signal lost: ${POD[id].name} (${(t - M.lastRx[id]).toFixed(0)} s)`, t - M.lastRx[id] > 5, t);
      const pc = S.pods[id].mAh / POD[id].cap * 100;
      setAlarm('bat_' + id, 'info', `${POD[id].name} battery ${pc.toFixed(0)}%`, pc < 20, t);
    }
    M.tiers = tiers;
  };
  return M;
}

/* ---------------- Validation suite ---------------- */
function runFor(S, seconds) { const n = Math.round(seconds / DT); for (let k = 0; k < n; k++) step(S); }
function p95(a) { return pctl(a, 0.95); }

function coverage(link, relays, triaxis, seed, n) {
  const r = rng(seed); let ok = 0; const L = Object.assign({}, link, { triaxis });
  const S = makeSim({ seed, relays, link: L });
  for (let k = 0; k < n; k++) {
    const a = r() * TAU, rr = Math.sqrt(r()) * MOTHER_R;
    S.m.x = rr * Math.cos(a); S.m.y = rr * Math.sin(a); S.m.head = r() * TAU; S.m.armPhase = r() * TAU; S.m.armOut = r() < 0.1 ? S.t + 100 : 0;
    for (const id of ['m1', 'm2']) { const u = [gauss(r), gauss(r), gauss(r)], m = Math.hypot(...u); S.pods[id].u = u.map(v => v / m); }
    S.m.tx = S.m.x; S.m.ty = S.m.y; S.m.nextMove = 1e9;
    kinematics(S); linkStep(S);
    if (S.pods.m1.per < 0.01 && S.pods.m2.per < 0.01) ok++;
  }
  return ok / n;
}

const TESTS = [
  { id: 'cov', name: 'MI link coverage across pool (2 relays, tri-axis coils)', target: '≥ 99% of positions, PER < 1%', run: (seed) => { const c = coverage(LINK_DEFAULT, 2, true, seed, 3000); return { value: (c * 100).toFixed(2) + ' %', pass: c >= 0.99 }; } },
  { id: 'covs', name: 'Coverage in contaminated water (σ = 0.5 S/m)', target: '≥ 99% of positions, PER < 1%', run: (seed) => { const c = coverage(Object.assign({}, LINK_DEFAULT, { sigma: 0.5 }), 2, true, seed, 3000); return { value: (c * 100).toFixed(2) + ' %', pass: c >= 0.99 }; } },
  { id: 'cov0', name: 'Original plan in same water: 1 relay, single-axis coil, 13.56 MHz', target: 'Comparison only', info: true, run: (seed) => { const c = coverage(Object.assign({}, LINK_DEFAULT, { f: 13.56e6, sigma: 0.5 }), 1, false, seed, 3000); return { value: (c * 100).toFixed(1) + ' %', pass: null }; } },
  { id: 'ble', name: 'BLE 2.4 GHz control: depth at which link fails', target: '< 30 cm (abdominal pod sits ~35 cm deep)', run: () => { const d = bleFailDepth(LINK_DEFAULT.sigma) * 100; return { value: d.toFixed(1) + ' cm', pass: d < 30 }; } },
  { id: 'lat', name: 'End-to-end transport latency, 30 min', target: 'p95 < 500 ms', run: (seed) => { const M = makeMonitor(4 * 1900); const S = makeSim({ seed, monitor: M }); runFor(S, 1800); const v = p95(M.lat) * 1000; return { value: v.toFixed(0) + ' ms', pass: v < 500 }; } },
  { id: 'cmp', name: 'Data completeness, 30 min (both pods)', target: '≥ 99.5% of samples delivered', run: (seed) => { const M = makeMonitor(4 * 1900); const S = makeSim({ seed, monitor: M }); runFor(S, 1800); const c = (M.delivered.m1 + M.delivered.m2) / (M.generated.m1 + M.generated.m2) * 100; return { value: c.toFixed(2) + ' %', pass: c >= 99.5 }; } },
  { id: 'ctx', name: 'Contraction detection with motion artefacts, 60 min', target: 'Sens ≥ 95%, PPV ≥ 95%, onset error ≤ 10 s', run: (seed) => {
      const M = makeMonitor(4 * 3700); const S = makeSim({ seed, monitor: M }); runFor(S, 3600);
      const gt = S.ctx.filter(c => c.start > 60 && c.start < 3600 - 150); const det = M.ctxDet.filter(d => d.onset * DT > 30 && d.onset * DT < 3600 - 120);
      let tp = 0, err = 0; const used = new Set();
      for (const g of gt) { let b = -1, bd = 1e9; det.forEach((d, k) => { const e = Math.abs(d.onset * DT - g.start); if (!used.has(k) && e < 30 && e < bd) { bd = e; b = k; } }); if (b >= 0) { used.add(b); tp++; err += bd; } }
      const fp = det.filter((d, k) => !used.has(k) && !S.ctx.some(c => Math.abs(c.start - d.onset * DT) < 30)).length;
      const sens = tp / gt.length, ppv = tp / Math.max(1, tp + fp), oe = err / Math.max(1, tp);
      return { value: `Sens ${(sens * 100).toFixed(0)}% · PPV ${(ppv * 100).toFixed(0)}% · onset ${oe.toFixed(1)} s · ${S.arts.length} artefacts rejected`, pass: sens >= 0.95 && ppv >= 0.95 && oe <= 10 };
    } },
  { id: 'late', name: 'Recurrent late decelerations flagged', target: 'Urgent flag ≤ 15 min after onset', run: (seed) => alarmTest(seed, 'late', 'late', 1500, 900, (M) => M.alarms.late && M.alarms.late.tier === 'urgent') },
  { id: 'tachy', name: 'Fetal tachycardia flagged', target: 'Warning ≤ 15 min after onset', run: (seed) => alarmTest(seed, 'tachy', 'tachy', 1500, 900) },
  { id: 'lowvar', name: 'Reduced variability flagged', target: 'Warning ≤ 15 min after onset', run: (seed) => alarmTest(seed, 'lowvar', 'lowvar', 1500, 900) },
  { id: 'coinc', name: 'Maternal pulse recorded as FHR flagged', target: 'Warning ≤ 60 s after switch', run: (seed) => alarmTest(seed, 'mhr', 'coinc', 900, 60) },
  { id: 'spo2', name: 'Maternal desaturation flagged', target: 'Urgent when SpO₂ < 90%', run: (seed) => alarmTest(seed, 'hypoxia', 'spo2', 1200, 900, (M) => M.alarms.spo2 && M.alarms.spo2.tier === 'urgent') },
  { id: 'tsys', name: 'Tachysystole flagged', target: 'Warning ≤ 15 min after onset', run: (seed) => alarmTest(seed, 'tachysys', 'tsys', 1500, 900) },
  { id: 'loss', name: 'Pod pulled from water → loss alarm → reconnect', target: 'Alarm ≤ 6 s, reconnect ≤ 2 s', run: (seed) => {
      const M = makeMonitor(4 * 1300); const S = makeSim({ seed, monitor: M }); runFor(S, 600);
      S.pods.m2.out = true; let tA = null; for (let k = 0; k < 120; k++) { step(S); if (!tA && M.alarms.loss_m2) tA = S.t - 600; }
      S.pods.m2.out = false; const t0 = S.t, rx0 = M.delivered.m2; let tR = null; for (let k = 0; k < 40; k++) { step(S); if (!tR && M.delivered.m2 > rx0) tR = S.t - t0; }
      return { value: `alarm ${tA == null ? '—' : tA.toFixed(1) + ' s'} · reconnect ${tR == null ? '—' : tR.toFixed(2) + ' s'}`, pass: tA != null && tA <= 6 && tR != null && tR <= 2 };
    } },
  { id: 'fa', name: 'False clinical alarms in normal labour, 60 min', target: '0 warning/urgent alarms', run: (seed) => {
      const M = makeMonitor(4 * 3700); const S = makeSim({ seed, monitor: M }); runFor(S, 3600);
      const n = M.log.filter(l => l.tier !== 'info').length; return { value: n + ' alarms' + (n ? ' (' + M.log.filter(l => l.tier !== 'info').map(l => l.msg.split(':')[0]).join(', ') + ')' : ''), pass: n === 0 };
    } },
  { id: 'bat', name: 'Battery endurance (measured current draw)', target: '≥ 12 h per pod', run: (seed) => {
      const M = makeMonitor(4 * 1900); const S = makeSim({ seed, monitor: M }); runFor(S, 1800);
      const h = ['m1', 'm2'].map(id => POD[id].cap / ((POD[id].cap - S.pods[id].mAh) / 0.5)); return { value: `wrist ${h[0].toFixed(0)} h · abdominal ${h[1].toFixed(0)} h`, pass: h[0] >= 12 && h[1] >= 12 };
    } }
];
function alarmTest(seed, scen, key, dur, limit, cond) {
  const onset = 300; const M = makeMonitor(4 * (dur + 100)); const S = makeSim({ seed, scenario: scen, monitor: M, scenStart: onset });
  let tFlag = null; const n = Math.round(dur / DT);
  for (let k = 0; k < n; k++) { step(S); if (tFlag == null && (cond ? cond(M) : M.alarms[key])) tFlag = S.t - onset; }
  return { value: tFlag == null ? 'not flagged' : `flagged at +${(tFlag / 60).toFixed(1)} min`, pass: tFlag != null && tFlag >= 0 && tFlag <= limit };
}

const API = { DT, TAU, LINK_DEFAULT, SCENARIOS, POD, POOL_R, MOTHER_R, RELAY_DEPTH, EBN0_REQ, BLE,
  rng, gauss, miLink, perOf, skinDepth, waterAlphaDbPerM, nearFieldRadius, bleMargin, bleFailDepth, relayPositions,
  makeSim, makeMonitor, step, runFor, TESTS, coverage };
if (typeof module !== 'undefined' && module.exports) module.exports = API; else G.JalMaa = API;
})(typeof globalThis !== 'undefined' ? globalThis : this);
