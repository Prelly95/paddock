// Draws the four canvas-based instruments: MiniMap, ADI, compass, wind rose.

const ADI = (() => {
  const canvas = document.getElementById('adiCanvas');
  const ctx    = canvas.getContext('2d');
  const W = canvas.width;
  const H = canvas.height;
  const CX = W / 2;
  const CY = H / 2;
  const R  = Math.min(W, H) / 2 - 2;

  let roll  = 0;
  let pitch = 0;

  function draw() {
    ctx.clearRect(0, 0, W, H);

    ctx.save();
    ctx.beginPath();
    ctx.arc(CX, CY, R, 0, Math.PI * 2);
    ctx.clip();

    // rotate by roll
    ctx.translate(CX, CY);
    ctx.rotate(-roll * Math.PI / 180);

    // pitch offset: 1 degree = 1.8px at this scale
    const pitchOffset = pitch * 1.8;

    // sky
    ctx.fillStyle = getComputedStyle(document.documentElement).getPropertyValue('--adi-sky').trim();
    ctx.fillRect(-R, -R - pitchOffset, R * 2, R * 2);

    // ground
    ctx.fillStyle = getComputedStyle(document.documentElement).getPropertyValue('--adi-ground').trim();
    ctx.fillRect(-R, -pitchOffset, R * 2, R * 2);

    // horizon line
    ctx.strokeStyle = getComputedStyle(document.documentElement).getPropertyValue('--adi-horizon').trim();
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(-R, -pitchOffset);
    ctx.lineTo(R,  -pitchOffset);
    ctx.stroke();

    // pitch ladder — every 10 degrees
    ctx.strokeStyle = 'rgba(214,220,211,.5)';
    ctx.fillStyle   = 'rgba(214,220,211,.6)';
    ctx.lineWidth   = 1;
    ctx.font        = '8px monospace';
    ctx.textAlign   = 'center';
    for (let deg = -30; deg <= 30; deg += 10) {
      if (deg === 0) continue;
      const y = -pitchOffset + deg * -1.8;
      const w = deg % 20 === 0 ? 28 : 16;
      ctx.beginPath();
      ctx.moveTo(-w, y);
      ctx.lineTo(w,  y);
      ctx.stroke();
      if (deg % 20 === 0) {
        ctx.fillText(Math.abs(deg), w + 8, y + 3);
        ctx.fillText(Math.abs(deg), -w - 8, y + 3);
      }
    }

    ctx.restore();

    // border ring
    ctx.strokeStyle = 'rgba(39,47,49,.9)';
    ctx.lineWidth   = 3;
    ctx.beginPath();
    ctx.arc(CX, CY, R, 0, Math.PI * 2);
    ctx.stroke();

    // roll arc ticks
    ctx.save();
    ctx.translate(CX, CY);
    ctx.strokeStyle = 'rgba(214,220,211,.4)';
    ctx.lineWidth   = 1;
    for (const angle of [-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60]) {
      ctx.save();
      ctx.rotate(angle * Math.PI / 180);
      ctx.beginPath();
      ctx.moveTo(0, -R);
      ctx.lineTo(0, -R + (angle % 30 === 0 ? 7 : 4));
      ctx.stroke();
      ctx.restore();
    }
    ctx.restore();

    // roll pointer
    ctx.save();
    ctx.translate(CX, CY);
    ctx.rotate(-roll * Math.PI / 180);
    ctx.fillStyle   = getComputedStyle(document.documentElement).getPropertyValue('--accent').trim();
    ctx.strokeStyle = getComputedStyle(document.documentElement).getPropertyValue('--accent').trim();
    ctx.lineWidth   = 1.5;
    ctx.beginPath();
    ctx.moveTo(0,  -R + 12);
    ctx.lineTo(-5, -R + 20);
    ctx.lineTo(5,  -R + 20);
    ctx.closePath();
    ctx.fill();
    ctx.restore();

    // fixed center cross
    ctx.strokeStyle = getComputedStyle(document.documentElement).getPropertyValue('--accent').trim();
    ctx.lineWidth   = 1.5;
    ctx.beginPath();
    ctx.moveTo(CX - 20, CY);
    ctx.lineTo(CX - 6,  CY);
    ctx.moveTo(CX + 6,  CY);
    ctx.lineTo(CX + 20, CY);
    ctx.moveTo(CX, CY - 4);
    ctx.lineTo(CX, CY + 4);
    ctx.stroke();
  }

  return {
    update(r, p) {
      roll  = r;
      pitch = p;
      draw();
    },
    init: draw
  };
})();


const Compass = (() => {
  const canvas = document.getElementById('compassCanvas');
  const ctx    = canvas.getContext('2d');
  const W = canvas.width;
  const H = canvas.height;
  const CX = W / 2;
  const CY = H / 2;
  const R  = Math.min(W, H) / 2 - 2;

  let heading = 0;

  const CARDINALS = ['N','NE','E','SE','S','SW','W','NW'];

  function draw() {
    ctx.clearRect(0, 0, W, H);

    // background
    ctx.fillStyle = getComputedStyle(document.documentElement).getPropertyValue('--compass-ring').trim();
    ctx.beginPath();
    ctx.arc(CX, CY, R, 0, Math.PI * 2);
    ctx.fill();

    // ring border
    ctx.strokeStyle = getComputedStyle(document.documentElement).getPropertyValue('--line').trim();
    ctx.lineWidth   = 1.5;
    ctx.beginPath();
    ctx.arc(CX, CY, R, 0, Math.PI * 2);
    ctx.stroke();

    ctx.save();
    ctx.translate(CX, CY);
    ctx.rotate(-heading * Math.PI / 180);

    // degree ticks
    ctx.strokeStyle = 'rgba(214,220,211,.3)';
    ctx.lineWidth   = 1;
    for (let i = 0; i < 360; i += 5) {
      const rad = i * Math.PI / 180;
      const len = i % 10 === 0 ? 7 : 4;
      ctx.save();
      ctx.rotate(rad);
      ctx.beginPath();
      ctx.moveTo(0, -R + 2);
      ctx.lineTo(0, -R + 2 + len);
      ctx.stroke();
      ctx.restore();
    }

    // cardinals
    ctx.textAlign    = 'center';
    ctx.textBaseline = 'middle';
    CARDINALS.forEach((lbl, i) => {
      const angle = i * 45 * Math.PI / 180;
      const dist  = R - 18;
      const x = Math.sin(angle) * dist;
      const y = -Math.cos(angle) * dist;
      ctx.font      = i % 2 === 0 ? 'bold 10px monospace' : '8px monospace';
      ctx.fillStyle = lbl === 'N'
        ? getComputedStyle(document.documentElement).getPropertyValue('--compass-north').trim()
        : 'rgba(214,220,211,.8)';
      ctx.fillText(lbl, x, y);
    });

    ctx.restore();

    // fixed lubber line
    ctx.strokeStyle = getComputedStyle(document.documentElement).getPropertyValue('--accent').trim();
    ctx.lineWidth   = 2;
    ctx.beginPath();
    ctx.moveTo(CX, CY - R + 2);
    ctx.lineTo(CX, CY - R + 14);
    ctx.stroke();

    // center dot
    ctx.fillStyle = getComputedStyle(document.documentElement).getPropertyValue('--accent').trim();
    ctx.beginPath();
    ctx.arc(CX, CY, 3, 0, Math.PI * 2);
    ctx.fill();
  }

  return {
    update(hdg) {
      heading = hdg;
      draw();
    },
    init: draw
  };
})();


const WindRose = (() => {
  const canvas = document.getElementById('windCanvas');
  const ctx    = canvas.getContext('2d');
  const W = canvas.width;
  const H = canvas.height;
  const CX = W / 2;
  const CY = H / 2;
  const R  = Math.min(W, H) / 2 - 4;

  let dir   = 0;
  let speed = 0;

  function draw() {
    ctx.clearRect(0, 0, W, H);

    // background ring
    ctx.strokeStyle = getComputedStyle(document.documentElement).getPropertyValue('--line').trim();
    ctx.lineWidth   = 1;
    ctx.beginPath();
    ctx.arc(CX, CY, R, 0, Math.PI * 2);
    ctx.stroke();

    // inner dashed ring
    ctx.setLineDash([3, 4]);
    ctx.beginPath();
    ctx.arc(CX, CY, R * 0.55, 0, Math.PI * 2);
    ctx.stroke();
    ctx.setLineDash([]);

    // cardinal crosses
    ctx.strokeStyle = 'rgba(214,220,211,.15)';
    ctx.lineWidth   = 1;
    ctx.beginPath();
    ctx.moveTo(CX, CY - R); ctx.lineTo(CX, CY + R);
    ctx.moveTo(CX - R, CY); ctx.lineTo(CX + R, CY);
    ctx.stroke();

    // speed-mapped arrow length
    const maxSpd  = 20;
    const arrowR  = Math.min(speed / maxSpd, 1) * (R - 10) + 10;
    const rad     = dir * Math.PI / 180;

    const tipX  = CX + Math.sin(rad) * arrowR;
    const tipY  = CY - Math.cos(rad) * arrowR;

    // arrow shaft
    const arrowColor = speed > 0
      ? getComputedStyle(document.documentElement).getPropertyValue('--info').trim()
      : getComputedStyle(document.documentElement).getPropertyValue('--muted').trim();
    ctx.strokeStyle = arrowColor;
    ctx.lineWidth   = 2;
    ctx.beginPath();
    ctx.moveTo(CX, CY);
    ctx.lineTo(tipX, tipY);
    ctx.stroke();

    // arrowhead
    if (speed > 0) {
      const headLen = 10;
      const headAngle = 0.45;
      ctx.fillStyle = arrowColor;
      ctx.beginPath();
      ctx.moveTo(tipX, tipY);
      ctx.lineTo(
        tipX - headLen * Math.sin(rad - headAngle),
        tipY + headLen * Math.cos(rad - headAngle)
      );
      ctx.lineTo(
        tipX - headLen * Math.sin(rad + headAngle),
        tipY + headLen * Math.cos(rad + headAngle)
      );
      ctx.closePath();
      ctx.fill();
    }

    // center dot
    ctx.fillStyle = getComputedStyle(document.documentElement).getPropertyValue('--muted').trim();
    ctx.beginPath();
    ctx.arc(CX, CY, 3, 0, Math.PI * 2);
    ctx.fill();
  }

  return {
    update(d, s) {
      dir   = d;
      speed = s;
      draw();
    },
    init: draw
  };
})();


// ── MiniMap ──────────────────────────────────────────────────────────────────
// Plots a trail of recent GPS positions on a canvas.  The view auto-centres
// on the rover and scales to keep the whole trail visible.

const MiniMap = (() => {
  const canvas  = document.getElementById('minimapCanvas');
  const ctx     = canvas.getContext('2d');
  const W       = canvas.width;
  const H       = canvas.height;
  const PAD     = 10;
  const MAX_PTS = 300;

  const trail = [];   // { lat, lon }
  let heading = 0;

  // Equirectangular projection: degrees → pixels relative to trail centre.
  function project(lat, lon, cLat, cLon, scale) {
    const x =  (lon - cLon) * Math.cos(cLat * Math.PI / 180) * scale;
    const y = -(lat - cLat) * scale;
    return { x: W / 2 + x, y: H / 2 + y };
  }

  function draw() {
    ctx.clearRect(0, 0, W, H);

    // background
    ctx.fillStyle = getComputedStyle(document.documentElement).getPropertyValue('--panel2').trim();
    ctx.fillRect(0, 0, W, H);

    // border
    ctx.strokeStyle = getComputedStyle(document.documentElement).getPropertyValue('--line').trim();
    ctx.lineWidth   = 1;
    ctx.strokeRect(0.5, 0.5, W - 1, H - 1);

    if (trail.length < 2) {
      // placeholder cross when no data
      ctx.strokeStyle = getComputedStyle(document.documentElement).getPropertyValue('--muted').trim();
      ctx.lineWidth   = 1;
      ctx.beginPath();
      ctx.moveTo(W / 2 - 8, H / 2); ctx.lineTo(W / 2 + 8, H / 2);
      ctx.moveTo(W / 2, H / 2 - 8); ctx.lineTo(W / 2, H / 2 + 8);
      ctx.stroke();
      return;
    }

    // compute bounding box of all trail points
    let minLat = Infinity, maxLat = -Infinity;
    let minLon = Infinity, maxLon = -Infinity;
    for (const p of trail) {
      if (p.lat < minLat) minLat = p.lat;
      if (p.lat > maxLat) maxLat = p.lat;
      if (p.lon < minLon) minLon = p.lon;
      if (p.lon > maxLon) maxLon = p.lon;
    }

    const cLat = (minLat + maxLat) / 2;
    const cLon = (minLon + maxLon) / 2;
    const cosLat = Math.cos(cLat * Math.PI / 180);

    // scale to fit bounding box inside canvas (with padding)
    const dLat = Math.max(maxLat - minLat, 1e-6);
    const dLon = Math.max((maxLon - minLon) * cosLat, 1e-6);
    const DEG  = 111_320; // metres per degree latitude
    const scaleY = (H - PAD * 2) / (dLat * DEG);
    const scaleX = (W - PAD * 2) / (dLon * DEG);
    const scale  = Math.min(scaleX, scaleY) * DEG;

    // grid lines (subtle)
    ctx.strokeStyle = getComputedStyle(document.documentElement).getPropertyValue('--line').trim();
    ctx.lineWidth   = 0.5;
    ctx.beginPath();
    ctx.moveTo(W / 2, 0); ctx.lineTo(W / 2, H);
    ctx.moveTo(0, H / 2); ctx.lineTo(W, H / 2);
    ctx.stroke();

    // trail
    ctx.strokeStyle = getComputedStyle(document.documentElement).getPropertyValue('--info').trim();
    ctx.lineWidth   = 1.5;
    ctx.lineJoin    = 'round';
    ctx.globalAlpha = 0.6;
    ctx.beginPath();
    trail.forEach((p, i) => {
      const { x, y } = project(p.lat, p.lon, cLat, cLon, scale);
      if (i === 0) ctx.moveTo(x, y);
      else         ctx.lineTo(x, y);
    });
    ctx.stroke();
    ctx.globalAlpha = 1;

    // rover position marker
    const last    = trail[trail.length - 1];
    const { x, y } = project(last.lat, last.lon, cLat, cLon, scale);

    // heading arrow
    ctx.save();
    ctx.translate(x, y);
    ctx.rotate(heading * Math.PI / 180);
    ctx.fillStyle = getComputedStyle(document.documentElement).getPropertyValue('--accent').trim();
    ctx.beginPath();
    ctx.moveTo(0, -9);
    ctx.lineTo(-5, 5);
    ctx.lineTo(0, 2);
    ctx.lineTo(5, 5);
    ctx.closePath();
    ctx.fill();
    ctx.restore();

    // scale bar (bottom-left)
    const barMetres = 5;
    const barPx     = barMetres * scale / DEG;
    ctx.strokeStyle = getComputedStyle(document.documentElement).getPropertyValue('--muted').trim();
    ctx.lineWidth   = 1.5;
    ctx.beginPath();
    ctx.moveTo(PAD, H - PAD);
    ctx.lineTo(PAD + barPx, H - PAD);
    ctx.stroke();
    ctx.fillStyle  = getComputedStyle(document.documentElement).getPropertyValue('--muted').trim();
    ctx.font       = '8px monospace';
    ctx.textAlign  = 'left';
    ctx.fillText(`${barMetres} m`, PAD, H - PAD - 3);
  }

  return {
    update(lat, lon, hdg) {
      if (lat === 0 && lon === 0) return;
      const last = trail[trail.length - 1];
      if (!last || last.lat !== lat || last.lon !== lon) {
        trail.push({ lat, lon });
        if (trail.length > MAX_PTS) trail.shift();
      }
      heading = hdg;
      draw();
    },
    init: draw
  };
})();
