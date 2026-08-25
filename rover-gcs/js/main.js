function showToast(msg = 'done', duration = 1800) {
  const t = document.getElementById('toast');
  t.textContent = msg;
  t.classList.add('show');
  setTimeout(() => t.classList.remove('show'), duration);
}

function setStatus(msg) {
  document.getElementById('statusMsg').textContent = msg;
}

// ── telemetry display ────────────────────────────────────────────────────────

function formatUptime(seconds) {
  const h = Math.floor(seconds / 3600);
  const m = Math.floor((seconds % 3600) / 60);
  const s = seconds % 60;
  if (h > 0) return `${h}h ${m}m`;
  if (m > 0) return `${m}m ${s}s`;
  return `${s}s`;
}

function onTelemetry(t) {
  // HUD items
  document.getElementById('hudAlt').innerHTML = `ALT <b>${t.alt.toFixed(1)} m</b>`;
  document.getElementById('hudSpd').innerHTML = `SPD <b>${t.gndSpeed.toFixed(1)} m/s</b>`;
  document.getElementById('hudBat').innerHTML = `BAT <b>${t.batPct.toFixed(0)}%</b>`;
  document.getElementById('hudSat').innerHTML = `SAT <b>${t.sats}</b>`;

  // instruments
  ADI.update(t.roll, t.pitch);
  Compass.update(t.yaw);
  WindRose.update(t.windDir, t.windSpeed);
  MiniMap.update(t.lat, t.lon, t.yaw);

  // nav values
  document.getElementById('valRoll').textContent    = `${t.roll.toFixed(1)}°`;
  document.getElementById('valPitch').textContent   = `${t.pitch.toFixed(1)}°`;
  document.getElementById('valHeading').textContent = `${t.yaw.toFixed(0).padStart(3, '0')}°`;
  document.getElementById('valWindDir').textContent = `${t.windDir.toFixed(0)}°`;
  document.getElementById('valWindSpd').textContent = `${t.windSpeed.toFixed(1)} m/s`;

  document.getElementById('valLat').textContent    = t.lat.toFixed(6);
  document.getElementById('valLon').textContent    = t.lon.toFixed(6);
  document.getElementById('valAlt').textContent    = `${t.alt.toFixed(1)} m`;
  document.getElementById('valGndSpd').textContent = `${t.gndSpeed.toFixed(1)} m/s`;
  document.getElementById('valVSpd').textContent   = `${t.vSpeed.toFixed(2)} m/s`;
  document.getElementById('valHdop').textContent   = t.hdop.toFixed(2);

  document.getElementById('valBatPct').textContent = `${t.batPct.toFixed(0)}%`;
  document.getElementById('valBatV').textContent   = `${t.batV.toFixed(2)} V`;
  document.getElementById('valBatA').textContent   = `${t.batA.toFixed(1)} A`;
  document.getElementById('valRssi').textContent   = `${t.rssi.toFixed(0)} dBm`;
  document.getElementById('valUptime').textContent = formatUptime(t.uptime);
  document.getElementById('valCpu').textContent    = `${t.cpu.toFixed(0)}%`;

  // header badges
  const connBadge = document.getElementById('connBadge');
  const modeBadge = document.getElementById('modeBadge');
  const armBadge  = document.getElementById('armBadge');

  if (t.connected) {
    connBadge.textContent = 'connected';
    connBadge.className   = 'badge connected';
  } else {
    connBadge.textContent = 'disconnected';
    connBadge.className   = 'badge';
  }

  modeBadge.textContent = t.mode || '—';
  modeBadge.className   = t.mode ? 'badge mode' : 'badge';

  if (t.armed) {
    armBadge.textContent = 'armed';
    armBadge.className   = 'badge armed';
  } else {
    armBadge.textContent = 'disarmed';
    armBadge.className   = 'badge';
  }

  // Hz counter
  document.getElementById('telemHz').textContent = `${Telemetry.getHz()} Hz`;
}

// ── panel resize ─────────────────────────────────────────────────────────────

function initResizer() {
  const divider  = document.getElementById('divider');
  const layout   = document.querySelector('.workspace');
  const ctrlPane = document.querySelector('.control-panel');

  divider.addEventListener('pointerdown', e => {
    e.preventDefault();
    divider.classList.add('dragging');

    const onMove = e => {
      const rect  = layout.getBoundingClientRect();
      const used  = rect.width - 6;
      const raw   = rect.right - e.clientX;
      const clamped = Math.min(Math.max(raw, 180), used * 0.6);
      layout.style.gridTemplateColumns = `1fr 6px ${clamped}px`;
    };

    const onUp = () => {
      divider.classList.remove('dragging');
      window.removeEventListener('pointermove', onMove);
      window.removeEventListener('pointerup',   onUp);
    };

    window.addEventListener('pointermove', onMove);
    window.addEventListener('pointerup',   onUp);
  });
}

// ── tabs ──────────────────────────────────────────────────────────────────────

function initTabs() {
  const tabs = document.querySelectorAll('.tab-btn');
  tabs.forEach(btn => btn.addEventListener('click', () => {
    tabs.forEach(t => t.setAttribute('aria-selected', 'false'));
    document.querySelectorAll('.tab-panel').forEach(p => p.setAttribute('hidden', ''));
    btn.setAttribute('aria-selected', 'true');
    const panel = document.getElementById(btn.dataset.tab);
    if (panel) panel.removeAttribute('hidden');
  }));
}

// ── connection controls ───────────────────────────────────────────────────────

function initConnection() {
  const btnConnect    = document.getElementById('btnConnect');
  const btnDisconnect = document.getElementById('btnDisconnect');
  const btnStream     = document.getElementById('btnStream');

  btnConnect.addEventListener('click', () => {
    const host  = document.getElementById('mavHost').value.trim();
    const port  = document.getElementById('mavPort').value;
    const wsUrl = `ws://${host}:${port}`;
    setStatus(`connecting to ${wsUrl}…`);
    Telemetry.connect(wsUrl);
    btnConnect.disabled    = true;
    btnDisconnect.disabled = false;
    showToast('connecting…');
    // re-enable connect button if the socket fails immediately
    const watchdog = setTimeout(() => {
      if (!Telemetry.state.connected) {
        btnConnect.disabled    = false;
        btnDisconnect.disabled = true;
        setStatus('connection failed');
      }
    }, 3000);
    Telemetry.subscribe(t => {
      if (t.connected) {
        clearTimeout(watchdog);
        setStatus(`ws://${host}:${port}`);
        showToast('connected');
      }
    });
  });

  btnDisconnect.addEventListener('click', () => {
    Telemetry.disconnect();
    btnConnect.disabled    = false;
    btnDisconnect.disabled = true;
    setStatus('disconnected');
    showToast('disconnected');
  });

  btnStream.addEventListener('click', () => {
    const url  = document.getElementById('streamUrl').value.trim();
    const type = document.getElementById('streamType').value;
    if (!url) { showToast('enter stream URL first'); return; }
    // stub — real implementation would open WebSocket / WebRTC / MJPEG src
    document.getElementById('feedOverlay').classList.add('hidden');
    showToast(`stream: ${type}`);
    setStatus(`stream: ${url}`);
  });
}

// ── mission controls ──────────────────────────────────────────────────────────

function initMission() {
  document.getElementById('btnSetMode').addEventListener('click', () => {
    const mode = document.getElementById('flightMode').value;
    Telemetry.setMode(mode);
    showToast(`mode → ${mode}`);
  });

  document.getElementById('btnArm').addEventListener('click', () => {
    Telemetry.setArmed(true);
    showToast('armed');
  });

  document.getElementById('btnDisarm').addEventListener('click', () => {
    Telemetry.setArmed(false);
    showToast('disarmed');
  });

  document.getElementById('btnGoTo').addEventListener('click', () => {
    const lat = document.getElementById('wpLat').value;
    const lon = document.getElementById('wpLon').value;
    const alt = document.getElementById('wpAlt').value;
    if (!lat || !lon) { showToast('enter lat/lon first'); return; }
    showToast(`goto ${parseFloat(lat).toFixed(4)}, ${parseFloat(lon).toFixed(4)}`);
    setStatus(`waypoint: ${lat}, ${lon} @ ${alt} m`);
  });
}

// ── param controls ────────────────────────────────────────────────────────────

function initParams() {
  document.getElementById('btnGetParam').addEventListener('click', () => {
    const name = document.getElementById('paramName').value.trim().toUpperCase();
    if (!name) { showToast('enter param name'); return; }
    // stub — would send MAV_CMD_REQUEST_MESSAGE / PARAM_REQUEST_READ
    document.getElementById('paramName').value = name;
    showToast(`get ${name}`);
  });

  document.getElementById('btnSetParam').addEventListener('click', () => {
    const name = document.getElementById('paramName').value.trim().toUpperCase();
    const val  = document.getElementById('paramVal').value;
    if (!name) { showToast('enter param name'); return; }
    showToast(`${name} = ${val}`);
  });

  document.getElementById('btnSendQuick').addEventListener('click', () => {
    const items = document.querySelectorAll('.param-item');
    let count = 0;
    items.forEach(item => {
      const input = item.querySelector('.param-quick');
      if (input.value !== '') {
        count++;
        // stub — would send PARAM_SET for each
      }
    });
    if (count === 0) { showToast('no values entered'); return; }
    showToast(`sent ${count} params`);
  });

  // allow Enter to submit quick params
  document.querySelectorAll('.param-quick').forEach(input => {
    input.addEventListener('keydown', e => {
      if (e.key === 'Enter') {
        const name = input.closest('.param-item').dataset.param;
        const val  = input.value;
        if (val !== '') showToast(`${name} = ${val}`);
      }
    });
  });
}

// ── init ──────────────────────────────────────────────────────────────────────

document.addEventListener('DOMContentLoaded', () => {
  ADI.init();
  Compass.init();
  WindRose.init();
  MiniMap.init();

  initTabs();
  initResizer();
  initConnection();
  initMission();
  initParams();

  Telemetry.subscribe(onTelemetry);

  setStatus('ready — start sim server then click connect');
});
