// Telemetry state fed from the Rust sim server (or a real MAVLink bridge)
// over a WebSocket that streams 20 Hz JSON frames.
//
// Frame fields (camelCase, matches sim/src/main.rs):
//   connected, armed, mode, roll, pitch, yaw,
//   lat, lon, alt, gndSpeed, vSpeed, hdop, sats,
//   batPct, batV, batA, rssi, uptime, cpu,
//   windDir, windSpeed

const Telemetry = (() => {
  const state = {
    connected: false,
    armed:     false,
    mode:      '',
    roll:      0,
    pitch:     0,
    yaw:       0,
    lat:       0,
    lon:       0,
    alt:       0,
    gndSpeed:  0,
    vSpeed:    0,
    hdop:      0,
    sats:      0,
    batPct:    0,
    batV:      0,
    batA:      0,
    rssi:      0,
    uptime:    0,
    cpu:       0,
    windDir:   0,
    windSpeed: 0,
  };

  let ws          = null;
  let telemCount  = 0;
  let telemHz     = 0;
  let lastHzTime  = Date.now();
  let reconnTimer = null;

  const listeners = [];

  function subscribe(fn) { listeners.push(fn); }

  function emit() {
    listeners.forEach(fn => fn({ ...state }));
    telemCount++;
    const now = Date.now();
    if (now - lastHzTime >= 1000) {
      telemHz    = telemCount;
      telemCount = 0;
      lastHzTime = now;
    }
  }

  function getHz() { return telemHz; }

  function applyFrame(frame) {
    // Map camelCase JSON fields onto local state.
    // A real MAVLink bridge may use different key names — adapt here.
    Object.assign(state, {
      connected: frame.connected  ?? true,
      armed:     frame.armed      ?? state.armed,
      mode:      frame.mode       ?? state.mode,
      roll:      frame.roll       ?? state.roll,
      pitch:     frame.pitch      ?? state.pitch,
      yaw:       frame.yaw        ?? state.yaw,
      lat:       frame.lat        ?? state.lat,
      lon:       frame.lon        ?? state.lon,
      alt:       frame.alt        ?? state.alt,
      gndSpeed:  frame.gndSpeed   ?? state.gndSpeed,
      vSpeed:    frame.vSpeed     ?? state.vSpeed,
      hdop:      frame.hdop       ?? state.hdop,
      sats:      frame.sats       ?? state.sats,
      batPct:    frame.batPct     ?? state.batPct,
      batV:      frame.batV       ?? state.batV,
      batA:      frame.batA       ?? state.batA,
      rssi:      frame.rssi       ?? state.rssi,
      uptime:    frame.uptime     ?? state.uptime,
      cpu:       frame.cpu        ?? state.cpu,
      windDir:   frame.windDir    ?? state.windDir,
      windSpeed: frame.windSpeed  ?? state.windSpeed,
    });
    emit();
  }

  function connect(url) {
    if (ws) disconnect();

    ws = new WebSocket(url);

    ws.addEventListener('open', () => {
      state.connected = true;
      emit();
    });

    ws.addEventListener('message', e => {
      let frame;
      try { frame = JSON.parse(e.data); } catch { return; }
      applyFrame(frame);
    });

    ws.addEventListener('close', () => {
      state.connected = false;
      emit();
      ws = null;
    });

    ws.addEventListener('error', () => {
      // close event follows; let that handler reset state
    });
  }

  function disconnect() {
    clearTimeout(reconnTimer);
    if (ws) {
      ws.onclose = null;
      ws.close();
      ws = null;
    }
    state.connected = false;
    emit();
  }

  function setArmed(v) { state.armed = v; emit(); }
  function setMode(m)  { state.mode  = m; emit(); }

  return { subscribe, connect, disconnect, setArmed, setMode, getHz, state };
})();
