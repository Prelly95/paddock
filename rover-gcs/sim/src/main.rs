/// Rover GCS demo telemetry server.
///
/// Broadcasts simulated vehicle state as JSON over WebSocket on
/// ws://127.0.0.1:8765.  Every connected client receives the same 20 Hz
/// stream.  The simulation mirrors the original JS demo feed so the UI
/// instruments animate the same way.
///
/// Usage: cargo run --bin sim
///        cargo run --bin sim -- --port 9000

use std::{
    net::SocketAddr,
    sync::{Arc, Mutex},
    time::{Duration, Instant},
};

use futures_util::{SinkExt, StreamExt};
use serde::Serialize;
use tokio::{
    net::{TcpListener, TcpStream},
    time,
};
use tokio_tungstenite::{accept_async, tungstenite::Message};

// ── telemetry frame ──────────────────────────────────────────────────────────

#[derive(Serialize, Clone)]
#[serde(rename_all = "camelCase")]
struct Frame {
    connected:  bool,
    armed:      bool,
    mode:       String,
    roll:       f64,
    pitch:      f64,
    yaw:        f64,
    lat:        f64,
    lon:        f64,
    alt:        f64,
    gnd_speed:  f64,
    v_speed:    f64,
    hdop:       f64,
    sats:       u32,
    bat_pct:    f64,
    bat_v:      f64,
    bat_a:      f64,
    rssi:       f64,
    uptime:     u64,
    cpu:        f64,
    wind_dir:   f64,
    wind_speed: f64,
}

// ── simulation state ─────────────────────────────────────────────────────────

struct SimState {
    t:        f64,
    yaw:      f64,
    wind_dir: f64,
    start:    Instant,
}

impl SimState {
    fn new() -> Self {
        Self {
            t:        0.0,
            yaw:      0.0,
            wind_dir: 0.0,
            start:    Instant::now(),
        }
    }

    fn tick(&mut self) -> Frame {
        self.t        += 0.04;
        self.yaw       = (self.yaw + 0.3) % 360.0;
        self.wind_dir  = (self.wind_dir + 0.05) % 360.0;

        let t = self.t;
        Frame {
            connected:  true,
            armed:      false,
            mode:       "POSCTL".into(),
            roll:       (t * 0.7).sin() * 25.0,
            pitch:      (t * 0.4).sin() * 10.0,
            yaw:        self.yaw,
            lat:        47.641_468 + (t * 0.05).sin() * 0.001,
            lon:        -122.140_165 + (t * 0.04).cos() * 0.001,
            alt:        50.0 + (t * 0.3).sin() * 8.0,
            gnd_speed:  3.2 + (t * 0.6).sin() * 1.5,
            v_speed:    (t * 0.5).sin() * 0.8,
            hdop:       1.2 + (t * 0.1).sin().abs() * 0.5,
            sats:       12,
            bat_pct:    (87.0 - t * 0.02).max(0.0),
            bat_v:      14.8 - t * 0.001,
            bat_a:      4.2 + (t * 0.4).sin().abs(),
            rssi:       -65.0 + (t * 0.2).sin() * 10.0,
            uptime:     self.start.elapsed().as_secs(),
            cpu:        18.0 + (t * 0.8).sin() * 8.0,
            wind_dir:   self.wind_dir,
            wind_speed: 3.5 + (t * 0.3).sin() * 1.2,
        }
    }
}

// ── shared broadcast channel ─────────────────────────────────────────────────

type Clients = Arc<Mutex<Vec<tokio::sync::mpsc::UnboundedSender<String>>>>;

// ── main ─────────────────────────────────────────────────────────────────────

#[tokio::main]
async fn main() {
    let port = std::env::args()
        .skip_while(|a| a != "--port")
        .nth(1)
        .and_then(|p| p.parse::<u16>().ok())
        .unwrap_or(8765);

    let addr: SocketAddr = ([127, 0, 0, 1], port).into();
    let listener = TcpListener::bind(addr).await.expect("bind failed");
    println!("rover-gcs-sim  ws://{addr}  (20 Hz)");

    let clients: Clients = Arc::new(Mutex::new(Vec::new()));

    // broadcast task: tick sim at 20 Hz and push JSON to all clients
    let clients_tx = clients.clone();
    tokio::spawn(async move {
        let mut sim      = SimState::new();
        let mut interval = time::interval(Duration::from_millis(50));
        loop {
            interval.tick().await;
            let frame = sim.tick();
            let json  = serde_json::to_string(&frame).unwrap();
            let mut guard = clients_tx.lock().unwrap();
            guard.retain(|tx| tx.send(json.clone()).is_ok());
        }
    });

    // accept loop
    loop {
        let Ok((stream, peer)) = listener.accept().await else { continue };
        let clients = clients.clone();
        tokio::spawn(handle_client(stream, peer, clients));
    }
}

async fn handle_client(stream: TcpStream, peer: SocketAddr, clients: Clients) {
    let Ok(ws) = accept_async(stream).await else {
        eprintln!("handshake failed for {peer}");
        return;
    };
    println!("client connected: {peer}");

    let (mut ws_tx, mut ws_rx) = ws.split();
    let (tx, mut rx) = tokio::sync::mpsc::unbounded_channel::<String>();

    clients.lock().unwrap().push(tx);

    // forward broadcast messages to the WebSocket
    let send_loop = async {
        while let Some(msg) = rx.recv().await {
            if ws_tx.send(Message::Text(msg.into())).await.is_err() {
                break;
            }
        }
    };

    // drain incoming messages (ping/pong handled by tungstenite automatically)
    let recv_loop = async {
        while let Some(Ok(_)) = ws_rx.next().await {}
    };

    tokio::select! {
        _ = send_loop => {},
        _ = recv_loop => {},
    }

    println!("client disconnected: {peer}");
}
