use futures_util::{SinkExt, StreamExt};
use serde_json::{json, Value};
use std::{net::TcpListener, process::{Child, Command, Stdio}, time::Duration};
use tokio_tungstenite::{connect_async, tungstenite::Message, MaybeTlsStream, WebSocketStream};

type Ws = WebSocketStream<MaybeTlsStream<tokio::net::TcpStream>>;

struct Server(Child);
impl Drop for Server {
    fn drop(&mut self) {
        let _ = self.0.kill();
    }
}

async fn spawn_server(password: &str) -> (Server, String) {
    let port = TcpListener::bind("127.0.0.1:0").unwrap().local_addr().unwrap().port();
    let child = Command::new(env!("CARGO_BIN_EXE_athena-server"))
        .env("PORT", port.to_string())
        .env("ADMIN_PASSWORD", password)
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .spawn()
        .unwrap();
    for _ in 0..100 {
        if tokio::net::TcpStream::connect(("127.0.0.1", port)).await.is_ok() {
            return (Server(child), format!("ws://127.0.0.1:{port}/ws"));
        }
        tokio::time::sleep(Duration::from_millis(50)).await;
    }
    panic!("server did not start");
}

async fn next(ws: &mut Ws) -> Message {
    tokio::time::timeout(Duration::from_secs(5), async {
        loop {
            match ws.next().await.expect("socket closed").expect("ws error") {
                Message::Ping(_) | Message::Pong(_) => continue,
                m => return m,
            }
        }
    })
    .await
    .expect("timed out waiting for message")
}

async fn next_json(ws: &mut Ws) -> Value {
    match next(ws).await {
        Message::Text(t) => serde_json::from_str(&t).unwrap(),
        other => panic!("expected text, got {other:?}"),
    }
}

async fn next_binary(ws: &mut Ws) -> Vec<u8> {
    match next(ws).await {
        Message::Binary(b) => b.to_vec(),
        other => panic!("expected binary, got {other:?}"),
    }
}

fn auth(password: &str) -> Message {
    Message::Text(json!({"type":"auth","password":password}).to_string().into())
}

#[tokio::test]
async fn relay_protocol() {
    let (_server, url) = spawn_server("hunter2").await;

    let (mut viewer, _) = connect_async(&url).await.unwrap();
    assert_eq!(next_json(&mut viewer).await, json!({"type":"status","viewers":1,"adminConnected":false}));

    let (mut admin, _) = connect_async(&url).await.unwrap();
    assert_eq!(next_json(&mut viewer).await, json!({"type":"status","viewers":2,"adminConnected":false}));
    assert_eq!(next_json(&mut admin).await["viewers"], 2);

    // wrong password
    admin.send(auth("nope")).await.unwrap();
    assert_eq!(next_json(&mut admin).await, json!({"type":"auth_result","success":false}));
    assert_eq!(next_json(&mut admin).await["adminConnected"], false);
    assert_eq!(next_json(&mut viewer).await["adminConnected"], false);

    // binary from a non-admin is dropped (verified below: viewer's next binary is `small`)
    admin.send(Message::Binary(vec![9, 9, 9].into())).await.unwrap();

    // right password
    admin.send(auth("hunter2")).await.unwrap();
    assert_eq!(next_json(&mut admin).await, json!({"type":"auth_result","success":true}));
    assert_eq!(next_json(&mut admin).await["adminConnected"], true);
    assert_eq!(next_json(&mut viewer).await, json!({"type":"status","viewers":2,"adminConnected":true}));

    // 7-byte and 300-byte binaries relayed unchanged
    let small = vec![0xA5, 0x7F, 0x01, b'x', 0x12, 0x34, 0x00];
    let big: Vec<u8> = (0..300u32).map(|i| (i * 7 % 256) as u8).collect();
    admin.send(Message::Binary(small.clone().into())).await.unwrap();
    assert_eq!(next_binary(&mut viewer).await, small);
    admin.send(Message::Binary(big.clone().into())).await.unwrap();
    assert_eq!(next_binary(&mut viewer).await, big);

    // late-joining viewer gets the latest frame on connect, then status
    let (mut late, _) = connect_async(&url).await.unwrap();
    assert_eq!(next_binary(&mut late).await, big);
    assert_eq!(next_json(&mut late).await, json!({"type":"status","viewers":3,"adminConnected":true}));
    assert_eq!(next_json(&mut viewer).await["viewers"], 3);
    assert_eq!(next_json(&mut admin).await["viewers"], 3);

    // admin closes -> admin_disconnected, then status
    admin.close(None).await.unwrap();
    assert_eq!(next_json(&mut viewer).await, json!({"type":"admin_disconnected"}));
    assert_eq!(next_json(&mut viewer).await, json!({"type":"status","viewers":2,"adminConnected":false}));
    assert_eq!(next_json(&mut late).await, json!({"type":"admin_disconnected"}));
}
