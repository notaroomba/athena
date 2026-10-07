# athena-server

WebSocket relay for Athena telemetry, mirroring the Cyberboard server. One
"admin" client (the laptop plugged into the flight computer over USB) pushes the
raw link-frame byte stream; the server fans it out unchanged to every other
connected viewer and replays the last chunk to anyone who joins late. It never
parses the payload.

## Run

```sh
cd software/server
ADMIN_PASSWORD=secret PORT=3001 cargo run --release
curl http://127.0.0.1:3001/health   # -> ok
```

Tests: `cargo test` (spawns the built binary on a free port).

Railway: deploy the repo with root directory `/software/server`; Railpack
detects the Cargo project. Set `ADMIN_PASSWORD`, use `/health` as the health
check path.

## Env vars

| var              | default | meaning                                                        |
| ---------------- | ------- | -------------------------------------------------------------- |
| `PORT`           | `3001`  | listen port (bound on 0.0.0.0)                                 |
| `ADMIN_PASSWORD` | `admin` | password for `auth` (a warning is printed when unset)          |

A `.env` file is loaded via dotenvy if present.

## HTTP

- `GET /health` -> `200 ok`
- `GET /ws` -> WebSocket

## WebSocket protocol

Client -> server (JSON text):

```json
{"type":"auth","password":"..."}
```

A successful auth makes that socket an admin. The admin then sends **binary**
messages: chunks of the raw Athena link-frame stream
(`[0xA5][type][len][payload][crc lo][crc hi]`, any length, split however the
serial port delivered them). Binary from a non-admin is ignored.

Server -> clients (JSON text unless noted):

| message | when |
| --- | --- |
| `{"type":"status","viewers":N,"adminConnected":bool}` | to everyone on every connect, disconnect and auth attempt. `viewers` counts all sockets, admin included. |
| `{"type":"auth_result","success":bool}` | to the client that sent `auth` |
| `{"type":"admin_disconnected"}` | to everyone when an admin socket closes |
| binary | admin chunks, relayed byte-for-byte to every other client; the latest one is replayed to a newly connected client |
