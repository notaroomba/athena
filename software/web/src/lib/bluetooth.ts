// Web Bluetooth source: the DA14531MOD on the TPU (UART7, 57600 8N1).
//
// Two firmwares are handled. DSPS (Renesas "serial port service") is a transparent UART bridge: every
// byte the TPU writes to the module arrives as a notification and goes straight into the frame decoder.
// CodeLess (the factory firmware on the module) is an AT-command interpreter: it can be connected to
// and talked to, but it does not forward UART data, so with it the dashboard only shows its replies.
// The DSPS image is installed over the air with Renesas' SUOTA app (CodeLess advertises SUOTA).
// Characteristics are picked by their properties (notify-only = device->host, write-only = host->device),
// so a different UUID revision of the service still works.

export interface BluetoothCallbacks {
  onChunk: (bytes: Uint8Array) => void;
  onDisconnect: () => void;
  onInfo?: (line: string) => void;
}

const DSPS_SERVICE = "0783b03e-8535-b5a0-7140-a304d2495cb7";
const CODELESS_SERVICE = "866d3b04-e674-40dc-9c05-b7f91bec6e83";
const CODELESS_INBOUND = "914f8fb9-e8cd-411d-b7d1-14594de45425"; // host -> module (AT commands)
const CODELESS_OUTBOUND = "3bb535aa-50b2-4fbe-aa09-6b06dc59a404"; // module -> host (replies)
const CODELESS_FLOW = "e2048b39-d4f9-4a45-9f25-1856c10d5639"; // 1 byte: 0x01 = outbound has data

let device: BluetoothDevice | null = null;
let rxChar: BluetoothRemoteGATTCharacteristic | null = null; // host -> module
let mode: "dsps" | "codeless" | null = null;

export function isBluetoothSupported(): boolean {
  return typeof navigator !== "undefined" && "bluetooth" in navigator;
}

export async function connectBluetooth(cb: BluetoothCallbacks): Promise<string> {
  if (!isBluetoothSupported()) throw new Error("Web Bluetooth is not supported here. Use Chrome/Edge over https.");
  device = await navigator.bluetooth.requestDevice({
    filters: [{ services: [DSPS_SERVICE] }, { services: [CODELESS_SERVICE] }, { namePrefix: "DSPS" }, { namePrefix: "CodeLess" }, { namePrefix: "Athena" }],
    optionalServices: [DSPS_SERVICE, CODELESS_SERVICE],
  });
  const dev = device;
  dev.addEventListener("gattserverdisconnected", () => {
    if (device === dev) {
      device = null;
      rxChar = null;
      mode = null;
      cb.onDisconnect();
    }
  });
  const server = await dev.gatt!.connect();
  const name = dev.name || "DA14531";

  let svc: BluetoothRemoteGATTService;
  try {
    svc = await server.getPrimaryService(DSPS_SERVICE);
    mode = "dsps";
  } catch {
    svc = await server.getPrimaryService(CODELESS_SERVICE);
    mode = "codeless";
  }
  const chars = await svc.getCharacteristics();

  if (mode === "dsps") {
    let tx: BluetoothRemoteGATTCharacteristic | null = null,
      flow: BluetoothRemoteGATTCharacteristic | null = null;
    for (const c of chars) {
      const w = c.properties.write || c.properties.writeWithoutResponse;
      if (c.properties.notify && !w) tx = c;
      else if (w && !c.properties.notify) rxChar = c;
      else if (w && c.properties.notify) flow = c;
    }
    if (!tx) throw new Error("DSPS service without a notify characteristic");
    tx.addEventListener("characteristicvaluechanged", (e) => {
      const v = (e.target as BluetoothRemoteGATTCharacteristic).value;
      if (v) cb.onChunk(new Uint8Array(v.buffer, v.byteOffset, v.byteLength));
    });
    await tx.startNotifications();
    if (flow) {
      try {
        await flow.writeValue(new Uint8Array([0x01])); // XON: ready to receive
      } catch {
        /* optional */
      }
    }
    cb.onInfo?.(`[bluetooth] ${name}: DSPS serial bridge, streaming`);
    return `${name} (DSPS)`;
  }

  // CodeLess: subscribe to the flow-control characteristic and read the outbound buffer when it signals data
  const by = (uuid: string) => chars.find((c) => c.uuid === uuid) ?? null;
  const flow = by(CODELESS_FLOW);
  const outbound = by(CODELESS_OUTBOUND);
  rxChar = by(CODELESS_INBOUND);
  if (flow && outbound) {
    flow.addEventListener("characteristicvaluechanged", async () => {
      try {
        const v = await outbound.readValue();
        cb.onChunk(new Uint8Array(v.buffer, v.byteOffset, v.byteLength));
      } catch {
        /* ignore */
      }
    });
    await flow.startNotifications();
  }
  cb.onInfo?.(`[bluetooth] ${name}: CodeLess AT firmware - replies only; flash the DSPS image (SUOTA) for telemetry`);
  return `${name} (CodeLess)`;
}

/** Sends bytes to the module: DSPS forwards them to the TPU's UART7 (not wired to the TPU RX yet), CodeLess treats them as AT text. */
export async function writeBluetooth(bytes: Uint8Array): Promise<boolean> {
  if (!rxChar) return false;
  try {
    for (let i = 0; i < bytes.length; i += 20) {
      const chunk = new Uint8Array(bytes.subarray(i, i + 20)); // fresh ArrayBuffer-backed copy for the GATT API
      if (rxChar.properties.writeWithoutResponse) await rxChar.writeValueWithoutResponse(chunk);
      else await rxChar.writeValue(chunk);
    }
    return true;
  } catch (e) {
    console.warn("bluetooth write error:", e);
    return false;
  }
}

export function disconnectBluetooth(): void {
  const d = device;
  device = null;
  rxChar = null;
  mode = null;
  try {
    d?.gatt?.disconnect();
  } catch {
    /* ignore */
  }
}

export function bluetoothMode(): "dsps" | "codeless" | null {
  return mode;
}
