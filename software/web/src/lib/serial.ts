// WebSerial source: the MPU/TPU enumerate as an STMicroelectronics USB CDC port.

const BAUD = 115200;

export interface SerialCallbacks {
  /** Raw chunk as read from the port; feed it to the Decoder (and relay it when admin). */
  onChunk: (bytes: Uint8Array) => void;
  onDisconnect: () => void;
}

let port: SerialPort | null = null;
let reader: ReadableStreamDefaultReader<Uint8Array> | null = null;
let writer: WritableStreamDefaultWriter<Uint8Array> | null = null;

export function isSerialSupported(): boolean {
  return typeof navigator !== "undefined" && "serial" in navigator;
}

export async function connectSerial(callbacks: SerialCallbacks): Promise<string> {
  if (!isSerialSupported()) {
    throw new Error("WebSerial is not supported. Use Chrome/Edge on HTTPS or localhost.");
  }

  port = await navigator.serial.requestPort();
  await port.open({ baudRate: BAUD });

  const info = port.getInfo();
  const label = `usb ${info.usbVendorId?.toString(16)}:${info.usbProductId?.toString(16)}`;

  try {
    writer = port.writable?.getWriter() ?? null;
  } catch {
    writer = null;
  }

  // Read loop: runs until the reader is cancelled or the device goes away.
  const p = port;
  (async () => {
    try {
      reader = p.readable!.getReader();
      for (;;) {
        const { value, done } = await reader.read();
        if (done) break;
        if (value) callbacks.onChunk(value);
      }
    } catch (e) {
      console.warn("serial read error:", e);
    } finally {
      try {
        reader?.releaseLock();
      } catch {
        /* ignore */
      }
      reader = null;
      try {
        writer?.releaseLock();
      } catch {
        /* ignore */
      }
      writer = null;
      try {
        await p.close();
      } catch {
        /* ignore */
      }
      if (port === p) port = null;
      callbacks.onDisconnect();
    }
  })();

  return label;
}

export async function disconnectSerial(): Promise<void> {
  try {
    await reader?.cancel();
  } catch {
    /* ignore */
  }
  // the read loop's finally block closes the port and fires onDisconnect
}

/** Sends raw bytes (a CMD frame or console characters) to the open port. */
export async function writeSerial(bytes: Uint8Array): Promise<boolean> {
  if (!writer) return false;
  try {
    await writer.write(bytes);
    return true;
  } catch (e) {
    console.warn("serial write error:", e);
    return false;
  }
}
