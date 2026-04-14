/* Robot R2 TurboWarp/Scratch Extension — v15 (stable sensor monitors + line detect + disconnect + pin labels)

   Problem solved:
   - Scratch monitors evaluate reporter/boolean blocks many times per second.
   - If blocks send BLE commands on each evaluation, BLE UART gets congested and
     ultrasonic updates appear "rare".

   This version:
   - Polls Distance/Touch/Lux/Sound in the background at controlled rates.
   - All sensor blocks are synchronous: they return cached values.

   Firmware (Arduino) expectations:
   - BLE UART: service 0xFFE0 / characteristic 0xFFE1 (HM-10/BT24 style)
   - Commands:
     Movement (continuous heartbeat): U D L R Q E
     Stop: S
     Horn: B
     Rear LED: t (ON), u (OFF)
     Front RGB power: f (ON), g (OFF)
     Parking color: Cx where x in {W,R,G,B,A,P,C}
     Line follow: Z (ON), z (OFF)
     Sensors:
       Ultrasonic: ?  -> "D:<cm>\n"
       Touch:      Y  -> "T:<0/1>\n"
       LDR:        H  -> "L:<lux>\n"
       Sound:      J  -> "S:<level>\n"
       Line:       M  -> "N:<0/1>\n" (any line sensor detects line)
*/

(function (Scratch) {
  "use strict";

  const SERVICE_UUID = "0000ffe0-0000-1000-8000-00805f9b34fb";
  const CHAR_UUID    = "0000ffe1-0000-1000-8000-00805f9b34fb";

  const encoder = new TextEncoder();
  const decoder = new TextDecoder();

  class RobotR2 {
    constructor() {
      this.device = null;
      this.chr = null;
      this.connected = false;

      // RX line buffer
      this._rxBuf = "";

      // Cached sensor values
      this.lastDistanceCm = -1;
      this.lastTouch = 0;   // 0/1
      this.lastLux = -1;
      this.lastSound = -1;
      this.lastLineDetected = 0; // 0/1

      // BLE write queue (prevents GATT overlap)
      this._writeChain = Promise.resolve();

      // Drive heartbeat
      this._driveCmd = "S";
      this._driveTimer = null;

      // Line follow state
      this._lineFollow = false;

      // Background polling
      this._pollTimer = null;
      this._pollBusy = false;
      this._lastPoll = { dist: 0, touch: 0, line: 0, lux: 0, sound: 0 };

      // Poll intervals (ms)
      this._distGapStop = 220;   // ~4.5 Hz while stopped
      this._distGapMove = 650;   // lower priority while moving
      this._touchGap    = 500;   // 2 Hz
      this._luxGap      = 900;   // ~1.1 Hz
      this._soundGap    = 900;   // ~1.1 Hz
      this._lineGap     = 250;   // 4 Hz (line detected?)

      // Bindings
      this._onNotify = this._onNotify.bind(this);
      this._onDisconnected = this._onDisconnected.bind(this);
    }

    getInfo() {
      return {
        id: "robotR2",
        name: "Robot R2",

        color1: "#00979D",
        color2: "#007A7F",
        color3: "#005B5F",

        blocks: [
         // { blockType: Scratch.BlockType.LABEL, text: "Robot R2 (BLE)" },
          { blockType: Scratch.BlockType.BUTTON, text: "Σύνδεση", func: "connect" },
          { blockType: Scratch.BlockType.BUTTON, text: "Αποσύνδεση", func: "disconnect" },
          { opcode: "isConnected", blockType: Scratch.BlockType.BOOLEAN, text: "συνδεδεμένο;" },

          "---",

          { blockType: Scratch.BlockType.LABEL, text: "Κίνηση" },
          {
            opcode: "move",
            blockType: Scratch.BlockType.COMMAND,
            text: "κινήσου [DIR]",
            arguments: {
              DIR: { type: Scratch.ArgumentType.STRING, menu: "dirMenu", defaultValue: "U" }
            }
          },
          {
            opcode: "moveFor",
            blockType: Scratch.BlockType.COMMAND,
            text: "κινήσου [DIR] για [SECS] δευτ.",
            arguments: {
              DIR:  { type: Scratch.ArgumentType.STRING, menu: "dirMenu", defaultValue: "U" },
              SECS: { type: Scratch.ArgumentType.NUMBER, defaultValue: 1 }
            }
          },
          { opcode: "stop", blockType: Scratch.BlockType.COMMAND, text: "STOP" },

          "---",

          { blockType: Scratch.BlockType.LABEL, text: "Φώτα & Ήχος" },
          { opcode: "beep", blockType: Scratch.BlockType.COMMAND, text: "κόρνα" },
          {
            opcode: "rearLed",
            blockType: Scratch.BlockType.COMMAND,
            text: "πίσω LED (D9) [STATE]",
            arguments: { STATE: { type: Scratch.ArgumentType.STRING, menu: "onOffMenu", defaultValue: "ON" } }
          },
          {
            opcode: "frontPower",
            blockType: Scratch.BlockType.COMMAND,
            text: "μπροστινό RGB (D12) [STATE]",
            arguments: { STATE: { type: Scratch.ArgumentType.STRING, menu: "onOffMenu", defaultValue: "ON" } }
          },
          {
            opcode: "parkingColor",
            blockType: Scratch.BlockType.COMMAND,
            text: "χρώμα RGB [COL]",
            arguments: { COL: { type: Scratch.ArgumentType.STRING, menu: "colorMenu", defaultValue: "W" } }
          },

          "---",

          { blockType: Scratch.BlockType.LABEL, text: "Αισθητήρες" },
          { opcode: "distanceCm", blockType: Scratch.BlockType.REPORTER, text: "απόσταση (cm) (TRIG A0, ECHO D2)" },
          { opcode: "touchPressed", blockType: Scratch.BlockType.BOOLEAN, text: "αισθητήρας αφής (D11) πατημένος;" },
          { opcode: "lux", blockType: Scratch.BlockType.REPORTER, text: "φως (lux) (A4)" },
          { opcode: "soundLevel", blockType: Scratch.BlockType.REPORTER, text: "επίπεδο ήχου (A6)" },
          { opcode: "lineDetected", blockType: Scratch.BlockType.BOOLEAN, text: "εντόπισες γραμμή; (A1/A2/A3)" },

          "---",


          { blockType: Scratch.BlockType.LABEL, text: "Αυτονομία" },
          {
            opcode: "lineFollow",
            blockType: Scratch.BlockType.COMMAND,
            text: "παρακολούθηση γραμμής [STATE]",
            arguments: { STATE: { type: Scratch.ArgumentType.STRING, menu: "onOffMenu", defaultValue: "ON" } }
          }
        ],

        menus: {
          dirMenu: {
            acceptReporters: true,
            items: [
              { text: "μπροστά", value: "U" },
              { text: "πίσω", value: "D" },
              { text: "αριστερά", value: "L" },
              { text: "δεξιά", value: "R" },
              { text: "μπροστά-αριστερά", value: "Q" },
              { text: "μπροστά-δεξιά", value: "E" }
            ]
          },
          onOffMenu: {
            acceptReporters: true,
            items: [
              { text: "ON",  value: "ON" },
              { text: "OFF", value: "OFF" }
            ]
          },
          colorMenu: {
            acceptReporters: true,
            items: [
              { text: "⚪", value: "W" },
              { text: "🔴", value: "R" },
              { text: "🟢", value: "G" },
              { text: "🔵", value: "B" },
              { text: "🟠", value: "A" },
              { text: "🟣", value: "P" },
              { text: "🟦", value: "C" }
            ]
          }
        }
      };
    }

    _requireConnected() {
      return !!(this.connected && this.device?.gatt?.connected && this.chr);
    }

    _write(text) {
      const t = String(text);
      this._writeChain = this._writeChain.then(async () => {
        if (!this.chr) return;
        const data = encoder.encode(t);
        if (this.chr.properties && this.chr.properties.writeWithoutResponse) {
          await this.chr.writeValueWithoutResponse(data);
        } else {
          await this.chr.writeValue(data);
        }
      }).catch(() => {});
      return this._writeChain;
    }

    _onNotify(event) {
      try {
        const chunk = decoder.decode(event.target.value);
        this._rxBuf += chunk;

        let idx;
        while ((idx = this._rxBuf.indexOf("\n")) >= 0) {
          const line = this._rxBuf.slice(0, idx).trim();
          this._rxBuf = this._rxBuf.slice(idx + 1);

          if (line.startsWith("D:")) {
            const v = parseFloat(line.slice(2));
            // Keep last valid distance; ignore -1 or NaN
            if (isFinite(v) && v >= 0) this.lastDistanceCm = v;
          } else if (line.startsWith("T:")) {
            const v = parseInt(line.slice(2), 10);
            this.lastTouch = (v === 1) ? 1 : 0;
          } else if (line.startsWith("L:")) {
            const v = parseFloat(line.slice(2));
            if (isFinite(v)) this.lastLux = v;
          } else if (line.startsWith("S:")) {
            const v = parseFloat(line.slice(2));
            if (isFinite(v)) this.lastSound = v;
          } else if (line.startsWith("N:")) {
            const v = parseInt(line.slice(2), 10);
            this.lastLineDetected = (v === 1) ? 1 : 0;
          }
        }
      } catch (_) {}
    }

    _onDisconnected() {
      this.connected = false;
      try {
        if (this.chr) this.chr.removeEventListener("characteristicvaluechanged", this._onNotify);
      } catch (_) {}

      this.device = null;
      this.chr = null;
      this._rxBuf = "";

      this.lastDistanceCm = -1;
      this.lastTouch = 0;
      this.lastLux = -1;
      this.lastSound = -1;

      this._stopHeartbeat();
      this._stopPoll();
    }

    _startHeartbeat() {
      if (this._driveTimer) return;
      this._driveTimer = setInterval(() => {
        if (!this._requireConnected()) return;
        if (!this._driveCmd) return;
        this._write(this._driveCmd);
      }, 150);
    }

    _stopHeartbeat() {
      if (this._driveTimer) {
        clearInterval(this._driveTimer);
        this._driveTimer = null;
      }
      this._driveCmd = "S";
    }

    _startPoll() {
      if (this._pollTimer) return;
      this._pollBusy = false;
      this._lastPoll = { dist: 0, touch: 0, line: 0, lux: 0, sound: 0 };

      this._pollTimer = setInterval(() => {
        this._pollTick();
      }, 90);
    }

    _stopPoll() {
      if (this._pollTimer) {
        clearInterval(this._pollTimer);
        this._pollTimer = null;
      }
      this._pollBusy = false;
    }

    _pollTick() {
      if (!this._requireConnected()) return;
      if (this._pollBusy) return;

      const now = Date.now();
      const moving = (this._driveTimer && this._driveCmd && this._driveCmd !== "S");
      const distGap = moving ? this._distGapMove : this._distGapStop;

      let cmd = null;
      if (now - this._lastPoll.dist >= distGap) {
        cmd = "?";
        this._lastPoll.dist = now;
      } else if (now - this._lastPoll.touch >= this._touchGap) {
        cmd = "Y";
        this._lastPoll.touch = now;
      } else if (now - this._lastPoll.line >= this._lineGap) {
        cmd = "M";
        this._lastPoll.line = now;
      } else if (now - this._lastPoll.lux >= this._luxGap) {
        cmd = "H";
        this._lastPoll.lux = now;
      } else if (now - this._lastPoll.sound >= this._soundGap) {
        cmd = "J";
        this._lastPoll.sound = now;
      }

      if (!cmd) return;

      this._pollBusy = true;
      this._write(cmd).finally(() => {
        this._pollBusy = false;
      });
    }

    // ----- UI BUTTON -----
    async connect() {
      if (!navigator.bluetooth) throw new Error("Web Bluetooth δεν υποστηρίζεται εδώ.");
      if (this._requireConnected()) return;

      const device = await navigator.bluetooth.requestDevice({
        filters: [{ services: [SERVICE_UUID] }],
        optionalServices: [SERVICE_UUID]
      });

      device.addEventListener("gattserverdisconnected", this._onDisconnected);

      const server = await device.gatt.connect();
      const service = await server.getPrimaryService(SERVICE_UUID);
      const chr = await service.getCharacteristic(CHAR_UUID);

      try {
        await chr.startNotifications();
        chr.addEventListener("characteristicvaluechanged", this._onNotify);
      } catch (_) {
        // Some stacks still work without explicit notifications; we'll keep going.
      }

      this.device = device;
      this.chr = chr;
      this.connected = true;

      // safe state
      this._lineFollow = false;
      await this._write("S");

      this._startPoll();
    }

    async disconnect() {
      // User-initiated disconnect
      try {
        this._stopHeartbeat();
        this._stopPoll();
        this._lineFollow = false;
      } catch (_) {}

      try {
        if (this.device?.gatt?.connected) {
          this.device.gatt.disconnect();
          return;
        }
      } catch (_) {}

      // Fallback cleanup if already disconnected
      this._onDisconnected();
    }

    isConnected() {
      return this._requireConnected();
    }

    // ----- MOVEMENT -----
    async move(args) {
      if (!this._requireConnected()) return;
      if (this._lineFollow) {
        this._lineFollow = false;
        await this._write("z");
        await this._write("S");
      }
      const cmd = String(args.DIR || "U").trim().toUpperCase();
      this._driveCmd = cmd;
      this._startHeartbeat();
      await this._write(cmd);
    }

    async moveFor(args) {
      if (!this._requireConnected()) return;
      if (this._lineFollow) {
        this._lineFollow = false;
        await this._write("z");
        await this._write("S");
      }

      const cmd = String(args.DIR || "U").trim().toUpperCase();
      let secs = Number(args.SECS);
      if (!isFinite(secs) || secs <= 0) secs = 0.1;

      const wasMoving = (this._driveCmd && this._driveCmd !== "S" && !!this._driveTimer);
      if (wasMoving) this._stopHeartbeat();

      this._driveCmd = cmd;
      await this._write(cmd);
      await new Promise(r => setTimeout(r, secs * 1000));

      await this._write("S");
      this._driveCmd = "S";
      if (wasMoving) this._startHeartbeat();
    }

    async stop() {
      if (!this._requireConnected()) return;
      if (this._lineFollow) {
        this._lineFollow = false;
        await this._write("z");
      }
      this._stopHeartbeat();
      await this._write("S");
    }

    // ----- LIGHTS & SOUND -----
    async beep() {
      if (!this._requireConnected()) return;
      await this._write("B");
    }

    async rearLed(args) {
      if (!this._requireConnected()) return;
      const on = String(args.STATE || "ON").toUpperCase() === "ON";
      await this._write(on ? "t" : "u");
    }

    async frontPower(args) {
      if (!this._requireConnected()) return;
      const on = String(args.STATE || "ON").toUpperCase() === "ON";
      await this._write(on ? "f" : "g");
    }

    async parkingColor(args) {
      if (!this._requireConnected()) return;
      const col = String(args.COL || "W").trim().toUpperCase();
      if (!col) return;
      // Send as a single 2-byte write (C + code) to avoid interleaving.
      await this._write("C" + col);
    }

    // ----- AUTONOMY -----
    async lineFollow(args) {
      if (!this._requireConnected()) return;
      const on = String(args.STATE || "ON").toUpperCase() === "ON";
      if (on) {
        this._lineFollow = true;
        this._stopHeartbeat();
        await this._write("Z");
      } else {
        this._lineFollow = false;
        await this._write("z");
      }
    }

    // ----- SENSORS (sync cached) -----
    distanceCm() {
      return this._requireConnected() ? this.lastDistanceCm : -1;
    }

    touchPressed() {
      return this._requireConnected() ? (this.lastTouch === 1) : false;
    }

    lux() {
      return this._requireConnected() ? this.lastLux : -1;
    }

    soundLevel() {
      return this._requireConnected() ? this.lastSound : -1;
    }

    lineDetected() {
      return this._requireConnected() ? (this.lastLineDetected === 1) : false;
    }
  }

  Scratch.extensions.register(new RobotR2());
})(Scratch);
