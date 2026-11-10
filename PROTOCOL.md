# K4 serial protocol

This is the wire protocol of K4-class USB mini laser engravers (80 × 76 mm,
0.05 mm pitch, CH340 USB-serial bridge), as implemented by `k4laser`.

It was reconstructed for interoperability by reading the IL of the vendor's
Windows application "Laser_K4ble V2.7 (2022-06-25)" (`Laser Framework4.exe`,
a .NET WinForms program). No vendor code is included in this project. Facts
below are tagged by how they are known:

- **[vendor]**: read directly from the vendor application.
- **[sim]**: implemented and checked against the built-in simulator only.
- **[hw]**: observed on real hardware.
- **[?]**: unknown or unconfirmed. Contributions welcome.

## Transport

- 115200 baud, 8N1, no flow control. **[vendor]**
- USB bridge: QinHeng CH340, `1a86:7523`. **[hw]** It has no serial number,
  so two CH340 devices get the same `/dev/serial/by-id` name; use
  `/dev/serial/by-path`.
- Half-duplex request/reply: the host sends one frame and waits for the reply
  before sending the next. **[vendor]**

## Framing (host → device)

```
[opcode u8][length u16 big-endian][payload …]
```

`length` counts the whole frame, header included. Commands without arguments
are four bytes, with one zero byte of payload: `[op, 0x00, 0x04, 0x00]`.
**[vendor]**

All multi-byte integers are big-endian. Coordinates are machine pixels of
0.05 mm (508 dpi), with the origin at the top-left of the work area as the
vendor preview shows it. **[vendor]**

## Replies (device → host)

| Bytes | Meaning |
|---|---|
| `09` | ACK: command accepted / ready for the next data frame |
| `08` | resend the last data frame |
| 3 bytes | reply to `0xFF` (model) |
| `FF FF FF FE` | offline-job flash erase finished |

The vendor polls for the reply byte in a 10 ms loop, with a timeout per
command. **[vendor]** `STOP` (`0x16`) gets no reply that the vendor waits for.
**[vendor]**

Each data frame (raster row, vector batch) is ACKed when the device can take
the next one, so the ACK paces the stream. A high depth or power makes rows
slow, so the host timeout has to be long. **[vendor]**, **[sim]**

## Commands

The vendor method names (pinyin) are the evidence trail.

| Op | Name | Payload | Vendor method | Notes |
|---|---|---|---|---|
| `0x0A` | HANDSHAKE | none | `lianjie`, `ji_huo` | ACK within 400 ms |
| `0x28` | SETTINGS | 16 bytes, see below | `she_zhi` | Sent on every connect |
| `0xFF` | MODEL | none | `du_ban_ben` | 3-byte reply |
| `0x17` | HOME | none | `dao_yuandian` | Head to origin (0, 0) |
| `0x01` | MOVE_REL | dx s16, dy s16 | `yidong` | Relative head move, laser off |
| `0x06` | RESET | none | `fu_wei` | Sent before every job |
| `0x14` | START | x u16, y u16 | `kai_shi` | Begin an online job at (x, y) |
| `0x09` | RASTER_ROW | depth u16, power u16, row u16, bits | `diaoke2` | One raster line |
| `0x1D` | VECTOR | depth u16, power u16, n × (x0 y0 x1 y1 u16) | `diaoke_lijing` | ≤ 80 segments per frame |
| `0x16` | STOP | none | `ting_zhi` | Ends a job or aborts it |
| `0x18` | PAUSE | none | `zan_ting` | |
| `0x19` | RESUME | none | `ji_xu` | |
| `0x20` | FRAME_ON | w u16, h u16, x u16, y u16 | `kai_kuang` | Trace a rectangle with the low-power beam |
| `0x21` | FRAME_OFF | none | `guan_kuang` | |
| `0x23` | OFFLINE_JOB | x, y, w, h, power, depth, chunks (u16 each) | `kai_shi_tuo_ji` | Store a job in flash |
| `0x22` | OFFLINE_ROW | inverted bits + checksum | `tuo_ji` | |
| `0xFD` / `0xFE` | AUX on / off | none | `yi_dong_z(true/false)` | Hidden, see below |

### Power and depth

- `power` is in per mille, 0–1000. The vendor slider shows 0–100 % and
  multiplies by 10. **[vendor]**
- `depth` (vendor: "carving depth") is 0–100. It is most likely the dwell time
  per pixel; the unit is unknown. **[?]**

### Raster rows (`0x09`)

`row` is the line index inside the job, from the START origin. `bits` is one
bit per pixel, MSB first, leftmost pixel first, `1` = burn, padded to a whole
byte. The vendor skips blank rows. **[vendor]**

### Vector batches (`0x1D`)

Segments are relative to the START origin. The vendor sends them in batches of
640 coordinate bytes, which is 80 segments. **[vendor]** `k4laser` orders
polylines nearest-neighbour to shorten travel moves.

### Settings (`0x28`)

The defaults are the values shipped in the vendor `pei_zhi.ini`. The names
translate the labels of the vendor's settings dialog. **[vendor]**

| Offset | Type | Key (`config.toml`) | Vendor label | Default |
|---|---|---|---|---|
| 0 | u8 | `standby_intensity` | 待机激光强度 (standby laser intensity) | 65 |
| 1 | u16 | `line_delay` | 行延时 (line delay) | 100 |
| 3 | u16 | `max_power` | 最大激光功率 (max laser power) | 1000 |
| 5 | u8 | `microsteps` | 细分数 (microstepping) | 8 |
| 6 | u8 | `x_backlash` | X轴换向补偿 (X reversal compensation) | 1 |
| 7 | u8 | `y_backlash` | Y轴换向补偿 (Y reversal compensation) | 1 |
| 8 | u16 | `accel_steps` | 变速阶梯数 (acceleration steps) | 200 |
| 10 | u16 | `speed_min` | 速度下限 (speed lower limit) | 650 |
| 12 | u16 | `speed_max` | 速度上限 (speed upper limit) | 400 |
| 14 | u16 | `frame_speed` | 框定位速度 (framing speed) | 10 |

With the defaults, the frame is
`28 00 13 41 00 64 03 E8 08 01 01 00 C8 02 8A 01 90 00 0A`. `speed_min` and
`speed_max` look like timer periods, which would explain why the "lower limit"
is the larger number. **[?]**

### Model (`0xFF`)

The reply is three bytes. The vendor chooses the work area from byte 3.
**[vendor]**

| Byte 3 | Work area |
|---|---|
| 4, 31 | 1600 × 1520 px = 80 × 76 mm (K4) |
| 21, 22 | 3100 × 3500 px |

The vendor `pei_zhi.ini` stores the model as `3,1,4`.

### Framing (`0x20` / `0x21`)

`FRAME_ON w h x y` starts tracing a w × h rectangle and `FRAME_OFF` ends it.
The vendor uses this pair to show where a job will burn. **[vendor]** The
beam probably runs at `standby_intensity` and loops until `FRAME_OFF`. **[?]** `k4laser` first moves the head to the job's top-left
corner with `MOVE_REL`, then sends `FRAME_ON w h 0 0`. **[vendor]**, **[sim]**
Whether `x`, `y` are offsets from the head or absolute positions is not
confirmed. **[?]**

### Offline jobs (`0x23` / `0x22`)

Some models store a raster job in flash and burn it from a button, without a
host connection.

- `OFFLINE_JOB` carries x, y, w, h, power, depth and a chunk count of
  `(w·h/8) / 4069 + 1`. The device erases flash and replies `FF FF FF FE`.
  **[vendor]**
- Each `OFFLINE_ROW` carries the row bits **inverted** (0 = burn) followed by a
  checksum: the byte sum of the frame, negated when it exceeds 255, and
  truncated to 8 bits. The vendor calls this `get_jiayan`. **[vendor]**

`k4laser` encodes these frames but does not use them yet.

### `0xFD` / `0xFE`

`yi_dong_z(bool)` sends `FD 00 04 00` (true) or `FE 00 04 00` (false) and waits
up to 3 s for an ACK. It is called only from `button1_Click` and
`button2_Click`. Neither handler is attached to a control on any form, so the
V2.7 UI never sends these bytes. **[vendor]** "yi dong z" reads as "move Z",
but the K4 has no Z axis. These are an on/off pair, perhaps for an auxiliary
output such as a fan or air assist. This is untested. **[?]**

`k4laser aux on|off` sends them. Use it only with the machine attended.

## Sequences

Connect:

```
HANDSHAKE → SETTINGS → MODEL → HOME
```

The vendor finds the machine by sending `HANDSHAKE` to **every** serial port
and waiting 400 ms on each for an ACK. **[vendor]** Writing binary data to an
unrelated serial device is unsafe, so `k4laser` never does this: it only
opens the port it is given. It retries `HANDSHAKE` three times, 0.5 s apart.

Online job, per operation and per pass:

```
MOVE_REL to (x, y) → RESET → START x y → data frames (each ACKed) → STOP
```

After `STOP`, `k4laser` treats the head position as unknown and homes before
the next absolute positioning. **[sim]**

Framing:

```
MOVE_REL to (x, y) → FRAME_ON w h 0 0 … FRAME_OFF
```

## Open questions

- Are `START` coordinates absolute, or relative to the head position?
- What is the unit of `depth`, and how does it interact with `line_delay`?
- How exactly does the device behave after `STOP`? Does it hold or lose its
  position?
- What do `0xFD` / `0xFE` do?
- What are the work areas of other model codes?

To capture evidence, run `k4laser … --trace FILE` for the host side. The
output of `k4laser decode FILE` can be attached to an issue.
