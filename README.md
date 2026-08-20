# pipecam — Linux V4L2 driver for the "Look Kellyop lem01camera" USB pipe camera

Makes a vendor-specific USB pipe/endoscope camera (`3456:4321`) work as a native
`/dev/videoN` capture device. It is **not** a UVC device, so no in-tree driver
binds it and it is invisible as a webcam on a stock system.

* **Format:** 450×450, `YUYV` (YUV 4:2:2), full-range
* **Frame rate:** ~20–27 fps, sensor-controlled (see [Frame rate](#frame-rate))
* **Status:** `v4l2-compliance` 57/57, 0 failures, 0 warnings
* **Developed and tested on:** Fedora 43, kernel 7.1.7

---

## Is this my camera?

This driver is for one specific vendor protocol, not endoscopes in general.
**Most USB endoscopes are plain UVC and need no driver at all.** Check:

```sh
lsusb | grep 3456:4321
```

You have this device if **all** of the following hold:

1. `lsusb` shows `ID 3456:4321`, with `iProduct` = `lem01camera` and
   `iManufacturer` = `Look Kellyop`.
2. **No `/dev/video*` node appears** when you plug it in, and it does not show
   up in `v4l2-ctl --list-devices`.
3. The manual or packaging tells you to install the **LinkBack** app
   (Android `com.lemai.linkback`, iOS `com.jmllm.linkback`).

If your manual names a *different* app — UseePlus, Xscope, i-See-Pro,
SUP-ANESOK, "Smart Endoscope" — it is a different device and this driver does
not apply.

> `Look Kellyop` is not a retail brand and `0x3456` is not a USB-IF assigned
> vendor ID (`3456:4321` is a placeholder). Searching for either will not find
> your product. Identify by the descriptor strings above.

## Supported hardware

### Retail products

These cameras are white-labelled heavily and **the brand on the box is not a
reliable identifier** — the same brand ships incompatible units. Only one retail
product has been positively confirmed to use this protocol:

| Brand | Model / ASIN | Probe | Confirmation |
|---|---|---|---|
| Ruycllo | "A8" — `B0GXYT79BM` (amazon.de) | 8 mm, 5 m semi-rigid, 6 LED | listing image instructs installing "Linkback" |

For contrast, Ruycllo `B0FBM53JDK` is a near-identical 8 mm product from the
*same brand* that uses a different app (`i-See-Pro`) and is **not** supported.
Use the descriptor test above, not the brand.

### Hardware variants

The vendor app ships `assets/config/cam_config.json` enumerating seven camera
variants. **450×450 is a documented vendor mode**, which is what this device
streams and what the driver implements:

| Variant | Sensor | Native resolution | Bytes/frame | Status |
|---------|--------|-------------------|-------------|--------|
| 000001 | BF2013 | **450×450** | 405000 | **works** |
| 000003 | GC0328 | **450×450** | 405000 | **works** |
| 000005 | BF20A6 | **450×450** | 405000 | **works** |
| 000004 | BF2013 | 400×400 | 320000 | not implemented |
| 000011 | BF20A6 | 400×400 | 320000 | not implemented |
| 000002 | GC0309 | 544×408 (+512×384, 448×336, 320×240) | 443904 | not implemented |
| 000009 | SP0A39 | 608×456 (+512×384, 448×336, 320×240) | 554496 | not implemented |

All five sensors are cheap VGA-class CMOS parts (GalaxyCore `GC*`, BYD Micro
`BF*`, SuperPix `SP*`) with no JPEG encoder — which is why the stream is raw YUV
rather than MJPEG. Any "1920P" claim on the packaging is marketing: a VGA sensor
behind a cropped transport window, upscaled by the app.

**Which variant is mine?** Unknown, and the driver does not need to know. The
tested unit streams 450×450, so it is one of BF2013/GC0328/BF20A6. A plausible
guess is that the config `sn` is the last six digits of the USB serial, but that
is **unverified and contradicted here**: this unit's serial ends `000002`, which
would imply the GC0309 544×408 variant, yet it demonstrably streams 450×450
(405000 bytes/frame, not 443904).

If your unit is one of the other variants the driver will bind but deliver no
frames, because every frame fails the 405000-byte size check; `dmesg` will not
report frames. Adjusting `PIPECAM_WIDTH`/`PIPECAM_HEIGHT` in `pipecam.c` should
suffice — all variants are packed YUYV, so the framing logic is unchanged.

## Why a custom driver is needed

| | |
|---|---|
| `bDeviceClass` | 239 / 2 / 1 (Misc, Interface Association) |
| Interface 0 | class `0xFF`, subclass `0xF0`, string **"iAP Interface"** |
| Interface 1 | class `0xFF`, subclass `0xF0`, string **"com.linkback.protocol"** |

Both interfaces are vendor-specific. The device is built as an **Apple MFi
accessory**: interface 0 is the iAP (iPod Accessory Protocol) channel, and
interface 1 carries the External Accessory protocol `com.linkback.protocol`.

`wTotalLength` is `0x57` (87 bytes) — *exactly* the standard descriptors with
nothing left over, so the device carries **no UVC class descriptors at all**.
There is no VideoStreaming descriptor for `uvcvideo` to parse, so no amount of
quirking or ID forcing will bind it.

This driver claims **interface 1 only**. Interface 0 is deliberately left
unclaimed — the vendor's own Android app contains no iAP2/MFi code either and
drives the device with plain libusb, so the iAP channel is only used when the
accessory is attached to an iPhone.

## The wire protocol

### Starting and stopping the stream

Interface 1 has three altsettings:

| Alt | Endpoints | Role |
|-----|-----------|------|
| 0 | bulk OUT `0x02` | idle — **no IN endpoint, so the stream is off** |
| 1 | bulk IN `0x82`, bulk OUT `0x02` | **streaming (used by this driver)** |
| 2 | isochronous IN `0x83`, 3×1024 B/µframe | alternative path (unused here) |

There is **no vendor "start" command**. Selecting altsetting 1 starts the data
flow; selecting altsetting 0 stops it, because that altsetting has no IN
endpoint. That is the driver's entire streamon/streamoff mechanism.

### Payload framing

The stream is a sequence of variable-length logical packets, each carrying a
12-byte UVC-style header:

```
byte 0     bHeaderLength   always 12
byte 1     bmHeaderInfo    bit0 FID, bit1 EOF, bit2 PTS, bit3 SCR, bit7 EOH
byte 2..5  PTS
byte 6..11 SCR
```

Observed header-info bytes are `0x8C`/`0x8D` (`EOH|SCR|PTS`, frame ID toggling
between frames) and `0x8E`/`0x8F` on the last packet of a frame.

**Packets are aligned to 512-byte boundaries within a bulk transfer.** This is
the single detail that matters most for a reimplementation: a new header begins
at every 512-byte offset inside the URB buffer, *not* once per transfer. In a
measurement of 20000 transfers, byte 512 of every one of the 8895 transfers
longer than 512 bytes was `0x0C` — a header length. A driver that strips 12
bytes once per transfer produces a subtly **sheared** image that still looks
almost right.

Packet payloads are therefore **not** uniform: a full 512-byte USB packet
carries 500 bytes of payload, but each transfer ends in a short packet carrying
less. Measured over 138950 packets, 50.3% carried exactly 500 bytes and the rest
were shorter. A frame is **~1040 packets** (1037–1046 observed), always totalling
**exactly 405000 payload bytes**.

### Frame geometry and pixel format

```
450 × 450, 900 bytes per line, 405000 bytes per frame, YUYV 4:2:2, full range
```

Luma occupies even byte offsets, chroma odd offsets. The sensor emits
**full-range** YCbCr — measured luma spans 6..254, with ~3% of pixels above the
limited-range white point of 235 — so the driver reports
`V4L2_QUANTIZATION_FULL_RANGE`.

### Frame rate

**The frame rate is not fixed.** The sensor runs auto-exposure and lowers its
rate in poor light; 20.7 fps and 27.0 fps have both been measured on the same
unit under different lighting, each rock-steady at the time (<0.1 ms jitter).

The host cannot select the rate, so the driver deliberately does not implement
`VIDIOC_ENUM_FRAMEINTERVALS`, `VIDIOC_G_PARM`, or `VIDIOC_S_PARM`. Advertising
a range there would promise selectable frame periods that the hardware cannot
provide. Each completed buffer instead receives its real monotonic capture
timestamp, which preserves the exposure-driven cadence for consumers.

Bus throughput scales with the rate — roughly 8.4 MB/s at 20 fps and 11.2 MB/s
at 27 fps.

## Build and install

Requires `kernel-devel` matching your running kernel, plus `gcc` and `make`.

**Minimum kernel: 6.8.** The driver uses `vb2_queue.min_queued_buffers`, which
was renamed from `min_buffers_needed` in 6.8. On older kernels the build fails;
substituting the old field name is the only change needed.

### Quick build (does not survive a kernel update)

```sh
make
sudo insmod ./pipecam.ko
```

### Permanent install via DKMS (recommended)

```sh
sudo dnf install -y dkms kernel-devel     # or: apt install dkms linux-headers-$(uname -r)
sudo mkdir -p /usr/src/pipecam-1.0
sudo cp pipecam.c Makefile dkms.conf /usr/src/pipecam-1.0/
sudo dkms add     -m pipecam -v 1.0
sudo dkms build   -m pipecam -v 1.0
sudo dkms install -m pipecam -v 1.0
```

The module then autoloads on plug-in via its `MODULE_DEVICE_TABLE`.
To remove: `sudo dkms remove -m pipecam -v 1.0 --all`

> **Secure Boot:** this is an out-of-tree, unsigned module. With Secure Boot
> enforcing you must enroll a MOK and sign the module, or it will be refused.

## Usage

Find the node — it is whichever `/dev/video*` reports `3456:4321`:

```sh
v4l2-ctl --list-devices
```

Then:

```sh
v4l2-ctl -d /dev/video2 --list-formats-ext
ffplay -f v4l2 -i /dev/video2
ffmpeg -f v4l2 -i /dev/video2 -frames:v 1 shot.png
vlc v4l2:///dev/video2
```

Access requires membership in the `video` group (or root).

### Note on the 450×450 resolution

450 is even, so YUYV macropixels are fine, but it is **not** a multiple of 4, 8
or 16. Most tools cope, but some GStreamer paths, hardware encoders and browser
capture stacks assume aligned dimensions. If a consumer chokes, scale it:

```sh
ffmpeg -f v4l2 -i /dev/video2 -vf scale=448:448 ...
```

## Troubleshooting

### `Device or resource busy` (EBUSY)

V4L2 gives one client exclusive access to the queue. `open()` itself succeeds,
but `VIDIOC_REQBUFS` / starting to stream returns `EBUSY` while another process
is using the device. This is normal, not a fault, and clears when that process
exits. Find the holder:

```sh
sudo fuser -v /dev/video2      # or: sudo lsof /dev/video2
```

A common cause is a media player left running after a failed attempt — it keeps
the descriptor open while showing an error.

### VLC

`vlc /dev/video2` does **not** work: VLC treats a bare path as a *file* and
tries to demux it, reporting `filesystem stream error: read error`. Use the
`v4l2://` access module:

```sh
vlc v4l2:///dev/video2
```

Note that the failed file-mode attempt leaves VLC running and holding the
device, which then causes `EBUSY` for everything else until you close it.

### Stream stops and `dmesg` says "endpoint failing"

The driver stops immediately on an endpoint stall, or after 256 consecutive
transient URB errors, and reports the failure to userspace (`DQBUF` returns
`EIO`) rather than stalling silently. Restart the capture to recover; if it
persists, replug the device or reset it in place (the `usbreset` command is
provided by `usbutils`):

```sh
sudo usbreset 3456:4321
```

Each new stream explicitly clears a previous USB endpoint halt before
submitting transfers.

### Capture interrupted by suspend or USB reset

The driver preserves the registered video node and open file handles across a
system suspend/resume or USB reset. If capture was active, it stops all URBs and
puts the VB2 queue into error rather than continuing a partial frame. Userspace
must issue `STREAMOFF`, requeue its buffers, and issue `STREAMON`; applications
that close and reopen the node already perform an equivalent recovery.

## Verification performed

The hardware results below are the established baseline from before the current
interval-UAPI and suspend/reset hardening. They remain regression targets and
must be rerun on the physical camera; the wire format and successful-frame path
did not change, while malformed-frame rejection became stricter.

| Test | Result |
|---|---|
| `v4l2-compliance -d /dev/videoN -s` | **57 passed, 0 failed, 0 warnings** |
| `v4l2-compliance -d /dev/videoN -s --expbuf-device /dev/videoM` | **59 passed, including DMABUF import** |
| DKMS 3.4 add/build/install in isolated trees | **clean** |
| Live stream, 600 frames | **zero dropped** (V4L2 sequence contiguous; driver accounts for discarded frames) |
| Forced 500 ms buffer starvation | **10 dropped frames exposed** as one V4L2 sequence gap |
| Frame interval stability | <0.1 ms jitter at a given light level |
| Liveness | **no byte-identical consecutive frames** |
| Frame sizing | every frame exactly 405000 bytes |
| Timestamp-derived rate | 26.987 fps measured from completed buffers |
| `ffmpeg` frame retention | 135 frames in 5.00 s = no loss in the tested capture |
| Concurrent access | second client correctly gets `EBUSY` |
| Hot-unplug while streaming | clean `EIO`; no oops, WARN or leak |
| fd held open across unplug, closed after | clean |
| Rebind, and `rmmod`/`insmod` cycles | clean |
| `checkpatch.pl` | 0 errors |

Image correctness was confirmed visually — a decoded frame is sharp and
geometrically straight, with no shear, which independently validates the 900-byte
stride and the per-512-byte header handling.

## How this was determined

No public documentation, driver, or protocol description for this device
existed. The protocol was recovered empirically:

1. **Descriptor dump** (`lsusb -v` as root) revealed the vendor-specific classes
   and, decisively, the `iAP Interface` / `com.linkback.protocol` strings.
2. **Blind endpoint probe** showed the device streams unprompted on EP `0x82`
   once altsetting 1 is selected — no init sequence is required.
3. **Header identification:** `bHeaderLength = 12` with info byte `0x8C` decodes
   as `EOH|SCR|PTS`, and 2 + 4 (PTS) + 6 (SCR) = 12 exactly — a self-consistent
   match for a UVC-style payload header.
4. **Geometry by autocorrelation** of the payload stream: a strong peak at lag
   **900** with clean harmonics at 1800/2700/3600/4500, giving 450 rows of 900
   bytes.
5. **Pixel format by component statistics:** luma on even offsets (std ≈ 79),
   chroma on odd offsets centred on 128 (std ≈ 8), with no planar split — packed
   4:2:2, luma first.
6. **Vendor app analysis** (decompiled APK) supplied the variant table and
   confirmed the device is driven with plain libusb and matched by USB class
   rather than VID/PID.

The ASCII-looking bytes `"23xx"` and `".00"` visible early in the stream are a
red herring — they are PTS/SCR clock values inside the 12-byte header, not a
version string.

## Limitations and unexplored areas

* **Chroma component order is unconfirmed.** Luma/chroma *positions* are certain,
  but every scene captured during development was essentially neutral (both
  chroma channels within a few counts of 128), so U-vs-V order could not be
  distinguished. If colours look swapped on a saturated target, change
  `V4L2_PIX_FMT_YUYV` to `V4L2_PIX_FMT_YVYU` in `pipecam_enum_fmt()` and
  `pipecam_fill_fmt()`.
* **Resolution is fixed at 450×450.** The vendor app changes resolution and pixel
  clock through control transfers, but the request constants live in an
  obfuscated Dart AOT snapshot and were not recovered.
* **No controls are exposed** (brightness, LED intensity). The app pushes named
  sensor register lists to the device; that command set was not reverse
  engineered.
* **The isochronous path (altsetting 2) is unused.** The vendor app uses both;
  the bulk path here is stable with zero errors over sustained capture, so
  isochronous bandwidth reservation was not worth taking on.
* **No transparent stream continuation after suspend/reset.** The node and file
  handles survive, but an active capture queue enters error and must be restarted
  with `STREAMOFF`, buffer requeueing, and `STREAMON`.

## License

GPL-2.0-or-later. See [LICENSE](LICENSE).
