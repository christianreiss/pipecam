# pipecam — Linux V4L2 driver for the "Look Kellyop lem01camera" USB pipe camera

Turns the USB pipe/endoscope camera `3456:4321` into a native `/dev/videoN`
capture device.

* **Format:** 450×450, `YUYV` (YUV 4:2:2), ~20 fps
* **Status:** passes `v4l2-compliance` 57/57, 0 failures, 0 warnings
* **Kernel tested:** 7.1.7-100.fc43.x86_64 (Fedora 43)

---

## Why a custom driver is needed

The camera is **not** a UVC device, so `uvcvideo` cannot drive it:

| | |
|---|---|
| `bDeviceClass` | 239 / 2 / 1 (Misc, Interface Association) |
| Interface 0 | class `0xFF`, subclass `0xF0`, string **"iAP Interface"** |
| Interface 1 | class `0xFF`, subclass `0xF0`, string **"com.linkback.protocol"** |

Both interfaces are vendor-specific. The device is built as an **Apple MFi
accessory**: interface 0 is the iAP (iPod Accessory Protocol) control channel,
and interface 1 exposes the External Accessory protocol `com.linkback.protocol`
that its companion mobile app talks to.

Crucially, `wTotalLength` is `0x57` (87 bytes), which is *exactly* the standard
descriptors with nothing left over — the device carries **no UVC class
descriptors at all**. There is no VideoControl or VideoStreaming descriptor for
`uvcvideo` to parse, so no amount of quirking or `usbcore` ID forcing will bind
it.

This driver binds **interface 1 only**. Interface 0 (iAP) is deliberately left
unclaimed.

## The wire protocol

Established by capturing and analysing the live device (see *How this was
determined* below).

### Starting and stopping the stream

Interface 1 has three altsettings:

| Alt | Endpoints | Role |
|-----|-----------|------|
| 0 | bulk OUT `0x02` | idle — **no IN endpoint, so the stream is off** |
| 1 | bulk IN `0x82`, bulk OUT `0x02` | **streaming (used by this driver)** |
| 2 | isochronous IN `0x83`, 3×1024 B/µframe | alternative streaming path (unused) |

There is **no vendor "start" command**. Selecting altsetting 1 starts the data
flow; selecting altsetting 0 stops it, because that altsetting has no IN
endpoint at all. This is the driver's entire streamon/streamoff mechanism.

### Payload framing

Every **512-byte USB packet** carries its own UVC-style payload header. Note
this is per *USB packet*, not per *bulk transfer* — this is the single most
important detail in the whole protocol:

```
byte 0     bHeaderLength   always 12
byte 1     bmHeaderInfo    bit0 FID, bit1 EOF, bit2 PTS, bit3 SCR, bit7 EOH
byte 2..5  PTS
byte 6..11 SCR
```

Observed header-info bytes are `0x8C`/`0x8D` (`EOH|SCR|PTS`, with the frame ID
toggling between frames) and `0x8E`/`0x8F` for the last packet of each frame.

So each packet carries 500 bytes of payload, and one frame is exactly
**810 packets × 500 bytes = 405000 bytes**.

> **Implementation warning.** A driver that treats one URB as one payload and
> strips 12 bytes once will produce a subtly *sheared* image that still looks
> almost right. The header must be stripped from every 512-byte packet inside
> the URB buffer. URB buffers are therefore sized as a multiple of 512 so a
> short-packet completion always lands on a packet boundary.

### Frame geometry and pixel format

```
450 × 450, 900 bytes per line, 405000 bytes per frame, YUYV 4:2:2, ~20 fps
```

## Build and install

Requires `kernel-devel` matching your running kernel, plus `gcc` and `make`.

### Quick build (does not survive a kernel update)

```sh
make
sudo insmod ./pipecam.ko
```

### Permanent install via DKMS (recommended)

Fedora ships new kernels frequently; DKMS rebuilds the module automatically so
it keeps working after an update.

```sh
sudo dnf install -y dkms kernel-devel
sudo mkdir -p /usr/src/pipecam-1.0
sudo cp pipecam.c Makefile dkms.conf /usr/src/pipecam-1.0/
sudo dkms add     -m pipecam -v 1.0
sudo dkms build   -m pipecam -v 1.0
sudo dkms install -m pipecam -v 1.0
```

The module then autoloads on plug-in via its `MODULE_DEVICE_TABLE`.

To remove: `sudo dkms remove -m pipecam -v 1.0 --all`

> **Secure Boot:** this is an out-of-tree, unsigned module. Secure Boot is
> currently **disabled** on this machine, so it loads fine. If you ever enable
> it you must enroll a MOK and sign the module, or it will be refused.

## Usage

Find the node (it is whichever `/dev/video*` reports `3456:4321`):

```sh
v4l2-ctl --list-devices
```

Inspect and capture:

```sh
v4l2-ctl -d /dev/video2 --list-formats-ext
ffplay -f v4l2 -input_format yuyv422 -video_size 450x450 -i /dev/video2
ffmpeg -f v4l2 -input_format yuyv422 -video_size 450x450 -i /dev/video2 -frames:v 1 shot.png
```

Access requires membership in the `video` group (or root).

### Note on the 450×450 resolution

450 is even, so YUYV macropixels are fine, but it is **not** a multiple of 4, 8
or 16. Most tools handle it, but some GStreamer paths, hardware encoders and
browser capture stacks assume aligned dimensions and may complain. If a
consumer chokes, scale to a friendly size, e.g.:

```sh
ffmpeg -f v4l2 -input_format yuyv422 -video_size 450x450 -i /dev/video2 \
       -vf scale=448:448 ...
```

## Verification performed

| Test | Result |
|---|---|
| `v4l2-compliance -d /dev/video2 -s` | **57 passed, 0 failed, 0 warnings** |
| Live stream, 600 frames | **zero dropped** (V4L2 sequence 0→599, span exactly 599) |
| Frame interval | 48.20–48.29 ms (0.09 ms jitter), 20.72 fps sustained |
| Liveness | **0 byte-identical consecutive frames**; inter-frame MAD 2.3–10.5 |
| Recorded H.264 file | 450×450, 20 fps, exactly 200 frames in 10.000 s |
| Frame sizing end-to-end | 5 frames captured = exactly 5 × 405000 bytes |
| Image correctness | sharp and geometrically correct — no shear or tearing |
| 5× streamon/streamoff cycles | clean, no errors |
| Concurrent open | correctly returns `EBUSY` |
| Hot-unplug while streaming | app gets clean `EIO` on `DQBUF`; node removed; no oops/leak |
| Rebind after unbind | works |
| 3× `rmmod`/`insmod` cycles | clean |
| `dmesg` across all tests | no warnings, no errors |

## How this was determined

No public documentation or driver for this device could be found, so the
protocol was recovered empirically from the device itself:

1. **Descriptor dump** (`lsusb -v` as root) revealed the vendor-specific classes
   and, decisively, the `iAP Interface` / `com.linkback.protocol` interface
   strings.
2. **Blind endpoint probe** showed the device streams unprompted on EP `0x82`
   once altsetting 1 is selected — no init sequence is required.
3. **Header identification:** `bHeaderLength = 12` with info byte `0x8C`
   decodes as `EOH|SCR|PTS`, and 2 + 4 (PTS) + 6 (SCR) = 12 bytes exactly —
   a self-consistent match for a UVC-style payload header. 99.99% of 172,459
   captured packets had `bHeaderLength = 12`.
4. **Geometry by autocorrelation** of the payload stream: a strong peak at lag
   **900** (r = 0.89) with clean harmonics at 1800/2700/3600/4500. Combined with
   the 405000-byte frame size this gives exactly 450 rows of 900 bytes.
5. **Pixel format by component statistics:** luma sits on even byte offsets
   (mean 74, std 79) and chroma on odd offsets (mean **128.1**, std 8.1),
   with no planar split — i.e. packed 4:2:2 with luma first.
6. **Visual confirmation:** decoding a captured frame as 450×450 YUYV produced a
   sharp, correctly proportioned image.

The ASCII-looking bytes `"23xx"` and `".00"` visible early in the stream are a
red herring — they are just PTS/SCR clock field values inside the 12-byte
header, not a version string.

## Hardware background

Determined by locating and decompiling the vendor's companion app ("LinkBack",
Android `com.lemai.linkback`, iOS `com.jmllm.linkback` — the iOS build is what
declares `com.linkback.protocol`). No public driver, decompilation or protocol
description for this device existed prior to this work.

* **Sensor:** GalaxyCore **GC0309** — 1/9" VGA, 648×488 array, outputs Bayer /
  RGB565 / **YCbCr 4:2:2**, ≤30 fps. It has no JPEG encoder, which is why the
  stream is raw YUV rather than MJPEG.
* **The vendor app drives this device with plain libusb** — it claims the
  interface and sets the altsetting, exactly as this driver does. There is **no
  iAP2/MFi code in the Android app at all**; the iAP channel is used only when
  the accessory is plugged into an iPhone. Ignoring interface 0 is correct.
* The app matches the device **by USB class (239/2), not by VID/PID**, which is
  why the VID/PID appears in no public database.
* The app changes resolution and pixel clock via **control transfers**
  (`writeControlEndpointToResolution`, `writeControlEndpointToClk`). The exact
  request constants live in an obfuscated Dart AOT snapshot and were not
  recovered.

### A note on resolution

The vendor app's `assets/config/cam_config.json` maps this unit's serial suffix
(`000002`) to a GC0309 at 544×408. **This device measurably streams 450×450**
(405000 bytes/frame, stride 900) in its default power-on mode, which is what
this driver implements. 544×408 would be 443904 bytes/frame and would render
visibly sheared; it does not match the capture. The discrepancy is most likely
because the app selects a mode via control transfer at startup, whereas this
driver uses the device's own default. Recorded here as an open question.

## Limitations and unexplored areas

These are outside the scope of "expose the camera as `/dev/video`":

* **Chroma component order is unconfirmed.** Luma/chroma *positions* are certain
  (proven by the std 80 vs std 6–9 split), but every scene captured during
  development was essentially neutral — both chroma channels sat within ~2 of
  128 — so U-vs-V order could not be distinguished. If colours look swapped on a
  saturated target, change `V4L2_PIX_FMT_YUYV` to `V4L2_PIX_FMT_YVYU` in
  `pipecam_enum_fmt()` and `pipecam_fill_fmt()`.
* **Resolution is fixed at 450×450.** Mode switching is known to be possible via
  control transfers, but the request constants were not recovered.
* **No controls are exposed** (brightness, LED intensity). The app pushes named
  GC0309 register lists to the device; that command set was not reverse-engineered.
* **The isochronous path (altsetting 2) is unused.** The vendor app uses both;
  the bulk path here is stable at ~8 MB/s with zero errors over sustained
  capture, so isochronous bandwidth reservation was not worth taking on.
