# Rift CV1 Control Center

GTK4/libadwaita control center for running an **Oculus Rift CV1** on Linux
with SteamVR, wrapping the OpenHMD stack:
[SteamVR-OpenHMD](https://github.com/ChristophHaag/SteamVR-OpenHMD) +
[OpenHMD `rift-kalman-filter`](https://github.com/thaytan/OpenHMD) +
[ouvrt](https://github.com/pH5/ouvrt) for Touch controller pairing.

## Features

- **Status card**: headset USB/HID, tracking sensors (count + USB-3 link
  check), controller battery (while SteamVR runs), HDMI link, room
  calibration, driver registration, OpenVR runtime, SteamVR — with a smart
  suggestion banner and an optional USB-wedge auto-fix.
- **Wake test** — verifies tracking + display link end to end.
- **Fix USB** — driver reattach, then USB reset, for the "HID not bound
  after the headset reboots itself" wedge.
- **Touch pairing wizard** — reboots the headset radio into pairing mode via
  ouvrt, bonds both controllers, reboots back automatically.
- **Runtime switching** between SteamVR and WiVRn (xrizer).
- **Camera view & placement guide** — live sensor debug stream (PipeWire)
  with placement diagrams for play, calibration, pairing and testing.
- **Advanced tools** — tracking quality test, in-SteamVR pose test, room
  calibration info/reset, headset serial/firmware info, EDID/USB/kernel/log
  inspection, diagnostics export.
- **Setup & install** — clones and builds the whole stack, installs udev
  rules, registers the driver, adds a desktop entry.

## Usage

GUI:

```sh
python3 rift_cv1_center.py
```

CLI (scriptable, no GUI/GTK needed):

```sh
python3 rift_cv1_center.py status            # exit code 1 if something is wrong
python3 rift_cv1_center.py fix-usb
python3 rift_cv1_center.py test              # wake test (15 s)
python3 rift_cv1_center.py switch-runtime steamvr
python3 rift_cv1_center.py room [--reset]
python3 rift_cv1_center.py info | devices | export
```

## Configuration

Defaults assume the stack lives in `~/git/`. Override any path via
`~/.config/rift-cv1-center/config.json`:

```json
{
  "steamvr_openhmd": "~/src/SteamVR-OpenHMD",
  "ouvrt_dir": "~/src/ouvrt",
  "steamvr_path": "~/.local/share/Steam/steamapps/common/SteamVR",
  "steam_logs": "~/.local/share/Steam/logs",
  "openhmd_config": "~/.config/openhmd"
}
```

or environment variables (`RIFT_CV1_STEAMVR_OPENHMD=…` etc.), which take
precedence.

## Layout

```
rift_cv1_center.py    entry point (GUI or CLI)
riftcv1/
  config.py           paths & constants (+ config file / env overrides)
  hw.py               USB sysfs, hidraw, EDID, USB reset/reattach
  runtime.py          openvrpaths, runtime switching, SteamVR device query
  install.py          build/install steps for the stack
  diagnostics.py      one-shot diagnostic tools & report export
  cli.py / gui.py     the two frontends
  pairing.py          ouvrt DBus + Touch pairing wizard
  camview.py          sensor camera stream + placement guides
  pose_test.py        in-SteamVR pose stability test (runs in the venv)
  vrinfo.py           SteamVR device/battery query (runs in the venv)
```

The `openvr` Python bindings live in a private venv (created on first use
of the pose test) — either `./venv` or
`~/.local/share/rift-cv1-center/venv`.
