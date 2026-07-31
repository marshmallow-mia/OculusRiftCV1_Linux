#!/usr/bin/env python3
"""Is SteamVR actually delivering frames, or reprojecting them?

The pose has been measured clean - 0.64 mm residual, no oscillation at any
frequency, against the Oculus runtime's 0.27 mm - so the reported overshoot is
not coming from tracking. The same tracking code under Monado feels better,
which points at presentation. But that comparison has a confound: hello_xr is
trivial to render, and a SteamVR application that misses frame deadlines gets
reprojected, which by itself looks like the world lagging and then catching up.

So measure it instead of arguing about it. SteamVR's compositor publishes its
own frame statistics; this samples them while the symptom is happening.

  m_nNumDroppedFrames     frames the compositor never got in time
  m_nNumFramePresents     >1 means the same frame was shown repeatedly
  m_nNumMisPresented      frames that missed their vsync
  m_nReprojectionFlags    non-zero means the compositor synthesised motion
  m_flPreSubmitGpuMs etc  where the time actually goes

Run it in a second terminal while SteamVR is up and the problem is visible:

  venv/bin/python tools/frame_timing.py --duration 30
"""
import argparse
import time

import openvr


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=30.0)
    ap.add_argument("--hz", type=float, default=20.0)
    a = ap.parse_args()

    openvr.init(openvr.VRApplication_Background)
    comp = openvr.VRCompositor()
    if comp is None:
        print("no compositor - is SteamVR running?")
        return

    fields = ("m_nNumFramePresents", "m_nNumMisPresented", "m_nNumDroppedFrames",
              "m_nReprojectionFlags", "m_flPreSubmitGpuMs", "m_flPostSubmitGpuMs",
              "m_flTotalRenderGpuMs", "m_flCompositorRenderGpuMs",
              "m_flCompositorIdleCpuMs", "m_flClientFrameIntervalMs")
    acc = {f: [] for f in fields}
    frames_seen = set()
    n = 0
    t_end = time.time() + a.duration
    while time.time() < t_end:
        try:
            ok, t = comp.getFrameTiming(0)   # pyopenvr returns (result, timing)
        except Exception as e:
            print("getFrameTiming failed:", e)
            break
        if ok:
            idx = getattr(t, "m_nFrameIndex", None)
            if idx is not None and idx in frames_seen:
                time.sleep(1.0 / a.hz)
                continue
            if idx is not None:
                frames_seen.add(idx)
            for f in fields:
                v = getattr(t, f, None)
                if v is not None:
                    acc[f].append(float(v))
            n += 1
        time.sleep(1.0 / a.hz)
    openvr.shutdown()

    if not n:
        print("no frame timing returned")
        return

    print("sampled %d distinct frames over %.0f s\n" % (n, a.duration))
    import statistics as st
    for f in fields:
        v = acc[f]
        if not v:
            continue
        print("  %-28s mean %9.3f  max %9.3f" % (f, st.mean(v), max(v)))

    print()
    drop = sum(acc["m_nNumDroppedFrames"])
    mis = sum(acc["m_nNumMisPresented"])
    rep = sum(1 for x in acc["m_nReprojectionFlags"] if x)
    multi = sum(1 for x in acc["m_nNumFramePresents"] if x > 1)
    print("  dropped frames total      : %d" % drop)
    print("  mis-presented total       : %d" % mis)
    print("  frames shown more than once: %d of %d (%.0f%%)" % (multi, n, 100.0 * multi / n))
    print("  frames with reprojection  : %d of %d (%.0f%%)" % (rep, n, 100.0 * rep / n))
    print()
    if multi > n * 0.1 or rep > n * 0.1 or drop:
        print("  VERDICT: the compositor is NOT delivering one fresh frame per")
        print("  display refresh. Reprojected and repeated frames look exactly")
        print("  like the world lagging and catching up, regardless of how good")
        print("  the tracking is. Fix the frame rate before anything else.")
    else:
        print("  VERDICT: frames are being delivered cleanly, so the artifact is")
        print("  not frame pacing - it is in the rendering geometry itself.")


if __name__ == "__main__":
    main()
