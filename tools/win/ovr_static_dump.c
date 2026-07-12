/*
 * tools/win/ovr_static_dump.c
 *
 * Dump LibOVR tracker extrinsics + HMD config to JSON.
 * This is the Tier-0 capture: run once with the headset on desk, sensors idle.
 *
 * Build (MSVC):
 *   cl /I %OVR_SDK%\LibOVR\Include ovr_static_dump.c ^
 *      /link /LIBPATH:%OVR_SDK%\LibOVR\Lib\Windows\x64\Release LibOVR.lib
 *
 * Build (MinGW-w64):
 *   gcc -I%OVR_SDK%/LibOVR/Include ovr_static_dump.c ^
 *       -L%OVR_SDK%/LibOVR/Lib/Windows/x64/Release -lLibOVR -lm -o ovr_static_dump.exe
 *
 * Usage:
 *   ovr_static_dump.exe [outfile.json] [wait_s]
 *   (defaults: stdout, 30 s)
 *
 * Tracker poses are only solved once the runtime is actively tracking the
 * HMD — until then ovr_GetTrackerPose returns a placeholder (0,0,-1) with
 * pose_valid=false. We therefore poll for up to wait_s seconds until every
 * connected tracker reports a valid pose. Put the headset on its taped spot,
 * in view of both sensors, with the proximity sensor covered (tape or cloth
 * over it) so the runtime treats it as worn.
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <windows.h>
#include <OVR_CAPI.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const char *origin_name(ovrTrackingOrigin o) {
    switch (o) {
        case ovrTrackingOrigin_EyeLevel:   return "EyeLevel";
        case ovrTrackingOrigin_FloorLevel: return "FloorLevel";
        default:                           return "Unknown";
    }
}

int main(int argc, char *argv[]) {
    FILE *out = stdout;
    if (argc > 1) {
        out = fopen(argv[1], "w");
        if (!out) { perror(argv[1]); return 1; }
    }

    ovrResult r = ovr_Initialize(NULL);
    if (OVR_FAILURE(r)) {
        fprintf(stderr, "ovr_Initialize failed: %d\n", (int)r);
        return 1;
    }

    ovrSession  ses;
    ovrGraphicsLuid luid;
    r = ovr_Create(&ses, &luid);
    if (OVR_FAILURE(r)) {
        fprintf(stderr, "ovr_Create failed: %d\n", (int)r);
        ovr_Shutdown();
        return 1;
    }

    /* Session origin defaults to EyeLevel regardless of the user's floor
     * setup; switch to FloorLevel so tracker heights are above-floor values
     * comparable with physically measured mounting heights. */
    if (OVR_FAILURE(ovr_SetTrackingOriginType(ses, ovrTrackingOrigin_FloorLevel)))
        fprintf(stderr, "WARNING: could not set FloorLevel origin\n");

    ovrHmdDesc       hmd    = ovr_GetHmdDesc(ses);
    unsigned int     ntk    = ovr_GetTrackerCount(ses);
    ovrTrackingOrigin orig  = ovr_GetTrackingOriginType(ses);

    /* Wait until every connected tracker has a solved pose (see header). */
    double wait_s = (argc > 2) ? atof(argv[2]) : 30.0;
    int all_valid = 0;
    for (double waited = 0.0; waited < wait_s; waited += 0.25) {
        unsigned int valid = 0, connected = 0;
        for (unsigned int i = 0; i < ntk; i++) {
            ovrTrackerPose tp = ovr_GetTrackerPose(ses, i);
            if (tp.TrackerFlags & ovrTracker_Connected)   connected++;
            if (tp.TrackerFlags & ovrTracker_PoseTracked) valid++;
        }
        if (connected && valid == connected) { all_valid = 1; break; }
        /* Poll tracking state too — keeps the session marked active. */
        ovr_GetTrackingState(ses, ovr_GetTimeInSeconds(), ovrFalse);
        ovrSessionStatus ss;
        ovr_GetSessionStatus(ses, &ss);
        fprintf(stderr, "\rwaiting for tracker poses: %u/%u valid (%.0fs/%.0fs)"
                "  [present:%d mounted:%d visible:%d]   ",
                valid, connected, waited, wait_s,
                (int)ss.HmdPresent, (int)ss.HmdMounted, (int)ss.IsVisible);
        Sleep(250);
    }
    fprintf(stderr, "\n");
    if (!all_valid)
        fprintf(stderr, "WARNING: timed out — dumping anyway, but poses with "
                        "pose_valid=false are placeholders, NOT extrinsics.\n"
                        "Is the HMD in view of both sensors with the proximity "
                        "sensor covered?\n");

    fprintf(out, "{\n");
    fprintf(out, "  \"runtime_version\": \"%d.%d.%d\",\n",
            OVR_MAJOR_VERSION, OVR_MINOR_VERSION, OVR_PATCH_VERSION);
    fprintf(out, "  \"hmd_product\": \"%s\",\n",  hmd.ProductName);
    fprintf(out, "  \"hmd_manufacturer\": \"%s\",\n", hmd.Manufacturer);
    fprintf(out, "  \"hmd_serial\": \"%s\",\n",   hmd.SerialNumber);
    fprintf(out, "  \"hmd_firmware\": \"%d.%d\",\n",
            hmd.FirmwareMajor, hmd.FirmwareMinor);
    fprintf(out, "  \"hmd_type\": %d,\n",         (int)hmd.Type);
    fprintf(out, "  \"tracking_origin\": \"%s\",\n", origin_name(orig));
    fprintf(out, "  \"tracker_count\": %u,\n",    ntk);
    fprintf(out, "  \"trackers\": [\n");

    for (unsigned int i = 0; i < ntk; i++) {
        ovrTrackerPose tp = ovr_GetTrackerPose(ses, i);
        ovrTrackerDesc td = ovr_GetTrackerDesc(ses, i);

        double hfov_deg = td.FrustumHFovInRadians * 180.0 / M_PI;
        double vfov_deg = td.FrustumVFovInRadians * 180.0 / M_PI;

        fprintf(out, "    {\n");
        fprintf(out, "      \"index\": %u,\n", i);
        fprintf(out, "      \"connected\": %s,\n",
                (tp.TrackerFlags & ovrTracker_Connected)   ? "true" : "false");
        fprintf(out, "      \"pose_valid\": %s,\n",
                (tp.TrackerFlags & ovrTracker_PoseTracked) ? "true" : "false");

        /* Pose in the tracking-origin frame */
        fprintf(out, "      \"pose\": {\n");
        fprintf(out, "        \"position\":    [%.9f, %.9f, %.9f],\n",
                tp.Pose.Position.x, tp.Pose.Position.y, tp.Pose.Position.z);
        fprintf(out, "        \"orientation\": [%.9f, %.9f, %.9f, %.9f]\n",
                tp.Pose.Orientation.x, tp.Pose.Orientation.y,
                tp.Pose.Orientation.z, tp.Pose.Orientation.w);
        fprintf(out, "      },\n");

        /* LeveledPose: yaw removed, so floor-plane position + tilt only */
        fprintf(out, "      \"leveled_pose\": {\n");
        fprintf(out, "        \"position\":    [%.9f, %.9f, %.9f],\n",
                tp.LeveledPose.Position.x,
                tp.LeveledPose.Position.y,
                tp.LeveledPose.Position.z);
        fprintf(out, "        \"orientation\": [%.9f, %.9f, %.9f, %.9f]\n",
                tp.LeveledPose.Orientation.x, tp.LeveledPose.Orientation.y,
                tp.LeveledPose.Orientation.z, tp.LeveledPose.Orientation.w);
        fprintf(out, "      },\n");

        /* Frustum */
        fprintf(out, "      \"frustum_hfov_deg\": %.4f,\n", hfov_deg);
        fprintf(out, "      \"frustum_vfov_deg\": %.4f,\n", vfov_deg);
        fprintf(out, "      \"frustum_near_m\":   %.4f,\n", td.FrustumNearZInMeters);
        fprintf(out, "      \"frustum_far_m\":    %.4f\n",  td.FrustumFarZInMeters);
        fprintf(out, "    }%s\n", i + 1 < ntk ? "," : "");
    }

    fprintf(out, "  ]\n");
    fprintf(out, "}\n");

    if (out != stdout) fclose(out);
    ovr_Destroy(ses);
    ovr_Shutdown();
    return 0;
}
