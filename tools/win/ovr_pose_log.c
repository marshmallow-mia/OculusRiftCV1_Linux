/*
 * tools/win/ovr_pose_log.c
 *
 * 1 kHz pose logger — Tier-1 capture.
 * Writes two CSVs simultaneously:
 *   out_now.csv  — ovr_GetTrackingState(..., latencyMarker=FALSE, predictTime=now)
 *   out_pred.csv — same call but predictTime = ovr_GetPredictedDisplayTime(session, 0)
 * The delta between them sizes their forward-prediction window exactly.
 *
 * Build: same flags as ovr_static_dump.c
 *
 * Usage:
 *   ovr_pose_log.exe <out_now.csv> <out_pred.csv> [duration_s] [pred_ms]
 *   duration_s defaults to 60, pred_ms to 22 (~2 frames at 90 Hz).
 *
 * NOTE: ovr_GetPredictedDisplayTime only works for sessions that submit
 * frames, so we size prediction with an explicit horizon instead: the pred
 * CSV holds the pose predicted pred_ms ahead of now. Comparing it against
 * the now-pose logged pred_ms later measures the predictor's accuracy.
 *
 * CSV columns (both files):
 *   t_ovr,t_wall,
 *   hpx,hpy,hpz,hqx,hqy,hqz,hqw,
 *   hvx,hvy,hvz,hwx,hwy,hwz,
 *   hax,hay,haz,halx,haly,halz,
 *   h_status,
 *   c0px,c0py,c0pz,c0qx,c0qy,c0qz,c0qw,c0status,
 *   c1px,c1py,c1pz,c1qx,c1qy,c1qz,c1qw,c1status
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <windows.h>
#include <timeapi.h>   /* timeBeginPeriod — link winmm.lib */
#include <OVR_CAPI.h>

#define CSV_HEADER \
    "t_ovr,t_wall," \
    "hpx,hpy,hpz,hqx,hqy,hqz,hqw," \
    "hvx,hvy,hvz,hwx,hwy,hwz," \
    "hax,hay,haz,halx,haly,halz," \
    "h_status," \
    "c0px,c0py,c0pz,c0qx,c0qy,c0qz,c0qw,c0status," \
    "c1px,c1py,c1pz,c1qx,c1qy,c1qz,c1qw,c1status\n"

static double wall_time_s(void) {
    LARGE_INTEGER freq, cnt;
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&cnt);
    return (double)cnt.QuadPart / (double)freq.QuadPart;
}

static void write_row(FILE *f, double t_ovr, double t_wall,
                      const ovrTrackingState *ts) {
    const ovrPoseStatef *hp = &ts->HeadPose;
    fprintf(f,
        "%.6f,%.6f,"
        "%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,"
        "%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,"
        "%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,"
        "%u,",
        t_ovr, t_wall,
        hp->ThePose.Position.x,    hp->ThePose.Position.y,    hp->ThePose.Position.z,
        hp->ThePose.Orientation.x, hp->ThePose.Orientation.y,
        hp->ThePose.Orientation.z, hp->ThePose.Orientation.w,
        hp->LinearVelocity.x,      hp->LinearVelocity.y,      hp->LinearVelocity.z,
        hp->AngularVelocity.x,     hp->AngularVelocity.y,     hp->AngularVelocity.z,
        hp->LinearAcceleration.x,  hp->LinearAcceleration.y,  hp->LinearAcceleration.z,
        hp->AngularAcceleration.x, hp->AngularAcceleration.y, hp->AngularAcceleration.z,
        ts->StatusFlags);

    for (int h = 0; h < 2; h++) {
        const ovrPoseStatef *cp = &ts->HandPoses[h];
        fprintf(f,
            "%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%u%s",
            cp->ThePose.Position.x,    cp->ThePose.Position.y,    cp->ThePose.Position.z,
            cp->ThePose.Orientation.x, cp->ThePose.Orientation.y,
            cp->ThePose.Orientation.z, cp->ThePose.Orientation.w,
            ts->HandStatusFlags[h],
            h == 0 ? "," : "\n");
    }
}

int main(int argc, char *argv[]) {
    if (argc < 3) {
        fprintf(stderr, "Usage: %s <out_now.csv> <out_pred.csv> [duration_s]\n",
                argv[0]);
        return 1;
    }
    double duration = (argc >= 4) ? atof(argv[3]) : 60.0;
    double pred_s   = ((argc >= 5) ? atof(argv[4]) : 22.0) / 1000.0;

    FILE *f_now  = fopen(argv[1], "w");
    FILE *f_pred = fopen(argv[2], "w");
    if (!f_now || !f_pred) { perror("fopen"); return 1; }

    ovrResult r = ovr_Initialize(NULL);
    if (OVR_FAILURE(r)) {
        fprintf(stderr, "ovr_Initialize failed: %d\n", (int)r);
        return 1;
    }

    ovrSession ses; ovrGraphicsLuid luid;
    r = ovr_Create(&ses, &luid);
    if (OVR_FAILURE(r)) {
        fprintf(stderr, "ovr_Create failed: %d\n", (int)r);
        ovr_Shutdown();
        return 1;
    }

    fputs(CSV_HEADER, f_now);
    fputs(CSV_HEADER, f_pred);

    timeBeginPeriod(1); /* 1 ms scheduler granularity so Sleep(1) ~= 1 ms */
    double t0 = wall_time_s();

    fprintf(stderr, "Logging for %.0f s. Press Ctrl-C to stop early.\n", duration);

    while (1) {
        double t_wall = wall_time_s();
        if (t_wall - t0 >= duration) break;

        double t_ovr  = ovr_GetTimeInSeconds();
        double t_pred = t_ovr + pred_s;

        ovrTrackingState ts_now  = ovr_GetTrackingState(ses, t_ovr,  ovrFalse);
        ovrTrackingState ts_pred = ovr_GetTrackingState(ses, t_pred, ovrFalse);

        write_row(f_now,  t_ovr, t_wall, &ts_now);
        write_row(f_pred, t_pred, t_wall, &ts_pred);

        /* ~1 kHz */
        Sleep(1);
    }

    timeEndPeriod(1);
    fprintf(stderr, "Done. Rows written.\n");

    fclose(f_now);
    fclose(f_pred);
    ovr_Destroy(ses);
    ovr_Shutdown();
    return 0;
}
