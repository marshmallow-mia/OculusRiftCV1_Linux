/* Drive the REAL fusion offline and measure what it does after motion stops.
 *
 * The complaint this exists to settle is "fast movement and the image still
 * takes a bit to stop moving; when not moving at all it feels correct" — a
 * tail, not latency. Four candidate causes have already been killed in the
 * headset, which is an expensive and low-resolution instrument. This runs the
 * driver's own rift-fusion-ovr.c and exponential-filter.c over a synthetic but
 * physically consistent motion profile, so overshoot and settling time come out
 * as numbers with no hardware and no wearer.
 *
 * Same idea as tools/calib_replay.c: link the real C, not a model of it.
 *
 * The IMU is synthesised to the fusion's own conventions (checked against
 * rift_fusion_ovr_imu_update): `accel` is SPECIFIC FORCE in the body frame, so
 * at rest it reads +g on world Y rotated into the body, and gyro is body-frame
 * angular velocity. Vision is fed at 60 Hz as the true pose. That means the
 * only thing under test is the estimator's response — the input is exact.
 *
 *   fusion_replay --profile turn|translate|still [options]
 *
 *     --no-euro           bypass the One Euro output filter
 *     --noise             add measured sensor noise (default: clean input)
 *     --csv FILE          write the full time series
 *
 * Gains are overridden through the driver's own env knobs, so attribution runs
 * against unmodified code:
 *   OHMD_RIFT_GAIN_POS / _VEL / _ACCEL   scale factors, 1.0 = shipping value
 *
 * Copyright 2026
 * SPDX-License-Identifier: BSL-1.0
 */
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "rift-fusion-ovr.h"
#include "exponential-filter.h"

#define IMU_HZ      500.0
#define VISION_HZ    60.0
#define GRAVITY       9.8

/* Settling bars: what counts as "stopped". */
#define SETTLE_POS_M   0.001
#define SETTLE_ROT_DEG 0.1

/* Deterministic noise, so a run is reproducible without carrying a seed file. */
static unsigned long rng_state = 0x2545F4914F6CDD1DUL;
static double urand(void)
{
	rng_state = rng_state * 6364136223846793005UL + 1442695040888963407UL;
	return (double)((rng_state >> 11) & 0xFFFFFFFFUL) / (double)0xFFFFFFFFUL;
}
static double nrand(double sigma)
{
	/* Box-Muller; only one of the pair is used, which is fine here. */
	double u1 = urand(), u2 = urand();
	if (u1 < 1e-12)
		u1 = 1e-12;
	return sigma * sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
}

/* A trapezoidal rate profile with cosine ramps: accelerate to V over Tr, hold
 * for Th, decelerate to rest over Tr. Analytic, so the acceleration handed to
 * the accelerometer is exactly consistent with the velocity handed to the
 * integrator - any disagreement in the output is the estimator's, not the
 * test's. */
typedef struct { double v, a; } rate;

static rate trapezoid(double t, double V, double Tr, double Th)
{
	rate r = { 0.0, 0.0 };

	if (t <= 0.0)
		return r;
	if (t < Tr) {
		r.v = V * (1.0 - cos(M_PI * t / Tr)) / 2.0;
		r.a = V * M_PI / (2.0 * Tr) * sin(M_PI * t / Tr);
	} else if (t < Tr + Th) {
		r.v = V;
	} else if (t < 2.0 * Tr + Th) {
		double u = t - (Tr + Th);
		r.v = V * (1.0 + cos(M_PI * u / Tr)) / 2.0;
		r.a = -V * M_PI / (2.0 * Tr) * sin(M_PI * u / Tr);
	}
	return r;
}

typedef struct {
	const char *name;
	double V, Tr, Th;   /* peak rate, ramp, hold */
	bool rotate;        /* yaw about world Y, else translate along world X */
} profile;

static const profile PROFILES[] = {
	/* 400 deg/s peak is the top of what the horizon telemetry actually
	 * recorded in the headset (481-676 deg/s peak, 76-101 mean). */
	{ "turn",      400.0 * M_PI / 180.0, 0.15, 0.30, true  },
	{ "translate", 1.20,                 0.20, 0.30, false },
	{ "still",     0.0,                  0.10, 0.10, false },
};

int main(int argc, char **argv)
{
	const char *want = "turn", *csv_path = NULL;
	bool use_euro = true, noisy = false;
	double vision_latency_ms = 30.0;
	double hold_override = -1.0;
	double vision_bias_mm = 0.0;
	double dropout_ms = 0.0;
	const profile *pr = NULL;
	int i;

	for (i = 1; i < argc; i++) {
		if (!strcmp(argv[i], "--profile") && i + 1 < argc)
			want = argv[++i];
		else if (!strcmp(argv[i], "--csv") && i + 1 < argc)
			csv_path = argv[++i];
		else if (!strcmp(argv[i], "--latency-ms") && i + 1 < argc)
			vision_latency_ms = atof(argv[++i]);
		else if (!strcmp(argv[i], "--hold") && i + 1 < argc)
			hold_override = atof(argv[++i]);
		else if (!strcmp(argv[i], "--vision-bias") && i + 1 < argc)
			vision_bias_mm = atof(argv[++i]);
		else if (!strcmp(argv[i], "--dropout-ms") && i + 1 < argc)
			dropout_ms = atof(argv[++i]);
		else if (!strcmp(argv[i], "--no-euro"))
			use_euro = false;
		else if (!strcmp(argv[i], "--noise"))
			noisy = true;
		else {
			fprintf(stderr, "unknown argument: %s\n", argv[i]);
			return 2;
		}
	}
	for (i = 0; i < (int)(sizeof(PROFILES) / sizeof(PROFILES[0])); i++) {
		if (!strcmp(PROFILES[i].name, want))
			pr = PROFILES + i;
	}
	if (pr == NULL) {
		fprintf(stderr, "unknown profile: %s (turn|translate|still)\n", want);
		return 2;
	}

	profile prof = *pr;
	if (hold_override >= 0.0)
		prof.Th = hold_override;
	pr = &prof;

	double motion_end = 2.0 * pr->Tr + pr->Th;
	double t_total = motion_end + 8.0;   /* long enough to see a 1.8 s tail */

	posef init;
	ovec3f_set(&init.pos, 0, 0, 0);
	oquatf_set(&init.orient, 0, 0, 0, 1);

	rift_fusion_ovr f;
	rift_fusion_ovr_init(&f, &init, RIFT_FUSION_OVR_MAX_SLOTS);

	/* Vision does not arrive at the instant of exposure: the frame has to be
	 * read out, blob-searched and solved. The tracker models that with delay
	 * slots - the fix is applied against the fusion state AS IT WAS at the
	 * exposure - so the harness has to do the same or it would be testing an
	 * estimator that gets impossibly fresh data. */
	struct {
		bool used;
		uint64_t exposure_ts, deliver_ts;
		posef pose;
		int slot;
	} pending[RIFT_FUSION_OVR_MAX_SLOTS];
	memset(pending, 0, sizeof(pending));
	uint64_t vision_latency_ns = (uint64_t)(vision_latency_ms * 1e6);

	exp_filter_pose euro;
	exp_filter_pose_init(&euro);

	FILE *csv = NULL;
	if (csv_path != NULL) {
		csv = fopen(csv_path, "w");
		if (csv == NULL) {
			perror(csv_path);
			return 1;
		}
		fprintf(csv, "t,true_x,true_yaw_deg,est_x,est_yaw_deg,"
			"pos_err_mm,rot_err_deg,accel_off_mag,lin_vel_mag\n");
	}

	/* Truth, integrated at the IMU rate from the same analytic rate profile
	 * that generates the sensor readings. */
	double true_x = 0.0, true_yaw = 0.0;
	double dt = 1.0 / IMU_HZ;
	uint64_t next_vision = 0;
	uint64_t vision_step = (uint64_t)(1e9 / VISION_HZ);

	/* Post-motion statistics */
	double overshoot_pos = 0.0, overshoot_rot = 0.0;
	double settle_pos = -1.0, settle_rot = -1.0;
	double peak_bias = 0.0, bias_at_end = 0.0;
	double max_rot_err_all = 0.0;
	int n_glitch = 0;
	double final_true_x = 0.0, final_true_yaw = 0.0;
	bool have_final = false;

	int n = (int)(t_total * IMU_HZ);
	for (i = 0; i <= n; i++) {
		double t = i * dt;
		uint64_t ts = (uint64_t)(t * 1e9);
		rate r = trapezoid(t, pr->V, pr->Tr, pr->Th);

		/* advance truth */
		if (pr->rotate)
			true_yaw += r.v * dt;
		else
			true_x += r.v * dt;

		if (!have_final && t >= motion_end) {
			final_true_x = true_x;
			final_true_yaw = true_yaw;
			have_final = true;
		}

		/* truth pose */
		posef truth;
		ovec3f_set(&truth.pos, (float)true_x, 0, 0);
		{
			vec3f axis = {{ 0, 1, 0 }};
			oquatf_init_axis(&truth.orient, &axis, (float)true_yaw);
		}

		/* Synthesise the IMU in the fusion's conventions: body-frame gyro,
		 * and body-frame SPECIFIC FORCE (world accel plus g, rotated in). */
		vec3f gyro = {{ 0, 0, 0 }}, accel;
		vec3f accel_world = {{ 0, (float)GRAVITY, 0 }};

		if (pr->rotate)
			gyro.y = (float)r.v;
		else
			accel_world.x = (float)r.a;

		{
			quatf inv = truth.orient;
			oquatf_inverse(&inv);
			oquatf_get_rotated(&inv, &accel_world, &accel);
		}
		if (noisy) {
			gyro.x += (float)nrand(0.002); gyro.y += (float)nrand(0.002);
			gyro.z += (float)nrand(0.002);
			accel.x += (float)nrand(0.02); accel.y += (float)nrand(0.02);
			accel.z += (float)nrand(0.02);
		}

		rift_fusion_ovr_imu_update(&f, ts, &gyro, &accel, NULL, false);

		/* Expose at 60 Hz: claim a delay slot now, deliver the solved pose
		 * one vision latency later. Feeding exact truth means any residual
		 * that shows up is the estimator's own, not measurement error. */
		/* Vision drops out under fast motion: blur costs blobs and the
		 * pose search needs 10 matched LEDs. Suppress it through the end
		 * of the movement, which is when it really fails, and let it
		 * resume once the head is still. */
		bool vision_blind = dropout_ms > 0.0 &&
			t > motion_end - dropout_ms / 1000.0 && t < motion_end;

		if (ts >= next_vision && !vision_blind) {
			int s;
			for (s = 0; s < RIFT_FUSION_OVR_MAX_SLOTS; s++) {
				if (!pending[s].used)
					break;
			}
			if (s < RIFT_FUSION_OVR_MAX_SLOTS) {
				posef vis = truth;
				/* Viewpoint-dependent per-camera bias: turning the head
				 * changes which camera dominates the solve, stepping the
				 * vision estimate by a few mm. The estimator then has to
				 * chase that step - with whatever time constant it has. */
				if (t >= motion_end)
					vis.pos.x += (float)(vision_bias_mm / 1000.0);
				if (noisy) {
					vis.pos.x += (float)nrand(0.0015);
					vis.pos.y += (float)nrand(0.0015);
					vis.pos.z += (float)nrand(0.0015);
				}
				rift_fusion_ovr_prepare_delay_slot(&f, ts, s);
				pending[s].used = true;
				pending[s].exposure_ts = ts;
				pending[s].deliver_ts = ts + vision_latency_ns;
				pending[s].pose = vis;
				pending[s].slot = s;
			}
			next_vision += vision_step;
		}

		/* Deliver any fix whose processing time has elapsed. */
		{
			int s;
			for (s = 0; s < RIFT_FUSION_OVR_MAX_SLOTS; s++) {
				if (!pending[s].used || ts < pending[s].deliver_ts)
					continue;
				rift_fusion_ovr_pose_update(&f, pending[s].exposure_ts,
					&pending[s].pose, pending[s].slot, 1.0f, false);
				rift_fusion_ovr_release_delay_slot(&f, pending[s].slot);
				pending[s].used = false;
			}
		}

		/* What the driver would report: fusion output, then the One Euro
		 * stage the tracker runs on it. */
		posef out;
		vec3f vel, acc, av;
		rift_fusion_ovr_get_pose_at(&f, ts, &out, &vel, &acc, &av, NULL, NULL);
		if (use_euro) {
			posef filtered;
			exp_filter_pose_run(&euro, ts, &out, &filtered);
			out = filtered;
		}

		/* Errors against truth */
		double pos_err = fabs((double)out.pos.x - true_x);
		double dot = fabs((double)out.orient.x * truth.orient.x +
		                  (double)out.orient.y * truth.orient.y +
		                  (double)out.orient.z * truth.orient.z +
		                  (double)out.orient.w * truth.orient.w);
		if (dot > 1.0)
			dot = 1.0;
		double rot_err = 2.0 * acos(dot) * 180.0 / M_PI;

		if (rot_err > max_rot_err_all)
			max_rot_err_all = rot_err;
		if (rot_err > 5.0)
			n_glitch++;

		double bias = ovec3f_get_length(&f.accel_offset);
		if (bias > peak_bias)
			peak_bias = bias;

		if (csv != NULL) {
			double est_yaw = 2.0 * atan2((double)out.orient.y,
			                             (double)out.orient.w) * 180.0 / M_PI;
			fprintf(csv, "%.4f,%.6f,%.4f,%.6f,%.4f,%.4f,%.4f,%.6f,%.4f\n",
				t, true_x, true_yaw * 180.0 / M_PI, out.pos.x, est_yaw,
				pos_err * 1000.0, rot_err, bias,
				ovec3f_get_length(&f.lin_vel));
		}

		/* After motion ends: how far past the final truth does it go, and
		 * how long until it stops moving? */
		if (t >= motion_end) {
			double past_pos = fabs((double)out.pos.x - final_true_x);
			double past_rot = rot_err;

			if (past_pos > overshoot_pos)
				overshoot_pos = past_pos;
			if (past_rot > overshoot_rot)
				overshoot_rot = past_rot;
			if (past_pos > SETTLE_POS_M)
				settle_pos = t - motion_end;
			if (past_rot > SETTLE_ROT_DEG)
				settle_rot = t - motion_end;
			bias_at_end = bias;
		}
	}
	if (csv != NULL)
		fclose(csv);

	printf("profile %-9s  motion ends at %.2f s, watched for %.1f s after\n",
		pr->name, motion_end, t_total - motion_end);
	printf("  one-euro %-8s noise %s\n",
		use_euro ? "ON" : "BYPASSED", noisy ? "ON" : "off");
	printf("  overshoot past final truth : %7.2f mm   %7.3f deg\n",
		overshoot_pos * 1000.0, overshoot_rot);
	printf("  settling  (<%.0f mm / <%.1f deg) : %s",
		SETTLE_POS_M * 1000.0, SETTLE_ROT_DEG,
		settle_pos < 0 ? "pos already inside" : "");
	if (settle_pos >= 0)
		printf("%7.0f ms", settle_pos * 1000.0);
	printf("   %s", settle_rot < 0 ? "rot already inside" : "");
	if (settle_rot >= 0)
		printf("%7.0f ms", settle_rot * 1000.0);
	printf("\n");
	printf("  accel bias state           : peak %.4f  at end %.4f m/s^2\n",
		peak_bias, bias_at_end);
	printf("  worst orientation error EVER: %.3f deg  (%d samples over 5 deg)\n",
		max_rot_err_all, n_glitch);
	printf("  final true yaw             : %.1f deg\n", final_true_yaw * 180.0 / M_PI);
	return 0;
}
