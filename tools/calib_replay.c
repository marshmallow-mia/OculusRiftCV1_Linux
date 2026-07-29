/* Drive the REAL rift_cam_calib code with real capture data.
 *
 * The unit tests use synthetic geometry with exact truth. This runs the same C
 * the driver runs over co-observed exposures exported from a capture, so the
 * numbers in windows-vs-linux-tracking.md come from the driver's own
 * accumulator rather than a Python model of it.
 *
 * Input: one line per exposure, 14 floats:
 *   ref(px py pz qx qy qz qw)  other(px py pz qx qy qz qw)   [obj->cam]
 *
 * Usage:
 *   calib_replay pairs.txt                       fit, and report
 *   calib_replay pairs.txt --score px py pz qx qy qz qw
 *                                                residual (px) and
 *                                                camera-moved verdict of a
 *                                                calibration fitted elsewhere
 *                                                against THIS history — the
 *                                                held-out viewpoint test
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "rift-cam-calib.h"

static int read_pose(FILE *f, posef *p)
{
	double v[7];
	int i;
	for (i = 0; i < 7; i++) {
		if (fscanf(f, "%lf", v + i) != 1)
			return 0;
	}
	p->pos.x = (float) v[0];
	p->pos.y = (float) v[1];
	p->pos.z = (float) v[2];
	p->orient.x = (float) v[3];
	p->orient.y = (float) v[4];
	p->orient.z = (float) v[5];
	p->orient.w = (float) v[6];
	return 1;
}

static int load(const char *path, rift_cam_calib *c, int *out_n)
{
	FILE *f = fopen(path, "r");
	char line[4096];
	posef ref, other;
	int n = 0;

	if (f == NULL) {
		perror(path);
		return 0;
	}
	if (fgets(line, sizeof(line), f) == NULL) {   /* comment header */
		fclose(f);
		return 0;
	}
	rift_cam_calib_init(c);
	while (read_pose(f, &ref) && read_pose(f, &other)) {
		n++;
		rift_cam_calib_add(c, &ref, &other);
	}
	fclose(f);
	*out_n = n;
	return 1;
}

int main(int argc, char **argv)
{
	rift_cam_calib c;
	posef got;
	int n = 0;

	if (argc < 2) {
		fprintf(stderr, "usage: %s pairs.txt [--score px py pz qx qy qz qw]\n",
			argv[0]);
		return 2;
	}
	if (!load(argv[1], &c, &n))
		return 2;

	if (argc >= 10 && strcmp(argv[2], "--score") == 0) {
		posef cand;
		cand.pos.x = (float) atof(argv[3]);
		cand.pos.y = (float) atof(argv[4]);
		cand.pos.z = (float) atof(argv[5]);
		cand.orient.x = (float) atof(argv[6]);
		cand.orient.y = (float) atof(argv[7]);
		cand.orient.z = (float) atof(argv[8]);
		cand.orient.w = (float) atof(argv[9]);
		printf("%.4f %d\n", rift_cam_calib_residual_px(&c, &cand),
			rift_cam_calib_camera_moved(&c, &cand) ? 1 : 0);
		return 0;
	}

	if (!rift_cam_calib_get(&c, &got)) {
		fprintf(stderr, "no estimate after %d exposures\n", n);
		return 1;
	}

	printf("exposures fed          %d\n", n);
	printf("history                %u poses over %u viewpoint buckets\n",
		c.n_hist, c.bins_seen);
	printf("solves                 %u  (%u history resets)\n",
		c.n_solves, c.n_resets);
	printf("state                  %s%s\n",
		c.state == RIFT_CAM_CALIBRATED ? "CALIBRATED" :
		c.state == RIFT_CAM_ESTIMATED ? "ESTIMATED" : "UNCALIBRATED",
		c.settled ? " - SETTLED" : "");
	printf("residual               %.3f px\n", c.residual_px);
	printf("cam1->cam0             pos %.6f %.6f %.6f  quat %.6f %.6f %.6f %.6f\n",
		got.pos.x, got.pos.y, got.pos.z,
		got.orient.x, got.orient.y, got.orient.z, got.orient.w);
	printf("baseline               %.1f mm\n",
		sqrtf(got.pos.x * got.pos.x + got.pos.y * got.pos.y +
		      got.pos.z * got.pos.z) * 1000.0f);
	return 0;
}
