/* Drive the REAL rift_cam_calib_add() with real capture data.
 *
 * The unit tests use synthetic geometry with exact truth. This runs the same
 * C code over the 1024 co-observed exposures from
 * captures/lin/2026-07-29/live-joint.jsonl and must land on the same relative
 * pose the Python bootstrap does — otherwise the driver will not reproduce
 * the offline result on hardware no matter how good the maths is.
 *
 * Input: one line per exposure, 14 floats:
 *   ref(px py pz qx qy qz qw)  other(px py pz qx qy qz qw)   [obj->cam]
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

int main(int argc, char **argv)
{
	FILE *f;
	rift_cam_calib c;
	posef ref, other, got;
	int n = 0, used = 0, settled_at = -1;
	char line[4096];
	float dev_ang, dev_pos;

	if (argc < 2) {
		fprintf(stderr, "usage: %s pairs.txt [truth: px py pz qx qy qz qw]\n", argv[0]);
		return 2;
	}
	f = fopen(argv[1], "r");
	if (f == NULL) {
		perror(argv[1]);
		return 2;
	}
	/* skip the comment header */
	if (fgets(line, sizeof(line), f) == NULL)
		return 2;

	rift_cam_calib_init(&c);
	while (read_pose(f, &ref) && read_pose(f, &other)) {
		n++;
		if (rift_cam_calib_add(&c, &ref, &other))
			used++;
		if (settled_at < 0 && c.settled)
			settled_at = n;
	}
	fclose(f);

	if (!rift_cam_calib_get(&c, &got)) {
		fprintf(stderr, "no estimate after %d exposures\n", n);
		return 1;
	}
	rift_cam_calib_dev(&c, &dev_ang, &dev_pos);

	printf("exposures fed          %d\n", n);
	printf("folded into the mean   %u  (%u rejected, %u history resets)\n",
		c.n, c.n_rejected, c.n_resets);
	printf("settled after          %d exposures\n", settled_at);
	printf("state                  %s\n",
		c.state == RIFT_CAM_CALIBRATED ? "CALIBRATED" :
		c.state == RIFT_CAM_ESTIMATED ? "ESTIMATED" : "UNCALIBRATED");
	printf("per-exposure scatter   %.4f deg / %.3f mm\n",
		dev_ang * 57.2957795f, dev_pos * 1000.0f);
	printf("cam1->cam0             pos %.6f %.6f %.6f  quat %.6f %.6f %.6f %.6f\n",
		got.pos.x, got.pos.y, got.pos.z,
		got.orient.x, got.orient.y, got.orient.z, got.orient.w);
	printf("baseline               %.1f mm\n",
		sqrtf(got.pos.x * got.pos.x + got.pos.y * got.pos.y +
		      got.pos.z * got.pos.z) * 1000.0f);

	if (argc >= 9) {
		posef truth;
		float ang, pos;
		truth.pos.x = (float) atof(argv[2]);
		truth.pos.y = (float) atof(argv[3]);
		truth.pos.z = (float) atof(argv[4]);
		truth.orient.x = (float) atof(argv[5]);
		truth.orient.y = (float) atof(argv[6]);
		truth.orient.z = (float) atof(argv[7]);
		truth.orient.w = (float) atof(argv[8]);
		rift_cam_calib_compare(&c, &truth, &ang, &pos);
		printf("\nvs the Python bootstrap on the SAME data:\n");
		printf("  disagreement         %.4f deg / %.3f mm\n",
			ang * 57.2957795f, pos * 1000.0f);
		printf("  rejects it?          %s\n",
			rift_cam_calib_rejects(&c, &truth) ? "YES - the two do not agree"
			                                   : "no");
		return (ang * 57.2957795f < 0.5f && pos < 0.010f) ? 0 : 1;
	}
	return 0;
}
