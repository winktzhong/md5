#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "md5anim.h"

#define MD5_FLAG_POS_X (1<<0)
#define MD5_FLAG_POS_Y (1<<1)
#define MD5_FLAG_POS_Z (1<<2)
#define MD5_FLAG_ORI_X (1<<3)
#define MD5_FLAG_ORI_Y (1<<4)
#define MD5_FLAG_ORI_Z (1<<5)

/* -------------------------------------------------------------------------- */
/* construction step, internal usage only for both structs.                   */
/* -------------------------------------------------------------------------- */
struct md5hierarchy {
	char name[MD5_MAX_NAME_SZ];
	int parent, flags, start_index;
};

struct md5builder {
	struct { int frames, joints, animated_components; } num;
	int frame_rate;

	struct md5hierarchy *hierarchy;
	struct md5bbox *bounds;
	struct md5joint *base;
	float **framedata;
};

/* -------------------------------------------------------------------------- */

static int
md5parse_hierarchy(FILE *, struct md5hierarchy *, int);
static int
md5parse_bboxes(FILE *, struct md5bbox *, int);
static int
md5parse_baseframe(FILE *, struct md5joint *, int);
static int
md5parse_frames(FILE *, float **, int, int);
static void
md5anim_build_skeleton(struct md5builder *, float *, struct md5joint *);
static int
md5anim_validade_model(const struct md5builder *, const struct md5model *);

static void
md5builder_init(struct md5builder *);
static void
md5builder_end(struct md5builder *);

/* -------------------------------------------------------------------------- */

int md5anim_load(const char *fname,
		struct md5anim *anim,
		struct md5model *model) {
	int err=0;
	FILE *in;

	if (!(in = fopen(fname, "r"))) return -1;
	if ((err=md5anim_read(in, anim, model))) return err;
	fclose(in);

	return 0;
}

#define DONE(_err) { err=_err; goto done; }
int md5anim_read(FILE *in,
		struct md5anim *anim,
		struct md5model *model) {
	int i=-1, err=-1;
	char *cmdline=NULL;
	size_t cmdlinesz;

	struct md5builder build;
	md5builder_init(&build);

	if (!md5lex_checktk(in, "MD5Version")) DONE(1);
	if (!md5lex_readint(in, &i) || i != 10) DONE(2);

	if (!md5lex_checktk(in, "commandline")) DONE(3);
	md5lex_readstring(in, &cmdline, &cmdlinesz);
	free(cmdline); cmdline=NULL; /* throw it away. */

	if (!md5lex_checktk(in, "numFrames")) DONE(4);
	if (!md5lex_readint(in, &build.num.frames)) DONE(5);
	assert(build.bounds = malloc(sizeof(struct md5bbox) * build.num.frames));
	assert(build.framedata = malloc(sizeof(float *) * build.num.frames));

	if (!md5lex_checktk(in, "numJoints")) DONE(6);
	if (!md5lex_readint(in, &build.num.joints)) DONE(7);
	assert(build.hierarchy
			= malloc(sizeof(struct md5hierarchy) * build.num.joints));
	assert(build.base = malloc(sizeof(struct md5joint) * build.num.joints));

	if (!md5lex_checktk(in, "frameRate")) DONE(8);
	if (!md5lex_readint(in, &build.frame_rate)) DONE(9);

	if (!md5lex_checktk(in, "numAnimatedComponents")) DONE(10);
	if (!md5lex_readint(in, &build.num.animated_components)) DONE(11);
	for (i=0; i<build.num.frames; i++)
		assert(build.framedata[i]
			= malloc(sizeof(float) * build.num.animated_components));

	if (md5parse_hierarchy(in, build.hierarchy, build.num.joints)) DONE(12);
	if (md5parse_bboxes(in, build.bounds, build.num.frames)) DONE(13);
	if (md5parse_baseframe(in, build.base, build.num.joints)) DONE(14);
	if (md5parse_frames(in, build.framedata, build.num.frames,
				build.num.animated_components)) DONE(15);
	if (!md5anim_validade_model(&build, model)) DONE(16);

	assert(anim->joints = malloc(sizeof(struct md5joint *) * build.num.frames));
	for (i=0; i<build.num.frames; i++) {
		assert(anim->joints[i]
				= malloc(sizeof(struct md5joint) * build.num.joints));
		md5anim_build_skeleton(&build, build.framedata[i], anim->joints[i]);
	}
	anim->bounds = build.bounds; build.bounds = NULL;
	anim->num.joints = build.num.joints;
	anim->num.frames = build.num.frames;
done:
	md5builder_end(&build);
	return err;
}

void md5anim_end(struct md5anim *anim) {
	int i;

	for (i=0; i<anim->num.frames; i++)
		free(anim->joints[i]);
	free(anim->bounds);
}

/* -------------------------------------------------------------------------- */

static int md5anim_validade_model(const struct md5builder *build,
		const struct md5model *model) {
	int i;

	/* no info, assume it is correct. */
	if (!model->jinfo) return 1;

	if (build->num.joints != model->num.joints)
		return 0;
	for (i=0; i<build->num.joints; i++) {
		if (build->hierarchy[i].parent != model->jinfo[i].parent)
			return 0;
		if (strcmp(build->hierarchy[i].name, model->jinfo[i].name))
			return 0;
	}
	return 1;
}

static void md5anim_build_skeleton(struct md5builder *build,
		float *framedata,
		struct md5joint *out) {
	int joint;

	for (joint=0; joint<build->num.joints; joint++) {
		struct md5joint *base = &build->base[joint];

		int parent =      build->hierarchy[joint].parent;
		int start_index = build->hierarchy[joint].start_index;
		int flags =       build->hierarchy[joint].flags;
		int j=0;

		quat_t ori = base->ori;
		v3_t pos = base->pos;

		if (flags & MD5_FLAG_POS_X) pos.x = framedata[start_index + j++];
		if (flags & MD5_FLAG_POS_Y) pos.y = framedata[start_index + j++];
		if (flags & MD5_FLAG_POS_Z) pos.z = framedata[start_index + j++];

		if (flags & MD5_FLAG_ORI_X) ori.x = framedata[start_index + j++];
		if (flags & MD5_FLAG_ORI_Y) ori.y = framedata[start_index + j++];
		if (flags & MD5_FLAG_ORI_Z) ori.z = framedata[start_index + j++];

		quat_calcw(&ori);
		if (parent < 0) { /* root */
			out[joint].ori = ori;
			out[joint].pos = pos;
		} else {
			/* rotate pos acording to parent ori. */
			quat_rotatep(&pos, &out[parent].ori, &pos);
			v3_add(&out[joint].pos, &pos, &out[parent].pos);
			quat_mulq(&out[joint].ori, &out[parent].ori, &ori);
		}
	}
}

/* -------------------------------------------------------------------------- */
/* parsing */
/* -------------------------------------------------------------------------- */


static int md5parse_hierarchy(FILE *in, struct md5hierarchy *hierarchy,
		int count) {
	int i;
	char *name=NULL;
	size_t namesz;

	if (!md5lex_checktk(in, "hierarchy")) return 1;
	if (!md5lex_checktk(in, "{")) return 2;
	/* "name" parent<int> flags<int> startIndex<int> */
	for (i=0; i<count; i++) {
		struct md5hierarchy *hie = &hierarchy[i];

		if (!md5lex_readstring(in, &name, &namesz)) return 3;
		strncpy(hie->name, name, MD5_MIN(namesz, MD5_MAX_NAME_SZ));
		free(name); name=NULL; /* throw it away. */

		if (!md5lex_readint(in, &hie->parent)) return 4;
		if (!md5lex_readint(in, &hie->flags)) return 5;
		if (!md5lex_readint(in, &hie->start_index)) return 6;
	}
	if (!md5lex_checktk(in, "}")) return 7;
	return 0;
}

static int md5parse_bboxes(FILE *in, struct md5bbox *bounds, int count) {
	int i;

	if (!md5lex_checktk(in, "bounds")) return 1;
	if (!md5lex_checktk(in, "{")) return 2;
	for (i=0; i<count; i++) {
		struct md5bbox *box = &bounds[i];

		if (!md5lex_checktk(in, "(")) return 3;
		if (!md5lex_readfloat(in, &box->min.x)) return 4;
		if (!md5lex_readfloat(in, &box->min.y)) return 4;
		if (!md5lex_readfloat(in, &box->min.z)) return 4;
		if (!md5lex_checktk(in, ")")) return 5;

		if (!md5lex_checktk(in, "(")) return 6;
		if (!md5lex_readfloat(in, &box->max.x)) return 7;
		if (!md5lex_readfloat(in, &box->max.y)) return 7;
		if (!md5lex_readfloat(in, &box->max.z)) return 7;
		if (!md5lex_checktk(in, ")")) return 8;
	}
	if (!md5lex_checktk(in, "}")) return 9;
	return 0;
}

static int md5parse_baseframe(FILE *in, struct md5joint *bases, int count) {
	int i;

	if (!md5lex_checktk(in, "baseframe")) return 1;
	if (!md5lex_checktk(in, "{")) return 2;
	for (i=0; i<count; i++) {
		struct md5joint *base = &bases[i];

		if (!md5lex_checktk(in, "(")) return 3;
		if (!md5lex_readfloat(in, &base->pos.x)) return 4;
		if (!md5lex_readfloat(in, &base->pos.y)) return 4;
		if (!md5lex_readfloat(in, &base->pos.z)) return 4;
		if (!md5lex_checktk(in, ")")) return 5;

		if (!md5lex_checktk(in, "(")) return 6;
		if (!md5lex_readfloat(in, &base->ori.x)) return 7;
		if (!md5lex_readfloat(in, &base->ori.y)) return 7;
		if (!md5lex_readfloat(in, &base->ori.z)) return 7;
		quat_calcw(&base->ori);
		if (!md5lex_checktk(in, ")")) return 8;
	}
	if (!md5lex_checktk(in, "}")) return 9;
	return 0;
}

static int md5parse_frames(FILE *in, float **framedata,
		int count, int anim_comp) {
	int i, j;

	for (j=0; j<count; j++) {
		int fid;

		if (!md5lex_checktk(in, "frame")) return 1;
		if (!md5lex_readint(in, &fid)) return 2;
		if (!md5lex_checktk(in, "{")) return 3;

		for (i=0; i<anim_comp; i++) {
			if (!md5lex_readfloat(in, &framedata[j][i])) return 4;
		}
		if (!md5lex_checktk(in, "}")) return 5;
	}
	return 0;
}

/* -------------------------------------------------------------------------- */

static void md5builder_init(struct md5builder *build) {
	build->hierarchy = NULL;
	build->base = NULL;
	build->bounds = NULL;
	build->framedata = NULL;
}

static void md5builder_end(struct md5builder *build) {
	int i;

	/* free(NULL) is fine, a NOP by ANSI spec. */
	for (i=0; i<build->num.frames; i++)
		free(build->framedata[i]);
	free(build->hierarchy);
	free(build->bounds);
	free(build->framedata);
}
