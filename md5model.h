#ifndef MD5MODEL_H
#define MD5MODEL_H

#include <stdio.h>
#include <stdlib.h>
#include "quat.h"
#include "v3.h"
#include "v2.h"

#define MD5_MAX_SHADER_SZ (256)
#define MD5_MAX_NAME_SZ (64)

struct md5joint {
	v3_t pos;
	quat_t ori;
};

struct md5jinfo {
	char *name;
	int parent;
};

struct md5vertex {
	v3_t pos;
	v2_t st;
	int start;
	int count;
};

struct md5tri {
	int idx[3];
};

struct md5weight {
	v3_t pos;
	int joint;
	float bias;
};

struct md5mesh {
	struct { int verts, tris, weights; } num;
	struct md5vertex *verts;
	struct md5tri *tris;
	struct md5weight *weights;

	char *shader;
};

struct md5model {
	struct { int joints, meshes; } num;
	struct md5joint *base;
	struct md5jinfo *jinfo;
	struct md5mesh *meshes;
};

int md5model_load(const char *fname, struct md5model *md5);
int md5model_read(FILE *in, struct md5model *md5);
void md5model_mkmesh(struct md5mesh *mesh, struct md5joint *skel);

#endif /* MD5MODEL_H */
