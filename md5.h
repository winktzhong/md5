#ifndef MD5_H
#define MD5_H

#include <stdio.h>
#include <stdlib.h>
#include "geometry.h"

struct md5joint {
	char name[64];
	int parent;

	v3_t pos;
	v4_t ori;
};

struct md5vertex {
	v2_t st;
	v3_t pos;	/* final position after calc with bias */
	int start;
	int count;
};

struct md5tri {
	int idx[3];
};

struct md5weight {
	int joint;
	float bias;
	v3_t pos;
};

struct md5bbox {
	v3_t min, max;
};

struct md5mesh {
	struct { int verts, tris, weights; } num;
	struct md5vertex *verts;
	struct md5tri *tris;
	struct md5weight *weights;

	char shader[256];
};

struct md5model {
	struct { int joints, meshes; } num;
	struct md5joint *base;
	struct md5mesh *meshes;

	struct md5bbox bbox;
};

int md5read(const char *fname, struct md5model *md5);
void md5print(struct md5model *m);

#endif /* MD5_H */
