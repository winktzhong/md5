#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <assert.h>
#include "md5model.h"
#include "md5lex.h"

#define MD5MIN(a, b) ((a) < (b) ? (a) : (b))
#define MD5MAX(a, b) ((a) > (b) ? (a) : (b))

static int
parse_joints(FILE *, struct md5joint *, struct md5jinfo *, int);
static int
parse_meshes(FILE *, struct md5mesh *);
static int
parse_meshes_vertex(FILE *, struct md5vertex *, int);
static int
parse_meshes_tri(FILE *, struct md5tri *tris, int);
static int
parse_meshes_weight(FILE *, struct md5weight *, int);

/* -------------------------------------------------------------------------- */

int md5model_load(const char *fname, struct md5model *model) {
	FILE *in;

	if (!(in = fopen(fname, "r"))) return -1;
	if (md5model_read(in, model)) return -2;
	fclose(in);

	return 0;
}

int md5model_read(FILE *in, struct md5model *model) {
	char *cmdline=NULL;
	size_t cmdlinesz;
	int i, ver;

	/* MD5Version <int> */
	if (!md5lex_checktk(in, "MD5Version")) return 1;
	if (!md5lex_readint(in, &ver) || ver != 10) return 2;

	/* commandline "bla bla bla..." */
	if (!md5lex_checktk(in, "commandline")) return 3;
	md5lex_readstring(in, &cmdline, &cmdlinesz);
	free(cmdline); cmdline=NULL; /* throw it away. */

	/* numJoints <int> */
	if (!md5lex_checktk(in, "numJoints")) return 4;
	md5lex_readint(in, &model->num.joints);
	assert(model->base = malloc(sizeof(struct md5joint) * model->num.joints));
	assert(model->jinfo= malloc(sizeof(struct md5jinfo) * model->num.joints));

	/* numMeshes <int> */
	if (!md5lex_checktk(in, "numMeshes")) return 5;
	md5lex_readint(in, &model->num.meshes);
	assert(model->meshes = malloc(sizeof(struct md5mesh) * model->num.meshes));

	/* joints */
	if (!md5lex_checktk(in, "joints")) return 6;
	if (!md5lex_checktk(in, "{")) return 7;
	if (parse_joints(in, model->base, model->jinfo, model->num.joints))
		return 8;
	if (!md5lex_checktk(in, "}")) return 9;

	/* meshs */
	for (i=0; i<model->num.meshes; i++) {
		if (!md5lex_checktk(in, "mesh")) return 10;
		if (!md5lex_checktk(in, "{")) return 11;
		if (parse_meshes(in, &model->meshes[i])) return 12;
		if (!md5lex_checktk(in, "}")) return 13;
	}
	return 0;
}

void md5model_end(struct md5model *model) {
	int i;

	for(i=0; i<model->num.joints; i++) {

	}
	for(i=0; i<model->num.meshes; i++) {
		struct md5mesh *mesh = &model->meshes[i];
		free(mesh->verts);
		free(mesh->tris);
		free(mesh->weights);
	}
	free(model->meshes);
	free(model->base);
	free(model->jinfo);
}

/* -------------------------------------------------------------------------- */

static int parse_joints(FILE *in,
		struct md5joint *joint,
		struct md5jinfo *jinfo,
		int joints) {
	int i;

	/* "name" parent ( pos.x pos.y pos.z ) ( orient.x orient.y orient.z ) */
	for (i=0; i<joints; i++) {
		struct md5joint *jointi = &joint[i];
		struct md5jinfo *jinfoi = &jinfo[i];

		jinfoi->name = NULL;
		if (!md5lex_readstring(in, &jinfoi->name, NULL)) return 1;
		if (!md5lex_readint(in, &jinfoi->parent)) return 2;

		md5lex_checktk(in, "(");
		if (!md5lex_readfloat(in, &jointi->pos.x)) return 3;
		if (!md5lex_readfloat(in, &jointi->pos.y)) return 3;
		if (!md5lex_readfloat(in, &jointi->pos.z)) return 3;
		md5lex_checktk(in, ")");

		md5lex_checktk(in, "(");
		if (!md5lex_readfloat(in, &jointi->ori.x)) return 4;
		if (!md5lex_readfloat(in, &jointi->ori.y)) return 4;
		if (!md5lex_readfloat(in, &jointi->ori.z)) return 4;
		quat_calcw(&jointi->ori);
		md5lex_checktk(in, ")");
	}
	return 0;
}

void md5model_mkmesh(struct md5mesh *mesh, struct md5joint *skel)
{
	int j, k;

	for (j=0; j<mesh->num.verts; j++) {
		v3_make(&mesh->verts[j].pos, 0, 0, 0);
		const struct md5vertex *vertex = &mesh->verts[j];
		for (k=vertex->start; k<vertex->start + vertex->count; k++) {
			v3_t wv;
			const struct md5weight *weight = &mesh->weights[k];
			const struct md5joint *joint = &skel[weight->joint];

			quat_rotatep(&wv, &joint->ori, &weight->pos);
			mesh->verts[j].pos.x += (joint->pos.x + wv.x) * weight->bias;
			mesh->verts[j].pos.y += (joint->pos.y + wv.y) * weight->bias;
			mesh->verts[j].pos.z += (joint->pos.z + wv.z) * weight->bias;
		}
	}
}

static int parse_meshes(FILE *in, struct md5mesh *mesh) {

	/* shader "<string>" */
	if (!md5lex_checktk(in, "shader")) return 1;
	if (!md5lex_readstring(in, &mesh->shader, NULL)) return 2;

	/* numverts <int> */
	if (!md5lex_checktk(in, "numverts")) return 3;
	if (!md5lex_readint(in, &mesh->num.verts)) return 4;
	assert(mesh->verts = malloc(sizeof (struct md5vertex)* mesh->num.verts));
	if (parse_meshes_vertex(in, mesh->verts, mesh->num.verts)) return 5;

	/* numtris <int> */
	if (!md5lex_checktk(in, "numtris")) return 6;
	if (!md5lex_readint(in, &mesh->num.tris)) return 7;
	assert(mesh->tris = malloc(sizeof(struct md5tri)* mesh->num.tris));
	if (parse_meshes_tri(in, mesh->tris, mesh->num.tris)) return 8;

	/* numweights <int> */
	if (!md5lex_checktk(in, "numweights")) return 9;
	if (!md5lex_readint(in, &mesh->num.weights)) return 10;
	assert(mesh->weights = malloc(sizeof(struct md5weight)* mesh->num.weights));
	if (parse_meshes_weight(in, mesh->weights, mesh->num.weights)) return 11;

	return 0;
}

static int parse_meshes_vertex(FILE *in, struct md5vertex *verts, int count) {
	int i;

	/* vert, s, t, start_w, count_w) */
	for (i=0; i<count; i++) { 
		int vid;
		struct md5vertex *vert;

		if (!md5lex_checktk(in, "vert")) return 1;
		if (!md5lex_readint(in, &vid)) return 2;

		vert = &verts[vid];

		if (!md5lex_checktk(in, "(")) return 3;
		if (!md5lex_readfloat(in, &vert->st.x)) return 4;
		if (!md5lex_readfloat(in, &vert->st.y)) return 4;
		if (!md5lex_checktk(in, ")")) return 5;

		if (!md5lex_readint(in, &vert->start)) return 6;
		if (!md5lex_readint(in, &vert->count)) return 6;
	}
	return 0;
}

static int parse_meshes_tri(FILE *in, struct md5tri *tris, int count) {
	int i;

	/* tri triIndex vertIndex[0] vertIndex[1] vertIndex[2] */
	for (i=0; i<count; i++) {
		int tid;
		struct md5tri *tri;

		if (!md5lex_checktk(in, "tri")) return 1;
		if (!md5lex_readint(in, &tid)) return 2;
		tri = &tris[tid];

		if (!md5lex_readint(in, &tri->idx[0])) return 3;
		if (!md5lex_readint(in, &tri->idx[1])) return 3;
		if (!md5lex_readint(in, &tri->idx[2])) return 3;
	}
	return 0;
}

static int parse_meshes_weight(FILE *in, struct md5weight *weights, int count) {
	int i;
	/* weight weightIndex joint bias ( pos.x pos.y pos.z ) */
	for (i=0; i<count; i++) {
		int wid;
		struct md5weight *weight;

		if (!md5lex_checktk(in, "weight")) return 1;
		if (!md5lex_readint(in, &wid)) return 2;
		weight = &weights[wid];

		if (!md5lex_readint(in, &weight->joint)) return 3;
		if (!md5lex_readfloat(in, &weight->bias)) return 4;

		if (!md5lex_checktk(in, "(")) return 5;
		if (!md5lex_readfloat(in, &weight->pos.x)) return 6;
		if (!md5lex_readfloat(in, &weight->pos.y)) return 6;
		if (!md5lex_readfloat(in, &weight->pos.z)) return 6;
		if (!md5lex_checktk(in, ")")) return 7;
	}
	return 0;
}
