#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <assert.h>
#include "geometry.h"
#include "md5.h"

static int md5parse(FILE *in, struct md5model *md5);
void md5print(struct md5model *m);

static void md5lex_eatcomment(FILE *in);
static void md5lex_eatsp(FILE *in);

static int md5lex_checktk(FILE *in, char *s);
static int md5lex_readint(FILE *in, int *v);
static int md5lex_readfloat(FILE *in, float *f);
static int md5lex_readstring(FILE *in, char **sp, size_t *n);

static int md5parse_joints(FILE *in, struct md5joint *md5, int joints);
static int md5parse_mesh(FILE *in, struct md5mesh *md5);

/* -------------------------------------------------------------------------- */

int md5read(const char *fname, struct md5model *md5) {
	int i;
	FILE *in;

	if (!(in = fopen(fname, "r"))) return -1;

	if (md5parse(in, md5)) return -2;

	for (i=0; i<md5->num.meshes; i++) {
		struct md5mesh *mesh = &md5->meshes[i];
		int j;
		assert(i < md5->num.meshes);

		for (j=0; j<mesh->num.verts; j++) {
			struct md5vertex *vertex = &mesh->verts[j];
			fp_t vpos[3] = {0, 0, 0};
			int k;

			assert(j < mesh->num.verts);

			for (k=vertex->start; k<vertex->start + vertex->count; k++) {
				struct md5weight *weight = &mesh->weights[k];
				struct md5joint *j = &md5->base[weight->joint];
				v4_t wv;

				quat_rotatep(&wv, &j->ori, v4_make_from_v3(&wv, &j->pos));
				vpos[0] += (v3_getx(&j->pos) * v4_getx(&wv)) * weight->bias;
				vpos[1] += (v3_gety(&j->pos) * v4_gety(&wv)) * weight->bias;
				vpos[2] += (v3_getz(&j->pos) * v4_getz(&wv)) * weight->bias;
				v3_make(&vertex->pos, vpos[0], vpos[1], vpos[2]);
			}
		}
	}
	return 0;
}

#define FN_FAIL(_err) {err=_err; goto done; }
int md5parse(FILE *in, struct md5model *md5) {
	int err=0;
	char *string = NULL;
	size_t stringsz;
	int i, ver;

	/* MD5Version <int> */
	if (!md5lex_checktk(in, "MD5Version")) FN_FAIL(2);
	if (!md5lex_readint(in, &ver) || ver != 10) FN_FAIL(3);

	/* commandline "bla bla bla..." */
	if (!md5lex_checktk(in, "commandline")) FN_FAIL(4);
	md5lex_readstring(in, &string, &stringsz);
	free(string); string = NULL; /* throw it away. */

	/* numJoints <int> */
	if (!md5lex_checktk(in, "numJoints")) FN_FAIL(5);
	md5lex_readint(in, &md5->num.joints);
	assert(md5->base = malloc(sizeof(struct md5joint) * md5->num.joints));

	/* numMeshes <int> */
	if (!md5lex_checktk(in, "numMeshes")) FN_FAIL(6);
	md5lex_readint(in, &md5->num.meshes);
	assert(md5->meshes = malloc(sizeof(struct md5mesh) * md5->num.meshes));

	/* joints */
	if (!md5lex_checktk(in, "joints")) FN_FAIL(7);
	if (!md5lex_checktk(in, "{")) FN_FAIL(8);
	if (md5parse_joints(in, md5->base, md5->num.joints)) FN_FAIL(9);
	if (!md5lex_checktk(in, "}")) FN_FAIL(10);

	/* meshs */
	for (i=0; i<md5->num.meshes; i++) {
		if (!md5lex_checktk(in, "mesh")) FN_FAIL(11);
		if (!md5lex_checktk(in, "{")) FN_FAIL(12);
		if (md5parse_mesh(in, md5->meshes)) FN_FAIL(13);
		if (!md5lex_checktk(in, "}")) FN_FAIL(14);
	}

done:
	fclose(in);
	return err;
}
#undef FN_ERR

/* -------------------------------------------------------------------------- */

#define FN_FAIL(_err) {err=_err; goto done; }
static int md5parse_joints(FILE *in, struct md5joint *joint, int joints) {
	int err=0, i;
	char *string=NULL;
	size_t stringsz;

	float pos[3], ori[3];

	/* "name" parent ( pos.x pos.y pos.z ) ( orient.x orient.y orient.z ) */
	for (i=0; i<joints; i++) {
		struct md5joint *ji = &joint[i];

		if (!md5lex_readstring(in, &string, &stringsz)) FN_FAIL(1);
		strncpy(ji->name, string, sizeof((struct md5joint *)0)->name);

		if (!md5lex_readint(in, &ji->parent)) FN_FAIL(2);

		md5lex_checktk(in, "(");
		if (!md5lex_readfloat(in, &pos[0])) FN_FAIL(3);
		if (!md5lex_readfloat(in, &pos[1])) FN_FAIL(3);
		if (!md5lex_readfloat(in, &pos[2])) FN_FAIL(3);
		v3_make(&ji->pos, pos[0], pos[1], pos[2]);
		md5lex_checktk(in, ")");

		md5lex_checktk(in, "(");
		if (!md5lex_readfloat(in, &ori[0])) FN_FAIL(4);
		if (!md5lex_readfloat(in, &ori[1])) FN_FAIL(4);
		if (!md5lex_readfloat(in, &ori[2])) FN_FAIL(4);
		quat_makew(&ji->ori, ori[0], ori[1], ori[2]);
		md5lex_checktk(in, ")");
	}
done:
	free(string);
	if (err) printf("joints err: %d\n", err);
	return err;
}
#undef FN_ERR

#define FN_FAIL(_err) {err=_err; goto done; }
static int md5parse_mesh(FILE *in, struct md5mesh *mesh) {
	int err=0;
	char *string=NULL;
	size_t stringsz;
	int i;

	/* shader "<string>" */
	if (!md5lex_checktk(in, "shader")) FN_FAIL(1);
	if (!md5lex_readstring(in, &string, &stringsz)) FN_FAIL(2);
	strncpy(mesh->shader, string, sizeof(((struct md5mesh *)0)->shader));

	/* numverts <int> */
	if (!md5lex_checktk(in, "numverts")) FN_FAIL(3);
	if (!md5lex_readint(in, &mesh->num.verts)) FN_FAIL(4);
	assert(mesh->verts = malloc(sizeof (struct md5vertex)* mesh->num.verts));

	/* vert, s, t, start_w, count_w) */
	for (i=0; i<mesh->num.verts; i++) { 
		int vert;
		float s, t;

		if (!md5lex_checktk(in, "vert")) FN_FAIL(5);
		if (!md5lex_readint(in, &vert)) FN_FAIL(6);
		assert(vert < mesh->num.verts);

		if (!md5lex_checktk(in, "(")) FN_FAIL(7);
		if (!md5lex_readfloat(in, &s)) FN_FAIL(8);
		if (!md5lex_readfloat(in, &t)) FN_FAIL(8);
		v2_make(&mesh->verts[vert].st, s, t);
		if (!md5lex_checktk(in, ")")) FN_FAIL(9);

		if (!md5lex_readint(in, &mesh->verts[vert].start)) FN_FAIL(10);
		if (!md5lex_readint(in, &mesh->verts[vert].count)) FN_FAIL(10);
	}

	/* numtris <int> */
	if (!md5lex_checktk(in, "numtris")) FN_FAIL(11);
	if (!md5lex_readint(in, &mesh->num.tris)) FN_FAIL(12);
	assert(mesh->tris = malloc(sizeof(struct md5tri)* mesh->num.tris));

	/* tri triIndex vertIndex[0] vertIndex[1] vertIndex[2] */
	for (i=0; i<mesh->num.tris; i++) {
		int tid;
		if (!md5lex_checktk(in, "tri")) FN_FAIL(13);
		if (!md5lex_readint(in, &tid)) FN_FAIL(14);

		if (!md5lex_readint(in, &mesh->tris[tid].idx[0])) FN_FAIL(15);
		if (!md5lex_readint(in, &mesh->tris[tid].idx[1])) FN_FAIL(15);
		if (!md5lex_readint(in, &mesh->tris[tid].idx[2])) FN_FAIL(15);
	}

	/* numweights <int> */
	if (!md5lex_checktk(in, "numweights")) FN_FAIL(16);
	if (!md5lex_readint(in, &mesh->num.weights)) FN_FAIL(17);
	assert(mesh->weights = malloc(sizeof(struct md5weight)* mesh->num.weights));

	/* weight weightIndex joint bias ( pos.x pos.y pos.z ) */
	for (i=0; i<mesh->num.weights; i++) {
		int wid;
		float pos[3];

		if (!md5lex_checktk(in, "weight")) FN_FAIL(18);
		if (!md5lex_readint(in, &wid)) FN_FAIL(19);

		if (!md5lex_readint(in, &mesh->weights[wid].joint)) FN_FAIL(20);
		if (!md5lex_readfloat(in, &mesh->weights[wid].bias)) FN_FAIL(22);

		if (!md5lex_checktk(in, "(")) FN_FAIL(21);
		if (!md5lex_readfloat(in, &pos[0])) FN_FAIL(22);
		if (!md5lex_readfloat(in, &pos[1])) FN_FAIL(22);
		if (!md5lex_readfloat(in, &pos[2])) FN_FAIL(22);
		v3_make(&mesh->weights[wid].pos, pos[0], pos[1], pos[2]);
		if (!md5lex_checktk(in, ")")) FN_FAIL(23);
	}

done:
	free(string);
	if (err) printf("mesh err: %d\n", err);
	return err;
}
#undef FN_ERR

/* -------------------------------------------------------------------------- */

#define FFMT "%10.6f"
void md5print(struct md5model *m) {
	int i;

	/* joints */
	printf("joints: %d\n", m->num.joints);
	for (i=0; i<m->num.joints; i++) {
		struct md5joint *j = &m->base[i];
		printf("%-20s %3d ("FFMT","FFMT","FFMT") ("FFMT","FFMT","FFMT")\n",
				j->name, j->parent,
				j->pos.x, j->pos.y, j->pos.z,
				j->ori.x, j->ori.y, j->ori.z);
	}
}

static void md5lex_eatcomment(FILE *in) {
	int c;

tryagain:
	md5lex_eatsp(in);

	if ((c = fgetc(in)) == '/') {
		if ((c = fgetc(in)) == '/') {
			while ((c = fgetc(in)) && c != '\n'); /* comment end. */
			goto tryagain;
		} else ungetc(c, in);
	} else ungetc(c, in);
}

static void md5lex_eatsp(FILE *in) {
	int c;

	while ((c = fgetc(in)) && isspace(c));
	ungetc(c, in);
}

static int md5lex_checktk(FILE *in, char *s) {
	int c;

	md5lex_eatcomment(in);
	while (*s) {
		c = fgetc(in);
		if (c == EOF) return 1;
		if (c != *s++) return 0; /* match failed. */
	}
	return 1;
}

static int md5lex_readstring(FILE *in, char **sp, size_t *n) {
	int fail=1, c;
	size_t i=0, sz=8;
	char *s = *sp;

	assert(sp);
	if (!*sp) s = *sp = malloc(sz);
	else sz = *n;

	md5lex_eatcomment(in);
	if ((c = fgetc(in)) != '"') { /* not a string! */
		ungetc(c, in);
	} else while ((c = fgetc(in)) != EOF) {
		if (i == sz) s = *sp = realloc(*sp, sz *= 2);
		if (c == '"') { fail=0; break; }
		s[i++] = c;
	}
	s[i] = '\0';

	*n = sz;
	return !fail;
}

static int md5lex_readint(FILE *in, int *v) {
	md5lex_eatcomment(in);
	return fscanf(in, " %d", v);
}

static int md5lex_readfloat(FILE *in, float *f) {
	md5lex_eatcomment(in);
	return fscanf(in, " %f", f);
}
