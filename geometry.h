#ifndef V4_H
#define V4_H

typedef float fp_t;

typedef struct {
	fp_t x, y;
} v2_t;

typedef struct {
	fp_t x, y, z;
} v3_t;

typedef struct {
	fp_t x, y, z, w;
} v4_t;

/* -------------------------------------------------------------------------- */

v2_t *v2_make(v2_t *u, fp_t x, fp_t y);

#define v2_getx(_u) ((_u)->x)
#define v2_gety(_u) ((_u)->y)

/* -------------------------------------------------------------------------- */

v3_t *v3_make(v3_t *u, fp_t x, fp_t y, fp_t z);

#define v3_getx(_u) ((_u)->x)
#define v3_gety(_u) ((_u)->y)
#define v3_getz(_u) ((_u)->z)

/* -------------------------------------------------------------------------- */

v4_t *v4_make(v4_t *u, fp_t x, fp_t y, fp_t z);
v4_t *v4_make_from_v2(v4_t *u, const v2_t *v, fp_t z);
v4_t *v4_make_from_v3(v4_t *u, const v3_t *v);
v4_t *v4_zero(v4_t *u);

#define v4_getx(_u) ((_u)->x)
#define v4_gety(_u) ((_u)->y)
#define v4_getz(_u) ((_u)->z)
#define v4_getw(_u) ((_u)->w)

v4_t *v4_minus(v4_t *o, const v4_t *u);
v4_t *v4_normalize(v4_t *r, const v4_t *u);
v4_t *v4_scale(v4_t *o, const v4_t *u, fp_t k);

v4_t *v4_add(v4_t *o, const v4_t *u, const v4_t *v);
v4_t *v4_sub(v4_t *o, const v4_t *u, const v4_t *v);
v4_t *v4_cross(v4_t *r, const v4_t *u, const v4_t *v);
v4_t *v4_project(v4_t *r, const v4_t *u, const v4_t *v);

fp_t  v4_dot(const v4_t *u, const v4_t *v);
fp_t  v4_norm(const v4_t *u);
fp_t  v4_sproject(v4_t *r, const v4_t *u, const v4_t *v);

/* -------------------------------------------------------------------------- */

v4_t *quat_make(v4_t *q, fp_t x, fp_t y, fp_t z);
v4_t *quat_makew(v4_t *q, fp_t x, fp_t y, fp_t z);
fp_t  quat_normal(const v4_t *q);
v4_t *quat_normalize(v4_t *r, const v4_t *qa);
v4_t *quat_mulq(v4_t *r, const v4_t *qa, const v4_t *qb);
v4_t *quat_mulv(v4_t *r, const v4_t *qa, const v4_t *qb);
v4_t *quat_conjugate(v4_t *r, const v4_t *q);
v4_t *quat_rotatep(v4_t *r, const v4_t *q, const v4_t *p);

/* -------------------------------------------------------------------------- */

#endif /* V4_H */
