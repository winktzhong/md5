#include "v3.h"

static v3_t *v3_fill(v3_t *u, fp_t x, fp_t y, fp_t z) {
	u->x = x;
	u->y = y;
	u->z = z;
	return u;
}

v3_t *v3_make(v3_t *u, fp_t x, fp_t y, fp_t z) {
	return v3_fill(u, x, y, z);
}

v3_t *v3_minus(v3_t *r, const v3_t *u) {
	r->x = - u->x;
	r->y = - u->y;
	r->z = - u->z;

	return r;
}

v3_t *v3_normalize(v3_t *r, const v3_t *q) {
	fp_t normal = v3_norm(q);

	r->x = q->x / normal;
	r->y = q->y / normal;
	r->z = q->z / normal;

	return r;
}

v3_t *v3_scale(v3_t *r, const v3_t *u, fp_t k) {
	r->x = u->x / k;
	r->y = u->y / k;
	r->z = u->z / k;

	return r;
}

v3_t *v3_add(v3_t *r, const v3_t *u, const v3_t *v) {
	r->x = u->x + v->x;
	r->y = u->y + v->y;
	r->z = u->z + v->z;

	return r;
}

v3_t *v3_sub(v3_t *r, const v3_t *u, const v3_t *v) {
	r->x = u->x - v->x;
	r->y = u->y - v->y;
	r->z = u->z - v->z;

	return r;
}

v3_t *v3_cross(v3_t *r, const v3_t *u, const v3_t *v) {
	v3_t tmp;

	tmp.x = u->y*v->z - u->z*v->y;
	tmp.y = u->x*v->z - u->z*v->x;
	tmp.z = u->x*v->y - u->y*v->x;
	*r = tmp;

	return r;
}

v3_t *v3_project(v3_t *r, const v3_t *u, const v3_t *v) {
	/* vector projection: r = (u.v/v.v)v */
	return v3_scale(r, v, v3_dot(u, v) / v3_dot(v, v));
}

fp_t v3_dot(const v3_t *u, const v3_t *v) {
	return u->x * v->x + u->y * v->y + u->z * v->z;
}

fp_t v3_norm(const v3_t *u) {
	return sqrt(v3_dot(u, u));
}
