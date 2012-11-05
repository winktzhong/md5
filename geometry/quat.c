#include <math.h>
#include "v3.h"
#include "quat.h"

quat_t *quat_fill(quat_t *r, fp_t x, fp_t y, fp_t z, fp_t w) {
	r->w = w;
	r->x = x;
	r->y = y;
	r->z = z;
	return r;
}

quat_t *quat_make(quat_t *r, fp_t x, fp_t y, fp_t z) {
	return quat_fill(r, x, y, z, 1);
}

void quat_calcw(quat_t *r) {
	fp_t t = 1.0 - (r->x*r->x) - (r->y*r->y) - (r->z*r->z);

	if (t<0) r->w = 0;
	else r->w = -sqrt(t);
}

quat_t *quat_makev(quat_t *r, fp_t angle, const v3_t *u) {
	v3_t v;
	fp_t sin_a = sin(0.5 * angle);
	fp_t cos_a = cos(0.5 * angle);

	v3_normalize(&v, u);
	r->w =       cos_a;
	r->x = v.x * sin_a;
	r->y = v.y * sin_a;
	r->z = v.z * sin_a;
	return r;
}

fp_t quat_normal(quat_t *q) {
	return sqrt(q->x*q->x + q->y*q->y + q->z*q->z + q->w*q->w);
}

quat_t *quat_normalize(quat_t *r, const quat_t *q) {
	fp_t mag = sqrt(q->x*q->x + q->y*q->y + q->z*q->z + q->w*q->w);

	r->w = q->w / mag;
	r->x = q->x / mag;
	r->y = q->y / mag;
	r->z = q->z / mag;
	return r;
}

quat_t *quat_mulq(quat_t *out,  const quat_t *qa, const quat_t *qb) {
	quat_t tmp;
	tmp.w = qa->w*qb->w - qa->x*qb->x - qa->y*qb->y - qa->z*qb->z;
	tmp.x = qa->x*qb->w + qa->w*qb->x + qa->y*qb->z - qa->z*qb->y;
	tmp.y = qa->y*qb->w + qa->w*qb->y + qa->z*qb->x - qa->x*qb->z;
	tmp.z = qa->z*qb->w + qa->w*qb->z + qa->x*qb->y - qa->y*qb->x;
	*out = tmp;
	return out;
}

quat_t *quat_mulv(quat_t *out, const quat_t *q, const v3_t *v) {
	quat_t tmp;
	tmp.w = - q->x*v->x - q->y*v->y - q->z*v->z;
	tmp.x =   q->w*v->x + q->y*v->z - q->z*v->y;
	tmp.y =   q->w*v->y + q->z*v->x - q->x*v->z;
	tmp.z =   q->w*v->z + q->x*v->y - q->y*v->x;
	*out = tmp;
	return out;
}

quat_t *quat_conjugate(quat_t *r, const quat_t *q) {
	r->w = q->w;
	r->x =-q->x;
	r->y =-q->y;
	r->z =-q->z;
	return r;
}

v3_t *quat_rotatep(v3_t *out, const quat_t *q, const v3_t *in) {
	quat_t tmp, inv;

	quat_mulq(&tmp, quat_mulv(&tmp, q, in), quat_conjugate(&inv, q));
	return v3_make(out, tmp.x, tmp.y, tmp.z);
}
