#include <math.h>
#include "geometry.h"

v2_t *v2_make(v2_t *u, fp_t x, fp_t y)
{
	u->x = x;
	u->y = y;
	return u;
}

/* -------------------------------------------------------------------------- */

v3_t *v3_make(v3_t *u, fp_t x, fp_t y, fp_t z)
{
	u->x = x;
	u->y = y;
	u->z = z;
	return u;
}

/* -------------------------------------------------------------------------- */

static void v4_fill(v4_t *u, fp_t x, fp_t y, fp_t z, fp_t w)
{
	u->x = x;
	u->y = y;
	u->z = z;
	u->w = w;
}

v4_t *v4_make(v4_t *u, fp_t x, fp_t y, fp_t z)
{
	v4_fill(u, x, y, z, 0);
	return u;
}

v4_t *v4_make_from_v2(v4_t *u, const v2_t *v, fp_t z)
{
	v4_fill(u, v->x, v->y, 0, 0);
	return u;
}

v4_t *v4_make_from_v3(v4_t *u, const v3_t *v)
{
	v4_fill(u, v->x, v->y, v->z, 0);
	return u;
}

v4_t *v4_zero(v4_t *u)
{
	v4_fill(u, 0, 0, 0, 0);
	return u;
}

v4_t *v4_minus(v4_t *r, const v4_t *u)
{
	r->x = - u->x;
	r->y = - u->y;
	r->z = - u->z;
	r->w =   u->w;

	return r;
}

v4_t *v4_normalize(v4_t *r, const v4_t *q)
{
	/* may get a nan or infinity. */
	fp_t normal = v4_norm(q);

	r->x = q->x / normal;
	r->y = q->y / normal;
	r->z = q->z / normal;
	r->w = q->w / normal;

	return r;
}

v4_t *v4_scale(v4_t *r, const v4_t *u, fp_t k)
{
	r->x = u->x / k;
	r->y = u->y / k;
	r->z = u->z / k;
	r->w = u->w / k;

	return r;
}

v4_t *v4_add(v4_t *r, const v4_t *u, const v4_t *v)
{
	r->x = u->x + v->x;
	r->y = u->y + v->y;
	r->z = u->z + v->z;
	r->w = u->w + v->w;

	return r;
}

v4_t *v4_sub(v4_t *r, const v4_t *u, const v4_t *v)
{
	r->x = u->x - v->x;
	r->y = u->y - v->y;
	r->z = u->z - v->z;
	r->w = u->w - v->w;

	return r;
}

v4_t *v4_cross(v4_t *r, const v4_t *u, const v4_t *v)
{
	v4_t tmp;

	tmp.x = u->y*v->z - u->z*v->y;
	tmp.y = u->x*v->z - u->z*v->x;
	tmp.z = u->x*v->y - u->y*v->x;
	tmp.w = 0;
	if (u->w || v->w) tmp.w = 1;
	*r = tmp;

	return r;
}

v4_t *v4_project(v4_t *r, const v4_t *u, const v4_t *v)
{
	/* vector projection: r = (u.v/v.v)v */
	return v4_scale(r, v, v4_dot(u, v) / v4_dot(v, v));
}

fp_t v4_dot(const v4_t *u, const v4_t *v)
{
	return u->x * v->x + u->y * v->y + u->z * v->z;
}

fp_t v4_norm(const v4_t *u)
{
	return sqrt(v4_dot(u, u));
}

/* -------------------------------------------------------------------------- */

v4_t *quat_make(v4_t *q, fp_t x, fp_t y, fp_t z)
{
	v4_fill(q, x, y, z, 1);
	return q;
}

v4_t *quat_makew(v4_t *q, fp_t x, fp_t y, fp_t z)
{
	fp_t w, t = 1 - (x * x) - (y * y) - (z * z);

	if (t<0) w = 0;
	else w = -sqrt(t);

	v4_fill(q, x, y, z, w);

	return q;
}

v4_t *quat_mulq(v4_t *r, const v4_t *qa, const v4_t *qb)
{
	v4_t tmp;

	tmp.w = qa->w*qb->w - qa->x*qb->x - qa->y*qb->y - qa->z*qb->z;
	tmp.x = qa->x*qb->w + qa->w*qb->x + qa->y*qb->z - qa->z*qb->y;
	tmp.y = qa->y*qb->w + qa->w*qb->y + qa->z*qb->x - qa->x*qb->z;
	tmp.z = qa->z*qb->w + qa->w*qb->z + qa->x*qb->y - qa->y*qb->x;
	*r = tmp;

	return r;
}

v4_t *quat_mulv(v4_t *r, const v4_t *q, const v4_t *v)
{
	v4_t tmp;

	tmp.w = - (q->x*v->x - q->y*v->y - q->z*q->z);
	tmp.x =   (q->w*v->x + q->y*v->z - q->z*v->y);
	tmp.y =   (q->w*v->y + q->z*v->x - q->x*v->z);
	tmp.z =   (q->w*v->z + q->x*v->y - q->y*v->x);
	*r = tmp;

	return r;
}

v4_t *quat_conjugate(v4_t *r, const v4_t *q)
{
	return v4_minus(r, q);
}

v4_t *quat_rotatep(v4_t *r, const v4_t *q, const v4_t *p)
{
	v4_t tmp;

	/* r = Q.P.Q* */
	quat_conjugate(&tmp, q);
	quat_mulv(&tmp, p, &tmp);
	quat_mulq(&tmp, q, &tmp);
	*r = tmp;

	return r;
}
