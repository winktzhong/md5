#ifndef QUAT_H
#define QUAT_H

#include <math.h>
#include "geodefs.h"
#include "v3.h"

typedef struct {
	fp_t x, y, z, w;
} quat_t;

quat_t *quat_make(quat_t *q, fp_t x, fp_t y, fp_t z);
quat_t *quat_fill(quat_t *r, fp_t x, fp_t y, fp_t z, fp_t w);
quat_t *quat_makev(quat_t *q, fp_t angle, const v3_t *u);
void    quat_calcw(quat_t *r);

#define quat_getx(_u) ((_u)->x)
#define quat_gety(_u) ((_u)->y)
#define quat_getz(_u) ((_u)->z)
#define quat_getw(_u) ((_u)->w)

fp_t    quat_normal(quat_t *q);
quat_t *quat_normalize(quat_t *r, const quat_t *q);
quat_t *quat_add(quat_t *r, const quat_t *qa, const quat_t *qb);
quat_t *quat_sub(quat_t *r, const quat_t *qa, const quat_t *qb);
quat_t *quat_mulq(quat_t *out,  const quat_t *qa, const quat_t *qb);
quat_t *quat_mulv(quat_t *out, const quat_t *q, const v3_t *v);
quat_t *quat_conjugate(quat_t *r, const quat_t *q);
v3_t   *quat_rotatep(v3_t *out, const quat_t *q, const v3_t *in);

#endif /* QUAT_H */
