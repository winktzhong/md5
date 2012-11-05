#ifndef V3_H
#define V3_H

#include <math.h>
#include "geodefs.h"

typedef struct {
	fp_t x, y, z;
} v3_t;

v3_t *v3_make(v3_t *u, fp_t x, fp_t y, fp_t z);
v3_t *v3_zero(v3_t *u);

#define v3_getx(_u) ((_u)->x)
#define v3_gety(_u) ((_u)->y)
#define v3_getz(_u) ((_u)->z)

v3_t *v3_minus(v3_t *o, const v3_t *u);
v3_t *v3_normalize(v3_t *r, const v3_t *u);
v3_t *v3_scale(v3_t *o, const v3_t *u, fp_t k);

v3_t *v3_add(v3_t *o, const v3_t *u, const v3_t *v);
v3_t *v3_sub(v3_t *o, const v3_t *u, const v3_t *v);
v3_t *v3_cross(v3_t *r, const v3_t *u, const v3_t *v);
v3_t *v3_project(v3_t *r, const v3_t *u, const v3_t *v);

fp_t  v3_dot(const v3_t *u, const v3_t *v);
fp_t  v3_norm(const v3_t *u);
fp_t  v3_sproject(v3_t *r, const v3_t *u, const v3_t *v);

#endif /* V3_H */
