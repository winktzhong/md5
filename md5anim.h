#ifndef MD5ANIM_H
#define MD5ANIM_H

#include <stdio.h>
#include "md5model.h"
#include "md5lex.h"
#include "quat.h"
#include "v3.h"

struct md5bbox {
	v3_t min, max;
};

struct md5anim {
	struct {int joints, frames; } num;
	struct md5joint **joints;
	struct md5bbox *bounds;
};

int
md5anim_load(const char *, struct md5anim *, struct md5model *);
int
md5anim_read(FILE *, struct md5anim *, struct md5model *);
void
md5anim_end(struct md5anim *);

#endif /* MD5ANIM_H */
