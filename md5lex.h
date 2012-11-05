#ifndef MD5LEX_H
#define MD5LEX_H

#include <stdio.h>

#define MD5_MIN(a, b) ((a) < (b) ? (a) : (b))
#define MD5_MAX(a, b) ((a) > (b) ? (a) : (b))

void md5lex_eatcomment(FILE *in);
int md5lex_checktk(FILE *in, char *s);
int md5lex_readint(FILE *in, int *v);
int md5lex_readfloat(FILE *in, float *f);
int md5lex_readstring(FILE *in, char **stringp, size_t *n);

#endif /* MD5LEX_H */
