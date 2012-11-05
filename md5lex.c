#include <stdlib.h>
#include <assert.h>
#include <ctype.h>

#include "md5lex.h"

static void md5lex_eatsp(FILE *in);

void md5lex_eatcomment(FILE *in) {
	int c;

tryagain:
	md5lex_eatsp(in);

	if ((c = fgetc(in)) == '/') {
		if ((c = fgetc(in)) == '/') {
			while (((c = fgetc(in)) != EOF) && c != '\n'); /* comment end. */
			goto tryagain;
		} else ungetc(c, in);
	} else ungetc(c, in);
}

int md5lex_checktk(FILE *in, char *s) {
	int c;

	md5lex_eatcomment(in);
	while (*s) {
		c = fgetc(in);
		if (c == EOF) return 1;
		if (c != *s++) return 0; /* match failed. */
	}
	return 1;
}

int md5lex_readstring(FILE *in, char **sp, size_t *n) {
	int fail=1, c;
	size_t i=0, sz=8;
	char *s;

	assert(sp);
	if (!*sp) s = *sp = malloc(sz);
	else s = *sp, sz = *n;

	md5lex_eatcomment(in);
	if ((c = fgetc(in)) != '"') { /* not a string! */
		ungetc(c, in);
	} else while ((c = fgetc(in)) != EOF) {
		if (i == sz) s = *sp = realloc(*sp, sz *= 2);
		if (c == '"') { fail=0; break; }
		s[i++] = c;
	}
	s[i] = '\0';

	if (fail) {
		free(s);
		*sp = NULL;
	} else if (n) *n = sz;
	return !fail;
}

int md5lex_readint(FILE *in, int *v) {
	md5lex_eatcomment(in);
	return fscanf(in, " %d", v);
}

int md5lex_readfloat(FILE *in, float *f) {
	md5lex_eatcomment(in);
	return fscanf(in, " %f", f);
}

static void md5lex_eatsp(FILE *in) {
	int c;

	while ((c = fgetc(in)) && isspace(c));
	ungetc(c, in);
}
