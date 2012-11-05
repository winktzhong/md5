#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include "md5model.h"
#include "md5anim.h"
#include <GL/glew.h>
#include <allegro5/allegro.h>
#include <allegro5/allegro_opengl.h>

struct game {
	ALLEGRO_DISPLAY *display;
	ALLEGRO_TIMER *tick;
	ALLEGRO_EVENT_QUEUE *q;
};
#define FPS 60.0

static struct game G;
static struct md5model _model;
static struct md5anim _anim;

void md5mkmesh(struct md5mesh *mesh, struct md5joint *skel);

static void opengl_dump(void)
{
	fprintf(stderr, "------------\n"
			"GL_VENDOR:\t%s\n"
			"GL_RENDERER:\t%s\n"
			"GL_VERSION:\t%s\n"
			"GL_SL_VERSION:\t%s\n"
			"------------\n",
			glGetString(GL_VENDOR),
			glGetString(GL_RENDERER),
			glGetString(GL_VERSION),
			glGetString(GL_SHADING_LANGUAGE_VERSION));
}

void game_init(unsigned int w, unsigned int h)
{
	assert(glewInit());
	assert(al_init());
	assert((G.q = al_create_event_queue()));
	assert(G.tick = al_create_timer(1.0 / FPS));
	assert(al_install_mouse());
	assert(al_install_keyboard());

	al_set_new_display_flags(ALLEGRO_OPENGL);

	assert(G.display = al_create_display(w, h));

	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluPerspective (45.0, w/(GLdouble)h, 0.1, 1000.0);
	glTranslatef (0.0f, -25.0f, -150.0f);
	glRotatef (-90.0f, 1.0, 0.0, 0.0);

	opengl_dump();

	al_register_event_source(G.q, al_get_mouse_event_source());
	al_register_event_source(G.q, al_get_keyboard_event_source());
	al_register_event_source(G.q, al_get_display_event_source(G.display));
	al_register_event_source(G.q, al_get_timer_event_source(G.tick));

	al_start_timer(G.tick);
}

void drawskel(const struct md5joint *skeleton,
		const struct md5jinfo *jinfo,
		int num_joints) {
	int i;

	/* Draw each joint */
	glPointSize (5.0f);
	glColor3f (1.0f, 0.0f, 0.0f);
	glBegin (GL_POINTS);
	for (i = 0; i < num_joints; ++i)
		glVertex3fv ((float*)&skeleton[i].pos);
	glEnd ();
	glPointSize (1.0f);

	/* Draw each bone */
	glColor3f (0.0f, 1.0f, 0.0f);
	glBegin (GL_LINES);
	for (i = 0; i < num_joints; ++i)
	{
		if (jinfo[i].parent != -1)
		{
			glVertex3fv ((float*)&skeleton[jinfo[i].parent].pos);
			glVertex3fv ((float*)&skeleton[i].pos);
		}
	}
	glEnd();
}

void game_loop(void)
{
	int i, j;
	uint8_t isdone=0, redraw=1, frame=0;

	while(!isdone) {
		ALLEGRO_EVENT ev;
		al_wait_for_event(G.q,&ev);

		if(ev.type==ALLEGRO_EVENT_KEY_DOWN)
			if(ev.keyboard.keycode==ALLEGRO_KEY_ESCAPE)
				isdone = 1;
		if(ev.type==ALLEGRO_EVENT_TIMER)
			redraw = 1;
		if (redraw){
			glRotatef(0.2, 0, 0, 1);
			glClear(GL_COLOR_BUFFER_BIT);

			if (++frame >= _anim.num.frames) frame=0;
			printf("frame %d of %d\n", frame, _anim.num.frames);

			drawskel(_anim.joints[frame], _model.jinfo, _model.num.joints);
			md5mkmesh(&_model.meshes[0], _anim.joints[frame]);
			glColor3f (1.0f, 1.0f, 1.0f);

			glBegin(GL_LINE_STRIP);
			for(i=0; i<_model.meshes[0].num.tris; i++) {
				for(j=0; j<3; j++) {
					int tri = _model.meshes[0].tris[i].idx[j];
					struct md5vertex *verts = &_model.meshes[0].verts[tri];
					glVertex3fv((float *)&verts->pos);
				}
			}
			glEnd();

			al_flip_display();
		}
	}
}

void game_end(void)
{
	al_destroy_display(G.display);
}

void md5mkmesh(struct md5mesh *mesh, struct md5joint *skel)
{
	int j, k;

	for (j=0; j<mesh->num.verts; j++) {
		v3_make(&mesh->verts[j].pos, 0, 0, 0);
		const struct md5vertex *vertex = &mesh->verts[j];
		for (k=vertex->start; k<vertex->start + vertex->count; k++) {
			v3_t wv;
			const struct md5weight *weight = &mesh->weights[k];
			const struct md5joint *joint = &skel[weight->joint];

			quat_rotatep(&wv, &joint->ori, &weight->pos);
			mesh->verts[j].pos.x += (joint->pos.x + wv.x) * weight->bias;
			mesh->verts[j].pos.y += (joint->pos.y + wv.y) * weight->bias;
			mesh->verts[j].pos.z += (joint->pos.z + wv.z) * weight->bias;
		}
	}
}

int main(int argc, char *argv[]) {
	int err;

	assert(argc > 1);

	err = md5model_load(argv[1], &_model);
	if (err) printf("md5model: %d\n", err);

	err = md5anim_load(argv[2], &_anim, &_model);
	if (err) printf("md5anim: %d\n", err);
	md5mkmesh(&_model.meshes[0], _model.base);

	game_init(800, 600);
	game_loop();
	game_end();

	md5anim_end(&_anim);
	return 0;
}
