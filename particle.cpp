#include "particle.h"
#include "modelerdraw.h"
#include "force.h"


GLuint Particle::texID = 0;

Particle::Particle()
	:mass(1.0), position(Vec3f(0, 0, 0)), velocity(Vec3f(0, 0, 0)) {};

void Particle::draw(Camera* camera) {
	glPushMatrix();
	glTranslatef(position[0], position[1], position[2]);

	//Rotate to face camera
	Vec3f up = camera->getUp();
	Vec3f campos = camera->getPos();

	Vec3f d = position - campos;
	d.normalize();
	Vec3f right = d ^ up;

	GLfloat m[16];
	m[0] = right[0]; m[4] = up[0]; m[8] = -d[0]; m[12] = 0.0;
	m[1] = right[1]; m[5] = up[1]; m[9] = -d[1]; m[13] = 0.0;
	m[2] = right[2]; m[6] = up[2]; m[10] = -d[2]; m[14] = 0.0;
	m[3] = 0.0; m[7] = 0.0; m[11] = 0.0; m[15] = 1.0;

	glMultMatrixf(m);

	float c[] = { 0.01, 0.9, 0.01, 1.0 };

	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, c);

	glBindTexture(GL_TEXTURE_2D, texID);
	glBegin(GL_QUADS);
	glColor4d(0.0f, 1.0f, 1.0f, 1.0f);
	glTexCoord2f(0.0f, 0.0f); glVertex3f(-0.1f, -0.1f, 0.0f);
	glTexCoord2f(0.0f, 1.0f); glVertex3f(-0.1f, 0.1f, 0.0f);
	glTexCoord2f(1.0f, 1.0f); glVertex3f(0.1f, 0.1f, 0.0f);
	glTexCoord2f(1.0f, 0.0f); glVertex3f(0.1f, -0.1f, 0.0f);
	glEnd();
	glPopMatrix();
}

bool Particle::toofar() {
	return position.length() > 20;
}

void Particle::update(Vec3f v, Vec3f p)
{
	velocity = v;
	position = p;
}

void Particle::attachForce(Force f) {
	force.push_back(f);
}
