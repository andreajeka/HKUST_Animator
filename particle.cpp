#include "particle.h"
#include "modelerdraw.h"
#include "force.h"


GLuint Particle::texID = 0;

Particle::Particle()
	:mass(1.0), position(Vec3f(0, 0, 0)), velocity(Vec3f(0, 0, 0)) {};

/* Handle Individual Billboarding */
void Particle::draw(Camera* camera) {
	glPushMatrix();
	glTranslatef(position[0], position[1], position[2]);
	
	// Get camera up vector and position and the billboard position
	Vec3f up = camera->getUp();
	Vec3f cameraPos = camera->getPos();
	Vec3f d = cameraPos - position;
	d.normalize();

	// get the right vector by calculating the cross product of the two vectors
	Vec3f right = d ^ up;
	
	// calculate the up vector for the billboard with another cross product
	// this is the final up vector for the billboard.
	up = d ^ right;
	

	//  The rules state that in order to transform the local coordinates 
	// into the global coordinates we must multiply by this vector matrix.
	GLfloat matrix[16];

	matrix[0] = right[0]; matrix[4] = up[0]; matrix[8] = -d[0]; matrix[12] = 0.0;
	matrix[1] = right[1]; matrix[5] = up[1]; matrix[9] = -d[1]; matrix[13] = 0.0;
	matrix[2] = right[2]; matrix[6] = up[2]; matrix[10] = -d[2]; matrix[14] = 0.0;
	matrix[3] = 0.0; matrix[7] = 0.0; matrix[11] = 0.0; matrix[15] = 1.0;

	glMultMatrixf(matrix);

	float c[] = { 0.01, 0.9, 0.01, 1.0 };

	// Render billboard
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
