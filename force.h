#ifndef FORCE_H
#define FORCE_H
#include "vec.h"

class Particle;

class Force{
public:
	Force();
	Force(float x, float y, float z)
		:f(Vec3f(x, y, z)) {};
	Force(Vec3f fv)
		:f(fv) {};
	Vec3f f;

	virtual Vec3f getAcceleration(const Particle& p) const;
	virtual Vec3f getFeaturedForce() const;
};
#endif