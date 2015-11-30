#include "force.h"
#include "particle.h"

Vec3f Force::getAcceleration(const Particle& p) const{
	return f / p.mass;
}

Vec3f Force::getFeaturedForce() const{
	Vec3f something(50, 50, 50);
	return something;
}