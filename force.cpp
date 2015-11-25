#include "force.h"
#include "particle.h"

Vec3f Force::getAcceleration(const Particle& p) const{
	return f / p.mass;
}
