/***********************
 * ParticleSystem class
 ***********************/

/**
 * The particle system class simply "manages" a collection of particles.
 * Its primary responsibility is to run the simulation, evolving particles
 * over time according to the applied forces using Euler's method.
 * This header file contains the functions that you are required to implement.
 * (i.e. the rest of the code relies on this interface)
 * In addition, there are a few suggested state variables included.
 * You should add to this class (and probably create new classes to model
 * particles and forces) to build your system.
 */

#ifndef __PARTICLE_SYSTEM_H__
#define __PARTICLE_SYSTEM_H__

#include <FL/gl.h>
#include "vec.h"
#include "particle.h"
#include "force.h"
#include "camera.h"
#include <vector>
#include <map>

class ParticleSystem {

public:

	/** Constructor **/
	ParticleSystem();


	/** Destructor **/
	virtual ~ParticleSystem();

	/** Simulation fxns **/
	// This fxn should render all particles in the system,
	// at current time t.
	virtual void drawParticles(float t, Camera* camera);

	// This fxn should save the configuration of all particles
	// at current time t.
	virtual void bakeParticles(float t);

	// This function should compute forces acting on all particles
	// and update their state (pos and vel) appropriately.
	virtual void computeForcesAndUpdateParticles(float t);

	// This function should reset the system to its initial state.
	// When you need to reset your simulation, PLEASE USE THIS FXN.
	// It sets some state variables that the UI requires to properly
	// update the display.  Ditto for the following two functions.
	virtual void resetSimulation(float t);

	// This function should start the simulation
	virtual void startSimulation(float t);

	// This function should stop the simulation
	virtual void stopSimulation(float t);

	// This function should clear out your data structure
	// of baked particles (without leaking memory).
	virtual void clearBaked();	

	// This function load bake from data structure
	// return true if found, false otherwise
	virtual bool loadBaked(float t);

	// These accessor fxns are implemented for you
	float getBakeStartTime() { return bake_start_time; }
	float getBakeEndTime() { return bake_end_time; }
	float getBakeFps() { return bake_fps; }
	bool isSimulate() { return simulate; }
	bool isDirty() { return dirty; }
	void setDirty(bool d) { dirty = d; }

	void setFps(int fps) { bake_fps = fps; }
	void setMatrix(GLfloat m[]) {
		for (int i = 0; i<16; i++) { matrix[i] = m[i]; }
	}
	void setParticleStart(Vec3f pos, Vec3f vel) {
		if (simulate) {
			init_position = pos;
			init_velocity = vel;
		}
	}
	void addFieldForce(Force f) {
		fieldForce.push_back(f);
	}

	void rkProc(Particle& p, std::vector<Force>& f, float dt);

protected:
	GLfloat matrix[16];
	int number;
	float last_time;
	std::vector<Particle> particles;
	std::map<int, std::vector<Particle>> storeBake;
	std::vector<Force> fieldForce;
	int max_bake;
	Vec3f init_position;
	Vec3f init_velocity;

	/** Some baking-related state **/
	float bake_fps;						// frame rate at which simulation was baked
	float bake_start_time;				// time at which baking started 
										// These 2 variables are used by the UI for
										// updating the grey indicator 
	float bake_end_time;				// time at which baking ended

	/** General state variables **/
	bool simulate;						// flag for simulation mode
	bool dirty;							// flag for updating ui (don't worry about this)

};


#endif	// __PARTICLE_SYSTEM_H__
