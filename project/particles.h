#pragma once

#include <GL/glew.h>
#include <vector>
#include <glm/glm.hpp>
#include <glm/detail/type_vec3.hpp>
#include <glm/mat4x4.hpp>
#include <stdlib.h>
//#include "volRender.h"

using namespace glm;

namespace particles {
	struct FluidCube { //Gonna start w/ a 2d grid
		int size;
		float dt;
		float diff;
		float visc;
		float absorbtion;
		float scattering;

		float* s;
		float* density;

		float* Vx;
		float* Vy;
		float* Vz;

		float* Vx0;
		float* Vy0;
		float* Vz0;

		vec3 position;
		fvec3 minBound;
		fvec3 maxBound;
	};
	typedef struct FluidCube FluidCube;

	FluidCube* fcCreate(int size, int diffusion, int viscosity, float absorbtion, float scattering, float dt);
	void fluidCubeStep(FluidCube* cube);
	void fcFree(FluidCube* cube);
	void set_bnd(int b, float* x, int N);
	void fcAddDensity(FluidCube* cube, int x, int y, int z, float amount);
	void fcAddVelocity(FluidCube* cube, int x, int y, int z, float amountX, float amountY, float amountZ);
	void fcClearDensity(FluidCube* cube);
	float sampleDensity(FluidCube* fc, vec3 sample_pos);
	void lin_solve(int b, float* x, float* x0, float a, float c, int iter, int N);
	void diffuse(int b, float* x, float* x0, float diff, float dt, int iter, int N);
	void advect(int b, float* d, float* d0, float* u, float* v, float* velocZ, float dt, int N);
	void project(float* u, float* v, float* velocZ, float* p, float* div, int iter, int N);
	//void dens_step(int N, float* x, float* x0, float* u, float* v, float diff, float dt);
	//void vel_step(int N, float* u, float* v, float* u0, float* v0, float visc, float dt); //Packs all helper functions into one
}