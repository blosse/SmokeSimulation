#include "particles.h"
#include <algorithm>
#include <GL/glew.h>
#include <glm/glm.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <labhelper.h>
#include <stdlib.h>

using namespace glm;
using namespace std;
using namespace labhelper;

/*
Most code is borrowed from either:
Real-Time Fluid Dynamics for Games by Jos Stam
or
https://mikeash.com/pyblog/fluid-simulation-for-dummies.html (Which is an implementation of the paper by Stam)
*/

#define XYZ(x,y,z) ((x) + (y) * N + (z) * N * N)
//int N;

namespace particles {

	FluidCube* fcCreate(int size, int diffusion, int viscosity, float absorbtion, float scattering, float dt)
	{
		FluidCube* cube = (FluidCube*) malloc(sizeof(*cube));

		int n = size * size * size;//(size + 2) * (size + 2); TODO Fixa minnesproblemet

		cube->size = size;
		cube->dt = dt;
		cube->diff = diffusion;
		cube->visc = viscosity;
		cube->absorbtion = absorbtion;
		cube->scattering = scattering;

		cube->s	= (float*) calloc(n, sizeof(float)); //== dens_prev?
		cube->density = (float*)calloc(n, sizeof(float));

		cube->Vx = (float*) calloc(n, sizeof(float));
		cube->Vy = (float*) calloc(n, sizeof(float));
		cube->Vz = (float*) calloc(size * size * size, sizeof(float));

		cube->Vx0 = (float*) calloc(n, sizeof(float));
		cube->Vy0 = (float*) calloc(n, sizeof(float));
		cube->Vz0 = (float*) calloc(size * size * size, sizeof(float));

		cube->position = vec3(0, 0, 0);
		cube->minBound = fvec3(cube->position.x - size / 2, cube->position.y - size / 2, cube->position.z - size / 2);
		cube->maxBound = fvec3(cube->position.x + size / 2, cube->position.y + size / 2, cube->position.z + size / 2);

		return cube;
	}

	void fcFree(FluidCube* cube)
	{
		free(cube->s);
		free(cube->density);

		free(cube->Vx);
		free(cube->Vy);
		free(cube->Vz);

		free(cube->Vx0);
		free(cube->Vy0);
		free(cube->Vz0);

		free(cube);
	}

	//"horizontal component of the velocity should be zero on the vertical walls, while the vertical component of
	//the velocity should be zero on the horizontal walls"
	//This is straight from the paper, mikeash's code is somewhat different, might use that when moving to 3D
	void set_bnd(int b, float* x, int N) {
		for (int j = 1; j < N - 1; j++) { //TODO kolla om detta verkligen behöver göras i separata loopar
			for (int i = 1; i < N - 1; i++) {
				x[XYZ(i, j, 0)] = b == 3 ? -x[XYZ(i, j, 1)] : x[XYZ(i, j, 1)];
				x[XYZ(i, j, N - 1)] = b == 3 ? -x[XYZ(i, j, N - 2)] : x[XYZ(i, j, N - 2)];
			}
		}
		for (int k = 1; k < N - 1; k++) {
			for (int i = 1; i < N - 1; i++) {
				x[XYZ(i, 0, k)] = b == 2 ? -x[XYZ(i, 1, k)] : x[XYZ(i, 1, k)];
				x[XYZ(i, N - 1, k)] = b == 2 ? -x[XYZ(i, N - 2, k)] : x[XYZ(i, N - 2, k)];
			}
		}
		for (int k = 1; k < N - 1; k++) {
			for (int j = 1; j < N - 1; j++) {
				x[XYZ(0, j, k)] = b == 1 ? -x[XYZ(1, j, k)] : x[XYZ(1, j, k)];
				x[XYZ(N - 1, j, k)] = b == 1 ? -x[XYZ(N - 2, j, k)] : x[XYZ(N - 2, j, k)];
			}
		}

		x[XYZ(0, 0, 0)] = 0.33f * (x[XYZ(1, 0, 0)] + x[XYZ(0, 1, 0)] + x[XYZ(0, 0, 1)]);
		x[XYZ(0, N - 1, 0)] = 0.33f * (x[XYZ(1, N - 1, 0)] + x[XYZ(0, N - 2, 0)] + x[XYZ(0, N - 1, 1)]);
		x[XYZ(0, 0, N - 1)] = 0.33f * (x[XYZ(1, 0, N - 1)] + x[XYZ(0, 1, N - 1)] + x[XYZ(0, 0, N-2)]);
		x[XYZ(0, N - 1, N - 1)] = 0.33f * (x[XYZ(1, N - 1, N - 1)] + x[XYZ(0, N - 2, N - 1)] + x[XYZ(0, N - 1, N - 2)]);
		x[XYZ(N - 1, 0, 0)] = 0.33f * (x[XYZ(N - 2, 0, 0)] + x[XYZ(N - 1, 1, 0)] + x[XYZ(N - 1, 0, 1)]);
		x[XYZ(N - 1, N - 1, 0)] = 0.33f * (x[XYZ(N - 2, N - 1, 0)] + x[XYZ(N - 1, N - 2, 0)] + x[XYZ(N - 1, N - 1, 1)]);
		x[XYZ(N - 1, 0, N - 1)] = 0.33f * (x[XYZ(N - 2, 0, N - 1)] + x[XYZ(N - 1, 1, N - 1)] + x[XYZ(N - 1, 0, N - 2)]);
		x[XYZ(N - 1, N - 1, N - 1)] = 0.33f * (x[XYZ(N - 2, N - 1, N - 1)] + x[XYZ(N - 1, N - 2, N - 1)] + x[XYZ(N - 1, N - 1, N - 2)]);
	}

	void fcAddDensity(FluidCube* cube, int x, int y, int z,float amount) { //Take force input from mouse or something "outside force"
		int N = cube->size;
		cube->density[XYZ(x, y, z)] += amount;
		//float dens = cube->density[XYZ(x, y, z)];
		//printf("Added density at (%d,%d): %f\n", x, y, dens);
	}

	void fcAddVelocity(FluidCube* cube, int x, int y, int z, float amountX, float amountY, float amountZ)
	{
		int N = cube->size;
		int index = XYZ(x, y, z);

		cube->Vx[index] += amountX;
		cube->Vy[index] += amountY;
		cube->Vz[index] += amountZ;
	}

	void fcClearDensity(FluidCube* cube) {
		for (int i = 0; i < cube->size * cube->size * cube->size; i++) {
			cube->density[i] = 0;
		}
	}


	//Exchange densities with neighboring cells. The smoke "leaks" into the neighboring cells
	//This is the Ash implementation, the original does (1+4*a) instead of * cReciprocal
	//This is because the linear solve is broken out of the diffuse func from the paper
	void lin_solve(int b, float* x, float* x0, float a, float c, int iter, int N) {
		float cRecip = 1.0 / c;
		for (int k = 0; k < iter; k++) {
			for (int m = 1; m < N - 1; m++) {
				for (int j = 1; j < N - 1; j++) {
					for (int i = 1; i < N - 1; i++) {
						x[XYZ(i, j, m)] =
							(x0[XYZ(i, j, m)]
								+ a *(x[XYZ(i + 1, j, m)]
									+ x[XYZ(i - 1, j, m)]
									+ x[XYZ(i, j + 1, m)]
									+ x[XYZ(i, j - 1, m)]
									+ x[XYZ(i, j, m + 1)]
									+ x[XYZ(i, j, m - 1)])
									) * cRecip;
					}
				}
			}
			set_bnd(b, x, N);
		}
	}

	void diffuse(int b, float* x, float* x0, float diff, float dt, int iter, int N) {
		float a = dt * diff * (N - 2) * (N - 2);
		lin_solve(b, x, x0, a, 1 + 6 * a, iter, N); //This 4 should maybe be a 6 in 3D
	}

	//This badboy is from the paper, mike ash impl is somewhat different due to XYZz
	//This function looks at the velocity in each cell and traces it back it time and sees where it lands.
	//It then performes a weighted average of the cells around the spot and applies that value to the current cell.
	void advect(int b, float* d, float* d0, float* u, float* v, float* w, float dt, int N) {
		float i0, i1, j0, j1, k0, k1;

		float dtx = dt * (N - 2);
		float dty = dt * (N - 2);
		float dtz = dt * (N - 2);

		float s0, s1, t0, t1, u0, u1;
		float tmp1, tmp2, tmp3, x, y, z;

		float Nfloat = (float) N;
		float ifloat, jfloat, kfloat;
		int i, j, k;

		for (k = 1, kfloat = 1; k < N - 1; k++, kfloat++) {
			for (j = 1, jfloat = 1; j < N - 1; j++, jfloat++) {
				for (i = 1, ifloat = 1; i < N - 1; i++, ifloat++) {
					tmp1 = dtx * u[XYZ(i, j, k)];
					tmp2 = dty * v[XYZ(i, j, k)];
					tmp3 = dtz * w[XYZ(i, j, k)];
					x = ifloat - tmp1;
					y = jfloat - tmp2;
					z = kfloat - tmp3;

					if (x < 0.5f) x = 0.5f;
					if (x > Nfloat + 0.5f) x = Nfloat + 0.5f;
					i0 = floor(x);
					i1 = i0 + 1.0f;
					if (y < 0.5f) y = 0.5f;
					if (y > Nfloat + 0.5f) y = Nfloat + 0.5f;
					j0 = floor(y);
					j1 = j0 + 1.0f;
					if (z < 0.5f) z = 0.5f;
					if (z > Nfloat + 0.5f) z = Nfloat + 0.5f;
					k0 = floor(z);
					k1 = k0 + 1.0f;

					s1 = x - i0;
					s0 = 1.0f - s1;
					t1 = y - j0;
					t0 = 1.0f - t1;
					u1 = z - k0;
					u0 = 1.0f - u1;

					int i0i = (int) i0;
					int i1i = (int) i1;
					int j0i = (int) j0;
					int j1i = (int) j1;
					int k0i = (int) k0;
					int k1i = (int) k1;

					d[XYZ(i, j, k)] = 
						s0 * (t0 * (u0 * d0[XYZ(i0i, j0i, k0i)] + u1 * d0[XYZ(i0i, j0i, k1i)])
							+(t1 * (u0 * d0[XYZ(i0i, j1i, k0i)] + u1 * d0[XYZ(i0i, j1i, k1i)])))
					  + s1 * (t0 * (u0 * d0[XYZ(i1i, j0i, k0i)] + u1 * d0[XYZ(i1i, j0i, k1i)])
							+(t1 * (u0 * d0[XYZ(i1i, j1i, k0i)] + u1 * d0[XYZ(i1i, j1i, k1i)])));
				}
			}
		}
		set_bnd(b, d, N);
	}

	//Poisson equation, using Gauss-Seidel relaxation again to solve system of equations.
	//This implementation is from the original paper,
	void project(float* u, float* v, float* w, float* p, float* div, int iter, int N) {
		float invN = (float) 1 / N;
		for (int k = 1; k < N - 1; k++) {
			for (int j = 1; j < N - 1; j++) {
				for (int i = 1; i < N - 1; i++) {
					div[XYZ(i, j, k)] = -0.5f * (
						u[XYZ(i + 1, j, k)]
						- u[XYZ(i - 1, j, k)]
						+ v[XYZ(i, j + 1, k)]
						- v[XYZ(i, j - 1, k)]
						+ w[XYZ(i, j, k + 1)]
						- w[XYZ(i, j, k - 1)]
						) * invN;
					p[XYZ(i, j, k)] = 0;
				}
			}
		}
		set_bnd(0, div, N);
		set_bnd(0, p, N);
		lin_solve(0, p, div, 1, 6, iter, N);

		for (int k = 1; k < N - 1; k++) {
			for (int j = 1; j < N - 1; j++) {
				for (int i = 1; i < N - 1; i++) {
					u[XYZ(i, j, k)] -= 0.5f * (p[XYZ(i + 1, j, k)]
						- p[XYZ(i - 1, j, k)]) * N;
					v[XYZ(i, j, k)] -= 0.5f * (p[XYZ(i, j + 1, k)]
						- p[XYZ(i, j - 1, k)]) * N;
					w[XYZ(i, j, k)] -= 0.5f * (p[XYZ(i, j, k + 1)]
						- p[XYZ(i, j, k - 1)]) * N;
				}
			}
		}
		set_bnd(1, u, N);
		set_bnd(2, v, N);
		set_bnd(3, w, N);
	}


	void fluidCubeStep(FluidCube* cube)
	{
		int N = cube->size;
		float visc = cube->visc;
		float diff = cube->diff;
		float dt = cube->dt;
		float* Vx = cube->Vx;
		float* Vy = cube->Vy;
		float* Vz = cube->Vz;
		float* Vx0 = cube->Vx0;
		float* Vy0 = cube->Vy0;
		float* Vz0 = cube->Vz0;
		float* s = cube->s; //I wonder what this represents? Seems like it's just empty. Replaces the swap step from original impl somehow
		float* density = cube->density;

		diffuse(1, Vx0, Vx, visc, dt, 4, N); //Default is 4 iterations
		diffuse(2, Vy0, Vy, visc, dt, 4, N);
		diffuse(3, Vz0, Vz, visc, dt, 4, N);

		project(Vx0, Vy0, Vz0, Vx, Vy, 4, N);

		advect(1, Vx, Vx0, Vx0, Vy0, Vz0, dt, N);
		advect(2, Vy, Vy0, Vx0, Vy0, Vz0, dt, N);
		advect(3, Vz, Vz0, Vx0, Vy0, Vz0, dt, N);

		project(Vx, Vy, Vz, Vx0, Vy0, 4, N);

		diffuse(0, s, density, diff, dt, 4, N);
		advect(0, density, s, Vx, Vy, Vz, dt, N);
	}

	float sampleDensity(FluidCube* fc, vec3 sample_pos) {
		int N = fc->size; //If we want to scale the cube, maybe scale here by field-size/world-size
		vec3 grid_size = fc->maxBound - fc->minBound;

		vec3 local_position = (sample_pos - fc->minBound) / grid_size;
		vec3 voxel_position = local_position * vec3(N);

		//recalculate from world coords to "density coords", maybe this can be done w/ some nice matrix instead
		int x = (int)floor(voxel_position.x);
		int y = (int)floor(voxel_position.y);
		int z = (int)floor(voxel_position.z);

		return fc->density[XYZ(x, y, z)];
	}
} //namespace particles