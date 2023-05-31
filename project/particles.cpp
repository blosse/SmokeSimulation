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
Most code is borrowed from
Real-Time Fluid Dynamics for Games by Jos Stam
and extended into 3D
*/

#define XYZ(x,y,z) ((x) + (y) * N + (z) * N * N)
//int N;

namespace particles {

	FluidCube* fcCreate(int size, int diffusion, int viscosity, float absorbtion, float scattering, float dt)
	{
		FluidCube* cube = (FluidCube*) malloc(sizeof(*cube));

		int n = size * size * size;

		//Properties of the smoke and simulation
		cube->size = size;
		cube->dt = dt;
		cube->diff = diffusion;
		cube->visc = viscosity;
		cube->absorbtion = absorbtion;
		cube->scattering = scattering;

		//Used in final diffuse and advect step to "clear" the buffers
		cube->s	= (float*) calloc(n, sizeof(float));

		//Stores the smoke density of each voxel
		cube->density = (float*)calloc(n, sizeof(float));

		//Current velocity
		cube->Vx = (float*) calloc(n, sizeof(float));
		cube->Vy = (float*) calloc(n, sizeof(float));
		cube->Vz = (float*) calloc(n, sizeof(float));

		//Previous velocity
		cube->Vx0 = (float*) calloc(n, sizeof(float));
		cube->Vy0 = (float*) calloc(n, sizeof(float));
		cube->Vz0 = (float*) calloc(n, sizeof(float));

		//Position and AABB of the field
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
	//i.e negate the incoming vector component for each respective plane
	void set_bnd(int b, float* x, int N) {
		//Set values on z-planes
		for (int j = 1; j < N - 1; j++) {
			for (int i = 1; i < N - 1; i++) {
				x[XYZ(i, j, 0)] = b == 3 ? -x[XYZ(i, j, 1)] : x[XYZ(i, j, 1)];
				x[XYZ(i, j, N - 1)] = b == 3 ? -x[XYZ(i, j, N - 2)] : x[XYZ(i, j, N - 2)];
			}
		}
		//Set values on y-planes
		for (int k = 1; k < N - 1; k++) {
			for (int i = 1; i < N - 1; i++) {
				x[XYZ(i, 0, k)] = b == 2 ? -x[XYZ(i, 1, k)] : x[XYZ(i, 1, k)];
				x[XYZ(i, N - 1, k)] = b == 2 ? -x[XYZ(i, N - 2, k)] : x[XYZ(i, N - 2, k)];
			}
		}
		//Set values on x-planes
		for (int k = 1; k < N - 1; k++) {
			for (int j = 1; j < N - 1; j++) {
				x[XYZ(0, j, k)] = b == 1 ? -x[XYZ(1, j, k)] : x[XYZ(1, j, k)];
				x[XYZ(N - 1, j, k)] = b == 1 ? -x[XYZ(N - 2, j, k)] : x[XYZ(N - 2, j, k)];
			}
		}

		//Set corners to average of neighboring cells
		x[XYZ(0, 0, 0)] = 0.33f * (x[XYZ(1, 0, 0)] + x[XYZ(0, 1, 0)] + x[XYZ(0, 0, 1)]);
		x[XYZ(0, N - 1, 0)] = 0.33f * (x[XYZ(1, N - 1, 0)] + x[XYZ(0, N - 2, 0)] + x[XYZ(0, N - 1, 1)]);
		x[XYZ(0, 0, N - 1)] = 0.33f * (x[XYZ(1, 0, N - 1)] + x[XYZ(0, 1, N - 1)] + x[XYZ(0, 0, N-2)]);
		x[XYZ(0, N - 1, N - 1)] = 0.33f * (x[XYZ(1, N - 1, N - 1)] + x[XYZ(0, N - 2, N - 1)] + x[XYZ(0, N - 1, N - 2)]);
		x[XYZ(N - 1, 0, 0)] = 0.33f * (x[XYZ(N - 2, 0, 0)] + x[XYZ(N - 1, 1, 0)] + x[XYZ(N - 1, 0, 1)]);
		x[XYZ(N - 1, N - 1, 0)] = 0.33f * (x[XYZ(N - 2, N - 1, 0)] + x[XYZ(N - 1, N - 2, 0)] + x[XYZ(N - 1, N - 1, 1)]);
		x[XYZ(N - 1, 0, N - 1)] = 0.33f * (x[XYZ(N - 2, 0, N - 1)] + x[XYZ(N - 1, 1, N - 1)] + x[XYZ(N - 1, 0, N - 2)]);
		x[XYZ(N - 1, N - 1, N - 1)] = 0.33f * (x[XYZ(N - 2, N - 1, N - 1)] + x[XYZ(N - 1, N - 2, N - 1)] + x[XYZ(N - 1, N - 1, N - 2)]);
	}

	void fcAddDensity(FluidCube* cube, int x, int y, int z,float amount) {
		int N = cube->size;
		cube->density[XYZ(x, y, z)] += amount;
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


	//Gauss-Seidel iterative solver
	//Calc inverse of a matrix
	//Do this since "We want to find the densities which when diffused 
	//backward in time yield the densities we started with"
	void lin_solve(int b, float* x, float* x0, float diffusion_rate, float diff_const, int iter, int N) {
		float inv_diff = 1.0 / diff_const; //1 + 6 * diffusion_rate
		for (int k = 0; k < iter; k++) {
			for (int m = 1; m < N - 1; m++) {
				for (int j = 1; j < N - 1; j++) {
					for (int i = 1; i < N - 1; i++) {
						x[XYZ(i, j, m)] =
							(x0[XYZ(i, j, m)]
								+ diffusion_rate *(x[XYZ(i + 1, j, m)]
									+ x[XYZ(i - 1, j, m)]
									+ x[XYZ(i, j + 1, m)]
									+ x[XYZ(i, j - 1, m)]
									+ x[XYZ(i, j, m + 1)]
									+ x[XYZ(i, j, m - 1)])
									) * inv_diff;
					}
				}
			}
			set_bnd(b, x, N);
		}
	}

	//Diffuse using the gauss-seidel solver
	void diffuse(int b, float* x, float* x0, float diff, float dt, int iter, int N) {
		float diffusion_rate = dt * diff * (N - 2) * (N - 2);
		lin_solve(b, x, x0, diffusion_rate, 1 + 6 * diffusion_rate, iter, N); //This 4 should maybe be a 6 in 3D
	}

	//This function looks at a particle in each voxel and traces it back it time and sees where it lands.
	//It then performes a weighted average of the voxels around the spot and applies that value to the current voxel.
	void advect(int b, float* d, float* d0, float* u, float* v, float* w, float dt, int N) {
		float i0, i1, j0, j1, k0, k1;
		float s0, s1, t0, t1, u0, u1;
		float x, y, z;

		float dt0 = dt * (N - 2);

		for (int k = 1; k < N - 1; k++) {
			for (int j = 1; j < N - 1; j++) {
				for (int i = 1; i < N - 1; i++) {
					//Figure out where particle came from
					x = i - (dt0 * u[XYZ(i, j, k)]);
					y = j - (dt0 * v[XYZ(i, j, k)]);
					z = k - (dt0 * w[XYZ(i, j, k)]);

					//Check so that the particle did not end up outside the field
					if (x < 0.5f) x = 0.5f;
					if (x > N + 0.5f) x = N + 0.5f;
					i0 = floor(x);
					i1 = i0 + 1.0f;
					if (y < 0.5f) y = 0.5f;
					if (y > N + 0.5f) y = N + 0.5f;
					j0 = floor(y);
					j1 = j0 + 1.0f;
					if (z < 0.5f) z = 0.5f;
					if (z > N + 0.5f) z = N + 0.5f;
					k0 = floor(z);
					k1 = k0 + 1.0f;

					//Calc the distance a particle as moved
					s1 = x - i0;
					s0 = 1.0f - s1;
					t1 = y - j0;
					t0 = 1.0f - t1;
					u1 = z - k0;
					u0 = 1.0f - u1;

					//Average togheter the densities around the voxel which the "particle" came from, and store in the current voxel
					d[XYZ(i, j, k)] = 
						 s0 * (t0 * (u0 * d0[XYZ((int) i0, (int) j0, (int) k0)] + u1 * d0[XYZ((int) i0, (int) j0, (int) k1)])
							+ (t1 * (u0 * d0[XYZ((int) i0, (int) j1, (int) k0)] + u1 * d0[XYZ((int) i0, (int) j1, (int) k1)])))
						+s1 * (t0 * (u0 * d0[XYZ((int) i1, (int) j0, (int) k0)] + u1 * d0[XYZ((int) i1, (int) j0, (int) k1)])
							+ (t1 * (u0 * d0[XYZ((int) i1, (int) j1, (int) k0)] + u1 * d0[XYZ((int) i1, (int) j1, (int) k1)])));
				}
			}
		}
		set_bnd(b, d, N);
	}

	//This step makes sure that the "mass is conserved" i.e makes sure there is equal inflow and outflow in every voxel.
	//Builds on the fact that every velocity field can be described as the sum of a mass conserving field and gradient field
	//We get our incompressible field by subtracting the gradient from our current velocities
	void project(float* u, float* v, float* w, float* p, float* div, int iter, int N) {
		//Set up "matricies" to calc the gradient
		float invN =  1.f / N;
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
		//Reuse the gauss-seidel solver to solve the equations and get the gradient field
		lin_solve(0, p, div, 1, 6, iter, N);

		//Subtract gradient from out velocity field
		for (int k = 1; k < N - 1; k++) {
			for (int j = 1; j < N - 1; j++) {
				for (int i = 1; i < N - 1; i++) {
					u[XYZ(i, j, k)] -= 0.5f * (
						  p[XYZ(i + 1, j, k)]
						- p[XYZ(i - 1, j, k)]) * N;
					v[XYZ(i, j, k)] -= 0.5f * (
						  p[XYZ(i, j + 1, k)]
						- p[XYZ(i, j - 1, k)]) * N;
					w[XYZ(i, j, k)] -= 0.5f * (
						  p[XYZ(i, j, k + 1)]
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
		float* s = cube->s;
		float* density = cube->density;

		//Diffuse the velocities
		diffuse(1, Vx0, Vx, visc, dt, 4, N); //Default is 4 iterations
		diffuse(2, Vy0, Vy, visc, dt, 4, N);
		diffuse(3, Vz0, Vz, visc, dt, 4, N);

		//Keep them mass conserving
		project(Vx0, Vy0, Vz0, Vx, Vy, 4, N);

		//Advect velocities
		advect(1, Vx, Vx0, Vx0, Vy0, Vz0, dt, N);
		advect(2, Vy, Vy0, Vx0, Vy0, Vz0, dt, N);
		advect(3, Vz, Vz0, Vx0, Vy0, Vz0, dt, N);

		//Keep them mass conserving
		project(Vx, Vy, Vz, Vx0, Vy0, 4, N);

		//Finaly diffuse and advect the actual smoke densities
		diffuse(0, s, density, diff, dt, 4, N);
		advect(0, density, s, Vx, Vy, Vz, dt, N);
	}

	float sampleDensity(FluidCube* fc, vec3 sample_pos) {
		int N = fc->size;

		vec3 voxel_position = (sample_pos - fc->minBound);

		//recalculate from world coords to "density coords", maybe this can be done w/ some nice matrix instead
		int x = (int)floor(voxel_position.x);
		int y = (int)floor(voxel_position.y);
		int z = (int)floor(voxel_position.z);

		if ((x == 0) || (y == 0) || (z == 0) || (x == N) || (y == N) || (z == N)) {
			return 0;
		}

		return fc->density[XYZ(x, y, z)];
	}
} //namespace particles