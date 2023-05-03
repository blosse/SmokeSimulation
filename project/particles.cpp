#include "Particles.h"
#include <algorithm>
#include <GL/glew.h>
#include <glm/glm.hpp>
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

#define SWAP(x0,x) {float *tmp=x0;x0=x;x=tmp;} //Swaps two array pointers
#define XY(i,j) ((i)+(N*2)*(j))
//int N;

namespace particles {

	FluidCube* fcCreate(int size, int diffusion, int viscosity, float dt)
	{
		FluidCube* cube = (FluidCube*) malloc(sizeof(*cube));

		int n = size * size * size;//(size + 2) * (size + 2); TODO Fixa minnesproblemet

		cube->size = size;
		cube->dt = dt;
		cube->diff = diffusion;
		cube->visc = viscosity;

		cube->s	= (float*) calloc(n, sizeof(float)); //== dens_prev?
		cube->density = (float*)calloc(n, sizeof(float));

		cube->Vx = (float*) malloc(n * sizeof(float));
		cube->Vy = (float*)malloc(n * sizeof(float));
		//cube->Vz = (float*) calloc(size * size * size, sizeof(float));

		cube->Vx0 = (float*) malloc(n * sizeof(float));
		cube->Vy0 = (float*) malloc(n * sizeof(float));
		//cube->Vz0 = (float*) calloc(size * size * size, sizeof(float));

		return cube;
	}

	void fcFree(FluidCube* cube)
	{
		free(cube->s);
		free(cube->density);

		free(cube->Vx);
		free(cube->Vy);
		//free(cube->Vz);

		free(cube->Vx0);
		free(cube->Vy0);
		//free(cube->Vz0);

		free(cube);
	}

	//"horizontal component of the velocity should be zero on the vertical walls, while the vertical component of
	//the velocity should be zero on the horizontal walls"
	//This is straight from the paper, mikeash's code is somewhat different, might use that when moving to 3D
	void set_bnd(int b, float* x, int N) {
		int i;
		for (i = 1; i <= N; i++) {	
			x[XY(0, i)] = b == 1 ? -1 * x[XY(1, i)] : x[XY(1, i)];     
			x[XY(N + 1, i)] = b == 1 ? -1 * x[XY(N, i)] : x[XY(N, i)];
			x[XY(i, 0)] = b == 2 ? -1 * x[XY(i, 1)] : x[XY(i, 1)];
			x[XY(i, N + 1)] = b == 2 ? -1 * x[XY(i, N)] : x[XY(i, N)];
		}
		x[XY(0, 0)] = 0.5 * (x[XY(1, 0)] + x[XY(0, 1)]);
		x[XY(0, N + 1)] = 0.5 * (x[XY(1, N + 1)] + x[XY(0, N)]);
		x[XY(N + 1, 0)] = 0.5 * (x[XY(N, 0)] + x[XY(N + 1, 1)]);
		x[XY(N + 1, N + 1)] = 0.5 * (x[XY(N, N + 1)] + x[XY(N + 1, N)]);
	}

	void fcAddDensity(FluidCube* cube, int x, int y, /*int z,*/float amount) { //Take force input from mouse or something "outside force"
		int N = cube->size;
		cube->density[XY(x, y)] += amount;
		//float dens = cube->density[XY(x, y)];
		//printf("Added density at (%d,%d): %f\n", x, y, dens);
	}

	void fcAddVelocity(FluidCube* cube, int x, int y, /*int z,*/ float amountX, float amountY/*, float amountZ*/)
	{
		int N = cube->size;
		int index = XY(x, y);

		cube->Vx[index] += amountX;
		cube->Vy[index] += amountY;
		//cube->Vz[index] += amountZ;
	}

	//Exchange densities with neighboring cells. The smoke "leaks" into the neighboring cells
	//TODO Figure out if x and x0 should just be arrays or if theres a fancier way to do it
	//This is the Ash implementation, the original does (1+4*a) instead of * cReciprocal
	//This is because the linear solve is broken out of the diffuse func from the paper
	void lin_solve(int b, float* x, float* x0, float a, float c, int iter, int N) {
		float cReciprocal = 1.0 / c;
		for (int k = 0; k < iter; k++) {
			for (int j = 1; j < N-1; j++) {
				for (int i = 1; i < N-1; i++) {
					x[XY(i, j)] = (x0[XY(i, j)]
						+ a* (x0[XY(i - 1, j)] 
							+ x0[XY(i + 1, j)] 
							+ x0[XY(i, j - 1)]
							+ x0[XY(i, j + 1)]))
							* cReciprocal;
				}
			}
		set_bnd(b ,x , N);
		}
	}

	void diffuse(int b, float* x, float* x0, float diff, float dt, int iter, int N) {
		float a = dt * diff * (N - 2) * (N - 2);
		lin_solve(b, x, x0, a, 1 + 4 * a, iter, N); //This 4 should maybe be a 6 in 3D
	}

	//This badboy is from the paper, mike ash impl is somewhat different due to xyz
	//This function looks at the velocity in each cell and traces it back it time and sees where it lands.
	//It then performes a weighted average of the cells around the spot and applies that value to the current cell.
	void advect(int b, float* d, float* d0, float* u, float* v, /*float* velocZ,*/ float dt, int N) {
		float dt0 = dt * N;
		int i0, j0, i1, j1;
		float s0, t0, s1, t1, x, y;
		for (int i = 1; i < N; i++) {
			for (int j = 1; j < N; j++) {
				x = i - dt0 * u[XY(i, j)];
				y = j - dt0 * v[XY(i, j)];
				if (x < 0.5f) x = 0.5f;
				if (x > N + 0.5f) x = N + 0.5f;
				i0 = (int)x;
				i1 = i0 + 1;
				if (y < 0.5) y = 0.5f;
				if (y > N + 0.5) y = N + 0.5f;
				j0 = (int)y;
				j1 = j0 + 1;
				s1 = x - i0;
				s0 = 1 - s1;
				t1 = y - j0;
				t0 = 1 - t1;
				d[XY(i, j)] = s0 * (t0 * d0[XY(i0, j0)] + t1 * d0[XY(i0, j1)]) + //TODO Fixa minnesgrejjen
					s1 * (t0 * d0[XY(i1, j0)] + t1 * d0[XY(i1, j1)]);
			}
		}
		set_bnd(b, d, N);
	}

	//Poisson equation, using Gauss-Seidel relaxation again to solve system of equations.
	//This implementation is from the original paper,
	void project(float* u, float* v, /*float* velocZ,*/ float* p, float* div, int iter, int N) {
		float h = 1.0 / N;
		for (int i = 1; i <= N; i++) {
			for (int j = 1; j <= N; j++) {
				div[XY(i, j)] = -0.5f * (
					  u[XY(i + 1, j)] 
					- u[XY(i - 1, j)] 
					+ v[XY(i, j + 1)] 
					- v[XY(i, j - 1)])
					* h; //
				p[XY(i, j)] = 0;
			}
		}
		set_bnd(0, div, N);
		set_bnd(0, p, N);
		lin_solve(0, p, div, 1, 4, iter, N);

		for (int i = 1; i <= N; i++) {
			for (int j = 1; j <= N; j++) {
				u[XY(i, j)] -= 0.5 * (p[XY(i + 1, j)] - p[XY(i - 1, j)]) / h;
				v[XY(i, j)] -= 0.5 * (p[XY(i, j + 1)] - p[XY(i, j - 1)]) / h;
			}
		}
		set_bnd(1, u, N);
		set_bnd(2, v, N);
	}


	void fluidCubeStep(FluidCube* cube)
	{
		int N = cube->size;
		float visc = cube->visc;
		float diff = cube->diff;
		float dt = cube->dt;
		float* Vx = cube->Vx;
		float* Vy = cube->Vy;
		//float* Vz = cube->Vz;
		float* Vx0 = cube->Vx0;
		float* Vy0 = cube->Vy0;
		//float* Vz0 = cube->Vz0;
		float* s = cube->s; //I wonder what this represents? Seems like it's just empty. Replaces the swap step from original impl somehow
		float* density = cube->density;

		diffuse(1, Vx0, Vx, visc, dt, 20, N); //Default is 4 iterations
		diffuse(2, Vy0, Vy, visc, dt, 20, N);
		//diffuse(3, Vz0, Vz, visc, dt, 4, N);

		project(Vx0, Vy0, /*Vz0,*/ Vx, Vy, 20, N);

		advect(1, Vx, Vx0, Vx0, Vy0, /*Vz0,*/ dt, N);
		advect(2, Vy, Vy0, Vx0, Vy0, /*Vz0,*/ dt, N);

		project(Vx, Vy, /*Vz,*/ Vx0, Vy0, 20, N);

		diffuse(0, s, density, diff, dt, 20, N);
		advect(0, density, s, Vx, Vy, /*Vz,*/ dt, N);
	}
} //namespace particles