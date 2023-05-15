#pragma once

#include <GL/glew.h>
#include <vector>
#include <glm/glm.hpp>
#include <glm/detail/type_vec3.hpp>
#include <glm/mat4x4.hpp>
#include <stdlib.h>
#include "particles.h"
#include <labhelper.h>

using namespace glm;
using namespace std;
using namespace labhelper;
using namespace particles;

#define XY(i,j) ((i)+(N*j))
#define XYZ(x,y,z) ((x) + (y) * N + (z) * N * N)

class ray {
	public:
		vec3 origin;
		vec3 dir;
		vec3 invdir;
		ray(fvec3 or, fvec3 dr) {
			origin = or;
			dir = dr;
			invdir = fvec3{ 1 } / dr;
		}

		vec2 rayAABB_intersect(FluidCube* fc) {
			float tmin, tmax, tymin, tymax, tzmin, tzmax;
			vec2 result = { -1, -1 }; //result.x is t0 result.y is t1 [-1,-1] is no intersection, prob not so smart to use that return value as false.
			if (this->dir.x >= 0) {
				tmin = (fc->minBound.x - this->origin.x) * this->invdir.x;
				tmax = (fc->maxBound.x - this->origin.x) * this->invdir.x;
			}
			else {
				tmax = (fc->minBound.x - this->origin.x) * this->invdir.x;
				tmin = (fc->maxBound.x - this->origin.x) * this->invdir.x;
			}
			if (this->dir.y >= 0) {
				tymin = (fc->minBound.y - this->origin.y) * this->invdir.y;
				tymax = (fc->maxBound.y - this->origin.y) * this->invdir.y;
			}
			else {
				tymax = (fc->minBound.y - this->origin.y) * this->invdir.y;
				tymin = (fc->maxBound.y - this->origin.y) * this->invdir.y;
			}
			if (tmin > tmax || tymin > tmax)
				return result; //Test fail
			if (tymin > tmin)
				tmin = tymin;
			if (tymax < tmax)
				tmax = tymax;
			if (this->dir.y >= 0) {
				tzmin = (fc->minBound.z - this->origin.z) * this->invdir.z;
				tzmax = (fc->maxBound.z - this->origin.z) * this->invdir.z;
			}
			else {
				tzmax = (fc->minBound.z - this->origin.z) * this->invdir.z;
				tzmin = (fc->maxBound.z - this->origin.z) * this->invdir.z;
			}
			if (tmin > tzmax || tzmin > tmax)
				return result; //test fail
			if (tzmin > tmin)
				tmin = tzmin;
			if (tzmax < tmax)
				tmax = tzmax;
			return result = { tmin, tmax };
		}

	vec3 traceRay(FluidCube* fc, vec3* imgBuf, vec3 lightPos) {
		vec3 bg_color = { 0.1f, 0.1f, 0.1f };
		vec3 vl_color = { 0.9, 0.5, 0.5 };
		vec3 light_color = { 1, 1, 1 };
		vec2 intersect = rayAABB_intersect(fc);
		if (intersect == vec2(-1, -1)) return bg_color; //If the ray misses, just return background color
		
		float transparency = 1; //init transparency to 1
		vec3 result{ 0 }; //init volume color to 0
		int N = fc->size;
		float march_dist = intersect.y - intersect.x; //Distance the ray travels inside volume
		float step_size = 1;
		int num_samples = march_dist / step_size; //Maybe rethink how many samples we want, this should give 1 sample per distance unit

		for (int n = 0; n < num_samples; n++) {
			float t = intersect.x + step_size + (0.5 * n); //Find middle of sample -> 0.5*n
			vec3 sample_pos = this->origin + t * this->dir;
			vec3 light_dir = sample_pos - lightPos; //direction from sample_pos to position of light
			
			ray light_ray(lightPos, -light_dir);
			vec2 light_intersect = light_ray.rayAABB_intersect(fc);
			float light_t1 = ((sample_pos - light_ray.dir) / light_ray.origin).x; //Keep an eye on this, that .x swizzle might not be correct.

			//Calc volume density at sample
			float sample_attn = exp(-step_size * fc->density[XYZ((int)sample_pos.x, (int)sample_pos.y, (int)sample_pos.z)]); //Beers law TODO actually use the density field. 
			//Attenuate volume transparency by trans.val of current sample
			transparency *= sample_attn;
			if(transparency < 0.005) return bg_color * transparency + result; //if transparency is too low, no point in continuing calcs

			//In-scattering, we are getting fancy now
			if (light_intersect != vec2(-1, -1)) {
				float light_attn = exp(-light_intersect.y * fc->absorbtion); //The lazy way, should sample along the ray here.
				result += transparency * light_color * step_size;
			}
		}
		//Combine the bg color and thhe calculated color of the volume
		return bg_color * transparency + result;
	}
};

void renderVolume(float* renderBuffer, FluidCube* fc, vec3 lightPos, vec3 cameraPos) {
	/* TODO
	- Loop through all pixels of the renderTexture
	- Shoot and trace ray into the scene
		- This means figuring out how camera is positioned in relation to "screen" (fov)
	- Store result of the traced ray into renderBuffer
	*/
}


