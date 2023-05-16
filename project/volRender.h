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
		int sign[3];
		ray(fvec3 or, fvec3 dr) {
			origin = or;
			dir = dr;
			invdir = fvec3{ 1 } / dr;
			sign[0] = (invdir.x < 0);
			sign[1] = (invdir.y < 0);
			sign[2] = (invdir.z < 0);
		}

		vec2 rayAABB_intersect(FluidCube* fc) {
			//Should i update minBound and maxBound of the fc as the camera rotates??
			//What is return value on miss?
			//vec3 tMin = (fc->minBound - this->origin) * this->invdir;
			//vec3 tMax = (fc->maxBound - this->origin) * this->invdir;
			//
			//vec3 t0 = glm::min(tMin, tMax);
			//vec3 t1 = glm::max(tMin, tMax);
			//
			//float tNear = glm::max(glm::max(t0.x, t0.y), t0.z);
			//float tFar = glm::min(glm::min(t1.x, t1.y), t1.z);
			////printf("near/far: %f/%f\n", tNear, tFar);
			//return vec2(tNear, tFar);

			vec3 tMin = (fc->minBound - this->origin) / this->dir;
			vec3 tMax = (fc->maxBound - this->origin) / this->dir;

			//find largest min value
			float t0 = glm::max(glm::max(glm::min(tMin.x, tMax.x), glm::min(tMin.y, tMax.y)), glm::min(tMin.z, tMax.z));
			//find smallest max value
			float t1 = glm::min(glm::min(glm::max(tMin.x, tMax.x), glm::max(tMin.y, tMax.y)), glm::max(tMin.z, tMax.z));

			//if ray intersects box that is behind us
			if (t1 < 0) {
				return vec2(-1, -1);
			}

			//if t0 > t1 ray does not intersect box
			if (t0 > t1) {
				return vec2(-1, -1);
			}

			//We are inside box
			if (t0 < 0) {
				return vec2(0, t1);
			}
			//else we are intersecting the box
			return vec2(t0, t1);
		}

	vec3 traceRay(FluidCube* fc, vec3* imgBuf, vec3 lightPos) {
		vec3 bg_color = { 0.8f, 0.8f, 1.f };
		vec3 light_color = {0.f, 1000.f, 0.f};
		vec3 result{ 0 }; //init volume color to 0
		float phase = 1 / 4 * M_PI; //Basic phase term for isotropic scattering, might implement henyey-greenstein if time
		float transparency = 1; //init transparency to 1
		int N = fc->size;

		vec2 intersect = rayAABB_intersect(fc);
		//printf("intersect (%f, %f)\n", intersect.x, intersect.y);
		if (intersect == vec2(-1, -1)) {
			//printf("missed ray dir: %f,%f\n", this->dir.x, this->dir.y);
			return bg_color; //If the ray misses, just return background color
		}
		float march_dist = intersect.y - intersect.x; //Distance the ray travels inside volume
		float step_size = 1;
		int num_samples = march_dist / step_size; //Maybe rethink how many samples we want, this should give 1 sample per distance unit
		//printf("Num samples %d: \n", num_samples);

		for (int n = 0; n < num_samples; n++) {
			float t = intersect.x + step_size + (0.5 * n); //Find middle of sample -> 0.5*n
			vec3 sample_pos = this->origin + t * this->dir;
			vec3 light_dir = sample_pos - lightPos; //direction from sample_pos to position of light
			
			ray light_ray(lightPos, light_dir);
			vec2 light_intersect = light_ray.rayAABB_intersect(fc);
			//float light_t1 = light_intersect.y; //Should just be t1 right since light sample_pos will always be inside cube? //((sample_pos - light_ray.dir) / light_ray.origin).y;
			float density_sample = 1 - sampleDensity(fc, sample_pos);

			//Calc transmission at sample
			float sample_attn = exp(-step_size * (fc->absorbtion + fc->scattering)) * density_sample;//Beers law
			//Attenuate volume transparency by trans.val of current sample
			transparency *= sample_attn;
			if(transparency < 0.005) break; //if transparency is too low, no point in continuing calcs

			//In-scattering, we are getting fancy now
			if (light_intersect != vec2(-1, -1)) {
				int num_samples_light = (light_intersect.y / step_size);
				float tau = 0; //Accumulated transmission
				for (int nl = 0; nl < num_samples_light; nl++) {
					float tLight = step_size * (nl + 0.5);
					vec3 sample_pos_light = sample_pos + tLight * light_dir;
					tau += 1 - sampleDensity(fc, sample_pos_light);
				}
				float light_attn = exp(-tau * (fc->absorbtion + fc->scattering) * step_size); //The lazy way, should sample along the ray here.
				result += transparency * light_color * light_attn * fc->scattering * phase * step_size * density_sample;
			}
		}
		//Combine the bg color and thhe calculated color of the volume
		//printf("returning raycolor: %f, %f, %f\n", ret_val.x, ret_val.y, ret_val.z);
		return bg_color * transparency + result;
	}
};

void renderVolume(vec3* renderBuffer, std::vector<ray> rayBuffer, FluidCube* fc, vec3 lightPos, vec3 cameraPos, int img_width, int img_height, int cameraMoved) {
	int fov = 90;
	float aspectRatio = img_width / (float)img_height;
	int N = img_height;
	
	mat4 cameraToWorld;
	float scale = tan(radians(fov * 0.5)); //?
	vec3 origin;
	
	if (cameraMoved == 1 || rayBuffer.size() == 0) {
		for (int y = 0; y < img_height; y++) {
			for (int x = 0; x < img_width; x++) {
				float px = (2 * ((x + 0.5) / img_width)) - 1;
				float py = (2 * ((y + 0.5) / img_height)) - 1;

				//This is in camera-space rn
				//We are going to need inverse(lookAt(...)) here (inverse viewmatrix)
				//Might be good idea to store (cache) all ray directions before tracing, so we only have to recalc them when cam moves, might give speed up. 
				vec3 rayOrigin(0);
				vec3 rayDir = normalize(vec3(px, py, -1) - rayOrigin);
				ray ray(rayOrigin, rayDir);
				rayBuffer.push_back(ray);
			}
		}
	}
	//Looping over all pixels in output image
	for (int i = 0; i < img_width * img_height; i++) {
		renderBuffer[i] = rayBuffer[i].traceRay(fc, renderBuffer, lightPos);
	}

}


