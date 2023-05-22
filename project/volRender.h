#pragma once

#include <GL/glew.h>
#include <vector>
#include <glm/glm.hpp>
#include <glm/detail/type_vec3.hpp>
#include <glm/mat4x4.hpp>
#include <glm/gtx/transform.hpp>
#include <stdlib.h>
#include "particles.h"
#include <labhelper.h>

using namespace glm;
using namespace std;
using namespace labhelper;
using namespace particles;

#define XY(i,j) ((i)+(N*j))
#define XYZ(x,y,z) ((x) + (y) * N + (z) * N * N)

#define MT

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

			vec3 tMin = (fc->minBound - this->origin) * this->invdir;
			vec3 tMax = (fc->maxBound - this->origin) * this->invdir;

			//find largest min value
			float t0 = glm::max(glm::max(glm::min(tMin.x, tMax.x), glm::min(tMin.y, tMax.y)), glm::min(tMin.z, tMax.z));
			//find smallest max value
			float t1 = glm::min(glm::min(glm::max(tMin.x, tMax.x), glm::max(tMin.y, tMax.y)), glm::max(tMin.z, tMax.z));

			//if ray intersects box that is behind us
			if (t1 < 0) {
				printf("Box behind!");
				return vec2(-1, -1);
			}

			//if t0 > t1 ray does not intersect box
			if (t0 > t1) {
				return vec2(-1, -1);
			}

			//We are inside box
			if (t0 < 0) {
				printf("Inside box\n");
				return vec2(0, t1);
			}
			//else we are intersecting the box
			return vec2(t0, t1);
		}

	vec3 traceRay(FluidCube* fc, vec3* imgBuf, vec3 lightPos) {
		vec3 bg_color = { 1.f, 1.f, 0.95f };
		vec3 light_color = {1.f, 1.f, 1.f};
		//float phase = 1 / 4 * 3.141; //Basic phase term for isotropic scattering, might implement henyey-greenstein if time
		int N = fc->size;

		vec2 intersect = rayAABB_intersect(fc);
		if (intersect == vec2(-1, -1)) {
			return bg_color; //If the ray misses, just return background color
		}
		float march_dist = intersect.y - intersect.x; //Distance the ray travels inside volume
		float step_size = 1;
		float step_size_light = step_size + 1.f;
		int num_samples = (int) ceil(march_dist / step_size); //Maybe rethink how many samples we want, this should give 1 sample per distance unit
		float stride = march_dist / num_samples;
		
		float transparency = 1; //init transparency to 1 (Fully transparent)
		vec3 result{ 0 }; //init volume color to 0

		for (int n = 0; n < num_samples; n++) {
			//Sampling along primary ray
			float t = intersect.x + stride + (0.5f * n); //Find middle of sample -> 0.5*n
			vec3 sample_pos = this->origin + t * this->dir;
			float density_sample = sampleDensity(fc, sample_pos); //Should this be 1-? Maybe rethink how density works
			
			//Calc transmission at sample
			float sample_attn = exp(stride * (fc->absorbtion + fc->scattering) * -density_sample);//Beers law
			//Attenuate volume transparency by trans.val of current sample
			transparency *= sample_attn;
			if(transparency < 0.01) break; //if transparency is too low, no point in continuing calcs

			//Sample along light ray
			vec3 light_dir = normalize((sample_pos - lightPos)); //direction from position of light to sample position
			ray light_ray(lightPos, light_dir);
			vec2 light_intersect = light_ray.rayAABB_intersect(fc);
		

			//In-scattering, we are getting fancy now
			//Check if light ray intersects the box and if the density of the sample is greater than 0
			if (light_intersect != vec2(-1, -1) && density_sample > 0) {
				int num_samples_light = (int) ceil(light_intersect.y / step_size_light ); //Larger step size for light sample
				float stride_light = (light_intersect.y - light_intersect.x) / num_samples_light;
				float tau = 0; //Accumulated transmission
				for (int nl = 0; nl < num_samples_light-1; nl++) {
					float t_light = light_intersect.x + stride_light * (nl + 0.5);
					vec3 sample_pos_light = lightPos + t_light * light_dir;
					tau += sampleDensity(fc, sample_pos_light);
				}
				//printf("tau = %f\n", tau);
				float light_attn = exp(-tau * (fc->absorbtion + fc->scattering) * stride_light);
				result += light_color     //Light color
						* light_attn      //Transmission value of light ray
						* (1.f / 4.f * 3.14f) //Phase function very simple rn
						* fc->scattering  //Scattering coefficient
						* transparency    //Transmission of prime ray
						* stride //Can be thought of as dx in a riemann sum
						* density_sample; //Density at sample location along light ray

			}
		}
		return bg_color * transparency + result;
	}
};

void renderVolume(vec3* renderBuffer, std::vector<ray> rayBuffer, FluidCube* fc, vec3 lightPos, vec3 cameraPos, int img_width, int img_height, int cameraMoved) {
	int fov = 90;
	float aspectRatio =(float) (img_width / img_height);
	int N = img_height;
	
	mat4 viewMatrix = glm::lookAt(cameraPos, vec3(0, 0, -1), vec3(0.f, 1.f, 0.f));
	mat4 inverseView = inverse(viewMatrix);

	//mat4 projection = perspectiveFov(radians(fov), (float)img_width, (float)img_height,  -1.f, INFINITY);

	float scale = tan(radians(fov * 0.5)); //?
	vec3 origin;
	
	if (cameraMoved == 1 || rayBuffer.size() == 0) {
		for (int y = 0; y < img_height; y++) {
			for (int x = 0; x < img_width; x++) {
				float px = ((2 * ((x + 0.5) / img_width)) - 1) * aspectRatio * scale;
				float py = ((2 * ((y + 0.5) / img_height)) - 1) * scale;

				//This is in camera-space rn
				//We are going to need inverse(lookAt(...)) here (inverse viewmatrix)
				//Might be good idea to store (cache) all ray directions before tracing, so we only have to recalc them when cam moves, might give speed up. 

				vec3 rayOrigin = { 0, 0, 0 };
				vec3 rayTargetWorld = inverseView * vec4(px, py, -1, 0);
				vec3 rayDir = normalize(rayTargetWorld);

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

