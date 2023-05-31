#pragma once

#include <GL/glew.h>
#include <vector>
#include <glm/glm.hpp>
#include <glm/detail/type_vec3.hpp>
#include <glm/mat4x4.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtx/rotate_vector.hpp>
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

class camera {
public:
	float angle;
	float radius;
	mat4 viewMatrix;
	vec3 position;
	camera(float dist, float ang) {
		angle = ang;
		radius = dist;
		position = vec3(sin(angle) * radius, 0.f, cos(angle) * radius);
		viewMatrix = lookAt(position, vec3(0,0,0), vec3(0, 1, 0));
	}

	void move(float angle, float dist) {
		this->angle = angle;
		this->radius = dist;
		float x = sin(angle) * radius;
		float z = cos(angle) * radius;
		this->position = vec3(x, 5.f, z);
		this->viewMatrix = lookAt(this->position, vec3(0, 0, 0), vec3(0, 1, 0));
	}
};

class sphere {
public:
	vec3 position;
	float radius;
	float radius2;
	vec3 color;
	sphere(vec3 pos, float r, vec3 col) {
		position = pos;
		radius = r;
		color = col;
		radius2 = r * r;
	}
};

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

	//Geometric intersection test of ray and sphere
	vec2 raySphereIntersect(sphere sph) {
		vec3 L = sph.position - this->origin; //Vector from center of sphere to ray origin
		float t_proj = dot(L, this->dir); //"Hypotenusan"
		if (t_proj < 0) { 
			return vec2(-1, -1); //Sphere is behind ray origin
		} 
		float d2 = dot(L, L) - (t_proj * t_proj); //Pythagoras
		if (d2 > sph.radius) { 
			return vec2(-1, -1); //Ray misses
		}
		float t_halfcircle = sqrt(sph.radius2 - d2); //Pythagoras again
		return vec2(t_proj - t_halfcircle, t_proj + t_halfcircle); //Return points of intersection
	}

	//Generate jitter [-0.2, 0.2]
	float jitter() {
		return ((rand() / RAND_MAX) * 0.4f) - 0.2f;
	}

	vec3 traceRay(FluidCube* fc, vec3* imgBuf, sphere lightPos) {
		vec3 bg_color = { 0.15f, 0.15f, 0.18f };
		int N = fc->size;

		//If ray hits light source, set bg color to color of light
		vec2 intersect_sphere = raySphereIntersect(lightPos);
		if (intersect_sphere != vec2(-1, -1)) {
			bg_color = lightPos.color;
		}
		//If the ray misses, return background color
		vec2 intersect = rayAABB_intersect(fc);
		if (intersect == vec2(-1, -1)) {
			return bg_color;
		}
		//If light source is in front of density grid, return light color
		if ((intersect.x > intersect_sphere.y) && (intersect_sphere.x > -1.f))  {
			return lightPos.color;
		}

		float march_dist = intersect.y - intersect.x; //Distance the ray travels inside volume
		float step_size = 1.f; //Change sampling frequency here, large performance hit if lowered
		float step_size_light = step_size;
		int num_samples = (int) ceil(march_dist / step_size); //Maybe rethink how many samples we want, this should give 1 sample per distance unit
		float stride = march_dist / num_samples;
		
		float transparency = 1; //init transparency to 1 (Fully transparent)
		vec3 result{ 0 }; //init volume color to 0

		for (int n = 0; n < num_samples; n++) {
			//Sampling along primary ray
			float t = intersect.x + stride * (0.5f + n /*+ jitter()*/); //Find middle of sample -> 0.5*n, Jitter slows it down alot
			vec3 sample_pos = this->origin + t * this->dir;
			float density_sample = sampleDensity(fc, sample_pos); //Should this be 1-? Maybe rethink how density works
			
			//Calc transmission at sample
			float sample_attn = exp(stride * (fc->absorbtion + fc->scattering) * -density_sample);//Beers law
			//Attenuate volume transparency by trans.val of current sample
			transparency *= sample_attn;
			if(transparency < 0.01) break; //if transparency is too low, no point in continuing calcs

			//Calculate light ray from sample point and check intersection
			vec3 light_dir = normalize((sample_pos - lightPos.position)); //direction from position of light to sample position
			ray light_ray(lightPos.position, light_dir);
			vec2 light_intersect = light_ray.rayAABB_intersect(fc);

			//In-scattering, we are getting fancy now
			//Check if light ray intersects the box and if the density of the sample is greater than 0
			if (light_intersect != vec2(-1, -1) && density_sample > 0) {
				int num_samples_light = (int) ceil(light_intersect.y / step_size_light ); //Larger step size for light sample
				float stride_light = (light_intersect.y - light_intersect.x) / num_samples_light;
				float tau = 0; //Accumulated transmission
				for (int nl = 0; nl < num_samples_light-1; nl++) {
					float t_light = light_intersect.x + stride_light * (nl + 0.5f  /*+ jitter()*/);
					vec3 sample_pos_light = lightPos.position + t_light * light_dir;
					tau += sampleDensity(fc, sample_pos_light);
				}
				float light_attn = exp(-tau * (fc->absorbtion + fc->scattering) * stride_light);
				result += lightPos.color     //Light color
						* light_attn      //Transmission value of light ray
						* (1.f / 4.f * 3.14f) //Phase function very simple rn
						* fc->scattering  //Scattering coefficient
						* transparency    //Transmission of prime ray
						* stride          //Step size can be thought of as dx in a riemann sum
						* density_sample; //Density at sample location along light ray
			}
		}
		return bg_color * transparency + result;
	}
};



void renderVolume(vec3* renderBuffer, std::vector<ray> *rayBuffer, FluidCube* fc, sphere lightPos, camera cam, int img_width, int img_height, int cameraMoved) {
	
	mat4 viewMatrix = cam.viewMatrix;
	mat4 inverseView = inverse(viewMatrix);

	if (cameraMoved) {
		rayBuffer->clear();
		for (int y = 0; y < img_height; y++) {
			for (int x = 0; x < img_width; x++) {
				float px = ((2 * ((x + 0.5) / img_width)) - 1);
				float py = ((2 * ((y + 0.5) / img_height)) - 1);

				vec3 rayOrigin = cam.position;
				vec3 rayTargetWorld = inverseView * vec4(px, py, -1, 1);
				vec3 rayDir = normalize(rayTargetWorld - rayOrigin);
				
				ray ray(rayOrigin, rayDir);
				rayBuffer->push_back(ray);
			}
		}
	}
	//Looping over all pixels in output image
	for (int i = 0; i < img_width * img_height; i++) {
		renderBuffer[i] = rayBuffer->at(i).traceRay(fc, renderBuffer, lightPos);
	}
}


