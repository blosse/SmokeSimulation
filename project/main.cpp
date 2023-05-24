
#ifdef _WIN32
extern "C" _declspec(dllexport) unsigned int NvOptimusEnablement = 0x00000001;
#endif

#include <GL/glew.h>
#include <cmath>
#include <cstdlib>
#include <stdlib.h>
#include <algorithm>
#include <chrono>

#include <string>
#include <fstream>
#include <streambuf>
#include <SDL.h>

#include <labhelper.h>
#include <imgui.h>
#include "particles.h"
#include "volRender.h"

#include <perf.h>

#include <glm/glm.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtx/rotate_vector.hpp>
using namespace glm;

#include <Model.h>
#include "hdr.h"
#include "fbo.h"

#define SIZE 32
#define IMG_RES 128
#define ABSORBTION 0.01
#define SCATTERING 0.01
#define XY(i,j) ((i)+(N*j))
#define XYZ(x,y,z) ((x) + (y) * N + (z) * N * N)

///////////////////////////////////////////////////////////////////////////////
/*
TODOLIST
-Somehow implement collision detection, so that smoke can "wrap" around objects (voxelize objects)
*/



///////////////////////////////////////////////////////////////////////////////
// Various globals
///////////////////////////////////////////////////////////////////////////////
SDL_Window* g_window = nullptr;
int windowWidth, windowHeight;
float g_clearColor[3] = { 0.2f, 0.2f, 0.8f };

float currentTime = 0.0f;
float previousTime = 0.0f;
float deltaTime = 0.0f;

//Mouse input
ivec2 g_prevMouseCoords = { -1, -1 };
bool g_isMouseDragging = false;


///////////////////////////////////////////////////////////////////////////////
// Shader programs
///////////////////////////////////////////////////////////////////////////////
GLuint shaderProgram; // Shader used to draw the shadow map

///////////////////////////////////////////////////////////////////////////////
// Test globals
///////////////////////////////////////////////////////////////////////////////
GLuint cubeVAO;
GLuint positionBuffer;
GLuint colorBuffer;
GLuint indexBuffer;
GLuint pointBuffer;

glm::vec3 g_triangleColor = { 1, 1, 1 };
glm::vec3 g_pointPosition = { 0, 0, 0 };

//std::vector<glm::vec3> vertices;
//std::vector<glm::uvec3> indices;
std::vector<glm::vec3> colors;
std::vector<glm::vec3> points;

//////////////////////////////////////////////////////////////////////////////
// Stuff for the volumetric rendering
//////////////////////////////////////////////////////////////////////////////
GLuint framebuffer;
GLuint renderTexture;

GLuint VAO;
GLuint VBO;
GLuint EBO;

vec3 *imageBuff = new vec3[IMG_RES*IMG_RES];
std::vector<ray> *rayBuff = new std::vector<ray>();

camera cam(50.f, 0.f);
float camDistance = 50.f;
float camAngle = 0.f;

float vertices[] = {
	// positions          // colors           // texture coords
	 1.0f,  1.0f, 0.0f,   1.0f, 0.0f, 0.0f,   1.0f, 1.0f,   // top right
	 1.0f, -1.0f, 0.0f,   0.0f, 1.0f, 0.0f,   1.0f, 0.0f,   // bottom right
	-1.0f, -1.0f, 0.0f,   0.0f, 0.0f, 1.0f,   0.0f, 0.0f,   // bottom left
	-1.0f,  1.0f, 0.0f,   1.0f, 1.0f, 0.0f,   0.0f, 1.0f    // top left 
};

unsigned int indices[] = {
	0, 1, 3,   // first triangle
	1, 2, 3
};

//Default values, maybe implement some way to change via GUI
//int N = 9;
float dt = 0.1f, diff = 0.5f, visc = 0.5f;
float force = 5.0f, source = 100.0f;
int dvel = 0; //Draw velocity or not, this is not implemented yet

int grid_res = SIZE;
int grid_size = (grid_res + 1) * (grid_res + 1);

float light_x = 0;
float light_y = 24;

vec3 cameraPosition = { 0, 0, 30 };

vec2 prevCam = { 0, 0 };
vec2 newCam = { 0, 30 }; //x = angle y = distance
int camMoved = 1;


particles::FluidCube* fluidCube;
glm::ivec3 emitterPos = { SIZE / 2, 4, SIZE / 2 };
glm::vec3 emitterDir = { 0.0f, 0.5f, 0.f };

///////////////////////////////////////////////////////////////////////////////
// Models
///////////////////////////////////////////////////////////////////////////////

void loadShaders(bool is_reload)
{
	GLuint shader = labhelper::loadShaderProgram("../project/simple.vert", "../project/simple.frag", is_reload);
	if(shader != 0)
	{
		shaderProgram = shader;
	}
}

void emitter_driver(particles::FluidCube* cube) {

	for (int i = 0; i < 10; i++) {
		particles::fcAddDensity(cube, emitterPos.x + i, emitterPos.y , emitterPos.z, 10.f);
		particles::fcAddDensity(cube, emitterPos.x, emitterPos.y , emitterPos.z + i, 10.f);
	}

	particles::fcAddVelocity(cube, emitterPos.x, emitterPos.y, emitterPos.z, emitterDir.x, emitterDir.y, emitterDir.z);
	particles::fcAddVelocity(cube, emitterPos.x + 1, emitterPos.y, emitterPos.z, emitterDir.x, emitterDir.y + 0.1f, emitterDir.z);
	particles::fcAddVelocity(cube, emitterPos.x, emitterPos.y + 1, emitterPos.z, emitterDir.x-0.1f, emitterDir.y, emitterDir.z);
	particles::fcAddVelocity(cube, emitterPos.x + 1, emitterPos.y + 1, emitterPos.z, emitterDir.x, emitterDir.y - 0.1f, emitterDir.z);

	
	particles::fcAddVelocity(fluidCube, 2, SIZE / 2, SIZE / 2, 0.5f, 0.01f, 0.5f);
	particles::fcAddVelocity(fluidCube, SIZE - 5, SIZE / 2 - 5, SIZE / 2 - 5 , -0.4f, -0.3f, 0.5f);
}

static void draw_density(particles::FluidCube* fc, vec3* imgBuf, std::vector<ray> *rayBuf)
{	
	if (camMoved) {
		cam.move(camAngle, camDistance);
	}
	renderVolume(imgBuf, rayBuf, fc, vec3(light_x, light_y, 5.f), cam, IMG_RES, IMG_RES, camMoved);
	camMoved = 0;
	glBindTexture(GL_TEXTURE_2D, renderTexture);
	glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, IMG_RES, IMG_RES, GL_RGB, GL_FLOAT, imgBuf);
	//glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, N, N, 0, GL_RGB, GL_FLOAT, imgBuf);
	glBindTexture(GL_TEXTURE_2D, 0);
	
}

//vec3 insane_ray_trace(float* densityField, int size, vec3 rayOrigin, vec3 rayDir) {
//	
//}



///////////////////////////////////////////////////////////////////////////////
/// This function is called once at the start of the program and never again
///////////////////////////////////////////////////////////////////////////////
void initialize()
{
	ENSURE_INITIALIZE_ONLY_ONCE();

	//Init the image buffer w/ gray
	for (int i = 0; i < SIZE * SIZE; i++) {
		imageBuff[i].x = 0.1f;
		imageBuff[i].y = 0.1f;
		imageBuff[i].z = 0.1f;
		//imageBuff[i].w = 1.f;
	}

	///////////////////////////////////////////////////////////////////////
	//		Load Shaders
	///////////////////////////////////////////////////////////////////////
	loadShaders(false);

	//glGenFramebuffers(1, &framebuffer);
	//glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
	glGenTextures(1, &renderTexture);
	glBindTexture(GL_TEXTURE_2D, renderTexture);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, IMG_RES, IMG_RES, 0, GL_RGB, GL_FLOAT, imageBuff);

	glGenVertexArrays(1, &VAO);
	glBindVertexArray(VAO);

	glGenBuffers(1, &VBO);
	glGenBuffers(1, &EBO);
	
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_DYNAMIC_DRAW);
	
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
	glEnableVertexAttribArray(1);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*) 0);
	glEnableVertexAttribArray(0);


	//glEnable(GL_DEPTH_TEST); // enable Z-buffering
	glDisable(GL_CULL_FACE);
	glEnable(GL_PROGRAM_POINT_SIZE);

	///////////////////////////////////////////////////////////////////////////
	//init simulation cube 
	fluidCube = particles::fcCreate(SIZE, diff, visc, ABSORBTION, SCATTERING, dt);
	
	particles::fcAddVelocity(fluidCube, 1, 1, 1, -0.2f, 0.5f, 0.5f);
	particles::fcAddVelocity(fluidCube, 2, 1, 1, 0.f, 0.5f, 0.5f);
	particles::fcAddVelocity(fluidCube, 3, 1, 1, 0.2f, 0.5f, 0.5f);

	//Add some initial density
	//for (int z = 0; z < (fluidCube->size) / 2; z++) {
	//	for (int y = 0; y < (fluidCube->size / 2); y++) {
	//		for (int x = 0; x < fluidCube->size / 2; x++) {
	//		particles::fcAddDensity(fluidCube, x + 8, y + 8, z + 8, 10.f);
	//		}
	//
	//	}
	//}
	//ivec3 density_cloud = { 3, SIZE - 6, SIZE - 6 };
	//for (int z = 0; z < 4; z++) {
	//	for (int y = 0; y < 4; y++) {
	//		for (int x = 0; x < 4; x++) {
	//			particles::fcAddDensity(fluidCube, density_cloud.x, density_cloud.y, density_cloud.z, 0.8f);
	//		}
	//	}
	//}
	//
	//density_cloud = { SIZE - 10, SIZE - 16, SIZE / 2 };
	//for (int z = 0; z < 6; z++) {
	//	for (int y = 0; y < 6; y++) {
	//		for (int x = 0; x < 6; x++) {
	//			particles::fcAddDensity(fluidCube, density_cloud.x, density_cloud.y, density_cloud.z, 0.8f);
	//		}
	//	}
	//}

	//Add initial velocity
	for (int z = 4; z < SIZE - 6 ; z += 4) {
		particles::fcAddVelocity(fluidCube, SIZE / 2, z   , SIZE / 2, 1.f, 0.f, 0.f);
		particles::fcAddVelocity(fluidCube, SIZE / 2, z+ 1, SIZE / 2, 0.f, 0.f, 1.f);
		particles::fcAddVelocity(fluidCube, SIZE / 2, z+ 2, SIZE / 2, -1.f, 0.f, 0.f);
		particles::fcAddVelocity(fluidCube, SIZE / 2, z+ 3, SIZE / 2, 0.f, 0.f, -1.f);
	}
}

///////////////////////////////////////////////////////////////////////////////
/// This function will be called once per frame, so the code to set up
/// the scene for rendering should go here
///////////////////////////////////////////////////////////////////////////////
void display(void)
{
	//labhelper::perf::Scope s("Display");

	///////////////////////////////////////////////////////////////////////////
	// Check if window size has changed and resize buffers as needed
	///////////////////////////////////////////////////////////////////////////
	int w, h;
	SDL_GetWindowSize(g_window, &w, &h);
	glViewport(0, 0, w, h); // Set viewport
	
	glClearColor(0.1f, 0.1f, 0.1f, 0.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	//for (int z = 0; z < (fluidCube->size) / 2; z++) {
	//	for (int x = 0; x < (fluidCube->size / 2); x++) {
	//		particles::fcAddDensity(fluidCube, x + (fluidCube->size / 4), 5, z + (fluidCube->size / 4), 0.1f);
	//		particles::fcAddDensity(fluidCube, x + (fluidCube->size / 4), 6, z + (fluidCube->size / 4), 0.1f);
	//		particles::fcAddDensity(fluidCube, x + (fluidCube->size / 4), 7, z + (fluidCube->size / 4), 0.1f);
	//	}
	//}
	emitter_driver(fluidCube);
	draw_density(fluidCube, imageBuff, rayBuff);

	glUseProgram(shaderProgram);

	glBindTexture(GL_TEXTURE_2D, renderTexture);
	glBindVertexArray(VAO);
	//labhelper::setUniformSlow(shaderProgram, "color_uni", glm::vec3(0.1,0.1,0.3));
	//glDrawArrays(GL_TRIANGLES, 0, 6 * SIZE * SIZE);
	glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

	glUseProgram(0);
}


///////////////////////////////////////////////////////////////////////////////
/// This function is used to update the scene according to user input
///////////////////////////////////////////////////////////////////////////////
bool handleEvents(void)
{
	// check events (keyboard among other)
	SDL_Event event;
	bool quitEvent = false;
	while(SDL_PollEvent(&event))
	{
		labhelper::processEvent( &event );

		if(event.type == SDL_QUIT || (event.type == SDL_KEYUP && event.key.keysym.sym == SDLK_ESCAPE))
		{
			quitEvent = true;
		}
		if(event.type == SDL_KEYUP && event.key.keysym.sym == SDLK_g)
		{
			if ( labhelper::isGUIvisible() )
			{
				labhelper::hideGUI();
			}
			else
			{
				labhelper::showGUI();
			}
		}
		if (event.type == SDL_MOUSEBUTTONDOWN && event.button.button == SDL_BUTTON_LEFT
			&& (!labhelper::isGUIvisible() || !ImGui::GetIO().WantCaptureMouse))
		{
			g_isMouseDragging = true;
			int x;
			int y;
			SDL_GetMouseState(&x, &y);
			g_prevMouseCoords.x = x;
			g_prevMouseCoords.y = y;
			printf("Click (%d, %d)\n", x, 640 - y);

		}

		if (!(SDL_GetMouseState(NULL, NULL) & SDL_BUTTON(SDL_BUTTON_LEFT)))
		{
			g_isMouseDragging = false;
		}

		if (event.type == SDL_MOUSEMOTION && g_isMouseDragging)
		{
			//// More info at https://wiki.libsdl.org/SDL_MouseMotionEvent
			int delta_x = event.motion.x - g_prevMouseCoords.x;
			int delta_y = event.motion.y - g_prevMouseCoords.y;
			//float rotationSpeed = 0.1f;
			//mat4 yaw = rotate(rotationSpeed * deltaTime * -delta_x, worldUp);
			//mat4 pitch = rotate(rotationSpeed * deltaTime * -delta_y,
			//	normalize(cross(cameraDirection, worldUp)));
			//cameraDirection = vec3(pitch * yaw * vec4(cameraDirection, 0.0f));
			g_prevMouseCoords.x = event.motion.x;
			g_prevMouseCoords.y = event.motion.y;
			int x_view = g_prevMouseCoords.x;
			int y_view = (640 - g_prevMouseCoords.y);

			int x_grid = floor(SIZE * x_view / 640);
			int y_grid = floor(SIZE * y_view / 640);

			fcAddDensity(fluidCube, x_grid, y_grid, SIZE / 2, 1.f);
			fcAddDensity(fluidCube, x_grid - 1 , y_grid, SIZE / 2, 1.f);
			fcAddDensity(fluidCube, x_grid + 1, y_grid, SIZE / 2, 1.f);
			fcAddDensity(fluidCube, x_grid, y_grid+1, SIZE / 2, 1.f);
			fcAddDensity(fluidCube, x_grid, y_grid-1, SIZE / 2, 1.f);
			fcAddDensity(fluidCube, x_grid, y_grid, SIZE / 2 + 1 , 1.f);
			fcAddDensity(fluidCube, x_grid, y_grid, SIZE / 2 - 1 , 1.f);
		}

		if (event.type == SDL_MOUSEBUTTONDOWN && event.button.button == SDL_BUTTON_RIGHT
			&& (!labhelper::isGUIvisible() || !ImGui::GetIO().WantCaptureMouse))
		{
			g_isMouseDragging = true;
			int x;
			int y;
			SDL_GetMouseState(&x, &y);
			g_prevMouseCoords.x = x;
			g_prevMouseCoords.y = y;
			//printf("Click (%d, %d)\n", x, 640 - y);
			
			fcAddVelocity(fluidCube, SIZE / 2, SIZE / 2 + 1 ,SIZE / 2, 0, 10.f, 0.f);
			fcAddVelocity(fluidCube, SIZE / 2, SIZE / 2 - 1, SIZE / 2, 0.f, -10.f, 0.f);
			fcAddVelocity(fluidCube, SIZE / 2 + 1, SIZE / 2, SIZE / 2, 10.f, 0.f, 0.f);
			fcAddVelocity(fluidCube, SIZE / 2 - 1, SIZE / 2, SIZE / 2, -10.f, 0.f, 0.f);
			fcAddVelocity(fluidCube, SIZE / 2, SIZE / 2, SIZE / 2 + 1, 0.f, 0.f, 10.f);
			fcAddVelocity(fluidCube, SIZE / 2, SIZE / 2, SIZE / 2 - 1, 0.f, 0.f, -5.f);

		}

	}
	return quitEvent;
}


///////////////////////////////////////////////////////////////////////////////
/// This function is to hold the general GUI logic
///////////////////////////////////////////////////////////////////////////////
void gui()
{
	// ----------------- Set variables --------------------------
	ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate,
	            ImGui::GetIO().Framerate);
	// ----------------------------------------------------------

	if (ImGui::Button("Clear smoke")) {
		fcClearDensity(fluidCube);
	}
	if (ImGui::SliderFloat("Camera Distance", &camDistance, 20, 80)) {
		camMoved = 1;
	}
	if (ImGui::SliderFloat("Rotate Cube", &camAngle, -3.14, 3.14)) {
		camMoved = 1;
	}
	ImGui::SliderFloat("Light x", &light_x, -50.f, 50.0f);
	ImGui::SliderFloat("Light y", &light_y, -50.f, 50.0f);
	ImGui::SliderInt("Emitter x pos", &emitterPos.x, 1, SIZE - 1);
	ImGui::SliderInt("Emitter y pos", &emitterPos.y, 1, SIZE - 1);
	ImGui::SliderFloat("Emitter dir x ", &emitterDir.x, -1.f, 1.f);
	ImGui::SliderFloat("Emitter dir y ", &emitterDir.y, -1.f, 1.f);
	////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////

	//labhelper::perf::drawEventsWindow();
}

int main(int argc, char* argv[])
{
	g_window = labhelper::init_window_SDL("OpenGL Project", 640, 640);

	initialize();

	bool stopRendering = false;
	auto startTime = std::chrono::system_clock::now();

	while(!stopRendering)
	{
		//update currentTime
		std::chrono::duration<float> timeSinceStart = std::chrono::system_clock::now() - startTime;
		previousTime = currentTime;
		currentTime = timeSinceStart.count();
		deltaTime = currentTime - previousTime;

		// check events (keyboard among other)
		stopRendering = handleEvents();

		// Inform imgui of new frame
		labhelper::newFrame( g_window );

		//Drive the simulation
		particles::fluidCubeStep(fluidCube);

		// render to window
		display();

		// Render overlay GUI.
		gui();

		// Finish the frame and render the GUI
		labhelper::finishFrame();

		// Swap front and back buffer. This frame will now been displayed.
		SDL_GL_SwapWindow(g_window);
	}

	particles::fcFree(fluidCube);

	// Shut down everything. This includes the window and all other subsystems.
	labhelper::shutDown(g_window);
	return 0;
}
