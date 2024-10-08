
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
#define IMG_RES 128 // 128

#define ABSORBTION 0.005
#define SCATTERING 0.02
#define VISCOSITY 0.5f
#define DIFFUSION 0.5f
float dt = 0.1f;

#define XY(i,j) ((i)+(N*j))
#define XYZ(x,y,z) ((x) + (y) * N + (z) * N * N)


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
bool g_isRightMouseDragging = false;


///////////////////////////////////////////////////////////////////////////////
// Shader programs
///////////////////////////////////////////////////////////////////////////////
GLuint shaderProgram; // Shader used to draw the shadow map

///////////////////////////////////////////////////////////////////////////////
// Test globals
///////////////////////////////////////////////////////////////////////////////


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
int camMoved = 1;
int prevPuff = 0;

sphere light(vec3(0, 5, -32), 4, vec3(4,4, 4));
float light_x = 0;

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

particles::FluidCube* fluidCube;

glm::ivec3 emitterPos = { SIZE / 2, 2, SIZE / 2 };
glm::vec3 emitterDir = { 0.0f, 0.1f, 0.f };
bool emitSmoke = false;
bool wind = false;
bool puffs = false;

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

	for (int i = 0; i < 2; i++) {
		particles::fcAddDensity(cube, emitterPos.x + i, emitterPos.y , emitterPos.z, 18.f);
		particles::fcAddDensity(cube, emitterPos.x, emitterPos.y , emitterPos.z + i, 18.f);
	}

	particles::fcAddVelocity(cube, emitterPos.x, emitterPos.y, emitterPos.z, emitterDir.x, emitterDir.y, emitterDir.z);
	particles::fcAddVelocity(cube, emitterPos.x + 1, emitterPos.y, emitterPos.z, emitterDir.x, emitterDir.y + 0.1f, emitterDir.z);
	particles::fcAddVelocity(cube, emitterPos.x, emitterPos.y + 1, emitterPos.z, emitterDir.x-0.1f, emitterDir.y, emitterDir.z);
	particles::fcAddVelocity(cube, emitterPos.x + 1, emitterPos.y + 1, emitterPos.z, emitterDir.x, emitterDir.y - 0.1f, emitterDir.z);
}

void addPuffs(FluidCube* fc) {
	switch (prevPuff) {
		case 0:
			fcAddDensity(fc, 3, 3, 3, 10.f);
			fcAddDensity(fc, 4, 3, 3, 10.f);
			fcAddDensity(fc, 3, 3, 4, 10.f);
			fcAddDensity(fc, 4, 3, 4, 10.f);

			fcAddVelocity(fc, 3, 3, 3, 0.01f, 0.3f, 0.01f);
			fcAddVelocity(fc, 4, 3, 4, 0.01f, 0.3f, 0.01f);
			prevPuff = (int) currentTime % 4;
			break;
		case 1:
			fcAddDensity(fc, SIZE - 3, 3, 3, 10.f);
			fcAddDensity(fc, SIZE - 4, 3, 3, 10.f);
			fcAddDensity(fc, SIZE - 3, 3, 4, 10.f);
			fcAddDensity(fc, SIZE - 4, 3, 4, 10.f);

			fcAddVelocity(fc, SIZE - 3, 3, 3, -0.01f, 0.3f, 0.01f);
			fcAddVelocity(fc, SIZE - 4, 3, 4, -0.01f, 0.3f, 0.01f);
			prevPuff = (int)currentTime % 4;
			break;
		case 2:
			fcAddDensity(fc, SIZE - 3, 3, SIZE - 3, 10.f);
			fcAddDensity(fc, SIZE - 4, 3, SIZE - 3, 10.f);
			fcAddDensity(fc, SIZE - 3, 3, SIZE - 4, 10.f);
			fcAddDensity(fc, SIZE - 4, 3, SIZE - 4, 10.f);

			fcAddVelocity(fc, SIZE - 3, 3, SIZE - 3, -0.01f, 0.3f, -0.01f);
			fcAddVelocity(fc, SIZE - 4, 3, SIZE - 4, -0.01f, 0.3f, -0.01f);
			prevPuff = (int)currentTime % 4;
			break;
		case 3:
			fcAddDensity(fc, 3, 3, SIZE - 3, 10.f);
			fcAddDensity(fc, 4, 3, SIZE - 3, 10.f);
			fcAddDensity(fc, 3, 3, SIZE - 4, 10.f);
			fcAddDensity(fc, 4, 3, SIZE - 4, 10.f);

			fcAddVelocity(fc, 3, 3, SIZE - 3, 0.01f, 0.3f, -0.01f);
			fcAddVelocity(fc, 4, 3, SIZE - 4, 0.01f, 0.3f, -0.01f);
			prevPuff = (int)currentTime % 4;
			break;
	}
}

void addWind(FluidCube* fc) {
	if ((int)currentTime % 2) {
		for (int x = 1; x < SIZE; x += 2) {
			for (int y = 1; y < SIZE; y += 2) {
				fcAddVelocity(fluidCube, x, y, 1, 0, deltaTime, -2.f);
			}
		}
	}
	else {
		for (int x = 2; x < SIZE; x += 2) {
			for (int y = 2; y < SIZE; y += 2) {
				fcAddVelocity(fluidCube, x, y, 1, 0, -deltaTime, -1.f);
			}
		}
	}
}

void rotateLight(float angle) {
	float x = sin(angle) * -32;
	float z = cos(angle) * -32;
	light.position = vec3(x, 5, z);
}

static void draw_density(particles::FluidCube* fc, vec3* imgBuf, std::vector<ray>* rayBuf)
{
	if (camMoved) {
		cam.move(camAngle, camDistance);
	}
	renderVolume(imgBuf, rayBuf, fc, light, cam, IMG_RES, IMG_RES, camMoved);
	camMoved = 0;
	glBindTexture(GL_TEXTURE_2D, renderTexture);
	glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, IMG_RES, IMG_RES, GL_RGB, GL_FLOAT, imgBuf);
	//glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, N, N, 0, GL_RGB, GL_FLOAT, imgBuf);
	glBindTexture(GL_TEXTURE_2D, 0);
	
}



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
	fluidCube = particles::fcCreate(SIZE, DIFFUSION, VISCOSITY, ABSORBTION, SCATTERING, dt);
	
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
	
	if (emitSmoke) { emitter_driver(fluidCube); }
	if (wind) { addWind(fluidCube); }
	if (puffs) { addPuffs(fluidCube); }

	draw_density(fluidCube, imageBuff, rayBuff);



	glUseProgram(shaderProgram);

	glBindTexture(GL_TEXTURE_2D, renderTexture);
	glBindVertexArray(VAO);

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

			g_prevMouseCoords.x = event.motion.x;
			g_prevMouseCoords.y = event.motion.y;
			
			camAngle -= delta_x / 150.f;
			camMoved = 1;
		}

		if (event.type == SDL_MOUSEBUTTONDOWN && event.button.button == SDL_BUTTON_RIGHT
			&& (!labhelper::isGUIvisible() || !ImGui::GetIO().WantCaptureMouse))
		{
			g_isRightMouseDragging = true;
			int x;
			int y;
			SDL_GetMouseState(&x, &y);
			g_prevMouseCoords.x = x;
			g_prevMouseCoords.y = y;
			
		}

		if (!(SDL_GetMouseState(NULL, NULL) & SDL_BUTTON(SDL_BUTTON_RIGHT)))
		{
			g_isRightMouseDragging = false;
		}

		
		if (event.type == SDL_MOUSEMOTION && g_isRightMouseDragging)
		{
			//// More info at https://wiki.libsdl.org/SDL_MouseMotionEvent
			int delta_x = event.motion.x - g_prevMouseCoords.x;
			int delta_y = event.motion.y - g_prevMouseCoords.y;

			g_prevMouseCoords.x = event.motion.x;
			g_prevMouseCoords.y = event.motion.y;
			
			//float x_vel = delta_x / 640; //Window size
			//float y_vel = delta_y / 640; 
			//vec3 velocity = 5.f * normalize(cam.viewMatrix * vec4(delta_x, delta_y, 0, 1));
			//fcAddVelocity(fluidCube, SIZE / 2, SIZE / 2, SIZE / 2, velocity.x, velocity.y, velocity.z);
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
	ImGui::SameLine();
	ImGui::Checkbox("Smoke Emitter", &emitSmoke);
	ImGui::SameLine();
	ImGui::Checkbox("Add puffs", &puffs);
	ImGui::SameLine();
	ImGui::Checkbox("Add Wind", &wind);
	if (ImGui::SliderFloat("Rotate Light", &light_x, -3.14f, 3.14f)) {
		rotateLight(light_x);
	}
	ImGui::SliderFloat("Emitter dir x ", &emitterDir.x, -0.3f, 0.3f);
	ImGui::SliderFloat("Emitter dir y ", &emitterDir.y, -0.3f, 0.3f);
	ImGui::SliderInt("Emitter x pos", &emitterPos.x, 1, SIZE - 1);
	ImGui::SliderInt("Emitter y pos", &emitterPos.y, 1, SIZE - 1);
	if (ImGui::SliderFloat("Camera Distance", &camDistance, 20, 80)) {
		camMoved = 1;
	}
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
