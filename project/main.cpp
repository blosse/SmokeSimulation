
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

#include <perf.h>

#include <glm/glm.hpp>
#include <glm/gtx/transform.hpp>
using namespace glm;

#include <Model.h>
#include "hdr.h"
#include "fbo.h"

#define SIZE 32
#define ABSORBTION 0.5f
#define XY(i,j) ((i)+(N*2)*(j))
#define XYZ(x,y,z) ((x) + (y) * N + (z) * N * N)

///////////////////////////////////////////////////////////////////////////////
/*
TODOLIST
-Implement some draw_density type function
-Check out "marker and cell" grid (MAC grid)
-Extend solver to 3D
-Check out and possibly implement "vorticity confinement" 
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

vec3 *imageBuff = new vec3[SIZE*SIZE];

float vertices[] = {
	// positions          // colors           // texture coords
	 0.5f,  0.5f, 0.0f,   1.0f, 0.0f, 0.0f,   1.0f, 1.0f,   // top right
	 0.5f, -0.5f, 0.0f,   0.0f, 1.0f, 0.0f,   1.0f, 0.0f,   // bottom right
	-0.5f, -0.5f, 0.0f,   0.0f, 0.0f, 1.0f,   0.0f, 0.0f,   // bottom left
	-0.5f,  0.5f, 0.0f,   1.0f, 1.0f, 0.0f,   0.0f, 1.0f    // top left 
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

////////////////////////////////////////////////////////////////////////////////
// Generate Grid
////////////////////////////////////////////////////////////////////////////////

void generateGrid(int N, std::vector<glm::vec3> &vertices, std::vector<glm::uvec3> &indices, std::vector<glm::vec3> &colors, std::vector<glm::vec3> &points) {

	std::vector<glm::vec3> verticesTemp;
	for (int i = 0; i <= N; i++) { //Generate vertices
		for (int j = 0; j <= N; j++) {

			float x = ((float)j / (float)N - 0.5f);
			float y = ((float)i / (float)N - 0.5f);
			float z = 1.0f;
			verticesTemp.push_back(glm::vec3(x, y, z));
		}
	}

	for (int i = 0; i < N; i++) {
		for (int j = 0; j < N; j++) {
			int offset_horizontal = i * (N + 1);
			int offset_vertical = (i + 1) * (N + 1);

			int a = (j + offset_horizontal);
			int b = (j + 1 + offset_horizontal);
			int c = (j + offset_vertical);
			indices.push_back(glm::uvec3(a, b, c));
			vertices.push_back(verticesTemp[a]);
			vertices.push_back(verticesTemp[b]);
			vertices.push_back(verticesTemp[c]);

			a = (j + 1 + offset_horizontal);
			b = (j + 1 + offset_vertical);
			c = (j + offset_vertical);
			indices.push_back(glm::uvec3(a, b, c));
			vertices.push_back(verticesTemp[a]);
			vertices.push_back(verticesTemp[b]);
			vertices.push_back(verticesTemp[c]);
		}
	}

	for (int i = 0; i < vertices.size(); i++) {
		colors.push_back(glm::vec3(0.0, 0.0, 0.0));
	}
	
	for (char j = 1; j <= N; j++) {
		for (char i = 1; i <= N; i++) {
			float x = (i * (1 / N));
			float y = (j * (1 / N));
			float z = 1.0;
			points.push_back(glm::vec3(x, y, z));
		}
	}

	/////////////////////////////////////////////////////// Uncomment for debug
	//printf("Calculated vertices:");
	//for (int i = 0; i < vertices.size(); i++) {
	//	if (i % 3 == 0) {
	//		printf("\n");
	//	}
	//	printf("%f\n", vertices[i]);
	//}

	//printf("\nCalculated Indices:");
	//for (int j = 0; j < indices.size(); j++) { //Print calculated indices
	//	for (uint i = 0; i < 3; i++) {
	//		printf("%d ", indices[j][i]);
	//	}
	//	printf("\n");	
	//}
}

void emitter_driver(particles::FluidCube* cube) {
	particles::fcAddDensity(cube, emitterPos.x, emitterPos.y, emitterPos.z, 0.5f);
	particles::fcAddDensity(cube, emitterPos.x + 1, emitterPos.y, emitterPos.z, 0.5f);
	particles::fcAddDensity(cube, emitterPos.x, emitterPos.y + 1, emitterPos.z, 0.5f);
	particles::fcAddDensity(cube, emitterPos.x + 1, emitterPos.y + 1, emitterPos.z, 0.5f);

	particles::fcAddVelocity(cube, emitterPos.x, emitterPos.y, emitterPos.z, emitterDir.x, emitterDir.y, emitterDir.z);
	particles::fcAddVelocity(cube, emitterPos.x + 1, emitterPos.y, emitterPos.z, emitterDir.x, emitterDir.y + 0.1f, emitterDir.z);
	particles::fcAddVelocity(cube, emitterPos.x, emitterPos.y + 1, emitterPos.z, emitterDir.x-0.1f, emitterDir.y, emitterDir.z);
	particles::fcAddVelocity(cube, emitterPos.x + 1, emitterPos.y + 1, emitterPos.z, emitterDir.x, emitterDir.y - 0.1f, emitterDir.z);

	
	particles::fcAddVelocity(fluidCube, 2, SIZE / 2, SIZE / 2, 0.5f, 0.01f, 0.5f);
	particles::fcAddVelocity(fluidCube, SIZE - 5, SIZE / 2 - 5, SIZE / 2 - 5 , -0.4f, -0.3f, 0.5f);
}

static void draw_density(particles::FluidCube* fc, vec3* imgBuf)
{	
	int N = fc->size;
	for (int j = 1; j < N-1; j++) {
		for (int i = 1; i < N-1; i++) {
			float distance = 0; 
			for (int k = 1; k < N-1; k++) { //Loop through all densities along "ray" (z-axis in this case)
				distance += 1 * fc->density[XYZ(i, j, -k)]; // IS -Z INTO OR OUT FROM SCREEN???
				//printf("Density at %d,%d,%d is: %f\n", i, j, k, fc->density[XYZ(i, j, k)]);
			}
			float result = exp(-distance * ABSORBTION); //Beers law??
		}
		imgBuf[XY(j, j)] = vec3(1.f);
	}
	glBindTexture(GL_TEXTURE_2D, renderTexture);
	glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, N, N, GL_RGB, GL_FLOAT, imgBuf);
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

	//for (int i = 0; i < SIZE * SIZE; i++) {
	//	imageBuff[i].x = 1.f;
	//	imageBuff[i].y = 0.f;
	//	imageBuff[i].z = 1.f;
	//}

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
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, SIZE, SIZE, 0, GL_RGB, GL_FLOAT, imageBuff);

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
	fluidCube = particles::fcCreate(SIZE, diff,visc, dt);
	particles::fcAddDensity(fluidCube, 15, 15, (SIZE / 2), 0.5f);
	particles::fcAddDensity(fluidCube, 14, 11, (SIZE / 2), 0.5f);
	particles::fcAddDensity(fluidCube, 14, 11, (SIZE / 2), 0.5f);
	particles::fcAddDensity(fluidCube, 14, 12, (SIZE / 2), 0.5f);
	particles::fcAddDensity(fluidCube, 12, 13, (SIZE / 2), 0.5f);
	particles::fcAddDensity(fluidCube, 12, 14, (SIZE / 2), 0.5f);
	particles::fcAddDensity(fluidCube, 11, 15, (SIZE / 2), 0.5f);
	particles::fcAddDensity(fluidCube, 11, 16, (SIZE / 2), 0.5f);
	particles::fcAddVelocity(fluidCube, 1, 1, 1, -0.2f, 0.5f, 0.5f);
	particles::fcAddVelocity(fluidCube, 2, 1, 1, 0.f, 0.5f, 0.5f);
	particles::fcAddVelocity(fluidCube, 3, 1, 1, 0.2f, 0.5f, 0.5f);

	
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
	
	emitter_driver(fluidCube);
	draw_density(fluidCube, imageBuff);

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
			//dens[XY(x, y)] = source;
		}

		if (!(SDL_GetMouseState(NULL, NULL) & SDL_BUTTON(SDL_BUTTON_LEFT)))
		{
			g_isMouseDragging = false;
		}

		if (event.type == SDL_MOUSEMOTION && g_isMouseDragging)
		{
			//// More info at https://wiki.libsdl.org/SDL_MouseMotionEvent
			//int delta_x = event.motion.x - g_prevMouseCoords.x;
			//int delta_y = event.motion.y - g_prevMouseCoords.y;
			//float rotationSpeed = 0.1f;
			//mat4 yaw = rotate(rotationSpeed * deltaTime * -delta_x, worldUp);
			//mat4 pitch = rotate(rotationSpeed * deltaTime * -delta_y,
			//	normalize(cross(cameraDirection, worldUp)));
			//cameraDirection = vec3(pitch * yaw * vec4(cameraDirection, 0.0f));
			//g_prevMouseCoords.x = event.motion.x;
			//g_prevMouseCoords.y = event.motion.y;
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

	//ImGui::SliderFloat("Diffusion", &diff, 0.0f, 1.0f);
	//ImGui::SliderFloat("Viscosity", &visc, 0.0f, 1.0f);
	//ImGui::SliderInt("Emitter x pos", &emitterPos.x, 1, SIZE - 1);
	//ImGui::SliderInt("Emitter y pos", &emitterPos.y, 1, SIZE - 1);
	//ImGui::SliderFloat("Emitter dir x ", &emitterDir.x, -1.f, 1.f);
	//ImGui::SliderFloat("Emitter dir y ", &emitterDir.y, -1.f, 1.f);
	////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////

	//labhelper::perf::drawEventsWindow();
}

int main(int argc, char* argv[])
{
	g_window = labhelper::init_window_SDL("OpenGL Project", 600, 600);

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
