#version 420

layout(location = 0) in vec3 in_position;
layout(location = 1) in vec3 in_color;
layout(location = 2) in vec3 in_points;
//layout(location = 1) in vec3 in_color;

uniform vec3 color_uni = vec3(0,0,0);

// Task 3: Add an output variable to pass colors to the fragment shader
out vec3 color;

void main()
{
	gl_Position = vec4(in_position, 1.0);
	gl_PointSize = 5.0;

	// Task 3: Set the color variable to the vertex color
	color = in_color;
}