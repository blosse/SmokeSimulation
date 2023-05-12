#version 420

layout(location = 0) in vec3 in_position;
layout(location = 1) in vec2 in_tex_coords;
//layout(location = 1) in vec3 in_color;

uniform vec3 color_uni = vec3(0,0,0);

// Task 3: Add an output variable to pass colors to the fragment shader
//out vec3 color;
out vec3 color;
out vec2 texCoord;

void main()
{
	gl_Position = vec4(in_position, 1.0);
	//color = in_color;
	texCoord = in_tex_coords;
}