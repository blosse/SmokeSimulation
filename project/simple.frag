#version 420

// required by GLSL spec Sect 4.5.3 (though nvidia does not, amd does)
precision highp float;

// Task 3: Add an input variable for colors from the vertex shader

//in vec3 color;
in vec2 texCoord;

layout(location = 0) out vec4 fragmentColor;

uniform sampler2D renderTex;
//uniform vec3 triangleColor = vec3(1, 1, 1);

void main()
{
// Task 3: Set the output color to be the incoming interpolated color received
	//fragmentColor.rgb = color; //* triangleColor;
	fragmentColor = texture(renderTex, texCoord);
}	
