#version 410

in vec3 shadeColor;
in vec3 normals;
in vec2 textureCoordinates;

uniform float Alpha;
out vec4 outputColor;
 
void main()
{
	//outputColor = vec4(normals, 1);
	outputColor = vec4(shadeColor, Alpha);
    //outputColor = Color;
}