#version 410

// these are for the programmable pipeline system and are passed in
// by default from OpenFrameworks
layout(location = 0) in vec4 vertexPosition_ModelSpace;
layout(location = 1) in vec4 vertexColors;
layout(location = 2) in vec3 vertexNormal_ModelSpace;
layout(location = 3) in vec2 vertexTextureCoordinates;

out vec3 shadeColor;
out vec3 normal;
out vec2 textureCoordinates;

uniform mat4 ProjectionMatrix;
uniform mat4 ModelMatrix;
uniform mat4 ViewMatrix;

// light and color stuff
uniform float Alpha;
uniform vec3 LightPosition;
uniform vec3 AmbientColor;
uniform vec3 DiffuseColor;
uniform float AmbientScale;
uniform float DiffuseScale;
uniform float SpecularScale;
uniform float SpecularPow;

void main()
{
	gl_Position =  ProjectionMatrix * ViewMatrix * ModelMatrix * vertexPosition_ModelSpace;
	textureCoordinates = vertexTextureCoordinates;

	// light and color stuff
	// surface normal in eye coordinates
	normal = normalize((ViewMatrix * ModelMatrix * vec4(vertexNormal_ModelSpace,0)).xyz);  // Only correct if ModelMatrix does not scale the model ! Use its inverse transpose if not.
	// vertex position in eye coordinates
	vec4 vPos4 = vec4(vertexPosition_ModelSpace.xyz, 1.0);
	vec4 vEyePos4 = ViewMatrix * ModelMatrix * vPos4;
	vec3 vEyePos3 = vEyePos4.xyz / vEyePos4.w;
	// get vector to light source
	vec3 lightDir = normalize(LightPosition - vEyePos3);

	// ambient color
	shadeColor = AmbientColor * AmbientScale;

	// diffuse color
	float diff = max(0.0, dot(normal, lightDir));
	diff = sqrt(sqrt(diff));
	shadeColor += diff * DiffuseColor * DiffuseScale;

	// specular color
	vec3 vReflection = normalize(reflect(-lightDir, normal));
	float spec = max(0.0, dot(normal, vReflection));
	if(diff != 0.0) // if diffuse light is zero, don't even bother with the pow function
	{
		float fSpec = pow(spec, SpecularPow); 
		shadeColor += vec3(fSpec, fSpec, fSpec) * SpecularScale;
	}
}