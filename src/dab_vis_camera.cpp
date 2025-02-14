/** \file dab_vis_camera.cpp
*/

#include "dab_vis_camera.h"

using namespace dab;
using namespace dab::vis;

glm::vec3 Camera::sPosition = glm::vec3(0.0, 0.0, -2.0);
glm::vec3 Camera::sRotation = glm::vec3(180.0, 0.0, 0.0);
glm::vec4 Camera::sProjection = glm::vec4(10.0, 1.0, 0.1, 200.0);

Camera::Camera()
	: mPosition(sPosition)
	, mRotation(sRotation)
	, mProjection(sProjection)
{}

Camera::~Camera()
{}

const glm::vec3& 
Camera::position() const
{
	return mPosition;
}

const glm::vec3& 
Camera::rotation() const
{
	return mRotation;
}

const glm::vec4& 
Camera::projection() const
{
	return mProjection;
}

void 
Camera::setPosition(const glm::vec3& pPosition)
{
	mPosition = pPosition;
}

void 
Camera::setRotation(const glm::vec3& pRotation)
{
	mRotation = pRotation;
}

void 
Camera::setProjection(glm::vec4& pProjection)
{
	mProjection = pProjection;
}