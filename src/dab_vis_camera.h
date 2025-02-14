/** \file dab_vis_camera.h
*/

#pragma once

#include "ofVectorMath.h"
#include "dab_vis_body_transform.h"

namespace dab
{

namespace vis
{

class Camera
{
public:
	Camera();
	~Camera();

	const glm::vec3& position() const;
	const glm::vec3& rotation() const;
	const glm::vec4& projection() const;

	void setPosition(const glm::vec3& pPosition);
	void setRotation(const glm::vec3& pRotation);
	void setProjection(glm::vec4& pProjection);

protected:
	static glm::vec3 sPosition;
	static glm::vec3 sRotation;
	static glm::vec4 sProjection;

	glm::vec3 mPosition;
	glm::vec3 mRotation;
	glm::vec4 mProjection;
};

};

};