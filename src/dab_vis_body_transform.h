/** \file dab_vis_body_transform.h
*/

#pragma once

#include "ofVectorMath.h"

namespace dab
{

namespace vis
{
	
class BodyTransform
{
public:
	BodyTransform();
	BodyTransform(const BodyTransform& pTransform);
	~BodyTransform();

	BodyTransform& operator=(const BodyTransform& pTransform);

	const glm::vec3& position() const;
	const glm::quat& orientation() const;
	const glm::vec3& scale() const;
	const glm::mat4& transform() const;
	const glm::mat4& matrix() const;

	void setPosition(const glm::vec3& pPosition);
	void setOrientation(const glm::quat& pOrientation);
	void setScale(const glm::vec3& pScale);
	void setTransform(const glm::mat4& pTransform);
	void setMatrix(const glm::mat4& pTransform);

protected:
	static glm::vec3 sPosition;
	static glm::quat sOrientation;
	static glm::vec3 sScale;

	glm::vec3 mPosition;
	glm::quat mOrientation;
	glm::vec3 mScale;
	glm::mat4 mTransform;

	void updateTransform();
};

};

};