/** \file dab_vis_body_transform.cpp
*/

#include "dab_vis_body_transform.h"

using namespace dab;
using namespace dab::vis;

glm::vec3 BodyTransform::sPosition(0.0, 0.0, 0.0);
glm::quat BodyTransform::sOrientation(1.0, 0.0, 0.0, 0.0);
glm::vec3 BodyTransform::sScale(1.0, 1.0, 1.0);

BodyTransform::BodyTransform()
	: mPosition(sPosition)
	, mOrientation(sOrientation)
	, mScale(sScale)
{
	updateTransform();
}

BodyTransform::BodyTransform(const BodyTransform& pTransform)
	: mPosition(pTransform.mPosition)
	, mOrientation(pTransform.mOrientation)
	, mScale(pTransform.mScale)
	, mTransform(pTransform.mTransform)
{}

BodyTransform::~BodyTransform()
{}

BodyTransform&
BodyTransform::operator=(const BodyTransform& pTransform)
{
	mPosition = pTransform.mPosition;
	mOrientation = pTransform.mOrientation;
	mScale = pTransform.mScale;
	mTransform = pTransform.mTransform;

	return *this;
}

void
BodyTransform::updateTransform()
{
	mTransform = glm::translate(glm::mat4(1.0), mPosition);
	mTransform = mTransform * glm::toMat4(mOrientation);
	mTransform = glm::scale(mTransform, mScale);
}

const glm::vec3& 
BodyTransform::position() const
{
	return mPosition;
}

const glm::quat& 
BodyTransform::orientation() const
{
	return mOrientation;
}

const glm::vec3& 
BodyTransform::scale() const
{
	return mScale;
}

const glm::mat4& 
BodyTransform::transform() const
{
	return mTransform;
}

const glm::mat4& 
BodyTransform::matrix() const
{
	return mTransform;
}

void 
BodyTransform::setPosition(const glm::vec3& pPosition)
{
	mPosition = pPosition;
	updateTransform();
}

void 
BodyTransform::setOrientation(const glm::quat& pOrientation)
{
	mOrientation = glm::normalize(pOrientation);
	updateTransform();
}

void 
BodyTransform::setScale(const glm::vec3& pScale)
{
	mScale = pScale;
	updateTransform();
}

void 
BodyTransform::setTransform(const glm::mat4& pTransform)
{
	mTransform = pTransform;
}

void 
BodyTransform::setMatrix(const glm::mat4& pTransform)
{
	mTransform = pTransform;
}