/** \file dab_vis_body_part.cpp
*/

#include "dab_vis_body_part.h"
#include "dab_physics_body_part.h"
#include "dab_vis_body_shape.h"

using namespace dab;
using namespace dab::vis;

BodyPart::BodyPart(std::shared_ptr<physics::BodyPart> pPhysicsBodyPart, std::shared_ptr<BodyShape> pBodyShape)
	: mPhysicsBodyPart(pPhysicsBodyPart)
	, mBodyShape(pBodyShape)
	, mBody(nullptr)
{
	init();
}

BodyPart::~BodyPart()
{}

void
BodyPart::init()
{}

std::shared_ptr<Body>
BodyPart::body() const
{
	return mBody;
}

std::shared_ptr<physics::BodyPart>
BodyPart::physicsBodyPart()
{
	return mPhysicsBodyPart;
}

std::shared_ptr<BodyShape>
BodyPart::bodyShape()
{
	return mBodyShape;
}

const BodyTransform& 
BodyPart::transform() const
{
	return mTransform;
}

const glm::mat4& 
BodyPart::transformMatrix() const
{
	return mTransform.matrix();
}

const std::vector<glm::mat4> 
BodyPart::shapeTransformMatrices() const
{
	std::vector<glm::mat4> _shapeTransformMatrices;
	
	const glm::mat4& partTransform = mTransform.matrix();
	const glm::mat4& shapeTransform = mBodyShape->transform().matrix();


	if (std::dynamic_pointer_cast<BodyCompoundShape>(mBodyShape) == nullptr)
	{
		glm::mat4 _shapeTransformMatrix = partTransform * shapeTransform;
		_shapeTransformMatrices.push_back(_shapeTransformMatrix);
	}
	else
	{
		std::shared_ptr<BodyCompoundShape> _cshape = std::dynamic_pointer_cast<BodyCompoundShape>(mBodyShape);
		const std::vector< std::shared_ptr<BodyShape> >& _childShapes = _cshape->childShapes();
		const std::vector< BodyTransform >& _childTransforms = _cshape->childTransforms();

		int childCount = _childShapes.size();

		for (int cI = 0; cI < childCount; ++cI)
		{
			glm::mat4 _shapeTransformMatrix = partTransform * shapeTransform * _childTransforms[cI].matrix() * _childShapes[cI]->transform().matrix();
			_shapeTransformMatrices.push_back(_shapeTransformMatrix);
		}
	}

	return _shapeTransformMatrices;
}

void 
BodyPart::update()
{
	const glm::mat4& physicsMat = mPhysicsBodyPart->matrix();

	mTransform.setMatrix(physicsMat);
}

void
BodyPart::draw(ofShader& pShader)
{
	mBodyShape->drawFaces(mTransform.matrix(), pShader);
}

void
BodyPart::drawFaces(ofShader& pShader)
{
	mBodyShape->drawFaces(mTransform.matrix(), pShader);
}

void
BodyPart::drawWireframe(ofShader& pShader)
{
	mBodyShape->drawWireframe(mTransform.matrix(), pShader);
}