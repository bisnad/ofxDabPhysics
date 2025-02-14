/** \file dab_vis_body_part.h
*/

#pragma once

#include <memory>
#include "dab_vis_body_transform.h"
#include "ofShader.h"

namespace dab
{

namespace physics
{
	class BodyPart;
};

namespace vis
{

class Body;
class BodyShape;
class Material;

class BodyPart
{
public:
	friend class BodyVisualization;

	BodyPart(std::shared_ptr<physics::BodyPart> pPhysicsBodyPart, std::shared_ptr<BodyShape> pBodyShape);
	~BodyPart();

	std::shared_ptr<Body> body() const;
	std::shared_ptr<physics::BodyPart> physicsBodyPart();
	std::shared_ptr<BodyShape> bodyShape();

	const BodyTransform& transform() const;
	const glm::mat4& transformMatrix() const;
	const std::vector<glm::mat4> shapeTransformMatrices() const;

	void update();

	void draw(ofShader& pShader);
	void drawFaces(ofShader& pShader);
	void drawWireframe(ofShader& pShader);

protected:
	std::shared_ptr<Body> mBody;
	std::shared_ptr<physics::BodyPart> mPhysicsBodyPart;
	std::shared_ptr<BodyShape> mBodyShape;

	BodyTransform mTransform;

	void init();
};

};

}