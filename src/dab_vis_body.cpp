/** \file dab_vis_body.cpp
*/

#include "dab_vis_body.h"
#include "dab_vis_body_part.h"

using namespace dab;
using namespace dab::vis;

Body::Body(std::shared_ptr<physics::Body> pPhysicsBody)
	: mPhysicsBody(pPhysicsBody)
{}

Body::~Body()
{}

std::shared_ptr<physics::Body> 
Body::physicsBody()
{
	return mPhysicsBody;
}

bool 
Body::partExists(const std::string& pPartName)
{
	return mParts.contains(pPartName);
}

std::shared_ptr<BodyPart> 
Body::part(const std::string& pPartName) throw (dab::Exception)
{
	if (mParts.contains(pPartName) == false) throw dab::Exception("Vis Error: body part " + pPartName + " not found", __FILE__, __FUNCTION__, __LINE__);

	return mParts[pPartName];
}