/** \file dab_vis_body.h
*/

#pragma once

#include "dab_index_map.h"

namespace dab
{

namespace physics
{
	class Body;
};

namespace vis
{

class BodyPart;

class Body
{
public:
	friend class BodyVisualization;

	Body(std::shared_ptr<physics::Body> pPhysicsBody);
	~Body();

	std::shared_ptr<physics::Body> physicsBody();

	bool partExists(const std::string& pPartName);
	std::shared_ptr<BodyPart> part(const std::string& pPartName) throw (dab::Exception);

protected:
	std::shared_ptr<physics::Body> mPhysicsBody;

	IndexMap< std::string, std::shared_ptr<BodyPart> > mParts;
};

};

};