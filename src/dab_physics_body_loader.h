/** \file dab_physics_body_loader.h
*/

#pragma once

#include "dab_exception.h"

namespace dab
{

namespace physics
{

class BodyLoader
{
	struct BodyShapeConfig
	{

	};

	struct BodyPartConfig
	{

	};

	struct BodyJointConfig
	{

	};

public:
	virtual void load(const std::string& pFilenName) throw (dab::Exception) = 0;

protected:

};

};

};