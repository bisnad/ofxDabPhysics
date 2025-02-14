/** \file dab_physics_behavior.h
*/

#pragma once

#include <string>
#include <vector>
#include <map>
#include <memory>
#include "dab_value.h"
#include "dab_exception.h"

namespace dab
{

namespace physics
{

class Simulation;
class Body;
class BodyPart;
class BodyJoint;
class BodyMotor;

class Behavior
{
public:
	friend class Simulation;

	Behavior(const std::string& pName, const std::vector<std::shared_ptr<BodyPart>>& pParts, const std::vector<std::shared_ptr<BodyJoint>>& pJoints, const std::vector<std::shared_ptr<BodyMotor>>& pMotors);

	//virtual Behavior* copy() const = 0;

	std::shared_ptr<Body> body() const;
	const std::string& name() const;

	const std::vector<std::shared_ptr<BodyPart>> parts() const;
	const std::vector<std::shared_ptr<BodyJoint>> joints() const;
	const std::vector<std::shared_ptr<BodyMotor>> motors() const;
	const std::map<std::string, AbstractValue* > parameters() const;

	virtual void update() = 0;

	template< class Type> void get(const std::string& pParName, Type& pValue) throw (Exception);
	template< class Type> void get(const std::string& pParName, std::vector<Type>& pValues) throw (Exception);
	template< class Type, int Dim> void get(const std::string& pParName, std::array<Type, Dim>& pValues) throw (Exception);

	void set(const std::string& pParName, const AbstractValue& pValue) throw (Exception);
	template< class Type> void set(const std::string& pParName, Type pValue) throw (Exception);
	template< class Type> void set(const std::string& pParName, const std::initializer_list<Type>& pValues) throw (Exception);
	template< class Type> void set(const std::string& pParName, const std::vector<Type>& pValues) throw (Exception);
	template< class Type, int Dim> void set(const std::string& pParName, const std::array<Type, Dim>& pValues) throw (Exception);

protected:
	virtual void notifyParameterChange(const std::string& pParName);

	std::string mName;
	std::shared_ptr<Body> mBody;
	std::vector<std::shared_ptr<BodyPart>> mParts;
	std::vector<std::shared_ptr<BodyJoint>> mJoints;
	std::vector<std::shared_ptr<BodyMotor>> mMotors;

	std::map<std::string, AbstractValue* > mParameters;

	Value<bool> mActive;
};

template< class Type>
void
Behavior::get(const std::string& pParName, Type& pValue) throw (Exception)
{
	try
	{
		pValue = *(mParameters.at(pParName));
	}
	catch (dab::Exception& e)
	{
		throw;
	}
}

template< class Type>
void
Behavior::get(const std::string& pParName, std::vector<Type>& pValues) throw (Exception)
{
	try
	{
		pValues = *(mParameters.at(pParName));
	}
	catch (dab::Exception& e)
	{
		throw;
	}
}

template< class Type, int Dim>
void
Behavior::get(const std::string& pParName, std::array<Type, Dim>& pValues) throw (Exception)
{
	try
	{
		pValues = *(mParameters.at(pParName));
	}
	catch (dab::Exception& e)
	{
		throw;
	}
}

template< class Type >
void
Behavior::set(const std::string& pParName, Type pValue) throw (Exception)
{
	try
	{
		mParameters.at(pParName)->set(pValue);
		notifyParameterChange(pParName);
	}
	catch (dab::Exception& e)
	{
		throw;
	}
}


template< class Type > 
void 
Behavior::set(const std::string& pParName, const std::initializer_list<Type>& pValues) throw (Exception)
{
	try
	{
		set(pParName, std::vector<float>(pValues));
	}
	catch (dab::Exception& e)
	{
		throw;
	}
}

template< class Type >
void
Behavior::set(const std::string& pParName, const std::vector<Type>& pValues) throw (Exception)
{
	if (mParameters.find(pParName) == mParameters.end()) throw dab::Exception("Behavior Error: behavior " + mName + " has no parameter " + pParName, __FILE__, __FUNCTION__, __LINE__);

	//std::cout << "Behavior " << mName << " parameter " << pParName << " values ";
	//for (int vI = 0; vI < pValues.size(); ++vI) std::cout << pValues[vI] << " ";
	//std::cout << "\n";

	try
	{
		int valueCount = pValues.size();

		if (valueCount == 1)  mParameters.at(pParName)->set(pValues[0]);
		if (valueCount == 2)  mParameters.at(pParName)->set( std::array<Type, 2>({ pValues[0], pValues[1] }));
		if (valueCount == 3)  mParameters.at(pParName)->set(std::array<Type, 3>({ pValues[0], pValues[1], pValues[2] }));
		if (valueCount == 4)  mParameters.at(pParName)->set(std::array<Type, 4>({ pValues[0], pValues[1], pValues[2], pValues[3] }));
		if (valueCount == 5)  mParameters.at(pParName)->set(std::array<Type, 5>({ pValues[0], pValues[1], pValues[2], pValues[3], pValues[4] }));
		if (valueCount == 6)  mParameters.at(pParName)->set(std::array<Type, 6>({ pValues[0], pValues[1], pValues[2], pValues[3], pValues[4], pValues[5] }));

		notifyParameterChange(pParName);
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Behavior Error: failed to set parameter " + pParName + " for behavior " + mName, __FILE__, __FUNCTION__, __LINE__);
		throw;
	}
}

template< class Type, int Dim >
void
Behavior::set(const std::string& pParName, const std::array<Type, Dim>& pValues) throw (Exception)
{
	if (mParameters.find(pParName) == mParameters.end()) throw dab::Exception("Behavior Error: behavior " + mName + " has no parameter " + pParName, __FILE__, __FUNCTION__, __LINE__);

	try
	{
		mParameters.at(pParName)->set(pValues);
		notifyParameterChange(pParName);
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Behavior Error: failed to set parameter " + pParName + " for behavior " + mName, __FILE__, __FUNCTION__, __LINE__);
		throw;
	}
}

};

};
