/** \file dab_com_osc_control.cpp
*/

#include "dab_com_osc_control.h"
#include "dab_com_physics_osc_control.h"
#include "dab_com_visuals_osc_control.h"

using namespace dab;
using namespace dab::com;

OscControl::OscControl()
	: mPhysicsControl(nullptr)
	, mVisualsControl(nullptr)
{}

OscControl::~OscControl()
{}

std::shared_ptr<OscSender> 
OscControl::sender(const std::string& pSenderName) throw (dab::Exception)
{
	if (mSenders.contains(pSenderName) == false) throw dab::Exception("Osc Error: Sender " + pSenderName + " not found", __FILE__, __FUNCTION__, __LINE__);

	return mSenders[pSenderName];
}

std::shared_ptr<OscReceiver> 
OscControl::receiver(const std::string& pReceiverName) throw (dab::Exception)
{
	if (mReceivers.contains(pReceiverName) == false) throw dab::Exception("Osc Error: Receiver " + pReceiverName + " not found", __FILE__, __FUNCTION__, __LINE__);

	return mReceivers[pReceiverName];
}

void 
OscControl::createSender(const std::string& pSenderName, const std::string& pIp, int pPort) throw (dab::Exception)
{
	if (mSenders.contains(pSenderName) == true) throw dab::Exception("Osc Error: Sender " + pSenderName + " already exists", __FILE__, __FUNCTION__, __LINE__);

	try
	{
		mSenders.add(pSenderName, std::shared_ptr<OscSender>(new OscSender("pSenderName", pIp, pPort)));
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Osc Error: Failed to create sender " + pSenderName, __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

void 
OscControl::createReceiver(const std::string& pReceiverName, unsigned int pPort) throw (dab::Exception)
{
	if (mReceivers.contains(pReceiverName) == true) throw dab::Exception("Osc Error: Receiver " + pReceiverName + " already exists", __FILE__, __FUNCTION__, __LINE__);

	try
	{
		mReceivers.add(pReceiverName, std::shared_ptr<OscReceiver>(new OscReceiver(pReceiverName, pPort)));
		mReceivers[pReceiverName]->start();
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Osc Error: Failed to create receiver " + pReceiverName, __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

void 
OscControl::createPhysicsControl(const std::string& pReceiverName) throw (dab::Exception)
{
	if (mPhysicsControl != nullptr) throw dab::Exception("Osc Error: PhysicsControl already exists", __FILE__, __FUNCTION__, __LINE__);
	if (mReceivers.contains(pReceiverName) == false) throw dab::Exception("Osc Error: Receiver " + pReceiverName + " doesn't exists", __FILE__, __FUNCTION__, __LINE__);

	mPhysicsControl = std::shared_ptr<PhysicsOscControl>(new PhysicsOscControl());
	mReceivers[pReceiverName]->registerOscListener(std::weak_ptr<PhysicsOscControl>(mPhysicsControl));
}

void 
OscControl::createVisualsControl(const std::string& pReceiverName) throw (dab::Exception)
{
	if (mVisualsControl != nullptr) throw dab::Exception("Osc Error: VisualsControl already exists", __FILE__, __FUNCTION__, __LINE__);
	if (mReceivers.contains(pReceiverName) == false) throw dab::Exception("Osc Error: Receiver " + pReceiverName + " doesn't exists", __FILE__, __FUNCTION__, __LINE__);

	mVisualsControl = std::shared_ptr<VisualsOscControl>(new VisualsOscControl());
	mReceivers[pReceiverName]->registerOscListener(std::weak_ptr<VisualsOscControl>(mVisualsControl));
}

void 
OscControl::update()
{
	if (mPhysicsControl != nullptr) mPhysicsControl->update();
	if (mVisualsControl != nullptr) mVisualsControl->update();
}