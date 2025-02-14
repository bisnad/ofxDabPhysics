/** \file dab_com_osc_control.h
*/

#pragma once

#include "dab_singleton.h"
#include "dab_osc_receiver.h"
#include "dab_osc_sender.h"
#include "dab_index_map.h"

namespace dab
{ 

namespace com
{

class PhysicsOscControl;
class VisualsOscControl;

class OscControl : public Singleton<OscControl>
{
public:
	OscControl();
	~OscControl();

	std::shared_ptr<OscSender> sender(const std::string& pSenderName) throw (dab::Exception);
	std::shared_ptr<OscReceiver> receiver(const std::string& pReceiverName) throw (dab::Exception);

	void createSender(const std::string& pSenderName, const std::string& pIp, int pPort) throw (dab::Exception);
	void createReceiver(const std::string& pReceiverName, unsigned int pPort) throw (dab::Exception);

	void createPhysicsControl(const std::string& pReceiverName) throw (dab::Exception);
	void createVisualsControl(const std::string& pReceiverName) throw (dab::Exception);

	void update();

protected:
	IndexMap<std::string, std::shared_ptr<OscReceiver>> mReceivers;
	IndexMap<std::string, std::shared_ptr<OscSender>> mSenders;

	std::shared_ptr<PhysicsOscControl> mPhysicsControl;
	std::shared_ptr<VisualsOscControl> mVisualsControl;
};

};

};
