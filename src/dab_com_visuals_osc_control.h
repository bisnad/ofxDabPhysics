/** \file dab_com_physics_osc_control.h
*/

#pragma once

#include "dab_singleton.h"
#include "dab_osc_receiver.h"
#include <mutex>

namespace dab
{

	namespace com
	{

		class VisualsOscControl : public OscListener
		{
		public:
			VisualsOscControl();
			~VisualsOscControl();

			void notify(std::shared_ptr<OscMessage> pMessage);
			void update();
			void update(std::shared_ptr<OscMessage> pMessage);

		protected:
			unsigned int mMaxMessageQueueLength = 2048;
			std::deque< std::shared_ptr<OscMessage> > mMessageQueue;

			// camera control
			void setCameraProjection(const std::vector<_OscArg*>& pArgs) throw (dab::Exception);
			void setCameraPosition(const std::vector<_OscArg*>& pArgs) throw (dab::Exception);
			void setCameraRotation(const std::vector<_OscArg*>& pArgs) throw (dab::Exception);
			void setShapeTransparency(const std::vector<_OscArg*>& pArgs) throw (dab::Exception);
			void setShapeAmbientScale(const std::vector<_OscArg*>& pArgs) throw (dab::Exception);
			void setShapeDiffuseScale(const std::vector<_OscArg*>& pArgs) throw (dab::Exception);
			void setShapeSpecularScale(const std::vector<_OscArg*>& pArgs) throw (dab::Exception);
			void setShapeSpecularPow(const std::vector<_OscArg*>& pArgs) throw (dab::Exception);
			void setShapeAmbientColor(const std::vector<_OscArg*>& pArgs) throw (dab::Exception);
			void setShapeDiffuseColor(const std::vector<_OscArg*>& pArgs) throw (dab::Exception);
			void setShapeTextureName(const std::vector<_OscArg*>& pArgs) throw (dab::Exception);

			std::mutex mLock;
		};

	};

};
