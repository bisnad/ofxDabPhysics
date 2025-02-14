/** \file dab_com_visuals_osc_control.cpp
*/

#include "dab_com_visuals_osc_control.h"
#include "dab_vis_body_visualization.h"
#include "dab_vis_camera.h"
#include "dab_vis_body_shape.h"
#include "ofVectorMath.h"
#include <array>
#include <memory>

using namespace dab;
using namespace dab::com;

VisualsOscControl::VisualsOscControl()
	: OscListener()
{}

VisualsOscControl::~VisualsOscControl()
{}

void
VisualsOscControl::notify(std::shared_ptr<OscMessage> pMessage)
{
	mLock.lock();

	mMessageQueue.push_back(pMessage);
	if (mMessageQueue.size() > mMaxMessageQueueLength) mMessageQueue.pop_front();

	mLock.unlock();
}

void
VisualsOscControl::update()
{
	mLock.lock();

	while (mMessageQueue.size() > 0)
	{
		std::shared_ptr< OscMessage > oscMessage = mMessageQueue[0];

		update(oscMessage);

		mMessageQueue.pop_front();
	}

	mLock.unlock();
}

void
VisualsOscControl::update(std::shared_ptr<OscMessage> pMessage)
{
	try
	{
		std::string address = pMessage->address();

		//std::cout << "address " << address << "\n";

		const std::vector<_OscArg*>& arguments = pMessage->arguments();

		if (address.compare("/visuals/camera/projection") == 0) setCameraProjection(arguments);
		else if (address.compare("/visuals/camera/position") == 0) setCameraPosition(arguments);
		else if (address.compare("/visuals/camera/rotation") == 0) setCameraRotation(arguments);
		else if (address.compare("/visuals/shape/transparency") == 0) setShapeTransparency(arguments);
		else if (address.compare("/visuals/shape/ambientscale") == 0) setShapeAmbientScale(arguments);
		else if (address.compare("/visuals/shape/diffusescale") == 0) setShapeDiffuseScale(arguments);
		else if (address.compare("/visuals/shape/specularscale") == 0) setShapeSpecularScale(arguments);
		else if (address.compare("/visuals/shape/specularpow") == 0) setShapeSpecularPow(arguments);
		else if (address.compare("/visuals/shape/ambientcolor") == 0) setShapeAmbientColor(arguments);
		else if (address.compare("/visuals/shape/diffusecolor") == 0) setShapeDiffuseColor(arguments);
	}
	catch (dab::Exception& e)
	{
		std::cout << e << "\n";
	}
}

void 
VisualsOscControl::setCameraProjection(const std::vector<_OscArg*>& pArgs) throw (dab::Exception)
{
	if (pArgs.size() == 4)
	{
		std::array<float, 4> projection;

		projection[0] = *pArgs[0];
		projection[1] = *pArgs[1];
		projection[2] = *pArgs[2];
		projection[3] = *pArgs[3];

		try
		{
			dab::vis::BodyVisualization& visuals = dab::vis::BodyVisualization::get();
			visuals.camera()->setProjection(glm::vec4(projection[0], projection[1], projection[2], projection[3]));
		}
		catch (dab::Exception& e)
		{
			e += dab::Exception("Osc Error: failed to set camera projection", __FILE__, __FUNCTION__, __LINE__);
		}
	}
	else throw(dab::Exception("Osc Error: Wrong Parameters for /visuals/camera/projection", __FILE__, __FUNCTION__, __LINE__));
}

void 
VisualsOscControl::setCameraPosition(const std::vector<_OscArg*>& pArgs) throw (dab::Exception)
{
	if (pArgs.size() == 3)
	{
		std::array<float, 3> position;

		position[0] = *pArgs[0];
		position[1] = *pArgs[1];
		position[2] = *pArgs[2];

		//std::cout << "setCameraPosition " << position[0] << " " << position[1] << " " << position[2] << "\n";

		try
		{
			dab::vis::BodyVisualization& visuals = dab::vis::BodyVisualization::get();
			visuals.camera()->setPosition(glm::vec3(position[0], position[1], position[2]));
		}
		catch (dab::Exception& e)
		{
			e += dab::Exception("Osc Error: failed to set camera position", __FILE__, __FUNCTION__, __LINE__);
		}
	}
	else throw(dab::Exception("Osc Error: Wrong Parameters for /visuals/camera/position", __FILE__, __FUNCTION__, __LINE__));
}

void 
VisualsOscControl::setCameraRotation(const std::vector<_OscArg*>& pArgs) throw (dab::Exception)
{
	if (pArgs.size() == 3)
	{
		std::array<float, 3> rotation;

		rotation[0] = *pArgs[0];
		rotation[1] = *pArgs[1];
		rotation[2] = *pArgs[2];

		//std::cout << "setCameraRotation " << rotation[0] << " " << rotation[1] << " " << rotation[2] << "\n";

		try
		{
			dab::vis::BodyVisualization& visuals = dab::vis::BodyVisualization::get();
			visuals.camera()->setRotation(glm::vec3(rotation[0], rotation[1], rotation[2]));
		}
		catch (dab::Exception& e)
		{
			e += dab::Exception("Osc Error: failed to set camera rotation", __FILE__, __FUNCTION__, __LINE__);
		}
	}
	else throw(dab::Exception("Osc Error: Wrong Parameters for /visuals/camera/rotation", __FILE__, __FUNCTION__, __LINE__));
}

void 
VisualsOscControl::setShapeTransparency(const std::vector<_OscArg*>& pArgs) throw (dab::Exception)
{
	if (pArgs.size() == 2)
	{
		std::string shapeName = pArgs[0]->operator const std::string&();
		float transparency = *pArgs[1];

		try
		{
			dab::vis::BodyVisualization& visuals = dab::vis::BodyVisualization::get();
			std::shared_ptr<vis::BodyShape> shape = visuals.shape(shapeName);

			shape->material().setTransparency(transparency);
		}
		catch (dab::Exception& e)
		{
			e += dab::Exception("Osc Error: failed to set shape transparency", __FILE__, __FUNCTION__, __LINE__);
		}
	}
	if (pArgs.size() == 1)
	{
		float transparency = *pArgs[0];

		try
		{
			dab::vis::BodyVisualization& visuals = dab::vis::BodyVisualization::get();
			const std::vector<std::shared_ptr<vis::BodyShape>> shapes = visuals.shapes();

			int shapeCount = shapes.size();

			for (int sI = 0; sI < shapeCount; ++sI)
			{
				shapes[sI]->material().setTransparency(transparency);
			}
		}
		catch (dab::Exception& e)
		{
			e += dab::Exception("Osc Error: failed to set shape transparency", __FILE__, __FUNCTION__, __LINE__);
		}
	}
	else throw(dab::Exception("Osc Error: Wrong Parameters for /visuals/shape/transparency", __FILE__, __FUNCTION__, __LINE__));
}

void 
VisualsOscControl::setShapeAmbientScale(const std::vector<_OscArg*>& pArgs) throw (dab::Exception)
{
	if (pArgs.size() == 2)
	{
		std::string shapeName = pArgs[0]->operator const std::string&();
		float ambientScale = *pArgs[1];

		try
		{
			dab::vis::BodyVisualization& visuals = dab::vis::BodyVisualization::get();
			std::shared_ptr<vis::BodyShape> shape = visuals.shape(shapeName);

			shape->material().setAmbientScale(ambientScale);
		}
		catch (dab::Exception& e)
		{
			e += dab::Exception("Osc Error: failed to set shape ambient scale", __FILE__, __FUNCTION__, __LINE__);
		}
	}
	if (pArgs.size() == 1)
	{
		float ambientScale = *pArgs[0];

		try
		{
			dab::vis::BodyVisualization& visuals = dab::vis::BodyVisualization::get();
			const std::vector<std::shared_ptr<vis::BodyShape>> shapes = visuals.shapes();

			int shapeCount = shapes.size();

			for (int sI = 0; sI < shapeCount; ++sI)
			{
				shapes[sI]->material().setAmbientScale(ambientScale);
			}
		}
		catch (dab::Exception& e)
		{
			e += dab::Exception("Osc Error: failed to set shape ambient scale", __FILE__, __FUNCTION__, __LINE__);
		}
	}
	else throw(dab::Exception("Osc Error: Wrong Parameters for /visuals/shape/ambientscale", __FILE__, __FUNCTION__, __LINE__));
}

void 
VisualsOscControl::setShapeDiffuseScale(const std::vector<_OscArg*>& pArgs) throw (dab::Exception)
{
	if (pArgs.size() == 2)
	{
		std::string shapeName = pArgs[0]->operator const std::string&();
		float diffuseScale = *pArgs[1];

		try
		{
			dab::vis::BodyVisualization& visuals = dab::vis::BodyVisualization::get();
			std::shared_ptr<vis::BodyShape> shape = visuals.shape(shapeName);

			shape->material().setDiffuseScale(diffuseScale);
		}
		catch (dab::Exception& e)
		{
			e += dab::Exception("Osc Error: failed to set shape diffuse scale", __FILE__, __FUNCTION__, __LINE__);
		}
	}
	if (pArgs.size() == 1)
	{
		float diffuseScale = *pArgs[0];

		try
		{
			dab::vis::BodyVisualization& visuals = dab::vis::BodyVisualization::get();
			const std::vector<std::shared_ptr<vis::BodyShape>> shapes = visuals.shapes();

			int shapeCount = shapes.size();

			for (int sI = 0; sI < shapeCount; ++sI)
			{
				shapes[sI]->material().setDiffuseScale(diffuseScale);
			}
		}
		catch (dab::Exception& e)
		{
			e += dab::Exception("Osc Error: failed to set shape diffuse scale", __FILE__, __FUNCTION__, __LINE__);
		}
	}
	else throw(dab::Exception("Osc Error: Wrong Parameters for /visuals/shape/diffusescale", __FILE__, __FUNCTION__, __LINE__));
}

void 
VisualsOscControl::setShapeSpecularScale(const std::vector<_OscArg*>& pArgs) throw (dab::Exception)
{
	if (pArgs.size() == 2)
	{
		std::string shapeName = pArgs[0]->operator const std::string&();
		float specularScale = *pArgs[1];

		try
		{
			dab::vis::BodyVisualization& visuals = dab::vis::BodyVisualization::get();
			std::shared_ptr<vis::BodyShape> shape = visuals.shape(shapeName);

			shape->material().setSpecularScale(specularScale);
		}
		catch (dab::Exception& e)
		{
			e += dab::Exception("Osc Error: failed to set shape specular scale", __FILE__, __FUNCTION__, __LINE__);
		}
	}
	if (pArgs.size() == 1)
	{
		float specularScale = *pArgs[0];

		try
		{
			dab::vis::BodyVisualization& visuals = dab::vis::BodyVisualization::get();
			const std::vector<std::shared_ptr<vis::BodyShape>> shapes = visuals.shapes();

			int shapeCount = shapes.size();

			for (int sI = 0; sI < shapeCount; ++sI)
			{
				shapes[sI]->material().setSpecularScale(specularScale);
			}
		}
		catch (dab::Exception& e)
		{
			e += dab::Exception("Osc Error: failed to set shape specular scale", __FILE__, __FUNCTION__, __LINE__);
		}
	}
	else throw(dab::Exception("Osc Error: Wrong Parameters for /visuals/shape/specularscale", __FILE__, __FUNCTION__, __LINE__));
}

void 
VisualsOscControl::setShapeSpecularPow(const std::vector<_OscArg*>& pArgs) throw (dab::Exception)
{
	if (pArgs.size() == 2)
	{
		std::string shapeName = pArgs[0]->operator const std::string&();
		float specularPow = *pArgs[1];

		try
		{
			dab::vis::BodyVisualization& visuals = dab::vis::BodyVisualization::get();
			std::shared_ptr<vis::BodyShape> shape = visuals.shape(shapeName);

			shape->material().setSpecularPow(specularPow);
		}
		catch (dab::Exception& e)
		{
			e += dab::Exception("Osc Error: failed to set shape specular pow", __FILE__, __FUNCTION__, __LINE__);
		}
	}
	if (pArgs.size() == 1)
	{
		float specularPow = *pArgs[0];

		try
		{
			dab::vis::BodyVisualization& visuals = dab::vis::BodyVisualization::get();
			const std::vector<std::shared_ptr<vis::BodyShape>> shapes = visuals.shapes();

			int shapeCount = shapes.size();

			for (int sI = 0; sI < shapeCount; ++sI)
			{
				shapes[sI]->material().setSpecularPow(specularPow);
			}
		}
		catch (dab::Exception& e)
		{
			e += dab::Exception("Osc Error: failed to set shape specular pow", __FILE__, __FUNCTION__, __LINE__);
		}
	}
	else throw(dab::Exception("Osc Error: Wrong Parameters for /visuals/shape/specularpow", __FILE__, __FUNCTION__, __LINE__));
}

void 
VisualsOscControl::setShapeAmbientColor(const std::vector<_OscArg*>& pArgs) throw (dab::Exception)
{
	if (pArgs.size() == 4)
	{
		std::string shapeName = pArgs[0]->operator const std::string&();
		glm::vec3 ambientColorHSB;
		ambientColorHSB[0] = *pArgs[1];
		ambientColorHSB[1] = *pArgs[2];
		ambientColorHSB[2] = *pArgs[3];

		ofColor c = ofColor::fromHsb(ambientColorHSB[0] * 255.0, ambientColorHSB[1] * 255.0, ambientColorHSB[2] * 255.0);
		glm::vec3 ambientColorRGB;
		ambientColorRGB[0] = c.r / 255.0;
		ambientColorRGB[1] = c.g / 255.0;
		ambientColorRGB[2] = c.b / 255.0;

		try
		{
			dab::vis::BodyVisualization& visuals = dab::vis::BodyVisualization::get();
			std::shared_ptr<vis::BodyShape> shape = visuals.shape(shapeName);

			shape->material().setAmbientColor(ambientColorRGB);
		}
		catch (dab::Exception& e)
		{
			e += dab::Exception("Osc Error: failed to set shape ambient color", __FILE__, __FUNCTION__, __LINE__);
		}
	}
	if (pArgs.size() == 3)
	{
		glm::vec3 ambientColorHSB;
		ambientColorHSB[0] = *pArgs[0];
		ambientColorHSB[1] = *pArgs[1];
		ambientColorHSB[2] = *pArgs[2];

		ofColor c = ofColor::fromHsb(ambientColorHSB[0] * 255.0, ambientColorHSB[1] * 255.0, ambientColorHSB[2] * 255.0);
		glm::vec3 ambientColorRGB;
		ambientColorRGB[0] = c.r / 255.0;
		ambientColorRGB[1] = c.g / 255.0;
		ambientColorRGB[2] = c.b / 255.0;

		try
		{
			dab::vis::BodyVisualization& visuals = dab::vis::BodyVisualization::get();
			const std::vector<std::shared_ptr<vis::BodyShape>> shapes = visuals.shapes();

			int shapeCount = shapes.size();

			for (int sI = 0; sI < shapeCount; ++sI)
			{
				shapes[sI]->material().setAmbientColor(ambientColorRGB);
			}
		}
		catch (dab::Exception& e)
		{
			e += dab::Exception("Osc Error: failed to set shape ambient color", __FILE__, __FUNCTION__, __LINE__);
		}
	}
	else throw(dab::Exception("Osc Error: Wrong Parameters for /visuals/shape/ambientcolor", __FILE__, __FUNCTION__, __LINE__));
}

void 
VisualsOscControl::setShapeDiffuseColor(const std::vector<_OscArg*>& pArgs) throw (dab::Exception)
{
	if (pArgs.size() == 4)
	{
		std::string shapeName = pArgs[0]->operator const std::string&();
		glm::vec3 diffuseColorHSB;
		diffuseColorHSB[0] = *pArgs[1];
		diffuseColorHSB[1] = *pArgs[2];
		diffuseColorHSB[2] = *pArgs[3];

		ofColor c = ofColor::fromHsb(diffuseColorHSB[0] * 255.0, diffuseColorHSB[1] * 255.0, diffuseColorHSB[2] * 255.0);
		glm::vec3 diffuseColorRGB;
		diffuseColorRGB[0] = c.r / 255.0;
		diffuseColorRGB[1] = c.g / 255.0;
		diffuseColorRGB[2] = c.b / 255.0;

		try
		{
			dab::vis::BodyVisualization& visuals = dab::vis::BodyVisualization::get();
			std::shared_ptr<vis::BodyShape> shape = visuals.shape(shapeName);

			shape->material().setDiffuseColor(diffuseColorRGB);
		}
		catch (dab::Exception& e)
		{
			e += dab::Exception("Osc Error: failed to set shape diffuse color", __FILE__, __FUNCTION__, __LINE__);
		}
	}
	if (pArgs.size() == 3)
	{
		glm::vec3 diffuseColorHSB;
		diffuseColorHSB[0] = *pArgs[0];
		diffuseColorHSB[1] = *pArgs[1];
		diffuseColorHSB[2] = *pArgs[2];

		ofColor c = ofColor::fromHsb(diffuseColorHSB[0] * 255.0, diffuseColorHSB[1] * 255.0, diffuseColorHSB[2] * 255.0);
		glm::vec3 diffuseColorRGB;
		diffuseColorRGB[0] = c.r / 255.0;
		diffuseColorRGB[1] = c.g / 255.0;
		diffuseColorRGB[2] = c.b / 255.0;

		try
		{
			dab::vis::BodyVisualization& visuals = dab::vis::BodyVisualization::get();
			const std::vector<std::shared_ptr<vis::BodyShape>> shapes = visuals.shapes();

			int shapeCount = shapes.size();

			for (int sI = 0; sI < shapeCount; ++sI)
			{
				shapes[sI]->material().setDiffuseColor(diffuseColorRGB);
			}
		}
		catch (dab::Exception& e)
		{
			e += dab::Exception("Osc Error: failed to set shape ambient color", __FILE__, __FUNCTION__, __LINE__);
		}
	}
	else throw(dab::Exception("Osc Error: Wrong Parameters for /visuals/shape/ambientcolor", __FILE__, __FUNCTION__, __LINE__));
}

void 
VisualsOscControl::setShapeTextureName(const std::vector<_OscArg*>& pArgs) throw (dab::Exception)
{}