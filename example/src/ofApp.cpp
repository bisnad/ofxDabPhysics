#include "ofApp.h"
#include "ofxAssimpModelLoader.h"
#include "dab_value.h"
#include "dab_geom_mesh_manager.h"
#include "dab_physics_simulation.h"
#include "dab_physics_body_shape.h"
#include "dab_physics_body_part.h"
#include "dab_physics_universal_joint.h"
#include "dab_physics_universal_motor.h"
#include "dab_physics_body.h"
#include "dab_physics_behavior_force.h"
#include "dab_physics_behavior_random_force.h"
#include "dab_physics_behavior_rotation_target.h"
#include "dab_physics_behavior_random_rotation_target.h"
#include "dab_physics_behavior_target_attraction.h"
#include "dab_physics_behavior_volume.h"
#include "dab_physics_behavior_speed.h"
#include "dab_physics_neighbor_behavior_cohesion.h"
#include "dab_physics_neighbor_behavior_alignment.h"
#include "dab_vis_body_visualization.h"
#include "dab_vis_camera.h"
#include "dab_com_osc_control.h"
#include "dab_urdf_importer.h"
#include <btBulletDynamicsCommon.h>
#include "dab_value.h"
#include "dab_string_tools.h"

//--------------------------------------------------------------

void 
ofApp::setup()
{	
	// load body from urdf file
	try
	{
		dab::UrdfImporter& urdfImporter = dab::UrdfImporter::get();
		urdfImporter.loadURDF(ofToDataPath("3d_models/arm_6joints_base/robot.urdf")); // arm with 6 joints and fixed base
		//urdfImporter.loadURDF(ofToDataPath("3d_models/arm_17joints_base/robot.urdf")); // arm with 17 joints and fixed base
	}
	catch (dab::Exception& e)
	{
		std::cout << e << "\n";
	}

	setupPhysics();
	setupGraphics();
	setupOsc();

	dab::physics::Simulation::get().start();

	ofBackground(0, 0, 0);
	ofSetFrameRate(60);
	ofSetVerticalSync(true);
}

void 
ofApp::setupPhysics()
{
	dab::physics::Simulation& physics = dab::physics::Simulation::get();
	physics.setTimeStep(0.1);
	physics.setSupSteps(20);
	physics.setGravity(glm::vec3(0.0, 0.0, 0.0));

	try
	{
		// retrieve part and joint names of loaded body
		const std::vector<std::shared_ptr<dab::physics::BodyPart>>& bodyParts = physics.body("onshape")->parts();
		const std::vector<std::shared_ptr<dab::physics::BodyJoint>>& bodyJoints = physics.body("onshape")->joints();

		std::vector<std::string> partNames = {};
		for (auto bodyPart : bodyParts) partNames.push_back(bodyPart->name());

		std::vector<std::string> jointNames = {};
		for (auto bodyJoint : bodyJoints) jointNames.push_back(bodyJoint->name());

		// ´add generic behaviors
		std::shared_ptr<dab::physics::RandomForceBehavior> randomTipBehaviour = dab::physics::Simulation::get().addBehavior<dab::physics::RandomForceBehavior>("onshape", "random_tip", { partNames[partNames.size() - 1] }, {}, {});
		randomTipBehaviour->set("active", false);
		randomTipBehaviour->set("minDir", { -1.0f, -1.0f, -1.0f });
		randomTipBehaviour->set("maxDir", { 1.0f, 1.0f, 1.0f });
		randomTipBehaviour->set("minAmp", 1.0f);
		randomTipBehaviour->set("maxAmp", 1.0f);
		randomTipBehaviour->set("appInterval", 1.0f);
		randomTipBehaviour->set("randInterval", 5000.0f);

		std::shared_ptr<dab::physics::RandomForceBehavior> randomAllBehaviour = dab::physics::Simulation::get().addBehavior<dab::physics::RandomForceBehavior>("onshape", "random_all", partNames, {}, {});
		randomAllBehaviour->set("active", false);
		randomAllBehaviour->set("minDir", { -1.0f, -1.0f, -1.0f });
		randomAllBehaviour->set("maxDir", { 1.0f, 1.0f, 1.0f });
		randomAllBehaviour->set("minAmp", 1.0f);
		randomAllBehaviour->set("maxAmp", 1.0f);
		randomAllBehaviour->set("appInterval", 1.0f);
		randomAllBehaviour->set("randInterval", 5000.0f);

		std::shared_ptr<dab::physics::ForceBehavior> forceTipBehaviour = dab::physics::Simulation::get().addBehavior<dab::physics::ForceBehavior>("onshape", "force_tip", { partNames[partNames.size() - 1] }, {}, {});
		forceTipBehaviour->set("active", false);
		forceTipBehaviour->set("dir", { 1.0f, 0.0f, 0.0f });
		forceTipBehaviour->set("amp", 1.0f);
		forceTipBehaviour->set("appInterval", 1.0f);

		for (int jI = 0; jI < jointNames.size(); ++jI)
		{
			std::shared_ptr<dab::physics::RotationTargetBehavior> rotationTargetBehavior = dab::physics::Simulation::get().addBehavior<dab::physics::RotationTargetBehavior>("onshape", "rotation_" + std::to_string(jI), {}, {}, { jointNames[jI] });
			rotationTargetBehavior->set("active", false);
			rotationTargetBehavior->set("target", { 0.0f, 0.0f, 0.0f });
			rotationTargetBehavior->set("speed", 1.4f);
			rotationTargetBehavior->set("appInterval", 1.0f);
		}

		std::shared_ptr<dab::physics::SpeedBehavior> speedTipBehavior = dab::physics::Simulation::get().addBehavior<dab::physics::SpeedBehavior>("onshape", "speed_tip", { partNames[partNames.size() - 1] }, {}, {});
		speedTipBehavior->set("active", false);
		speedTipBehavior->set("speed", 1.0f);
		speedTipBehavior->set("amount", 1.0f);

		std::shared_ptr<dab::physics::SpeedBehavior> speedAllBehavior = dab::physics::Simulation::get().addBehavior<dab::physics::SpeedBehavior>("onshape", "speed_all", partNames, {}, {});
		speedAllBehavior->set("active", false);
		speedAllBehavior->set("speed", 1.0f);
		speedAllBehavior->set("amount", 1.0f);

		// add interaction behaviors
		std::shared_ptr<dab::physics::TargetAttractionBehavior> targetAttractionTipBehaviour = dab::physics::Simulation::get().addBehavior<dab::physics::TargetAttractionBehavior>("onshape", "targetpos_tip", { partNames[partNames.size() - 1] }, {}, {});
		targetAttractionTipBehaviour->set("active", false);
		targetAttractionTipBehaviour->set("targetPos", { 0.0f, 2.0f, 0.0f });
		targetAttractionTipBehaviour->set("maxDist", 10.0f);
		targetAttractionTipBehaviour->set("minAmp", 0.0f);
		targetAttractionTipBehaviour->set("maxAmp", 1.0f);
		targetAttractionTipBehaviour->set("appInterval", 1.0f);

		std::shared_ptr<dab::physics::TargetAttractionBehavior> targetAttractionAllBehaviour = dab::physics::Simulation::get().addBehavior<dab::physics::TargetAttractionBehavior>("onshape", "targetpos_all", partNames, {}, {});
		targetAttractionAllBehaviour->set("active", false);
		targetAttractionAllBehaviour->set("targetPos", { 0.0f, 2.0f, 0.0f });
		targetAttractionAllBehaviour->set("maxDist", 10.0f);
		targetAttractionAllBehaviour->set("minAmp", 0.0f);
		targetAttractionAllBehaviour->set("maxAmp", 1.0f);
		targetAttractionAllBehaviour->set("appInterval", 1.0f);

		// add movement quality behaviors
		std::shared_ptr<dab::physics::RandomForceBehavior> levitationBehaviour = dab::physics::Simulation::get().addBehavior<dab::physics::RandomForceBehavior>("onshape", "levitation", { partNames[partNames.size() - 1] }, {}, {});
		levitationBehaviour->set("active", false);
		levitationBehaviour->set("minDir", { -1.0f, -1.0f, -1.0f });
		levitationBehaviour->set("maxDir", { 1.0f, 1.0f, 1.0f });
		levitationBehaviour->set("minAmp", 1.0f);
		levitationBehaviour->set("maxAmp", 1.0f);
		levitationBehaviour->set("appInterval", 1.0f);
		levitationBehaviour->set("randInterval", 5000.0f);

		std::shared_ptr<dab::physics::RandomForceBehavior> particlesBehaviour = dab::physics::Simulation::get().addBehavior<dab::physics::RandomForceBehavior>("onshape", "particles", { partNames[partNames.size() - 1] }, {}, {});
		particlesBehaviour->set("active", false);
		particlesBehaviour->set("minDir", { -1.0f, -1.0f, -1.0f });
		particlesBehaviour->set("maxDir", { 1.0f, 1.0f, 1.0f });
		particlesBehaviour->set("minAmp", 1.0f);
		particlesBehaviour->set("maxAmp", 1.0f);
		particlesBehaviour->set("appInterval", 1.0f);
		particlesBehaviour->set("randInterval", 5000.0f);

		std::shared_ptr<dab::physics::RandomRotationTargetBehavior> thrustingBehavior = dab::physics::Simulation::get().addBehavior<dab::physics::RandomRotationTargetBehavior>("onshape", "thrusting", {}, {}, { jointNames[0], jointNames[1] });
		thrustingBehavior->set("active", false);
		thrustingBehavior->set("minTarget", { 0.0f, 0.0f, -1.4f });
		thrustingBehavior->set("maxTarget", { 0.0f, 0.0f, 1.4f });
		thrustingBehavior->set("speed", 1.4f);
		thrustingBehavior->set("appInterval", 1.0f);
		thrustingBehavior->set("randInterval", 2000.0f);

		std::shared_ptr<dab::physics::RandomRotationTargetBehavior> staccatoBehavior = dab::physics::Simulation::get().addBehavior<dab::physics::RandomRotationTargetBehavior>("onshape", "staccato", {}, {}, jointNames);
		staccatoBehavior->set("active", false);
		staccatoBehavior->set("minTarget", { 0.0f, 0.0f, -1.4f });
		staccatoBehavior->set("maxTarget", { 0.0f, 0.0f, 1.4f });
		staccatoBehavior->set("speed", 1.4f);
		staccatoBehavior->set("appInterval", 1.0f);
		staccatoBehavior->set("randInterval", 2000.0f);

		std::shared_ptr<dab::physics::RandomRotationTargetBehavior> fluidityBehavior = dab::physics::Simulation::get().addBehavior<dab::physics::RandomRotationTargetBehavior>("onshape", "fluidity", {}, {}, jointNames);
		fluidityBehavior->set("active", false);
		fluidityBehavior->set("minTarget", { 0.0f, 0.0f, -1.4f });
		fluidityBehavior->set("maxTarget", { 0.0f, 0.0f, 1.4f });
		fluidityBehavior->set("speed", 0.1f);
		fluidityBehavior->set("appInterval", 1.0f);
		fluidityBehavior->set("randInterval", 1000.0f);

		// make copies of body
		physics.copyBody("onshape", "onshape2");
		physics.copyBody("onshape", "onshape3");
		physics.copyBody("onshape", "onshape4");
		physics.copyBody("onshape", "onshape5");
		physics.copyBody("onshape", "onshape6");
		physics.copyBody("onshape", "onshape7");
		physics.copyBody("onshape", "onshape8");
		physics.copyBody("onshape", "onshape9");
		physics.copyBody("onshape", "onshape10");
		physics.copyBody("onshape", "onshape11");
		physics.copyBody("onshape", "onshape12");
		physics.copyBody("onshape", "onshape13");
		physics.copyBody("onshape", "onshape14");
		physics.copyBody("onshape", "onshape15");
		physics.copyBody("onshape", "onshape16");

		// retrieve names of all bodies
		const std::vector<std::shared_ptr<dab::physics::Body>>& bodies = physics.bodies();

		std::vector<std::string> bodyNames = {};
		for (auto body : bodies) bodyNames.push_back(body->name());

		// for multiple bodies, add neighbor behaviors
		if (bodies.size() > 1)
		{
			// cohesion among tip parts of bodies
			std::vector<std::shared_ptr<dab::physics::CohesionBehavior>> cohesionBehaviors = dab::physics::Simulation::get().addNeighborBehavior<dab::physics::CohesionBehavior>(bodyNames, "cohesion", { partNames[partNames.size() - 1] }, { partNames[partNames.size() - 1] }, {}, {}, {}, {});
			for (int bI = 0; bI < cohesionBehaviors.size(); ++bI)
			{
				cohesionBehaviors[bI]->set("active", false);
				cohesionBehaviors[bI]->set("minDist", 0.0f);
				cohesionBehaviors[bI]->set("maxDist", 10.0f);
				cohesionBehaviors[bI]->set("amount", 1.0f);
			}

			// evasion among tip parts of bodies
			std::vector<std::shared_ptr<dab::physics::CohesionBehavior>> evasionBehaviors = dab::physics::Simulation::get().addNeighborBehavior<dab::physics::CohesionBehavior>(bodyNames, "evasion", { partNames[partNames.size() - 1] }, { partNames[partNames.size() - 1] }, {}, {}, {}, {});
			for (int bI = 0; bI < cohesionBehaviors.size(); ++bI)
			{
				evasionBehaviors[bI]->set("active", false);
				evasionBehaviors[bI]->set("minDist", 0.0f);
				evasionBehaviors[bI]->set("maxDist", 10.0f);
				evasionBehaviors[bI]->set("amount", 1.0f);
			}

			// alignment among tip parts of bodies
			std::vector<std::shared_ptr<dab::physics::AlignmentBehavior>> alignmentBehaviors = dab::physics::Simulation::get().addNeighborBehavior<dab::physics::AlignmentBehavior>(bodyNames, "alignment", { partNames[partNames.size() - 1] }, { partNames[partNames.size() - 1] }, {}, {}, {}, {});
			for (int bI = 0; bI < alignmentBehaviors.size(); ++bI)
			{
				alignmentBehaviors[bI]->set("active", false);
				alignmentBehaviors[bI]->set("minDist", 0.0f);
				alignmentBehaviors[bI]->set("maxDist", 10.0f);
				alignmentBehaviors[bI]->set("linearAmount", 1.0f);
				alignmentBehaviors[bI]->set("angularAmount", 1.0f);
				alignmentBehaviors[bI]->set("amount", 1.0f);
			}
		}

		// change orientation and placement of bodies
		std::shared_ptr<dab::physics::BodyPart> base;

		if (bodies.size() == 1)
		{
			// center pos & vertical up
			base = physics.part("onshape", partNames[0]);
			base->setRotation(glm::quat(glm::vec3(PI / 2.0, 0.0, 0.0)));
			base->setPosition(glm::vec3(0.0, 0.0, 3.0));
		}
		else
		{
			// two rings of eight arms each, bottom ring with arms pointing up and top ring with arms pointing down
			// bottom ring for first half arms with arms pointing up
			{
				int body_offset = 0;
				int body_count = bodies.size() / 2;
				float ringPosRadius = 2.5;
				float ringStartPosAngle = -PI / 2.0;
				float ringPosAngleSpacing = PI * 2.0 / (float)body_count;
				float bodyBaseRotation = 0.0;

				for (int bI = body_offset; bI < body_offset + body_count; ++bI)
				{
					std::string bodyName = "onshape";

					if (bI > 0)
					{
						bodyName += std::to_string(bI + 1);
					}

					base = physics.part(bodyName, partNames[0]);

					float posAngle = ringStartPosAngle + float(bI) * ringPosAngleSpacing;
					float posX = ringPosRadius * cos(posAngle);
					float posY = ringPosRadius * sin(posAngle);
					bodyBaseRotation = -posAngle;

					base->setPosition(glm::vec3(posX, posY, 0.0));
					base->setRotation(glm::quat(glm::vec3(PI / 2.0, 0.0, 0.0)));
				}
			}
			// // top ring for second half of arms with arms pointing down
			int body_offset = bodies.size() / 2;
			int body_count = bodies.size() / 2;
			float ringPosRadius = 2.5;
			float ringStartPosAngle = -PI / 2.0;
			float ringPosAngleSpacing = PI * 2.0 / (float)body_count;
			float bodyBaseRotation = 0.0;

			for (int bI = body_offset; bI < body_offset + body_count; ++bI)
			{
				std::string bodyName = "onshape";

				if (bI > 0)
				{
					bodyName += std::to_string(bI + 1);
				}

				base = physics.part(bodyName, partNames[0]);

				float posAngle = ringStartPosAngle + float(bI) * ringPosAngleSpacing;
				float posX = ringPosRadius * cos(posAngle);
				float posY = ringPosRadius * sin(posAngle);
				bodyBaseRotation = -posAngle;

				base->setPosition(glm::vec3(posX, posY, 5.0));
				base->setRotation(glm::quat(glm::vec3(-PI / 2.0, 0.0, posAngle)));
			}
		}
	}
	catch (dab::Exception& e)
	{
		std::cout << e << "\n";
	}
}

void 
ofApp::setupGraphics()
{
	dab::physics::Simulation& physics = dab::physics::Simulation::get();
	dab::vis::BodyVisualization& visuals = dab::vis::BodyVisualization::get();
	visuals.camera()->setProjection(glm::vec4(20.0, 1.0, 0.1, 200.0));
	visuals.camera()->setPosition(glm::vec3(0.0, -4.0, -3.9));
	visuals.camera()->setRotation(glm::vec3(-80.0, 0.0, -24.0));

	try
	{
		ofBackground(0, 0, 0);
		ofSetFrameRate(60);
		ofSetVerticalSync(true);

		const std::vector<std::shared_ptr<dab::physics::Body>>& bodies = physics.bodies();

		// create additional visual bodies if there are multiple arms
		if (bodies.size() > 1)
		{
			for (int bI = 1; bI < bodies.size(); ++bI)
			{
				visuals.copyBody(bodies[0]->name(), bodies[bI]->name());
			}
		}

		// create visual target position
		visuals.addTargetPositions(1);
	}
	catch (dab::Exception& e)
	{
		std::cout << e << "\n";
	}
}

void
ofApp::setupOsc()
{	
	try
	{
		dab::com::OscControl& oscControl = dab::com::OscControl::get();

		oscControl.createReceiver("OscControlReceiver", 9003);
		oscControl.createPhysicsControl("OscControlReceiver");
		oscControl.createVisualsControl("OscControlReceiver");

		oscControl.createSender("OscPhysicsSender", "127.0.0.1", 9005);
	}
	catch (dab::Exception& e)
	{
		std::cout << e << "\n";
	}
}

void 
ofApp::resetPhysics()
{}

//--------------------------------------------------------------
void 
ofApp::update()
{
	updateOsc();
	updateGraphics();
}

void 
ofApp::updateOsc()
{
	// receive osc control data
	dab::com::OscControl::get().update();

	// send osc data
	dab::physics::Simulation& physics = dab::physics::Simulation::get();
	dab::com::OscControl& oscControl = dab::com::OscControl::get();
	std::shared_ptr<dab::OscSender> _sender = oscControl.sender("OscPhysicsSender");
	const std::vector<std::shared_ptr<dab::physics::Body>>& physicsBodies = physics.bodies();

	// send joint positions
	for (auto physicsBody : physicsBodies)
	{
		std::shared_ptr<dab::OscMessage> jointPositionMessage(new dab::OscMessage("/physics/joint/pos"));

		const std::string& bodyName = physicsBody->name();

		jointPositionMessage->add(bodyName);

		const std::vector<std::shared_ptr<dab::physics::BodyJoint>>& physicsJoints = physicsBody->joints();

		for (auto physicsJoint : physicsJoints)
		{
			btTransform jointTransform = physicsJoint->transform();
			const btVector3& jointPos = jointTransform.getOrigin();

			jointPositionMessage->add(jointPos[0]);
			jointPositionMessage->add(jointPos[1]);
			jointPositionMessage->add(jointPos[2]);
		}

		_sender->send(jointPositionMessage);
	}

	// send joint world angles
	for (auto physicsBody : physicsBodies)
	{
		std::shared_ptr<dab::OscMessage> jointRotationMessage(new dab::OscMessage("/physics/joint/rot"));

		const std::string& bodyName = physicsBody->name();

		jointRotationMessage->add(bodyName);

		const std::vector<std::shared_ptr<dab::physics::BodyJoint>>& physicsJoints = physicsBody->joints();

		for (auto physicsJoint : physicsJoints)
		{
			btTransform jointTransform = physicsJoint->transform();
			const btQuaternion& jointRot = jointTransform.getRotation();

			jointRotationMessage->add(jointRot[0]);
			jointRotationMessage->add(jointRot[1]);
			jointRotationMessage->add(jointRot[2]);
			jointRotationMessage->add(jointRot[3]);

			//std::cout << "joint " << physicsJoint->name() << " jrot " << jointRot[0] << " " << jointRot[1] << " " << jointRot[2] << " " << jointRot[3] << "\n";
		}

		_sender->send(jointRotationMessage);
	}

	// send joint relative angles
	for (auto physicsBody : physicsBodies)
	{
		std::shared_ptr<dab::OscMessage> jointRotationMessage(new dab::OscMessage("/physics/joint/relrot"));

		const std::string& bodyName = physicsBody->name();

		jointRotationMessage->add(bodyName);

		const std::vector<std::shared_ptr<dab::physics::BodyJoint>>& physicsJoints = physicsBody->joints();

		for (auto physicsJoint : physicsJoints)
		{
			std::shared_ptr<dab::physics::UniversalJoint> ujoint = std::dynamic_pointer_cast<dab::physics::UniversalJoint>(physicsJoint);

			if (ujoint == nullptr) continue;

			const std::array<float, 3>& jointAngles = ujoint->angles();

			//std::cout << "joint " << ujoint->name() << " angles " << jointAngles[0] << " " << jointAngles[1] << " " << jointAngles[2] << "\n";

			jointRotationMessage->add(jointAngles[0]);
			jointRotationMessage->add(jointAngles[1]);
			jointRotationMessage->add(jointAngles[2]);
		}

		_sender->send(jointRotationMessage);
	}
}

void 
ofApp::updateGraphics()
{
	dab::vis::BodyVisualization::get().update();

	// update visual target position
	try
	{
		std::array<float, 3> targetPos;
		//dab::physics::Simulation::get().body("onshape")->behavior("targetpos_tip")->get("targetPos", targetPos);
		dab::physics::Simulation::get().body("onshape")->behavior("targetpos_all")->get("targetPos", targetPos);
		
		dab::vis::BodyVisualization::get().setTargetPosition(0, glm::vec3(targetPos[0], targetPos[1], targetPos[2]));
	}
	catch (dab::Exception& e)
	{
		std::cout << "ofApp::updateGraphics failed: " << e << "\n";
	}
}

//--------------------------------------------------------------
void ofApp::draw()
{
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);

	dab::vis::BodyVisualization::get().draw();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key)
{
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}