#pragma once

#include "ofMain.h"
#include "ofVectorMath.h"
#include "ofShader.h"
#include "ofVbo.h"
#include "dab_vis_body_shape.h"
#include "dab_vis_body_part.h"
#include "dab_physics_behavior.h"

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);

	protected:
		void setupPhysics();
		void setupGraphics();
		void setupOsc();

		void resetPhysics();

		void updateGraphics();
		void updateOsc();
};
