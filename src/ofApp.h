#pragma once

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxGui.h"
#include "ofxOpenCv.h"

using namespace ofxCv;
using namespace cv;

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
		
    ofVideoGrabber cam;
    ofxCv::ContourFinder contourFinder;
    ofxCv::ContourFinder contourFinder2;

    
    ofImage img;
    ofxCvHaarFinder finder;
    
    ofxCv::ObjectFinder haarfinder;

    
    
    ofxPanel gui;
    ofParameter<float> minArea, maxArea, threshold;
    ofParameter<bool> holes;
    ofParameter<bool> holes2;

    ofParameter<float>          persistence;
    ofParameter<bool> simply;
    
    ofParameter<float>          mincanny;
    ofParameter<float>          maxcanny;

    ofParameter<float>          minArea2;
    ofParameter<float>          maxArea2;
    
    ofParameter<float>          mincanny2;
    ofParameter<float>          maxcanny2;
    
    ofParameter<float>          resample;
    ofParameter<float>          smooth;



    
    ofxCvGrayscaleImage     grayImage;
    ofxCvGrayscaleImage     canny;
    ofxCvColorImage            colorImg;

    ofxCvGrayscaleImage     canny2;

    
    
    ofMesh m_triangulation;
    vector<ofVec2f> m_points;
    
    vector<ofPolyline> allContours;
    vector<ofPolyline> eyes;
    vector<ofPolyline> mouth;

    
};
