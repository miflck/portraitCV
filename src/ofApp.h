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
    
    
    
    
    void makeNewPortrait();
    void makeContours();
    void makePolylines();
    
    
		
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
    ofParameter<float>          contrast;
    ofParameter<float>          brightness;

    ofParameter<float>          blur;

    ofParameter<bool> simply;
    
    ofParameter<float>          mincanny;
    ofParameter<float>          maxcanny;

    ofParameter<float>          minArea2;
    ofParameter<float>          maxArea2;
    
    ofParameter<float>          mincanny2;
    ofParameter<float>          maxcanny2;
    
    ofParameter<float>          resample;
    ofParameter<float>          smooth;
    
    
    ofParameter<float>          arclength1;
    ofParameter<float>          arclength2;
    ofParameter<float>          arclength3;
    ofParameter<float>          arclength4;







    
    ofxCvGrayscaleImage     grayImage;
    ofxCvGrayscaleImage     grayImageBlur;

    
    
    ofxCvGrayscaleImage     canny;
    ofxCvColorImage            colorImg;

    ofxCvGrayscaleImage     canny2;

    
    
    ofMesh m_triangulation;
    vector<ofVec2f> m_points;
    
    vector<ofPolyline> allContours;
    vector<ofPolyline> eyes;
    vector<ofPolyline> mouth;
    
    vector<ofPolyline> linesToDraw1;
    vector<ofPolyline> linesToDraw2;
    vector<ofPolyline> linesToDraw3;
    
    
    vector<ofPolyline> linesToAnimate;
    int animationPolylineIndex=0;
    int animationVerexIndex=1;

    bool record=false;

    

    int drawcounter=0;
    
    bool continousDraw=false;
    
};
