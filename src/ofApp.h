#pragma once

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxGui.h"
#include "ofxOpenCv.h"
#include "ofxSimpleSerial.h"


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
    
    
    // Serial
    ofxSimpleSerial    serial;
    string        message;
    bool        remember;
    void        onNewMessage(string & message);
    
    void sendFeed();
    void makeFeed();
    
    void makeBoundingRectFeed();

    
    
    void penUp();
    void penDown();
    void goHome();
    bool bGoHome;
    vector<string> commands;
    bool done=true;
    bool bSendFeed=false;

		
    ofVideoGrabber cam;
    ofxCv::ContourFinder contourFinder;
    ofxCv::ContourFinder contourFinder2;

    bool bMakeContours;
    ofImage img;
    ofxCvHaarFinder finder;
    
    ofxCv::ObjectFinder haarfinder;

    
    
    ofxPanel gui;

    
    ofParameter<bool> bShowCanny;
    ofParameter<bool> bShowImage;
    ofParameter<bool> bShowDebug;


    ofParameterGroup imageparameters;
    ofParameterGroup display;
    
    ofParameterGroup finder1;
    ofParameterGroup finder2;
    ofParameterGroup robot;


    ofParameterGroup polyline1;
    ofParameterGroup polyline2;
    ofParameterGroup polyline3;


    
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
    ofParameter<float>          cannyblur;
    ofParameter<float>          cannycontrast;
    ofParameter<float>          cannybrightness;



    ofParameter<float>          minArea2;
    ofParameter<float>          maxArea2;
    
    ofParameter<float>          mincanny2;
    ofParameter<float>          maxcanny2;
    
    
    ofParameter<bool> poly;
    ofParameter<bool> poly2;
    ofParameter<bool> poly3;

    
    
    ofParameter<float>          resample;
    ofParameter<float>          resample2;
    ofParameter<float>          resample3;

    
    ofParameter<float>          smooth;
    ofParameter<float>          smooth2;
    ofParameter<float>          smooth3;

    ofParameter<float>          simplify;
    ofParameter<float>          simplify2;
    ofParameter<float>          simplify3;
    
    
    ofParameter<float>          arclength1;
    ofParameter<float>          arclength2;
    ofParameter<float>          arclength3;
    ofParameter<float>          arclength4;
    
    ofParameter<int>            quadsmooth;
    ofParameter<int>            quadhard;
    ofParameter<float>          quadfine;



    ofParameter<float>          zoomfact;



    ofParameter<float>          sortthreshold;
    
    
    ofParameter<int>            dilateErode;

    ofParameter<int>            area1;
    ofParameter<int>            area2;
    ofParameter<int>            area3;

    
    
    ofParameter<int>          penUpPos;
    ofParameter<int>          penDownPos;
    ofParameter<int>          penHighUpPos;
    ofParameter<int>          penHighDownPos;

    ofParameter<int>          penIdlePos;

    ofParameter<int>          penDrawPos;


    ofParameter<float>          scaleScreen;


    
    ofxCvGrayscaleImage     grayImage;
    ofxCvGrayscaleImage     grayImageBlur;

    
    
    ofxCvGrayscaleImage     canny;
    ofxCvGrayscaleImage     meanCanny;

    
    
    ofxCvColorImage            colorImg;

    ofxCvGrayscaleImage     canny2;

    ofxCvGrayscaleImage     zoom;

    
    ofMesh m_triangulation;
    vector<ofVec2f> m_points;
    
    vector<ofPolyline> allContours;
    vector<ofPolyline> eyes;
    vector<ofPolyline> mouth;
    
    vector<ofPolyline> linesToDraw1;
    vector<ofPolyline> linesToDraw2;
    vector<ofPolyline> linesToDraw3;
    
    vector<ofPolyline> linesToPrint;

    
    
    vector<ofPolyline> linesToAnimate;
    int animationPolylineIndex=0;
    int animationVerexIndex=1;

    bool record=false;
    
    ofRectangle faceBoundingBox;

    

    int drawcounter=0;
    
    bool continousDraw=false;
    
    float drawScaleFact=2;
    
    int getTransX(float x);
    int getTransY(float y);
    
    string formGString(float x, float y);
    
    void turnIdle();
    void turnDraw();

    void penHighUp();
    void waitPen(int mil);
    void turnPen(int ang);


    
};
