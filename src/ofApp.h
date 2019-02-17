#pragma once

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxGui.h"
#include "ofxOpenCv.h"
#include "ofxSimpleSerial.h"
#include "ofxDmx.h"

#define IDLE 100
#define DRAWING 200

using namespace ofxCv;
using namespace cv;

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();
        void exit();
    
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
    
    
    int state=IDLE;
    int stateBefore=IDLE;

    
    void makeNewPortrait();
    void makeContours();
    void makePolylines();

    
    void makeEyePolylines();
    
    
    // TImers
    int timerduration=10;
    int inittime;
    
    int idleTimerDuration=60000;
    int initIdletime;
    
    int portraitTimerDuration=3000;
    int portraitinittime;
    bool bMakeNewPortraitwidthTimer=false;
    
    void makeNewPortraitWithTimer();
   void makeNewPortraitWithTimerFinished();
    
    
    
    // Serial
    ofxSimpleSerial    serial;
    string        message;
    void        onNewMessage(string & message);
    
    
    ofxSimpleSerial    button;
    void        onNewButtonMessage(string & message);

    
    
    
    void sendFeed();
    void sendFinalFeed();
    void clearCommands();
    void makeFeed();
    
    void makeBoundingRectFeed();

    
    
    void penUp();
    void penDown();
    void goHome();
    
    void waitPos();
    
    bool bGoHome;
    vector<string> commands;
    bool done=true;
    //bool bSendFeed=false;

		
    ofVideoGrabber cam;
    ofxCv::ContourFinder contourFinder;
    ofxCv::ContourFinder contourFinder2;

    bool bMakeContours;
    ofImage img;
    ofxCvHaarFinder finder;
    ofxCvHaarFinder hfinder2;

    
    ofxCv::ObjectFinder haarfinder;

    
    ofxPanel gui;
    ofxPanel polylinesPanel;
    ofxPanel cannyPanel;

    ofParameter<bool> bShowCanny;
    ofParameter<bool> bShowCanny2;
    ofParameter<bool> bShowImage;
    ofParameter<bool> bShowDebug;
    ofParameter<bool> bShowDebugLines;

    ofParameter<bool> bUseCanny1;

    ofParameterGroup imageparameters;
    ofParameter<float>          contrast;
    ofParameter<float>          brightness;
    ofParameter<float>          blur;
    ofParameter<float>          zoomfact;
    ofParameter<float>          facezoomfact;


    
    ofParameter<float>          scaleScreen;

    
    ofParameterGroup display;
    
    
    ofParameter<float> minArea, maxArea;
    ofParameter<bool> holes;
    ofParameter<bool> simply;

    
    ofParameterGroup finder1;
    
    ofParameterGroup canny1;
    ofParameterGroup canny2group;


    ofParameter<float>          mincanny;
    ofParameter<float>          maxcanny;
    ofParameter<float>          cannyblur;
    ofParameter<float>          cannycontrast;
    ofParameter<float>          cannybrightness;
    ofParameter<int>            dilateErode;
    ofParameter<float>          threshold;

    
    ofParameter<float>          minArea2;
    ofParameter<float>          maxArea2;
    ofParameter<float>          mincanny2;
    ofParameter<float>          maxcanny2;
    ofParameter<float>          cannyblur2;
    ofParameter<float>          cannycontrast2;
    ofParameter<float>          cannybrightness2;
    ofParameter<float>          threshold2;

    ofParameter<int>            dilateErode2;
    

    ofParameterGroup polyline1;
    ofParameterGroup polyline2;
    ofParameterGroup polyline3;
    ofParameterGroup polylinehalf;
    
    ofParameter<float>          resample;
    ofParameter<float>          resample2;
    ofParameter<float>          resample3;
    
    ofParameter<float>          smooth;
    ofParameter<float>          smooth2;
    ofParameter<float>          smooth3;
    


    ofParameter<float>          simplify;
    ofParameter<float>          simplify2;
    ofParameter<float>          simplify3;
    
    ofParameter<int>            area1min;
    ofParameter<int>            area1max;
    ofParameter<int>            area2min;
    ofParameter<int>            area2max;
    ofParameter<int>            area3min;
    ofParameter<int>            area3max;
    
    
    ofParameter<float>          polyhalf_percent;
    ofParameter<float>          resample_half;
    ofParameter<float>          smooth_half;
    ofParameter<float>          simplify_half;
    ofParameter<int>            area_half_min;
    ofParameter<int>            area_half_max;
    
    
    ofParameterGroup finder2;
    ofParameter<bool> holes2;

 
    
    

    ofParameter<bool> poly;
    ofParameter<bool> poly2;
    ofParameter<bool> poly3;
    ofParameter<bool> polymedian;
    ofParameter<bool> polyhalf;

    ofParameter<int>            quadsmooth;
    ofParameter<int>            quadhard;
    ofParameter<float>          quadfine;

    
    ofParameter<float>          arclength1;
    ofParameter<float>          arclength2;
    ofParameter<float>          arclength3;
    ofParameter<float>          arclength4;
    
  
    ofParameter<float>          sortthreshold;
    
    ofParameterGroup robot;
    ofParameter<int>          penUpPos;
    ofParameter<int>          penDownPos;
    ofParameter<int>          penHighUpPos;
    ofParameter<int>          penHighDownPos;
    ofParameter<int>          penIdlePos;
    ofParameter<int>          penDrawPos;
    ofParameter<int>            dipPosX;
    ofParameter<int>            dipPosY;
    
    ofParameter<int>            waitPosX;
    ofParameter<int>            waitPosY;


    ofParameter<int>            eyeAreamin;
    ofParameter<int>            eyeAreamax;
    ofParameter<float>          eyeresample;
    ofParameter<float>          eyesmooth;
    ofParameter<float>          eyesimplify;

    ofParameter<float>          eyequad;

    
    
    
    
    ofxCvGrayscaleImage     grayImage;
    ofxCvColorImage         colorImg;
    ofxCvGrayscaleImage     canny;
    ofxCvGrayscaleImage     canny2;
    ofxCvGrayscaleImage     zoom;
    ofxCvGrayscaleImage     face;

    
    
    
    ofMesh m_triangulation;
    vector<ofVec2f> m_points;
    vector<ofPolyline> allContours;
    vector<ofPolyline> eyes;
    vector<ofPolyline> mouth;
    vector<ofPolyline> linesToDraw1;
    vector<ofPolyline> half_linesToDraw1;

    vector<ofPolyline> linesToDraw2;
    vector<ofPolyline> linesToDraw3;
    vector<ofPolyline> medianlines;
    vector<ofPolyline> linesToPrint;

    
    

    
    vector<ofPolyline> linesToAnimate;
    int animationPolylineIndex=0;
    int animationVerexIndex=1;

    bool record=false;
    
    ofRectangle faceBoundingBox;
    ofRectangle faceBoundingBoxOriginal;
    ofRectangle eyeBoundingBox;

    int drawcounter=0;
    
    bool continousDraw=false;
    
    float drawScaleFact=1;
    
    int getTransX(float x);
    int getTransY(float y);
    
    string formGString(float x, float y);
    
    void turnIdle();
    void turnDraw();

    void penHighUp();
    void waitPen(int mil);
    void turnPen(int ang);
    
    void goDip();
    
    int xInMM=1400;
    int yInMM=1400;
    float scaleRatio;
    

// DMX
    ofxDmx dmx;
    int dmxValue;
    void turnLightsOn();
    void turnLightsOff();
    
    ofParameter<float> dimAmmount;

    
    
    ofTrueTypeFont  font;

    ofImage mask;

    bool bDrawGui=false;
    
    
    void drawDebugLines();
    ofFbo saveFrame;
    
  
    
};
