#include "ofApp.h"
#include "ofxDelaunay2D.h"

using namespace ofxCv;
using namespace cv;

cv::Mat cam_mat;
cv::Mat crop;

//--------------------------------------------------------------
void ofApp::setup(){
    
    img.load("michaelflueckiger.jpeg");
    img.crop(0, 0, 1000, 788);
    colorImg.allocate(1000,788);
    grayImage.allocate(1000,788);

    colorImg.setFromPixels(img.getPixels());
    grayImage=colorImg;
    //grayImage.threshold(200);
    //grayImage.erode();
    //grayImage.dilate();
    
    
   
    
   
    
    cam.setup(640, 480);
    gui.setup();
    gui.add(minArea.set("Min area", 1, 1, 100));
    gui.add(maxArea.set("Max area", 200, 1, 500));
    gui.add(threshold.set("Threshold", 128, 0, 255));
    gui.add(holes.set("Holes", true));
    gui.add(simply.set("Simple", false));

    gui.add(persistence.set("persistence", 15, 1, 100));
    gui.add(mincanny.set("mincanny", 80, 10, 200));
    gui.add(maxcanny.set("maxcanny", 100, 15, 300));
    
    gui.add(minArea2.set("Min area2", 1, 1, 300));
    gui.add(maxArea2.set("Max area2", 200, 1, 500));
    gui.add(mincanny2.set("mincanny2", 80, 10, 200));
    gui.add(maxcanny2.set("maxcanny2", 100, 15, 300));
    gui.add(holes2.set("Holes2", true));

    gui.add(resample.set("resample", 3, 1, 40));
    gui.add(smooth.set("smooth", 3, 1, 20));
    
    ofBackground(0);
    
}

//--------------------------------------------------------------
void ofApp::update(){
    
    
   /*  cam.update();
     if(cam.isFrameNew()) {
         colorImg.allocate(cam.getWidth(),cam.getHeight());

         colorImg.setFromPixels(cam.getPixels());
         grayImage.allocate(cam.getWidth(),cam.getHeight());
         grayImage=colorImg;

  /*   contourFinder.setMinAreaRadius(minArea);
     contourFinder.setMaxAreaRadius(maxArea);
     contourFinder.setThreshold(threshold);
     contourFinder.findContours(cam);
     contourFinder.setFindHoles(holes);*/
  //  }
 //   grayImage=colorImg;

 //  grayImage.brightnessContrast(brightness, contrast);
    canny.allocate(grayImage.getWidth(), grayImage.getHeight());
    cvCanny(grayImage.getCvImage(), canny.getCvImage(), mincanny, maxcanny,3);
    canny.dilate();
    canny.erode();
    canny.flagImageChanged();
    
    canny2.allocate(grayImage.getWidth(), grayImage.getHeight());
    cvCanny(grayImage.getCvImage(), canny2.getCvImage(), mincanny2, maxcanny2,3);
    canny2.dilate();
    canny2.erode();
    canny2.flagImageChanged();
    
    
    finder.setup("haarcascade_frontalface_default.xml");

    finder.findHaarObjects(grayImage,200,200);
    ofRectangle cur =finder.blobs[0].boundingRect;
    canny.setROI(cur.x, cur.y-200, cur.width, cur.height+200);
    if(finder.blobs.size()>0){
 
    
    cam_mat = toCv(canny);
    cv::Rect crop_roi = cv::Rect(cur.x, cur.y-100, cur.width, cur.height+150);
    crop = cam_mat(crop_roi).clone();
    
        
      
    
    contourFinder.setMinAreaRadius(minArea);
    contourFinder.setMaxAreaRadius(maxArea);
    contourFinder.setThreshold(threshold);
    contourFinder.setSimplify(simply);
    contourFinder.setFindHoles(holes);
    contourFinder.findContours(crop);
        
        
        int n = contourFinder.size();
        allContours.clear();
        for(int k=0;k<n;k++){
        ofPolyline contour=contourFinder.getPolyline(k);
            allContours.push_back(contour);
        }
        
        
        haarfinder.setup("haarcascade_mcs_eyepair_small.xml");
        haarfinder.setPreset(ObjectFinder::Fast);
        
        haarfinder.update(colorImg);

        
        
        
        cam_mat = toCv(canny2);
        cv::Rect crop_roi2 = cv::Rect(cur.x, cur.y-100, cur.width, cur.height+150);
        crop = cam_mat(crop_roi).clone();
        
        contourFinder2.setMinAreaRadius(minArea2);
        contourFinder2.setMaxAreaRadius(maxArea2);
        contourFinder2.setThreshold(threshold);
        contourFinder2.setSimplify(simply);
        contourFinder2.setFindHoles(holes2);
        contourFinder2.findContours(crop);
        
        
       // contourFinder.findContours(canny);

    }
    

}


static bool sortByCriteria(const ofPoint &a, const ofPoint &b){
    return a.x < b.x;
}

static bool sortByCriteriaX(const ofPolyline &a, const ofPolyline &b){
    return a.getBoundingBox().x < b.getBoundingBox().x;
}

//--------------------------------------------------------------
void ofApp::draw(){
   // cam.draw(0, 0);
    ofSetColor(255);
   // canny.draw(0,0);
    grayImage.draw(0,0);
   // contourFinder.draw();
    
    
    haarfinder.draw();

    
    vector<cv::Point> quad;
    vector<cv::Point> quad2;


    int n = contourFinder.size();
    int n2 = contourFinder2.size();

    //ofPolyline polyline;
    vector<ofPoint> points;
    vector<ofPolyline> polylines;


    for(int k=0;k<n;k++){
        if (contourFinder.getArcLength(k)>400){
            approxPolyDP(contourFinder.getContour(k),quad, 16 ,true);
        }else if (contourFinder.getArcLength(k)>200){
            approxPolyDP(contourFinder.getContour(k),quad, 5 ,true);
        }else if (contourFinder.getArcLength(k)<200){
            approxPolyDP(contourFinder.getContour(k),quad, 3 ,true);
        }else if (contourFinder.getArcLength(k)<50 && contourFinder.getArcLength(k)>30){
            approxPolyDP(contourFinder.getContour(k),quad, 1 ,true);
        }else{
            approxPolyDP(contourFinder.getContour(k),quad, 0.5 ,true);
        }
        
        
        
        ofSetColor(255);
        for(int i = 1; i < quad.size(); i++) {
            ofDrawLine(quad[i-1].x, quad[i-1].y, quad[i].x, quad[i].y);
            points.push_back(ofPoint(quad[i].x, quad[i].y));
        }
        
        ofPolyline polyline;
        //ofSetColor(255,0,0);
        
        for(int i = 0; i < quad.size(); i++) {
            polyline.addVertex(quad[i].x, quad[i].y);
            m_points.push_back(ofVec2f(quad[i].x, quad[i].y));

        }
        polylines.push_back(polyline);
        
    }
   // polyline = polyline.getResampledBySpacing(resample);

    
    //m_triangulation = ofxDelaunay2D::triangulate(m_points);

  
    
    ofSort(points, sortByCriteria);
    ofSort(polylines, sortByCriteriaX);

    
    ofPushMatrix();
    ofTranslate(500, 0);
   // polyline.draw();
    m_triangulation.drawWireframe();

    ofPopMatrix();
    
    ofPolyline polyline2;

    for(int i = 0; i < points.size(); i++) {
        polyline2.addVertex(points[i].x, points[i].y);
    }
    
    ofPushMatrix();
    ofTranslate(500, 0);
    ofPolyline p;
    for(int i = 0; i < polylines.size(); i++) {
        p.addVertices(polylines[i].getVertices());
        //polylines[i].draw();
        
    }
   // p.draw();
    

    
    ofPopMatrix();
    

    
    for(int k=0;k<n2;k++){
        
       /* if (contourFinder2.getArcLength(k)>400){
            approxPolyDP(contourFinder2.getContour(k),quad2, 16 ,true);
        }else if (contourFinder2.getArcLength(k)>200){
            approxPolyDP(contourFinder2.getContour(k),quad2, 5 ,true);
        }else if (contourFinder2.getArcLength(k)<200){
            approxPolyDP(contourFinder2.getContour(k),quad2, 3 ,true);
        }else if (contourFinder2.getArcLength(k)<50 && contourFinder.getArcLength(k)>30){
            approxPolyDP(contourFinder2.getContour(k),quad2, 1 ,true);
        }else{
            approxPolyDP(contourFinder2.getContour(k),quad2, 0.5 ,true);
        }*/
        
        approxPolyDP(contourFinder2.getContour(k),quad2, 0.05 ,true);

        
        ofPolyline polyline;
        ofSetColor(255);
        ofFill();
        
        for(int i = 0; i < quad2.size(); i++) {
            polyline.addVertex(quad2[i].x, quad2[i].y);
        }
        polyline.close();
        
        polyline = polyline.getResampledBySpacing(resample);
        polyline = polyline.getSmoothed(smooth);
        
        polyline.simplify();

        polyline.draw();
        
    }
    gui.draw();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

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
