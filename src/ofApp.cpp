#include "ofApp.h"
#include "ofxDelaunay2D.h"

using namespace ofxCv;
using namespace cv;

cv::Mat cam_mat;
cv::Mat crop;

//--------------------------------------------------------------
void ofApp::setup(){
   // ofSetLogLevel(OF_LOG_VERBOSE);
    img.load("michaelflueckiger.jpeg");
    img.crop(0, 0, 1000, 788);
    colorImg.allocate(1000,788);
    grayImage.allocate(1000,788);

    colorImg.setFromPixels(img.getPixels());
    grayImage=colorImg;
    //grayImage.threshold(200);
    //grayImage.erode();
    //grayImage.dilate();
    
    
   
    
   
    cam.listDevices();
    //cam.setDeviceID(1);

    cam.setup(1024,576);
    gui.setup();
    
    gui.add(contrast.set("contrast", 1, 0, 1));
    gui.add(brightness.set("brightness", 1, 0, 1));

    gui.add(blur.set("blur", 0, 0, 10));


    gui.add(arclength1.set("arclength1", 400, 0, 3000));
    gui.add(arclength2.set("arclength2", 200, 0, 3000));
    gui.add(arclength3.set("arclength3", 50, 0, 3000));
    gui.add(arclength4.set("arclength4", 30, 0, 3000));

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
    
    
    cam.update();
    
    
    
    if(continousDraw){
        
        makeNewPortrait();

        
    }
    
    /*  if(cam.isFrameNew()) {
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

    canny.contrastStretch();
 

    canny.flagImageChanged();
    
    canny2.allocate(grayImage.getWidth(), grayImage.getHeight());
    cvCanny(grayImage.getCvImage(), canny2.getCvImage(), mincanny2, maxcanny2,3);
    canny2.dilate();
    canny2.erode();
    canny2.flagImageChanged();
    
    
    finder.setup("haarcascade_frontalface_default.xml");

    finder.findHaarObjects(grayImage,200,200);
    
    
    if(finder.blobs.size()>0){

    ofRectangle cur =finder.blobs[0].boundingRect;
    ofDrawRectangle(cur);
    canny.setROI(cur.x, cur.y-200, cur.width, cur.height+200);
 
    
    cam_mat = toCv(canny);
    //cv::Rect crop_roi = cv::Rect(cur.x, cur.y-100, cur.width, cur.height+150);
        cv::Rect crop_roi = cv::Rect(cur.x, cur.y, cur.width, cur.height);

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

static bool sortByArea(const ofPolyline &a, const ofPolyline &b){
    return a.getPerimeter() > b.getPerimeter();
}



//--------------------------------------------------------------
void ofApp::draw(){
    cam.draw(ofGetWidth()-cam.getWidth()/3, grayImage.getHeight()/3,cam.getWidth()/3,cam.getHeight()/3);
    ofPushStyle();
    ofSetColor(255);
   canny.draw(ofGetWidth()-canny.getWidth()/3, canny.getHeight()/3*2,canny.getWidth()/3,canny.getHeight()/3);
    
    // canny.draw(0,0);
    ofPopStyle();
    grayImage.draw(ofGetWidth()-grayImage.getWidth()/3,0,grayImage.getWidth()/3,grayImage.getHeight()/3);
    ofPushMatrix();
   // ofScale(2,2);
   //contourFinder.draw();
    ofPopMatrix();
    
    haarfinder.draw();

    
    vector<cv::Point> quad;
    vector<cv::Point> quad2;


    int n = contourFinder.size();
    int n2 = contourFinder2.size();

    //ofPolyline polyline;
    vector<ofPoint> points;
    vector<ofPolyline> polylines;

    linesToDraw1.clear();
    linesToDraw2.clear();
    linesToDraw3.clear();

    
    vector<ofPolyline> contourslines=contourFinder.getPolylines();

  /*  for(int k=0;k<contourslines.size();k++){
    
    /*
    if (contourslines[k].getPerimeter()>arclength1){
        contourslines[k] = contourslines[k].getResampledBySpacing(200);
    }else if (contourslines[k].getPerimeter()>arclength2){
        contourslines[k] = contourslines[k].getResampledBySpacing(100);
    }else if (contourslines[k].getPerimeter()<arclength2){
        contourslines[k] = contourslines[k].getResampledBySpacing(50);
    }else if (contourslines[k].getPerimeter()<arclength3 && contourslines[k].getPerimeter()>arclength4){
        contourslines[k] = contourslines[k].getResampledBySpacing(3);
    }else{
        contourslines[k] = contourslines[k].getResampledBySpacing(0);
    }*/
        
        
   /*     contourslines[k] = contourslines[k].getResampledByCount(10);

        if(contourslines[k].getPerimeter()>50){
        linesToDraw1.push_back(contourslines[k]);
        }
      }*/
    
    
    for(int k=0;k<n;k++){
    /*  if (contourFinder.getArcLength(k)>arclength1){
            approxPolyDP(contourFinder.getContour(k),quad, 40 ,true);
        }else if (contourFinder.getArcLength(k)>arclength2){
            approxPolyDP(contourFinder.getContour(k),quad, 20 ,true);
        }else if (contourFinder.getArcLength(k)>arclength3){
            approxPolyDP(contourFinder.getContour(k),quad, 15 ,true);
        }else if (contourFinder.getArcLength(k)<arclength4 && contourFinder.getArcLength(k)>arclength4){
            approxPolyDP(contourFinder.getContour(k),quad, 5 ,true);
        }else{
            approxPolyDP(contourFinder.getContour(k),quad, 1 ,true);
        }*/
        
        approxPolyDP(contourFinder.getContour(k),quad, 0.5 ,true);

        
        if (contourFinder.getArcLength(k)>arclength4){
        approxPolyDP(contourFinder.getContour(k),quad, 3 ,true);
        }
        if (contourFinder.getArcLength(k)>arclength3){
            approxPolyDP(contourFinder.getContour(k),quad, 8 ,true);
        }
        if (contourFinder.getArcLength(k)>arclength2){
            approxPolyDP(contourFinder.getContour(k),quad, 10 ,true);
        }
        if (contourFinder.getArcLength(k)>arclength1){
            approxPolyDP(contourFinder.getContour(k),quad, 16 ,true);
        }
        
        
        
       // approxPolyDP(contourFinder.getContour(k),quad, 0.5 ,true);

        
        ofSetColor(255);
        for(int i = 1; i < quad.size(); i++) {
            //ofDrawLine(quad[i-1].x*2, quad[i-1].y*2, quad[i].x*2, quad[i].y*2);
            points.push_back(ofPoint(quad[i].x, quad[i].y));
        }
        
        ofPolyline polyline;
        ofPolyline polyline2;
        ofPolyline polyline3;


        //ofSetColor(255,0,0);
        
       for(int i = 0; i < quad.size(); i++) {
            polyline.addVertex(quad[i].x, quad[i].y);
        }
        for(int i =  quad.size()/2; i < quad.size(); i++) {
            polyline2.addVertex(quad[i].x, quad[i].y);
        }
        
        
        for(int i = 0; i < quad.size(); i++) {
            polyline3.addVertex(quad[i].x, quad[i].y);
        }
        
        //cout<<polyline.getArea()<<" "<<polyline.getPerimeter()<<endl;
  
        
        
        
        polyline = polyline.getResampledBySpacing(resample);
       // polyline = polyline.getSmoothed(smooth);
        
        polyline2 = polyline2.getResampledBySpacing(resample);
        polyline2 = polyline2.getSmoothed(smooth);
        
        polyline3 = polyline3.getResampledBySpacing(resample);
        polyline3 = polyline3.getSmoothed(smooth);
        
       // polyline.simplify();
        
        //if(ABS(polyline.getArea())>30){
            linesToDraw1.push_back(polyline);
        
    //}
       // linesToDraw1.push_back(polyline);
        linesToDraw2.push_back(polyline2);
        linesToDraw3.push_back(polyline3);

    }
    
    
    ofSort(linesToDraw1, sortByArea);

    
    
   // polyline = polyline.getResampledBySpacing(resample);

    
    //m_triangulation = ofxDelaunay2D::triangulate(m_points);

  
    
    //ofSort(points, sortByCriteria);
    //ofSort(polylines, sortByCriteriaX);

    
    ofPushMatrix();
  //  ofScale(0.7,0.7);
    ofTranslate(0, 500);
    
    ofPushStyle();
    ofSetColor(255, 0, 255);
   // cout<<" lines "<<linesToDraw1.size()<<" "<<drawcounter<<endl;
    if(drawcounter<linesToDraw1.size()){
        drawcounter++;
    }else{
        drawcounter=1;
        
    }
    
//linesToDraw1=linesToDraw1.reverse(linesToDraw1.begin(),linesToDraw1.end());

    for(int i = linesToDraw1.size()-1; i >= drawcounter; i--) {
       // cout<<linesToDraw1[linesToDraw1.size()-1].getPerimeter()<<endl;
       linesToDraw1[i].draw();
    }
    
    //cout<<linesToDraw1[drawcounter].getArea()<<" "<<linesToDraw1[drawcounter].getPerimeter()<<endl;

    
    ofTranslate(500, 0);

    for(int i = 0; i < drawcounter; i++) {
        linesToDraw1[i].draw();
    }

    ofSetColor(255,0,0);
    linesToDraw1[drawcounter].draw();
   

    ofPopStyle();
    ofPopMatrix();
    
    ofPushMatrix();
    ofTranslate(550, 0);
    ofPushStyle();
    ofSetColor(255, 0, 255);
    ofScale(1,1);
    for(int i = 0; i < linesToDraw1.size(); i++) {
       linesToDraw1[i].draw();
    }
    
    ofTranslate(500, 0);

    ofSetColor(0, 255, 255);

    for(int i = 0; i < linesToDraw2.size(); i++) {
        linesToDraw2[i].draw();
        
    }
    
    
    ofSetColor(255, 255, 0);

    ofTranslate(0, 500);
    for(int i = 0; i < linesToDraw3.size(); i++) {
        linesToDraw3[i].draw();
        
    }
    
   // polyline.draw();
   // m_triangulation.drawWireframe();
    ofPopStyle();

    ofPopMatrix();
    
    ofPolyline polyline2;

    for(int i = 0; i < points.size(); i++) {
        polyline2.addVertex(points[i].x, points[i].y);
    }
    
    ofPushMatrix();
    ofPushStyle();
    ofSetColor(0, 0, 255);
    ofTranslate(500, 0);
    ofPolyline p;
    for(int i = 0; i < polylines.size()/2; i++) {
        p.addVertices(polylines[i].getVertices());
        //polylines[i].draw();
        
    }
  //  p.draw();
    
    ofPopStyle();
    
    ofPopMatrix();
    

    
    for(int k=0;k<n2;k++){
        
  /*     if (contourFinder2.getArcLength(k)>400){
            approxPolyDP(contourFinder2.getContour(k),quad2, 16 ,true);
        }else if (contourFinder2.getArcLength(k)>200){
            approxPolyDP(contourFinder2.getContour(k),quad2, 5 ,true);
        }else if (contourFinder2.getArcLength(k)<200){
            approxPolyDP(contourFinder2.getContour(k),quad2, 3 ,true);
        }else if (contourFinder2.getArcLength(k)<50 && contourFinder.getArcLength(k)>30){
            approxPolyDP(contourFinder2.getContour(k),quad2, 1 ,true);
        }else{
            approxPolyDP(contourFinder2.getContour(k),quad2, 0.5 ,true);
        }
    */
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
void ofApp::makeNewPortrait(){
    colorImg.allocate(cam.getWidth(),cam.getHeight());
    colorImg.setFromPixels(cam.getPixels());
    grayImage.allocate(cam.getWidth(),cam.getHeight());
    grayImageBlur.allocate(cam.getWidth(),cam.getHeight());
    grayImage=colorImg;
    grayImageBlur=colorImg;
 //   grayImage.brightnessContrast(brightness, contrast);
    grayImage.blur(blur);

    grayImage.contrastStretch();
    grayImage.dilate();
    grayImage.erode();
    
    
    grayImageBlur.blur(blur);
    grayImageBlur.contrastStretch();
    grayImageBlur.dilate();
    grayImageBlur.erode();
    
    
}


//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    if(key == ' '){
        makeNewPortrait();
        
    }
    
    if(key == 'c'){
        continousDraw=!continousDraw;
        
    }

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
