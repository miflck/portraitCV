#include "ofApp.h"
#include "ofxDelaunay2D.h"

using namespace ofxCv;
using namespace cv;

cv::Mat cam_mat;
cv::Mat crop;


int baud = 115200;
char myByte = 0;
string cmd;

bool bUseArduino=false;

//--------------------------------------------------------------
void ofApp::setup(){
    
  serial.listDevices();
    if(bUseArduino) {
        vector <ofSerialDeviceInfo> deviceList = serial.getDeviceList();
        serial.setup(0, baud); //open the first device
        serial.startContinuousRead(false);
        ofAddListener(serial.NEW_MESSAGE,this,&ofApp::onNewMessage);
    }
    
    
    bool        bSendSerialMessage=false;            // a flag for sending serial
    message = "";
    remember = false;
    
    ofSetLogLevel(OF_LOG_VERBOSE);
    
    
    img.load("michaelflueckiger.jpeg");
    img.crop(0, 0, 1000, 788);
    colorImg.allocate(1000,788);
    grayImage.allocate(1000,788);
    meanCanny.allocate(grayImage.getWidth(), grayImage.getHeight());
    
    
    colorImg.setFromPixels(img.getPixels());
    grayImage=colorImg;
    //grayImage.threshold(200);
    //grayImage.erode();
    //grayImage.dilate();
    
    
    /*
    img.load("michaelflueckiger.jpeg");
    //img.crop(0, 0, 1000, 788);
    colorImg.allocate(img.getWidth(),img.getHeight());
    grayImage.allocate(img.getWidth(),img.getHeight());
    meanCanny.allocate(grayImage.getWidth(), grayImage.getHeight());


    colorImg.setFromPixels(img.getPixels());
    grayImage=colorImg;*/
    //grayImage.threshold(200);
    //grayImage.erode();
    //grayImage.dilate();
    
    
   
    
   
    cam.listDevices();
    cam.setDeviceID(0);

    cam.setup(1024,576);
    
    gui.setup();

    


    display.setName("Dispaly");
    display.add(bShowImage.set("bShowImage",false));
    display.add(bShowCanny.set("bShowCanny",false));
    gui.add(display);

    imageparameters.setName("Image");
    imageparameters.add(contrast.set("contrast",0, -1, 1));
    imageparameters.add(brightness.set("brightness", 0, -1, 1));
    imageparameters.add(blur.set("blur", 0, 0, 10));
    imageparameters.add(zoomfact.set("zoom", 1, 0, 2));
    gui.add(imageparameters);
    gui.getGroup("Image").minimize();
  
    finder1.setName("Contourfinder 1");

    finder1.add(dilateErode.set("dilateErode", 5, 0, 100));

    finder1.add(arclength1.set("arclength1", 400, 0, 3000));
    finder1.add(arclength2.set("arclength2", 200, 0, 3000));
    finder1.add(arclength3.set("arclength3", 50, 0, 3000));
    finder1.add(arclength4.set("arclength4", 30, 0, 3000));
    finder1.add(mincanny.set("mincanny", 80, 10, 500));
    finder1.add(maxcanny.set("maxcanny", 100, 15, 300));
    finder1.add(cannyblur.set("cannyblur", 0, 0, 10));

    finder1.add(quadhard.set("quadhard", 16, 0, 30));
    finder1.add(quadsmooth.set("quadsmooth", 8, 0, 16));

    finder1.add(minArea.set("Min area", 1, 1, 100));
    finder1.add(maxArea.set("Max area", 200, 1, 2000));
    finder1.add(threshold.set("Threshold", 128, 0, 255));
    finder1.add(holes.set("Holes", true));
    finder1.add(simply.set("Simple", false));
    
    finder1.add(resample.set("resample", 3, 1, 40));
    finder1.add(smooth.set("smooth", 3, 1, 20));
    finder1.add(sortthreshold.set("sortthreshold", 0.5, 0, 5));

    gui.add(finder1);

    
    
    finder2.setName("Contourfinder 2");


    finder2.add(minArea2.set("Min area2", 1, 1, 300));
    finder2.add(maxArea2.set("Max area2", 200, 1, 500));
    finder2.add(mincanny2.set("mincanny2", 80, 10, 200));
    finder2.add(maxcanny2.set("maxcanny2", 100, 15, 300));
    finder2.add(holes2.set("Holes2", true));
    gui.add(finder2);
    gui.getGroup("Contourfinder 2").minimize();

    //gui.add(persistence.set("persistence", 15, 1, 100));

    robot.setName("Robot");

    robot.add(penUpPos.set("penUpPos", 70, 0, 180));
    robot.add(penHighUpPos.set("penHighUpPos", 0, 0, 180));
    robot.add(penHighDownPos.set("penHighDownPos", 30, 0, 180));
    robot.add(penDownPos.set("penDownPos", 85, 0, 180));
    robot.add(penIdlePos.set("penIdlePos", 180, 0, 180));
    robot.add(penDrawPos.set("penDrawPos", 30, 0, 180));
    gui.add(robot);
    gui.getGroup("Robot").minimize();

    


    
    
    
    ofBackground(0);
    
    makeContours();
}

//--------------------------------------------------------------
void ofApp::update(){
    
    if(bUseArduino){
        if(message != "" && remember == false)
    {
        cout << "sending message: " << message << "\n";
        serial.writeString(message);
        message = "";
    }
    
    if (done && bSendFeed &&!bGoHome)sendFeed();
    if(done && bGoHome){
        goHome();
        bGoHome=false;
    }
        
    }
    
    
    cam.update();
    if(continousDraw){
    makeNewPortrait();
        makeContours();

    }
  if(record) makeContours();
    if(bMakeContours)  {
        makeContours();
        bMakeContours=false;
    }
}


static bool sortByCriteria(const ofPoint &a, const ofPoint &b){
    return a.x < b.x;
}

static bool sortByCriteriaX(const ofPolyline &a, const ofPolyline &b){
    return a.getBoundingBox().x < b.getBoundingBox().x;
}

static bool sortByArea(const ofPolyline &a, const ofPolyline &b){
    if(a.getPerimeter() > 50 || b.getPerimeter()>50){
        return a.getPerimeter() > b.getPerimeter();
    }else{
        return a.getBoundingBox().x < b.getBoundingBox().x;

    }
}

void ofApp::makeContours(){
    
    grayImage.contrastStretch();
    zoom=grayImage;
    
    zoom.transform(0, zoom.getWidth()/2, zoom.getHeight()/2, zoomfact, zoomfact, 0, 0);
   // if(canny.getWidth()!=grayImage.getWidth()){
        canny.allocate(grayImage.getWidth(), grayImage.getHeight());
        canny2.allocate(grayImage.getWidth(), grayImage.getHeight());
    if(meanCanny.getWidth()!=grayImage.getWidth()){
        meanCanny.allocate(grayImage.getWidth(), grayImage.getHeight());
    }
    //}
    


    cvCanny(zoom.getCvImage(), canny.getCvImage(), mincanny, maxcanny,3);
    
    //cvCornerHarris(zoom.getCvImage(), canny.getCvImage(),10,10);
    for(int i=0;i<dilateErode;i++){
        canny.dilate();
    }
    
    //canny.blur(cannyblur);
    
    for(int i=0;i<dilateErode;i++){
        canny.erode();
    }


   // cvCanny(canny.getCvImage(), canny.getCvImage(), mincanny, maxcanny,3);


  /*  canny.dilate();
    canny.erode();
    canny.dilate();
    canny.erode();
    canny.dilate();
    canny.erode();
    canny.dilate();
    canny.dilate();

    canny.erode();
    canny.erode();
   */

    
    canny.flagImageChanged();
    
    //meanCanny+=canny;
  //  meanCanny+=canny;
  //  meanCanny.erode();
   // meanCanny.erode();
 //   meanCanny.erode();
 //   meanCanny.erode();

    
    cvCanny(zoom.getCvImage(), canny2.getCvImage(), mincanny2, maxcanny2,3);
    canny2.dilate();
    canny2.erode();
    canny2.flagImageChanged();
  //  meanCanny-=canny2;
  //  meanCanny.absDiff(canny,canny2);

    
    finder.setup("haarcascade_frontalface_default.xml");
    finder.findHaarObjects(zoom,200,200);
    
    
    if(finder.blobs.size()>0){
        ofRectangle cur =finder.blobs[0].boundingRect;
        faceBoundingBox = finder.blobs[0].boundingRect;
       

        ofDrawRectangle(cur);
        
        int diffx,diffy,dx,dy,hy;
        int offsetty=100;
        
        
        diffx=cur.x+cur.width;
        
        diffy=cur.y-offsetty/2;
       
        if(cur.y-offsetty/2<=0){
            dy=0;
        }else{
            dy=cur.y-offsetty/2;
        }
        
        if(cur.y+cur.height+offsetty>=canny.getHeight()){
            hy=canny.getHeight()-dy;
        }else{
            hy=cur.height+offsetty;
        }
        
        faceBoundingBox.y-=dy;
        faceBoundingBox.height+=hy;
        
       canny.setROI(cur.x-200, cur.y-400, cur.width+200, cur.height+400);
        cam_mat = toCv(canny);
        //cv::Rect crop_roi = cv::Rect(cur.x, cur.y-100, cur.width, cur.height+150);
       // cv::Rect crop_roi = cv::Rect(cur.x, cur.y-dy, cur.width, cur.height+hy);
        cv::Rect crop_roi = cv::Rect(cur.x, dy, cur.width, hy);

        
        crop = cam_mat(crop_roi).clone();
        
        
       /* ofPoint sourcePoints[4];
        sourcePoints[0] = ofPoint(    cur.x,     cur.y);
        sourcePoints[1] = ofPoint(cur.x+cur.width, cur.y);
        sourcePoints[2] = ofPoint(cur.x+cur.width, cur.y+cur.height);
        sourcePoints[3] = ofPoint(    cur.x, cur.y+cur.height);
        ofPoint destPoints[4];
        destPoints[0]   = ofPoint(       0,        0);
        destPoints[1]   = ofPoint(     cam.getWidth(),        0);
        destPoints[2]   = ofPoint(     cam.getWidth(),      cam.getHeight());
        destPoints[3]   = ofPoint(       0,      cam.getHeight());*/
        //zoom.warpIntoMe(grayImage, sourcePoints, destPoints);
      
        //zoom.scale(2, 2);
        
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
        
        
        //haarfinder.setup("haarcascade_mcs_eyepair_small.xml");
        //haarfinder.setPreset(ObjectFinder::Fast);
        //haarfinder.update(colorImg);
        
        //cam_mat = toCv(canny2);
       // cv::Rect crop_roi2 = cv::Rect(cur.x, cur.y-100, cur.width, cur.height+150);
       // crop = cam_mat(crop_roi).clone();
        
        contourFinder2.setMinAreaRadius(minArea2);
        contourFinder2.setMaxAreaRadius(maxArea2);
        contourFinder2.setThreshold(threshold);
        contourFinder2.setSimplify(simply);
        contourFinder2.setFindHoles(holes2);
        contourFinder2.findContours(crop);
    }
    
    makePolylines();
    
}



void ofApp::makePolylines(){
  
    
    
    int n = contourFinder.size();
    int n2 = contourFinder2.size();
    
    //ofPolyline polyline;
    vector<ofPoint> points;
    vector<ofPolyline> polylines;
    
    linesToDraw1.clear();
    linesToDraw2.clear();
    linesToDraw3.clear();
    
    
    
    for(int k=0;k<n;k++){
        
        vector<cv::Point> quad;
        vector<cv::Point> quad2;
        vector<cv::Point> quad3;
        vector<cv::Point> quad4;


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
        
        //approxPolyDP(contourFinder.getContour(k),quad, 0.5 ,true);
        
       /* if (contourFinder.getArcLength(k)>arclength4&& contourFinder.getArcLength(k)<arclength3){
            approxPolyDP(contourFinder.getContour(k),quad, 3 ,true);
        }*/
        
     /*   if (contourFinder.getArcLength(k)<arclength1){
            approxPolyDP(contourFinder.getContour(k),quad, quadsmooth ,true);
        }
        
       else if (contourFinder.getArcLength(k)>arclength1){
            approxPolyDP(contourFinder.getContour(k),quad, quadhard ,true);
        }*/
        
        
        approxPolyDP(contourFinder.getContour(k),quad, quadhard ,true);
        approxPolyDP(contourFinder.getContour(k),quad2, quadsmooth ,true);
        approxPolyDP(contourFinder.getContour(k),quad3, 0.5 ,true);

        
        
       // approxPolyDP(contourFinder.getContour(k),quad, 16 ,true);

        ofSetColor(255);
        
        /* for(int i = 1; i < quad.size(); i++) {
         points.push_back(ofPoint(quad[i].x, quad[i].y));
         }*/
        
        ofPolyline polyline;
        ofPolyline polyline2;
        ofPolyline polyline3;
        
        for(int i = 0; i < quad.size(); i++) {
            polyline.addVertex(quad[i].x, quad[i].y);
        }
        for(int i =  quad.size()/2; i < quad.size(); i++) {
            polyline2.addVertex(quad[i].x, quad[i].y);
        }
        
        for(int i = 0; i < quad.size(); i++) {
            polyline3.addVertex(quad[i].x*1.3, quad[i].y*1.3);
        }
        
        
        
        
        polyline = polyline.getResampledBySpacing(resample);
        polyline = polyline.getSmoothed(smooth);
        polyline.simplify(sortthreshold);

        
        polyline2 = polyline2.getResampledBySpacing(resample);
        polyline2 = polyline2.getSmoothed(smooth);
        polyline2.simplify(sortthreshold);

        
        polyline3 = polyline3.getResampledBySpacing(resample);
        polyline3.simplify(sortthreshold);
        polyline3 = polyline3.getSmoothed(smooth);
        
        
        //cout<<"n "<<k<<" quad.size() "<<quad.size()<<" area "<<polyline3.getPerimeter()<<endl;

        
        //cout<<"n "<<k<<" arc length "<<contourFinder.getArcLength(k)<<endl;
        if(ABS(polyline3.getPerimeter())>1){
            
           /* vector<ofVec3f> vertices = polyline3.getVertices();
            for (int vertexIndex=0; vertexIndex<vertices.size(); vertexIndex++) {

            cout<<vertices[vertexIndex]<<" ";
            }
            cout<<endl;*/
            linesToDraw3.push_back(polyline);
        }
        
        if(ABS(polyline.getPerimeter())>1){
        linesToDraw1.push_back(polyline);
        }
        linesToDraw2.push_back(polyline);
        //linesToDraw3.push_back(polyline3);
        
    }
    
    
    ofSort(linesToDraw1, sortByArea);
    ofSort(linesToDraw3, sortByArea);

    if(record){

        linesToAnimate=linesToDraw1;
        linesToPrint=linesToDraw3;
        
      
        
        record=false;
        makeFeed();
    }
    
}


//--------------------------------------------------------------
void ofApp::draw(){
    
    zoom.draw(ofGetWidth()-grayImage.getWidth()/3,0,grayImage.getWidth()/3,grayImage.getHeight()/3);

    if(bShowImage){
        img.draw(0,0);
    }
    cam.draw(ofGetWidth()-cam.getWidth()/3, grayImage.getHeight()/3,cam.getWidth()/3,cam.getHeight()/3);
    ofPushStyle();
    ofSetColor(255);
    if(!bShowCanny){
         canny.draw(ofGetWidth()-canny.getWidth()/3, canny.getHeight()/3*2,canny.getWidth()/3,canny.getHeight()/3);
    }else{
       canny.draw(0,0);
    }
   //

    //canny2.draw(ofGetWidth()-canny.getWidth()/3, canny.getHeight()/3*3,canny.getWidth()/3,canny.getHeight()/3);
    //meanCanny.draw(ofGetWidth()-canny.getWidth()/3*2, canny.getHeight()/3*3,canny.getWidth()/3,canny.getHeight()/3);

    // canny.draw(0,0);
    ofPopStyle();
    //zoom.draw(ofGetWidth()-grayImage.getWidth()/3,0,grayImage.getWidth()/3,grayImage.getHeight()/3);
    
    //zoom.draw(ofGetWidth()-zoom.getWidth()/3,zoom.getHeight()/3*3,zoom.getWidth()/3,zoom.getHeight()/3);

   // ofDrawRectangle(faceBoundingBox);
    
    ofPushMatrix();
   // ofScale(2,2);
   //contourFinder.draw();
    //ofTranslate(faceBoundingBox.getPosition().x,faceBoundingBox.getPosition().y);
    ofPushStyle();
    ofNoFill();
    ofSetLineWidth(2);
    for(int i = 0; i < (int)contourFinder.getPolylines().size(); i++) {
         ofSetColor(255);
        /*if (contourFinder.getArcLength(i)>arclength4&& contourFinder.getArcLength(i)<arclength3){
            ofSetColor(0,0,255);
        }
        if (contourFinder.getArcLength(i)>arclength3 && contourFinder.getArcLength(i)<arclength3){
            ofSetColor(0,255,255);
        }
        if (contourFinder.getArcLength(i)>arclength2 && contourFinder.getArcLength(i)<arclength1){
            ofSetColor(0,255,0);
        }
        if (contourFinder.getArcLength(i)>arclength1){
            ofSetColor(255,0,0);
        }
        */
        
        
        if (contourFinder.getContourArea(i)>arclength4&& contourFinder.getContourArea(i)<arclength3){
            ofSetColor(0,0,255);
        }
        if (contourFinder.getContourArea(i)>arclength3 && contourFinder.getContourArea(i)<arclength2){
            ofSetColor(0,255,255);
        }
        if (contourFinder.getContourArea(i)>arclength2 && contourFinder.getContourArea(i)<arclength1){
            ofSetColor(0,255,0);
        }
        if (contourFinder.getContourArea(i)>arclength1){
            ofSetColor(255,0,0);
        }
       // ofSetLineWidth(3);
        
        contourFinder.getPolylines()[i].draw();
        //ofDrawRectangle(toOf(contourFinder.getBoundingRect(i)));
    }
    ofPopStyle();
    
    ofPopMatrix();
    
    haarfinder.draw();
  
    
    ofPushMatrix();
    //ofScale(0.7,0.7);
    ofTranslate(0, 0);
    ofPushStyle();
    for(int i = 0; i < linesToAnimate.size(); i++) {
        //linesToAnimate[i].draw();
       /* for (int p=0; p<100; p+=10) {
            ofVec3f point =  linesToAnimate[i].getPointAtPercent(p/100.0);  // Returns a point at a percentage along the polyline
            ofDrawCircle(point, 1);
        }*/
        
        //if(linesToAnimate[i].getPerimeter()>100){
        /*vector<ofVec3f> vertices = linesToAnimate[i].getVertices();
        float normalLength = 10;
        for (int vertexIndex=0; vertexIndex<vertices.size(); vertexIndex++) {
            ofVec3f vertex = vertices[vertexIndex];  // Get the vertex
            ofVec3f normal = linesToAnimate[i].getNormalAtIndex(vertexIndex) * normalLength;  // Scale the normal
            ofDrawLine(vertex-normal/2, vertex+normal/2);  // Center the scaled normal around the vertex
        }*/
            
        //}
        
    }
    
    /*
    if(linesToAnimate.size()>0){
        linesToAnimate[animationPolylineIndex].simplify();
        vector<ofVec3f> vertices = linesToAnimate[animationPolylineIndex].getVertices();
        for (int vertexIndex=1; vertexIndex<animationVerexIndex; vertexIndex++) {
            ofVec3f vertex = vertices[vertexIndex];  // Get the vertex
            ofVec3f vertex2 = vertices[vertexIndex-1];  // Get the vertex
            ofDrawLine(vertex, vertex2);  // Center the scaled normal around the vertex

        }
        animationVerexIndex++;
        if(animationVerexIndex>vertices.size()-1){
            animationVerexIndex=1;
            animationPolylineIndex++;
            if(animationPolylineIndex>linesToAnimate.size()-1){
                animationPolylineIndex=0;
            }
        }
    }
    
    for(int i = 0; i < animationPolylineIndex; i++) {
        linesToAnimate[i].draw();
    }
        
 */
   // for(int i = 0; i < linesToAnimate.size(); i++) {
    
  
    
    ofPopStyle();
    ofPopMatrix();
    
   
    
    ofPushMatrix();
  //  ofScale(0.7,0.7);
    ofTranslate(0, 500);
    
    ofPushStyle();
    ofSetColor(255, 0, 255);
   // cout<<" lines "<<linesToDraw1.size()<<" "<<drawcounter<<endl;
    if(drawcounter<linesToDraw1.size()){
        drawcounter++;
    }else{
        drawcounter=0;
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
  //  linesToDraw1[drawcounter].draw();

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
    
   /* ofPolyline polyline2;

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
    
    ofPopMatrix();*/
    

    
/*    for(int k=0;k<n2;k++){
        
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
/*        approxPolyDP(contourFinder2.getContour(k),quad2, 0.05 ,true);

        
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
 
    }*/
    gui.draw();
}

//--------------------------------------------------------------
void ofApp::makeNewPortrait(){
    if(colorImg.getWidth()!=cam.getWidth()){
    colorImg.allocate(cam.getWidth(),cam.getHeight());
    grayImage.allocate(cam.getWidth(),cam.getHeight());
    zoom.allocate(cam.getWidth(),cam.getHeight());
    grayImageBlur.allocate(cam.getWidth(),cam.getHeight());
    }
    
    colorImg.setFromPixels(cam.getPixels());

    grayImage=colorImg;
    grayImage.brightnessContrast(brightness, contrast);
    grayImageBlur=colorImg;
    grayImage.blur(blur);

   // grayImage.contrastStretch();
    grayImage.dilate();
    grayImage.erode();
    
    
    grayImageBlur.blur(blur);
    grayImageBlur.contrastStretch();
    grayImageBlur.dilate();
    grayImageBlur.erode();
    
    
}


void ofApp::sendFeed(){
    if (commands.size()>0) {
        cout<<commands.size()<<" commands in buffer"<<endl;
        string cmd = commands[0];
        cout<<"command " <<cmd<<endl;
        cmd+='\n';
        //myPort.write(cmd);
        commands.erase(commands.begin());
        done=false;
        
        
        message = cmd;
        remember = false;
        
    }
    
}

void ofApp::waitPen(int mil){
    string cmd = "M12 "+ofToString(mil)+"d";
    commands.push_back(cmd);
}

void ofApp::turnPen(int ang){
    string cmd = "M11 "+ofToString(ang)+"d";
    commands.push_back(cmd);
}

void ofApp::turnIdle(){
    penHighUp();
    waitPen(200);
    turnPen(penIdlePos);
    waitPen(200);
 
}

void ofApp::turnDraw(){
    penHighUp();
    waitPen(200);
    turnPen(penDrawPos);
    penUp();
}

void ofApp::penHighUp(){
    string cmd;
    cmd = "M1 "+ofToString(penHighUpPos)+"d";
    commands.push_back(cmd);
}


void ofApp::penUp(){
    string cmd;
    cmd = "M1 "+ofToString(penUpPos)+"d";
    commands.push_back(cmd);
}

void ofApp::penDown(){
    string cmd;
    cmd = "M1 "+ofToString(penDownPos)+"d";
    commands.push_back(cmd);
}

void ofApp::goHome(){
    commands.clear();
    penUp();
    string cmd = "G1 X"+ofToString(int(0))+" Y"+ofToString(int(0));
    commands.push_back(cmd);
    remember = false;
    bSendFeed=true;
    done=true;
}


void ofApp::makeFeed(){
    cout<<"Make Feed"<<linesToPrint.size()<<endl;
    commands.clear();
    for(int i = 0; i < linesToPrint.size(); i++) {
        linesToPrint[i].simplify();
        vector<ofVec3f> vertices = linesToPrint[i].getVertices();
        cout<<"verts "<<vertices.size()<<endl;
        
        //pen Up
        penUp();
        //string cmd = "G1 X"+ofToString(int(grayImage.getWidth()/2*drawZoomFact-vertices[0].x))+" Y"+ofToString(int(grayImage.getHeight()*drawZoomFact-vertices[0].y));
        
       //string cmd = "G1 X"+ofToString(int(faceBoundingBox.getWidth()/2-vertices[0].x))+" Y"+ofToString(int(faceBoundingBox.getHeight()-vertices[0].y));
        cmd=formGString(vertices[0].x,vertices[0].y);
        commands.push_back(cmd);
        
        //pen Down
        penDown();
       // waitPen(50);

        for (int vertexIndex=0; vertexIndex<vertices.size(); vertexIndex++) {
            ofVec3f vertex = vertices[vertexIndex];  // Get the vertex
           // string cmd = "G1 X"+ofToString(int(grayImage.getWidth()/2*drawZoomFact-vertex.x))+" Y"+ofToString(int(grayImage.getHeight()*drawZoomFact-vertex.y));
          //  string cmd = "G1 X"+ofToString(int(faceBoundingBox.getWidth()/2-vertex.x))+" Y"+ofToString(int(grayImage.getHeight()-vertex.y));
            cmd=formGString(vertex.x,vertex.y);

            commands.push_back(cmd);
        }

    }
    cout<<commands.size()<<endl;
    for (int i=0; i<commands.size(); i++) {
        //cout<<commands[i]<<endl;
    }
    
    bSendFeed=true;
    done=true;
    
}


int ofApp::getTransX(float x){
    int rX=int(drawScaleFact*(faceBoundingBox.getWidth()/2)-(x*drawScaleFact));
    return rX;
}


int ofApp::getTransY(float y){
    int rY=int(drawScaleFact*(faceBoundingBox.getHeight())-(y*drawScaleFact));
    return rY;
}

string ofApp::formGString(float x, float y){
     string s = "G1 X"+ofToString(getTransX(x))+" Y"+ofToString(getTransY(y));
    return s;
}

void ofApp::makeBoundingRectFeed(){
    cout<<"Make Bounding Rect Feed"<< faceBoundingBox.getPosition().x<<" "<<faceBoundingBox.getWidth()<<endl;
    commands.clear();

    penUp();

    
    string cmd = "G1 X"+ofToString(getTransX(0))+" Y"+ofToString(getTransY(0));
    commands.push_back(cmd);
    penDown();
    
    cmd = "G1 X"+ofToString(getTransX(faceBoundingBox.getWidth()))+" Y"+ofToString(getTransY(0));
    commands.push_back(cmd);
        
    cmd = "G1 X"+ofToString(getTransX(faceBoundingBox.getWidth()))+" Y"+ofToString(getTransY(faceBoundingBox.getHeight()));
    commands.push_back(cmd);
    
    cmd = "G1 X"+ofToString(getTransX(0))+" Y"+ofToString(getTransY(faceBoundingBox.getHeight()));
    commands.push_back(cmd);
    
    cmd = "G1 X"+ofToString(getTransX(0))+" Y"+ofToString(getTransY(0));
    commands.push_back(cmd);
    
    cout<<commands.size()<<endl;
    bSendFeed=true;
    done=true;
}




void ofApp::onNewMessage(string & message)
{
    cout << "onNewMessage, message: " << message << "\n";
    if(message=="OK"){
         done=true;
    }
    
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    if(key == ' '){
        makeNewPortrait();
        
    }
    
    
    if(key == 'h'){
        bGoHome=true;
    
    }
    
    if(key == 'C'){
        continousDraw=!continousDraw;
        
    }

    if(key=='r'){
        
        record=true;
    }
    
    if(key=='u'){
        penUp();
        remember = false;
        bSendFeed=true;
        done=true;

    }
    
    if(key=='d'){
        penDown();
        remember = false;
        bSendFeed=true;
        done=true;

    }
    
    if(key=='b'){
        makeBoundingRectFeed();
    }
    
    if(key=='i'){
        turnIdle();
        remember = false;
        bSendFeed=true;
        done=true;
    }
    
    if(key=='I'){
        turnDraw();
        remember = false;
        bSendFeed=true;
        done=true;
    }
    
    if(key=='c'){
        bMakeContours=true;
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
