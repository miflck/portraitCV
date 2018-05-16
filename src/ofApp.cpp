#include "ofApp.h"

using namespace ofxCv;
using namespace cv;

cv::Mat cam_mat;
cv::Mat crop;

cv::Mat cam_mat2;
cv::Mat crop2;



int baud = 115200;
char myByte = 0;
string cmd;

bool bUseArduino=true;


static bool sortByCriteria(const ofPoint &a, const ofPoint &b){
    return a.x < b.x;
}

static bool sortByCriteriaX(const ofPolyline &a, const ofPolyline &b){
    return a.getBoundingBox().x < b.getBoundingBox().x;
}

static bool sortByArea(const ofPolyline &a, const ofPolyline &b){
    if(a.getPerimeter() > 70 || b.getPerimeter()>70){
        return a.getPerimeter() > b.getPerimeter();
    }else{
        return a.getBoundingBox().x < b.getBoundingBox().x;
    }
}

static bool sortByDistance(const ofPolyline &a, const ofPolyline &b){
    ofVec2f n=ofVec2f(0,0);
    ofVec2f aV=a.getPointAtPercent(0);
    ofVec2f bV=b.getPointAtPercent(0);
    float d1=ofDist(aV.x,aV.y,n.x, n.y);
    float d2=ofDist(bV.x,bV.y,n.x, n.y);
    return d1>d2;
}





//--------------------------------------------------------------
void ofApp::setup(){
    
font.load("frabk.ttf", 50);

  serial.listDevices();
    if(bUseArduino) {
        vector <ofSerialDeviceInfo> deviceList = serial.getDeviceList();
       // serial.setup("tty.wchusbserial1420", baud); //open the first device
        serial.setup("tty.wchusbserial1420", baud); //open the first device

        serial.startContinuousRead(false);
        ofAddListener(serial.NEW_MESSAGE,this,&ofApp::onNewMessage);
        
        
        
       //button.setup("tty.usbmodem141221", 9600); //open the first device
        button.setup("tty.usbmodem1471", 9600);//mac mini
        
        // serial.setup("/dev/tty.usbserial-A70060V8", 9600);
        button.startContinuousRead();
        ofAddListener(button.NEW_MESSAGE,this,&ofApp::onNewButtonMessage);
        
        
    }
    
    
    bool        bSendSerialMessage=false;            // a flag for sending serial
    message = "";
    
    
    
    dmx.connect("tty.usbserial-EN210513"); // use the name
    dmxValue=0;
    
    
    //0ofSetLogLevel(OF_LOG_VERBOSE);
    
    
    img.load("michaelflueckiger.jpeg");
    //img.load("Brad_Pitt_Fury_2014.jpg");

    mask.load("mask.png");
    
    
    
    
    colorImg.setFromPixels(img.getPixels());
    grayImage=colorImg;
   
    
    cam.listDevices();
    cam.setDeviceID(0);

   // cam.setup(1920,1080);
    cam.setup( 1600, 896);
   
    
    face.allocate(1600, 896);

    
    
    gui.setup("General","general.xml",300,10);
    polylinesPanel.setup("Polylines","polylines.xml",10,10);
    cannyPanel.setup("Canny","canny.xml",10,10);

    display.setName("Dispaly");
    display.add(bShowImage.set("bShowImage",false));
    display.add(bShowCanny.set("bShowCanny",false));
    display.add(bShowCanny2.set("bShowCanny2",false));
    display.add(bShowDebug.set("bShowDebug",false));
    display.add(bShowDebugLines.set("bShowDebugLines",false));
    gui.add(display);

    imageparameters.setName("Image");
    imageparameters.add(contrast.set("contrast",0, -1, 1));
    imageparameters.add(brightness.set("brightness", 0, -1, 1));
    imageparameters.add(blur.set("blur", 0, 0, 10));
    imageparameters.add(zoomfact.set("zoom", 1, 0, 2));
    imageparameters.add(facezoomfact.set("facezoom", 1, 0, 2));

    gui.add(imageparameters);
    gui.getGroup("Image").minimize();
  
    canny1.setName("Canny 1");
    canny1.add(dilateErode.set("dilateErode", 2, 0, 5));
    canny1.add(mincanny.set("mincanny", 80, 10, 500));
    canny1.add(maxcanny.set("maxcanny", 100, 15, 300));
    canny1.add(cannyblur.set("cannyblur", 0, 0, 10));
    canny1.add(cannycontrast.set("cannycontrast", 0,-1, 1));
    canny1.add(cannybrightness.set("brightness", 0, -1, 1));
    canny1.add(threshold.set("Threshold", 128, 0, 255));

    cannyPanel.add(canny1);
    canny2group.setName("Canny 2");
    canny2group.add(dilateErode2.set("dilateErode2", 2, 0, 5));
    canny2group.add(mincanny2.set("mincanny2", 80, -100,2000));
    canny2group.add(maxcanny2.set("maxcanny2", 100, -100, 2000));
    canny2group.add(cannyblur2.set("cannyblur2", 0, 0, 10));
    canny2group.add(cannycontrast2.set("cannycontrast2", 0,-1, 1));
    canny2group.add(cannybrightness2.set("brightness2", 0, -1, 1));
    canny2group.add(threshold2.set("Threshold 2", 128, 0, 255));

    cannyPanel.add(canny2group);
    cannyPanel.add(bUseCanny1.set("bUseCanny1",false));

    finder1.setName("Contourfinder 1");
    finder1.add(minArea.set("Min area", 1, 1, 100));
    finder1.add(maxArea.set("Max area", 200, 1, 8000));
    finder1.add(holes.set("Holes", true));
    finder1.add(simply.set("Simple", false));
    
    polyline1.setName("Polyline 1");
    polyline1.add(quadhard.set("quadhard", 30, 0, 50));
    polyline1.add(resample.set("resample", 3, 0, 80));
    polyline1.add(smooth.set("smooth", 0, 0, 5));
    polyline1.add(simplify.set("simplify", 0, 0, 20));
    polyline1.add(area1min.set("area1 min", 3, 1, 100));
    polyline1.add(area1max.set("area1 max", 3, 1, 8000));
    finder1.add(polyline1);
    
    
    polylinehalf.setName("Polyline Half");
    polylinehalf.add(resample_half.set("resample_half", 3, 0, 80));
    polylinehalf.add(polyhalf_percent.set("polyhalf_percent", 5, 1, 50));
    polylinehalf.add(simplify_half.set("simplify_half", 0, 0, 20));
    polylinehalf.add(area_half_min.set("area_half_min", 3, 1, 200));
    polylinehalf.add(area_half_max.set("area_half_max", 3, 1, 8000));
    finder1.add(polylinehalf);

    polyline2.setName("Polyline 2");
    polyline2.add(quadsmooth.set("quadsmooth", 8, 0, 30));
    polyline2.add(resample2.set("resample2", 3, 0, 80));
    polyline2.add(smooth2.set("smooth2", 0, 0, 5));
    polyline2.add(simplify2.set("simplify2", 0, 0, 20));
    polyline2.add(area2min.set("area2 min", 3, 1, 500));
    polyline2.add(area2max.set("area2 max", 3, 1, 2000));
    finder1.add(polyline2);

    polyline3.setName("Polyline 3");
    polyline3.add(quadfine.set("quadfine", 1, 0, 10));
    polyline3.add(resample3.set("resample3", 3, 0, 80));
    polyline3.add(smooth3.set("smooth3", 0, 0, 5));
    polyline3.add(simplify3.set("simplify3", 0, 0, 20));
    polyline3.add(area3min.set("area3 min", 3, 1, 50));
    polyline3.add(area3max.set("area3 max", 3, 1, 500));
    finder1.add(polyline3);


    //finder1.add(sortthreshold.set("sortthreshold", 0.5, 0, 20));


    
    finder1.add(poly.set("poly",false));
    finder1.add(poly2.set("poly2",false));
    finder1.add(poly3.set("poly3",false));
    finder1.add(polyhalf.set("polyhalf",false));

    polylinesPanel.add(finder1);
   // gui.add(finder1);
    gui.add(scaleScreen.set("scaleScreen",false));

    
    

    
    finder2.setName("Contourfinder 2");
    finder2.add(minArea2.set("Min area2", 1, 1, 300));
    finder2.add(maxArea2.set("Max area2", 200, 1, 500));

    finder2.add(holes2.set("Holes2", true));
    
    
    
    
    finder2.add(eyeAreamin.set("eyeAreamin2", 1, 1, 300));
    finder2.add(eyeAreamax.set("eyeAreamax", 200, 1, 500));
    finder2.add(eyeresample.set("eyeresample", 0, 1, 30));
    finder2.add(eyesmooth.set("eyesmooth", 0, 1, 10));
    finder2.add(eyesimplify.set("eyesimplify", 0, 1, 10));
    finder2.add(eyequad.set("eyequad", 0, 0.1, 5));
    gui.add(finder2);
    //gui.getGroup("Contourfinder 2").minimize();

    //gui.add(persistence.set("persistence", 15, 1, 100));

    robot.setName("Robot");

    robot.add(penUpPos.set("penUpPos", 145, -100, 180));
    robot.add(penHighUpPos.set("penHighUpPos", 147, 0, 180));
    robot.add(penHighDownPos.set("penHighDownPos", 30, 0, 180));
    robot.add(penDownPos.set("penDownPos", 162, 0, 180));
    robot.add(penIdlePos.set("penIdlePos", 142, 0, 180));
    robot.add(penDrawPos.set("penDrawPos", 118, 0, 180));
    robot.add(dipPosX.set("dipPosX", 0, 0, -300));
    robot.add(dipPosY.set("dipPosY", 0, 0, -300));
    robot.add(waitPosX.set("waitPosX", 0, -1000, 1000));
    robot.add(waitPosY.set("waitPosY", 1000, 0, 1600));
    robot.add(dimAmmount.set("dimAmmount", 0, 0, 255));
    gui.add(robot);
    gui.getGroup("Robot").minimize();

    
    gui.loadFromFile("general.xml");
    polylinesPanel.loadFromFile("polylines.xml");
    cannyPanel.loadFromFile("canny.xml");
    
    ofBackground(0);
    //makeNewPortrait();
    makeContours();
    inittime=ofGetElapsedTimeMillis();
    initIdletime=ofGetElapsedTimeMillis();
    turnLightsOn();
}

//--------------------------------------------------------------
void ofApp::update(){
    
    if (stateBefore!=state){
        switch (state) {
            case IDLE:
                turnLightsOn();
                initIdletime=ofGetElapsedTimeMillis();
                break;
            default:
                break;
        }
    }
    
    if(ofGetElapsedTimeMillis()-initIdletime>idleTimerDuration){
        //goDip();
        initIdletime=ofGetElapsedTimeMillis();
    }
    
    
    switch (state) {
        case IDLE:
            break;
            
        case DRAWING:
            turnLightsOff();
            if(commands.size()<=0 &! bGoHome){
                turnDraw();
                waitPos();
                turnIdle();
                state=IDLE;
            }
            break;
            
        default:
            break;
    }
    
    // DMX
    dmx.setLevel(1, dmxValue);
    dmx.setLevel(2, dmxValue);
    dmx.setLevel(3, dmxValue);


    dmx.update();
    
    
    if(bUseArduino){
        if(message != "")
        {
        cout << "sending message: " << message << "\n";
        serial.writeString(message);
        message = "";
        }
        
        if (done &&!bGoHome){
            sendFeed();
        }
        
        if(done && bGoHome){
            goHome();
            bGoHome=false;
        }
    }
    
    cam.update();
    if(cam.isFrameNew()){
        ofxCvColorImage  c;
        c.allocate(cam.getWidth(),cam.getHeight());
        c.setFromPixels(cam.getPixels());
        if(face.getWidth()!=c.getWidth()){
            face.allocate(c.getWidth(), c.getHeight());
        }
        face=c;
        face.mirror(false,true);
        face.transform(0, face.getWidth()/2, face.getHeight()/2, facezoomfact, facezoomfact, 0, 0);
    }

    
    if(continousDraw){
        if(ofGetElapsedTimeMillis()-inittime>timerduration){
        makeNewPortrait();
        makeContours();
        inittime=ofGetElapsedTimeMillis();
        }
    }
    
    if(bMakeContours)  {
        makeContours();
        bMakeContours=false;
    }
    
    if(record){
       // makeContours();
        record=false;
        state=DRAWING;
        makeFeed();
    }
    
    if(bMakeNewPortraitwidthTimer && ofGetElapsedTimeMillis()-portraitinittime>portraitTimerDuration){
        makeNewPortraitWithTimerFinished();
    }

    
    
    
    
    stateBefore=state;
}




void ofApp::makeContours(){
    grayImage.contrastStretch();
    zoom=grayImage;
    zoom.transform(0, zoom.getWidth()/2, zoom.getHeight()/2, zoomfact, zoomfact, 0, 0);
   
    if(canny.getWidth()!=grayImage.getWidth()){
        canny.allocate(grayImage.getWidth(), grayImage.getHeight());
    }
    if(canny2.getWidth()!=grayImage.getWidth()){
        canny2.allocate(grayImage.getWidth(), grayImage.getHeight());
    }

    cvCanny(zoom.getCvImage(), canny.getCvImage(), mincanny, maxcanny,3);
    canny.blur(cannyblur);
    canny.threshold(threshold);

    for(int i=0;i<dilateErode;i++){
        canny.dilate();
    }

    for(int i=0;i<dilateErode;i++){
        canny.erode();
    }
    canny.brightnessContrast(cannybrightness,cannycontrast);
    canny.flagImageChanged();
   
    zoom.blur(blur);
    cvCanny(zoom.getCvImage(), canny2.getCvImage(), mincanny2, maxcanny2,5);
    
    for(int i=0;i<dilateErode2;i++){
        canny2.dilate();
    }
    for(int i=0;i<dilateErode2;i++){
        canny2.erode();
    }
    
    canny2.blur(cannyblur2);
    canny2.brightnessContrast(cannybrightness2,cannycontrast2);
    canny2.threshold(threshold2);
    canny2.flagImageChanged();
    
    finder.setup("haarcascade_frontalface_default.xml");
    finder.findHaarObjects(zoom,150,150);
   
    if(finder.blobs.size()>0){
        ofRectangle cur =finder.blobs[0].boundingRect;
        faceBoundingBox = finder.blobs[0].boundingRect;
        faceBoundingBoxOriginal=finder.blobs[0].boundingRect;
        
       
        
        // Find ROI
        int diffx,diffy,dx,dy,hy;
        int offsetty=150;
        diffx=cur.x+cur.width;
        diffy=cur.y-offsetty/2;
        if(cur.y-offsetty/2<=0){
            dy=cur.y;
        }else{
            dy=cur.y-offsetty/2;
        }
        if(cur.y+cur.height+offsetty>=canny.getHeight()){
            hy=canny.getHeight()-dy;
        }else{
            hy=cur.height+offsetty;
        }
        faceBoundingBox.y-=dy;
        faceBoundingBoxOriginal.y-=dy;
        faceBoundingBox.height=hy;

        if(bUseCanny1){
            cam_mat = toCv(canny);

        }else{
            cam_mat = toCv(canny2);
        }
        
        cv::Rect crop_roi = cv::Rect(cur.x, dy, cur.width, hy);
        crop = cam_mat(crop_roi).clone();
        
        contourFinder.setMinAreaRadius(minArea);
        contourFinder.setMaxAreaRadius(maxArea);
        contourFinder.setSimplify(simply);
        contourFinder.setFindHoles(holes);
        contourFinder.findContours(crop);
        
        
        int n = contourFinder.size();
        
        cout<<"Contours1: "<<n<<endl;
        
        
        
    
    }
    hfinder2.setup("haarcascade_mcs_eyepair_small.xml");
    //finder.setPreset(ObjectFinder::Fast);
    //haarfinder.update(colorImg);
    hfinder2.findHaarObjects(zoom);
    
    cout<<"eyefinder: "<<hfinder2.blobs.size()<<endl;

    
    if(hfinder2.blobs.size()>0){
        cout<<"eyes"<<hfinder2.blobs.size()<<endl;
        ofRectangle cur =hfinder2.blobs[0].boundingRect;
        eyeBoundingBox=cur;
        
        cam_mat2 = toCv(canny);
        cv::Rect crop_roi = cv::Rect(cur.x, cur.y, cur.width, cur.height);
        crop2 = cam_mat2(crop_roi).clone();
        
        contourFinder2.setMinAreaRadius(minArea2);
        contourFinder2.setMaxAreaRadius(maxArea2);
        contourFinder2.setThreshold(threshold);
        contourFinder2.setSimplify(simply);
        contourFinder2.setFindHoles(holes2);
        contourFinder2.findContours(crop2);
        
        int n2 = contourFinder2.size();
        cout<<"Contours2: "<<n2<<endl;

    }
    makePolylines();
  //  makeEyePolylines();
}



void ofApp::makeEyePolylines(){
    eyes.clear();
    vector<ofPolyline> polylines;
    int n = contourFinder2.size();
    for(int k=0;k<n;k++){
        vector<cv::Point> quad;
        approxPolyDP(contourFinder2.getContour(k),quad, eyequad ,true);
        ofPolyline polyline;
        for(int i = 0; i < quad.size(); i++) {
            polyline.addVertex(quad[i].x, quad[i].y);
        }
        polyline = polyline.getResampledBySpacing(eyeresample);
        polyline = polyline.getSmoothed(eyesmooth);
        polyline.simplify(eyesimplify);
        if(polyline.getPerimeter()>1&& polyline.getPerimeter()>eyeAreamin && ABS(polyline.getPerimeter())<eyeAreamax){
            eyes.push_back(polyline);
        }
    }
    cout<<"Eyes poly"<<eyes.size()<<endl;
}


void ofApp::makePolylines(){
  
    int n = contourFinder.size();
    
    //clear all lines
    linesToDraw1.clear();
    linesToDraw2.clear();
    linesToDraw3.clear();
    
    medianlines.clear();
    half_linesToDraw1.clear();
    linesToPrint.clear();
    linesToAnimate.clear();

    
    
    for(int k=0;k<n;k++){
        
        vector<cv::Point> quad;
        vector<cv::Point> quad2;
        vector<cv::Point> quad3;

        
        approxPolyDP(contourFinder.getContour(k),quad, quadhard ,true);
        approxPolyDP(contourFinder.getContour(k),quad2, quadsmooth ,true);
        approxPolyDP(contourFinder.getContour(k),quad3, quadfine ,true);

        
        ofSetColor(255);
        
        ofPolyline polyline;
        ofPolyline polyline2;
        ofPolyline polyline3;
        
        ofPolyline polyline_half;

        ofPolyline polyline_line;
        ofPolyline polyline_median;

        vector <ofVec2f> points;
        
        // get contour as polyline
        for(int i = 0; i < contourFinder.getContour(k).size(); i++) {
            polyline_line.addVertex(contourFinder.getContour(k)[i].x, contourFinder.getContour(k)[i].y);
        }
        
        for (int p=0; p<100; p+=10) {
            ofVec2f point =  polyline_line.getPointAtPercent(p/100.0);
            points.push_back(point);
        }
        
        vector <ofVec2f> middlepoints;
        for(int i = 0; i < points.size()/2; i++) {
           ofVec2f p= points[i]-points[points.size()-1];
            ofVec2f m= points[i]+p/2;
            middlepoints.push_back(m);
        }
        
        for(int i = 0; i < middlepoints.size(); i++) {
            polyline_median.addVertex(middlepoints[i].x, middlepoints[i].y);
        }
        medianlines.push_back(polyline_median);
        
        
        
        for(int i = 0; i < quad.size(); i++) {
            polyline.addVertex(quad[i].x, quad[i].y);
        }
        
    
        
        for(int i = 0; i < quad2.size(); i++) {
            polyline2.addVertex(quad2[i].x, quad2[i].y);
        }
        
        for(int i = 0; i < quad3.size(); i++) {
            polyline3.addVertex(quad3[i].x, quad3[i].y);
        }
        
        for (int p=0; p<50; p+=polyhalf_percent) {
            ofVec2f point =  polyline_line.getPointAtPercent(p/100.0);
            polyline_half.addVertex(point);
        }
        
        
        polyline = polyline.getResampledBySpacing(resample);
        polyline = polyline.getSmoothed(smooth);
        polyline.simplify(simplify);
        
        polyline2 = polyline2.getResampledBySpacing(resample2);
        polyline2 = polyline2.getSmoothed(smooth2);
        polyline2.simplify(simplify2);

        polyline3 = polyline3.getResampledBySpacing(resample3);
        polyline3 = polyline3.getSmoothed(smooth3);
        polyline3.simplify(simplify3);

        polyline_half = polyline_half.getResampledBySpacing(resample_half);
        polyline_half = polyline_half.getSmoothed(smooth_half);
        polyline_half.simplify(simplify_half);
        //cout<<"n "<<k<<" quad.size() "<<quad.size()<<" area "<<polyline3.getPerimeter()<<endl;

        
        //cout<<"n "<<k<<" arc length "<<contourFinder.getArcLength(k)<<endl;
      

            if(polyline.getPerimeter()>1&& polyline.getPerimeter()>area1min && ABS(polyline.getPerimeter())<area1max){
           /* if(linesToDraw1.size()>0){
                ofPolyline p;
                p.addVertex(linesToDraw1.back().getPointAtPercent(100));
                p.addVertex(polyline.getPointAtPercent(0));
                linesToDraw1.push_back(p);
            }*/
                linesToDraw1.push_back(polyline);
            }
        
        
        if(polyline_half.getPerimeter()>1&& polyline_half.getPerimeter()>area_half_min && ABS(polyline_half.getPerimeter())<area_half_max){
            /* if(linesToDraw1.size()>0){
             ofPolyline p;
             p.addVertex(linesToDraw1.back().getPointAtPercent(100));
             p.addVertex(polyline.getPointAtPercent(0));
             linesToDraw1.push_back(p);
             }*/
            half_linesToDraw1.push_back(polyline_half);
        }

        if(polyline2.getPerimeter()>1 && polyline2.getPerimeter()>area2min && ABS(polyline2.getPerimeter())<area2max){
            
           /* if(linesToDraw2.size()>0){
                ofPolyline p;
                p.addVertex(linesToDraw2.back().getPointAtPercent(100));
                p.addVertex(polyline2.getPointAtPercent(0));
                linesToDraw2.push_back(p);
            }*/
            
            linesToDraw2.push_back(polyline2);

        }
        
        
       
        
        if(ABS(polyline3.getPerimeter())>1&& polyline3.getPerimeter()>area3min && ABS(polyline3.getPerimeter())<area3max){
           /* if(linesToDraw3.size()>0){
            ofPolyline p;
            p.addVertex(linesToDraw3.back().getPointAtPercent(100));
            p.addVertex(polyline3.getPointAtPercent(0));
            linesToDraw3.push_back(p);
            }*/
            linesToDraw3.push_back(polyline3);
        }
        
        
    }
    
    
   ofSort(linesToDraw1, sortByArea);
   ofSort(linesToDraw2, sortByArea);
    ofSort(linesToDraw3, sortByArea);
    ofSort(half_linesToDraw1, sortByArea);

    
    //linesToPrint=linesToDraw1;
    //linesToPrint=half_linesToDraw1;
    if(poly){
        for(int i=0;i<linesToDraw1.size();i++){
            linesToPrint.push_back(linesToDraw1[i]);
        }
    }
    if(poly2){
        for(int i=0;i<linesToDraw2.size();i++){
            linesToPrint.push_back(linesToDraw2[i]);
        }
    }
    if(poly3){
        for(int i=0;i<linesToDraw3.size();i++){
            linesToPrint.push_back(linesToDraw3[i]);
        }
    }
    
    if(polyhalf){
        for(int i=0;i<half_linesToDraw1.size();i++){
            linesToPrint.push_back(half_linesToDraw1[i]);
        }
    }
    
    ofSort(linesToPrint, sortByArea);
    
    linesToAnimate=linesToDraw1;
    
    for(int i=0;i<linesToDraw2.size();i++){
        linesToAnimate.push_back(linesToDraw2[i]);
    }
    for(int i=0;i<linesToDraw3.size();i++){
        linesToAnimate.push_back(linesToDraw3[i]);
    }
    
    


    
}


//--------------------------------------------------------------
void ofApp::draw(){
    
    
    string s="beschäftigt...";
    ofRectangle bb;

    switch (state) {
        case IDLE:
            face.draw(ofGetWidth()/2-face.getWidth()/2,ofGetHeight()/2-face.getHeight()/2,face.getWidth(),face.getHeight());
            mask.draw(0,0, mask.getWidth(), mask.getHeight());
            //ofPushStyle();
            //ofNoFill();
            //ofSetColor(255,0,0);
            //ofDrawEllipse(face.getWidth()/2,face.getHeight()/2,600,face.getHeight()-50);
            //ofPopStyle();

            break;
        
          case DRAWING:
            ofPushStyle();
            ofSetColor(0, 0, 0);
            ofDrawRectangle(0, 0, ofGetWidth(), ofGetHeight());
            ofSetColor(255);
            
            bb=font.getStringBoundingBox(s, 0, 0);
            font.drawString(s, ofGetWidth()/2-bb.getWidth()/2, ofGetHeight()/2);
           // font.drawString(ofToString(commands.size())+" commands to go", ofGetWidth()/2-500, ofGetHeight()/2);
            ofPopStyle();

            break;
            
        default:
            
           // face.draw(0,0,face.getWidth(),face.getHeight());
           // mask.draw(0,0, mask.getWidth(), mask.getHeight());
            break;
            

    }
    
    
    if(bMakeNewPortraitwidthTimer){
        ofPushStyle();
        ofSetColor(255, 0, 0);
        font.drawString(ofToString(((portraitTimerDuration-(ofGetElapsedTimeMillis()-portraitinittime))/1000)+1), ofGetWidth()/2-20, ofGetHeight()/2);
        ofPopStyle();
    }
    

    
   // font.drawString(ofToString(idleTimerDuration-ofGetElapsedTimeMillis()-initIdletime)+" "+ofToString(idleTimerDuration), ofGetWidth()/2-500, ofGetHeight()/2);

    
    
    if(bShowDebug){
    zoom.draw(ofGetWidth()-zoom.getWidth()/3,0,zoom.getWidth()/3,zoom.getHeight()/3);

    if(bShowImage){
        grayImage.draw(0,0);
    }
        
    cam.draw(ofGetWidth()-cam.getWidth()/3, grayImage.getHeight()/3,cam.getWidth()/3,cam.getHeight()/3);
    ofPushStyle();
    ofSetColor(255);
    
    if(!bShowCanny){
         canny.draw(ofGetWidth()-canny.getWidth()/3, canny.getHeight()/3*2,canny.getWidth()/3,canny.getHeight()/3);
    }else{
       canny.draw(0,0);
    }
        
        if(!bShowCanny2){
            canny2.draw(ofGetWidth()-canny.getWidth()/3*2, canny.getHeight()/3*3,canny.getWidth()/3,canny.getHeight()/3);

        }else{
            canny2.draw(0,0);

        }
        ofPopStyle();
    }
    

    if(bShowDebugLines){
        
        int gutter=100;
        ofPushStyle();
        ofSetColor(0);
        ofDrawRectangle(0, 0, ofGetWidth(), ofGetHeight());
        ofPopStyle();
        ofPushMatrix();
        ofScale(scaleScreen,scaleScreen);
        ofPushStyle();
        ofNoFill();
        ofSetLineWidth(2);
        for(int i = 0; i < (int)contourFinder.getPolylines().size(); i++) {
             ofSetColor(255);
            
            if (contourFinder.getContourArea(i)>30){
                ofSetColor(255,0,0);
            }
            if (contourFinder.getContourArea(i)>80){
                ofSetColor(0,255,0);
            }
            if (contourFinder.getContourArea(i)>100){
                ofSetColor(0,0,255);
            }
            if (contourFinder.getContourArea(i)>300){
                ofSetColor(255,255,0);
            }
            
            if (contourFinder.getContourArea(i)>800){
                ofSetColor(0,255,255);
            }
            
            if (contourFinder.getContourArea(i)>5000){
                ofSetColor(255,0,255);
            }
            contourFinder.getPolylines()[i].draw();
        }
        ofPopStyle();
        ofPopMatrix();
    
        
        
        ofPushMatrix();
        ofTranslate(faceBoundingBox.getWidth()*scaleScreen+gutter*scaleScreen, 0);

        ofPushMatrix();
        ofScale(scaleScreen,scaleScreen);
        ofPushStyle();
        ofSetColor(255, 255, 255);
        if(drawcounter<linesToPrint.size()){
            drawcounter++;
        }else{
            drawcounter=0;
        }
        ofTranslate(0, 0);
        for(int i = 0; i < drawcounter; i++) {
            linesToPrint[i].draw();
        }
        ofNoFill();
        ofSetColor(255,0,0);
        ofDrawRectangle(0,0,faceBoundingBox.getWidth(),faceBoundingBox.getHeight());
        ofPopStyle();
        ofPopMatrix();
        
        ofTranslate(faceBoundingBox.getWidth()*scaleScreen+gutter*scaleScreen, 0);

        ofPushMatrix();
        ofPushStyle();
        ofSetColor(255, 255, 0);
        ofScale(scaleScreen,scaleScreen);
        for(int i = 0; i < linesToPrint.size(); i++) {
            linesToPrint[i].draw();
        }
        ofPopStyle();
        ofPopMatrix();
        
    
        ofPopMatrix();
        
        
        
        ofPushMatrix();
        ofTranslate(0, faceBoundingBox.getHeight()*scaleScreen);
        
        ofPushMatrix();
        ofScale(scaleScreen,scaleScreen);
        ofPushStyle();
        ofSetColor(255, 0, 255);
        for(int i = 0; i < linesToDraw1.size(); i++) {
            linesToDraw1[i].draw();
        }
        ofPopMatrix();
        
        ofTranslate(faceBoundingBox.getWidth()*scaleScreen+gutter*scaleScreen, 0);
        
        ofPushMatrix();
        ofScale(scaleScreen,scaleScreen);
        ofSetColor(0, 255, 255);
        for(int i = 0; i < half_linesToDraw1.size(); i++) {
            half_linesToDraw1[i].draw();
        }
        ofPopMatrix();
        
        ofSetColor(255, 255, 0);
        ofTranslate(faceBoundingBox.getWidth()*scaleScreen+gutter*scaleScreen, 0);
        ofPushMatrix();
        ofScale(scaleScreen,scaleScreen);
            for(int i = 0; i < linesToDraw2.size(); i++) {
                linesToDraw2[i].draw();
            }
        ofPopMatrix();
    
        ofTranslate(faceBoundingBox.getWidth()*scaleScreen+gutter*scaleScreen, 0);
        ofPushMatrix();
        ofScale(scaleScreen,scaleScreen);
        for(int i = 0; i < linesToDraw3.size(); i++) {
            linesToDraw3[i].draw();
        }
        ofPopMatrix();
        
    
        ofPopStyle();
        ofPopMatrix();
    
    
       
    
    
       
    
    }
    if(bDrawGui) {
        gui.draw();
        polylinesPanel.draw();
        cannyPanel.draw();
    }

}

void ofApp::makeNewPortraitWithTimer(){
    bMakeNewPortraitwidthTimer=true;
    portraitinittime=ofGetElapsedTimeMillis();
    
    
}


void ofApp::makeNewPortraitWithTimerFinished(){
    makeNewPortrait();
    makeContours();
    record=true;
    bMakeNewPortraitwidthTimer=false;

}


//--------------------------------------------------------------
void ofApp::makeNewPortrait(){
    if(colorImg.getWidth()!=cam.getWidth()){
    colorImg.allocate(cam.getWidth(),cam.getHeight());
    grayImage.allocate(cam.getWidth(),cam.getHeight());
    zoom.allocate(cam.getWidth(),cam.getHeight());
    }
    colorImg.setFromPixels(cam.getPixels());
    grayImage=colorImg;
    grayImage.brightnessContrast(brightness, contrast);
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
    }
}


void ofApp::sendFinalFeed(){
    if (commands.size()>0) {
        cout<<commands.size()<<" commands in buffer"<<endl;
        string cmd;

        for(int i=0;i<commands.size();i++){
            cmd += commands[i];
            cout<<"command " <<cmd<<endl;
            cmd+='\n';
        }
        done=false;
        message = cmd;
    }
}

void ofApp::clearCommands(){
    commands.clear();
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
    waitPen(200);
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
    penHighUp();
    string cmd = "G1 X"+ofToString(int(0))+" Y"+ofToString(int(0));
    commands.push_back(cmd);
    done=true;
}


void ofApp::waitPos(){
    penUp();
    string cmd = "G1 X"+ofToString(waitPosX)+" Y"+ofToString(waitPosY);
    commands.push_back(cmd);
    done=true;
}


void ofApp::goDip(){
    turnIdle();
    string cmd = "G1 X"+ofToString(dipPosX)+" Y"+ofToString(dipPosY);
    commands.push_back(cmd);
    waitPen(3000);
    cmd = "G1 X"+ofToString(dipPosX)+" Y"+ofToString(dipPosY+300);
    commands.push_back(cmd);
    waitPen(100);
    turnDraw();
    turnIdle();
    cmd = "G1 X"+ofToString(dipPosX)+" Y"+ofToString(dipPosY);
    commands.push_back(cmd);
    waitPen(3000);
    cmd = "G1 X"+ofToString(0)+" Y"+ofToString(0);
    commands.push_back(cmd);
    turnDraw();
    turnIdle();
    turnDraw();
    goHome();
    done=true;
}


void ofApp::makeFeed(){
    cout<<"Make Feed"<<linesToPrint.size()<<endl;
    
    scaleRatio=yInMM/faceBoundingBox.getHeight();
    drawScaleFact=scaleRatio;
    cout<<"Scale "<<scaleRatio<<" face height "<<faceBoundingBox.getHeight()<<endl;

    commands.clear();
    turnDraw();
    
    for(int i = 0; i < linesToPrint.size(); i++) {
        linesToPrint[i].simplify();
        vector<ofVec3f> vertices = linesToPrint[i].getVertices();
        penUp();
        cmd=formGString(vertices[0].x,vertices[0].y);
        commands.push_back(cmd);
        penDown();
        for (int vertexIndex=0; vertexIndex<vertices.size(); vertexIndex++) {
            ofVec3f vertex = vertices[vertexIndex];  // Get the vertex
            cmd=formGString(vertex.x,vertex.y);
            commands.push_back(cmd);
        }
    }
//    goHome();
//    turnIdle();
    cout<<commands.size()<<endl;
    for (int i=0; i<commands.size(); i++) {
        //cout<<commands[i]<<endl;
    }
    
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
    done=true;
}




void ofApp::onNewMessage(string & message)
{
    cout << "onNewMessage, message: " << message << "\n";
    if(message=="OK"){
         done=true;
    }
    
}

void ofApp::onNewButtonMessage(string & message)
{
    cout << "onNewButtonMessage, message: " << message << "\n";
    if(message=="1" && state == IDLE){
        makeNewPortrait();
        makeContours();
        record=true;
    }
    
    if(message=="-1"){
        commands.clear();
        bGoHome=true;
        turnLightsOn();
        state=IDLE;
    }
    
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    if(key == ' '){
        makeNewPortrait();
        
    }
    
    
    if(key == 'h'){
        commands.clear();
        bGoHome=true;
        turnLightsOn();
        state=IDLE;
    }
    
    if(key == 'C'){
        continousDraw=!continousDraw;
        
    }

    if(key=='r'){
        record=true;
    }
    
    if(key=='u'){
        penUp();
        done=true;

    }
    
    if(key=='d'){
        penDown();
        done=true;

    }
    
    if(key=='b'){
        scaleRatio=yInMM/faceBoundingBox.getHeight();
        drawScaleFact=scaleRatio;
        cout<<"Scale "<<faceBoundingBox.getHeight()<<" "<<scaleRatio<<endl;
        makeBoundingRectFeed();
    }
    
    if(key=='i'){
        turnIdle();
        done=true;
    }
    
    if(key=='I'){
        turnDraw();
        done=true;
    }
    
    if(key=='c'){
        bMakeContours=true;
    }
    
    
    if(key=='s'){
      //  gui.saveToFile("savesettings.xml");
      //  polylinesPanel.saveToFile("polylinessettings.xml");
        
        gui.saveToFile("general.xml");
        polylinesPanel.saveToFile("polylines.xml");
        cannyPanel.saveToFile("canny.xml");
        
    }
    
    if(key=='l'){
        gui.loadFromFile("general.xml");
        polylinesPanel.loadFromFile("polylines.xml");
        cannyPanel.loadFromFile("canny.xml");
    }


    
    if(key=='m'){
        goDip();
    }
 
    if(key=='1'){
        //dmxValue < 127 ? dmxValue =255 : dmxValue=0;
        
        dmxValue  =dimAmmount;

    }
    
    if(key=='2'){
        //dmxValue < 127 ? dmxValue =255 : dmxValue=0;
        
        dmxValue  =0;
        
    }
    
    if(key=='9'){
        bDrawGui=!bDrawGui;
    }
    
    
    if(key=='w'){
        turnDraw();
        waitPos();
        turnIdle();
    }
    
    
    if(key=='p'){
        makeNewPortraitWithTimer();
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

//--------------------------------------------------------------
void ofApp::exit(){
    dmxValue=0;
    dmx.setLevel(1, dmxValue);
    dmx.setLevel(2, dmxValue);
    dmx.setLevel(3, dmxValue);

    dmx.update();
    clearCommands();
    goHome();
    sendFinalFeed();
    serial.writeString(message);
    ofSleepMillis(4000);
    turnDraw();
    sendFinalFeed();
    serial.writeString(message);
    ofSleepMillis(2000);
}

void ofApp::turnLightsOn(){
    dmxValue =dimAmmount;
}
void ofApp:: turnLightsOff(){
     dmxValue=0;
}






