#include "ofApp.h"
#define SSTR(x) dynamic_cast< std::ostringstream & >((std::ostringstream() << std::dec << x)).str()

void ofApp::setup() {
	// of setup
	ofSetWindowTitle("Kinect2OSC");
	ofSetFrameRate(30);

	// kinect setup
	kinect.setRegistration(true);
	kinect.init();
	kinect.open();

	if (kinect.isConnected()) {
		ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
		ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
		ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
		ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
	}
	else {
		ofLogError() << "kinect not connected!";
	}

	// initial settings
	nearThreshold = 150;
	farThreshold = 240;
	polylineSimplfy = 4.0f;
	flipX = false;
	flipY = false;
	timeThreshold = 100;
	resetTimeThreshold = 500;
	lastTime = ofGetElapsedTimeMillis();
	lastResetTime = ofGetElapsedTimeMillis();
	flowMaxFeatures = 200;
	flowMaxLevel = 3;
	flowMinDistance = 4;
	flowQualityLevel = 0.01;

	// helping images setup
	kinectImage.allocate(kinect.width, kinect.height, OF_IMAGE_GRAYSCALE);
	nearImage.allocate(kinect.width, kinect.height, OF_IMAGE_GRAYSCALE);
	farImage.allocate(kinect.width, kinect.height, OF_IMAGE_GRAYSCALE);

	flowPyrLK.resetFlow();
	flowPyrLK.setMaxFeatures(flowMaxFeatures);
	flowPyrLK.setMaxLevel(flowMaxLevel);
	flowPyrLK.setMinDistance(flowMinDistance);
	flowPyrLK.setQualityLevel(flowQualityLevel);

	flowEnabled = false;

	// tabbed gui
	gui = new ofxUITabBar(10, 10, 400, 300);

	ofxUICanvas *guiMain = new ofxUICanvas();
	guiMain->setName("main");
	guiMain->addToggle("flip X", flipX);
	guiMain->addToggle("flip Y", flipY);
	guiMain->addSpacer();
	guiMain->addSlider("near", 0, 255, nearThreshold);
	guiMain->addSlider("far", 0, 255, farThreshold);
	guiMain->addSlider("angle", -30, 30, kinectAngle);
	guiMain->addSlider("simplify", 1.0, 20.0, polylineSimplfy);
	guiMain->addSlider("threshold", 10.0, 1000.0, timeThreshold);
	guiMain->autoSizeToFitWidgets();
	ofAddListener(guiMain->newGUIEvent, this, &ofApp::guiEvent);

	ofxUICanvas *guiFlow = new ofxUICanvas();
	guiFlow->setName("flow");
	guiFlow->addToggle("calculate flow", flowEnabled);
	guiFlow->addSpacer();
	guiFlow->addSlider("max features", 1, 500, flowMaxFeatures);
	guiFlow->addSlider("max level", 1, 8, flowMaxLevel);
	guiFlow->addSlider("min distance", 1, 16, flowMinDistance);
	guiFlow->addSlider("quality level", 0.001, 0.1, flowQualityLevel);
	guiMain->autoSizeToFitWidgets();
	ofAddListener(guiFlow->newGUIEvent, this, &ofApp::guiEvent);

	gui->addCanvas(guiMain);
	gui->addCanvas(guiFlow);
	gui->loadSettings("settings/", "ui-");

	// osc setup
	oscSender.setup(HOST, PORT);
}

void ofApp::update() {
	// update kinect
	kinect.update();

	if (kinect.isFrameNew() && ofGetElapsedTimeMillis() - lastTime > timeThreshold) {
		// update image
		kinectImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height, OF_IMAGE_GRAYSCALE);

		// flip image if needed
		if (flipX) { ofxCv::flip(kinectImage, kinectImage, 1); }
		if (flipY) { ofxCv::flip(kinectImage, kinectImage, 0); }
		kinectImage.update();

		// set near image
		nearImage.setFromPixels(kinectImage);
		ofxCv::threshold(kinectImage, nearImage, nearThreshold);

		// set far image
		farImage.setFromPixels(kinectImage);
		ofxCv::invert(farImage);
		ofxCv::threshold(farImage, farThreshold);
		ofxCv::invert(farImage);

		// update near and far images
		nearImage.update();
		farImage.update();

		// bitwise and near and far images
		ofxCv::bitwise_and(nearImage, farImage, kinectImage);
		kinectImage.update();

		// get contours for image
		contourFinder.findContours(kinectImage);
		int size = contourFinder.size();

		// clear last polylines
		blobPolylines.clear();

		// get blob data from contours and send through OSC
		ofxOscMessage oscMessageCount;
		ofxOscBundle oscBundle;

		// setup osc
		oscMessageCount.setAddress("/count");
		oscMessageCount.addIntArg(size);
		oscBundle.addMessage(oscMessageCount);

		// message for centroid and blob
		for (int i = 0; i < size; ++i) {
			// initialize messages
			ofxOscMessage oscMessageCentroids, oscMessageBlobs;

			// get centroid and polyline
			ofPoint centroid = ofxCv::toOf(contourFinder.getCentroid(i));
			ofPolyline polyline = contourFinder.getPolyline(i);

			string centroidAddr = "/centroid/";
			centroidAddr.append(SSTR(i));

			string blobAddr = "/blob/";
			blobAddr.append(SSTR(i));

			oscMessageCentroids.setAddress(centroidAddr);
			oscMessageBlobs.setAddress(blobAddr);

			polyline.simplify(polylineSimplfy);
			blobPolylines.push_back(polyline);

			for (ofPoint blobPoint : polyline.getVertices()) {
				oscMessageBlobs.addFloatArg(blobPoint.x / kinect.width);
				oscMessageBlobs.addFloatArg(blobPoint.y / kinect.height);
			}

			oscMessageCentroids.addFloatArg(centroid.x / kinect.width);
			oscMessageCentroids.addFloatArg(centroid.y / kinect.height);

			oscBundle.addMessage(oscMessageCentroids);
			oscBundle.addMessage(oscMessageBlobs);
		}

		// message for optical flow
		if (flowEnabled) {
			double minVal, maxVal;
			ofxCv::minMaxLoc(ofxCv::toCv(kinectImage), &minVal, &maxVal);

			vector<ofPoint> flowPoints;

			// fix for black mat crashing opencv
			if (minVal != 0 || maxVal != 0) {
				flowPyrLK.calcOpticalFlow(kinectImage);
				flowPoints = flowPyrLK.getCurrent();
			}

			if (flowPoints.empty() && ofGetElapsedTimeMillis() - lastResetTime > resetTimeThreshold) {
				flowPyrLK.resetFlow();
				lastResetTime = ofGetElapsedTimeMillis();
			}
			else {
				// create message and send
				ofxOscMessage oscFlowMessage;
				oscFlowMessage.setAddress("/flow/");
				ofRectangle testRect(0, 0, kinect.width, kinect.height);

				for (ofPoint flowPoint : flowPoints) {
					if (testRect.inside(flowPoint)) {
						oscFlowMessage.addFloatArg(flowPoint.x / kinect.width);
						oscFlowMessage.addFloatArg(flowPoint.y / kinect.height);
					}
				}

				oscBundle.addMessage(oscFlowMessage);
			}
		}

		// send bundle to client
		oscSender.sendBundle(oscBundle);

		// update time
		lastTime = ofGetElapsedTimeMillis();
	}
}

void ofApp::draw() {
	ofBackground(ofColor::gray);
	ofSetColor(ofColor::white);

	kinect.draw(420, 10, 400, 300);

	kinect.drawDepth(10, 320, 400, 300);

	kinectImage.draw(420, 320, 400, 300);

	ofPushMatrix();
	ofTranslate(420, 320);
	ofScale(400.0 / kinect.width, 300.0 / kinect.height);

	ofSetColor(ofColor::blue);
	contourFinder.draw();
	ofPopMatrix();

	ofSetColor(ofColor::yellow);
	flowPyrLK.draw(420, 320, 400, 300);

	ofPushMatrix();
	ofTranslate(420, 320);
	ofScale(400.0 / kinect.width, 300.0 / kinect.height);

	for (ofPolyline polyline : blobPolylines) {
		ofSetColor(ofColor::red);
		polyline.draw();

		ofPoint centroidPos = polyline.getCentroid2D();
		ofDrawSphere(centroidPos.x, centroidPos.y, 3);
	}
	ofPopMatrix();

	gui->draw();
}

void ofApp::exit() {
//	gui->saveSettings("settings/", "ui-");
	delete gui;

	kinect.setCameraTiltAngle(0);
	kinect.close();
}

void ofApp::keyPressed(int key) {
	switch (key) {
		case 's':
			gui->saveSettings("settings/", "ui-");
			break;

		case 'l':
			gui->loadSettings("settings/", "ui-");
			break;

		default:
			break;
	}
}

void ofApp::guiEvent(ofxUIEventArgs &event) {
	if (event.getName() == "near") {
		nearThreshold = (int)event.getSlider()->getScaledValue();
	}

	if (event.getName() == "far") {
		farThreshold = (int)event.getSlider()->getScaledValue();
	}

	if (event.getName() == "angle") {
		kinectAngle = (int)event.getSlider()->getScaledValue();
		kinect.setCameraTiltAngle(kinectAngle);
	}

	if (event.getName() == "simplify") {
		polylineSimplfy = event.getSlider()->getScaledValue();
	}

	if (event.getName() == "threshold") {
		timeThreshold = (int)event.getSlider()->getScaledValue();
	}

	if (event.getName() == "flip X") {
		flipX	= event.getButton()->getValue();
	}

	if (event.getName() == "flip Y") {
		flipY	= event.getButton()->getValue();
	}

	if (event.getName() == "calculate flow") {
		flowEnabled = event.getButton()->getValue();
		flowPyrLK.resetFlow();
	}

	if (event.getName() == "max features") {
		flowMaxFeatures = (int)event.getSlider()->getScaledValue();
		flowPyrLK.setMaxFeatures(flowMaxFeatures);
		flowPyrLK.resetFlow();
	}

	if (event.getName() == "max level") {
		flowMaxLevel = (int)event.getSlider()->getScaledValue();
		flowPyrLK.setMaxLevel(flowMaxLevel);
		flowPyrLK.resetFlow();
	}

	if (event.getName() == "min distance") {
		flowMinDistance = (int)event.getSlider()->getScaledValue();
		flowPyrLK.setMinDistance(flowMinDistance);
		flowPyrLK.resetFlow();
	}

	if (event.getName() == "quality level") {
		flowQualityLevel = event.getSlider()->getScaledValue();
		flowPyrLK.setQualityLevel(flowQualityLevel);
		flowPyrLK.resetFlow();
	}
}
