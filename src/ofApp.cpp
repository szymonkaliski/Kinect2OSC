#include "ofApp.h"

void ofApp::setup() {
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
	
	colorImg.allocate(kinect.width, kinect.height);
	grayImage.allocate(kinect.width, kinect.height);
	grayThreshNear.allocate(kinect.width, kinect.height);
	grayThreshFar.allocate(kinect.width, kinect.height);
	
	nearThreshold = 250;
	farThreshold = 240;
	
	ofSetFrameRate(60);
	
	nearThresholdSlider.addListener(this, &ofApp::nearThresholdChanged);
	farThresholdSlider.addListener(this, &ofApp::farThresholdChanged);
	angleSlider.addListener(this, &ofApp::angleChanged);
	
	gui.setup();
	gui.setPosition(10, 320);
	gui.add(nearThresholdSlider.setup("near", nearThreshold, 0, 255));
	gui.add(farThresholdSlider.setup("far", farThreshold, 0, 255));
	gui.add(angleSlider.setup("angle", 0, -30, 30));
	gui.loadFromFile("settings.xml");
	
	oscSender.setup(HOST, PORT);
}

void ofApp::update() {
	kinect.update();

	if (kinect.isFrameNew()) {
		grayImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
		
		grayThreshNear = grayImage;
		grayThreshFar = grayImage;
		
		grayThreshNear.threshold(nearThreshold, true);
		grayThreshFar.threshold(farThreshold);
		
		cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
		
		// update the cv images
		grayImage.flagImageChanged();
		
		// find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
		// also, find holes is set to true so we will get interior contours as well....
		contourFinder.findContours(grayImage, 10, (kinect.width * kinect.height) / 2, 20, false);
		
		// get blob data from contours and send through OSC
		ofxOscMessage oscMessageCentroids;
		ofxOscMessage oscMessageBoundingBoxes;
		
		oscMessageCentroids.setAddress("/centroids");
		oscMessageBoundingBoxes.setAddress("/boundingboxes");
		
		vector<ofxCvBlob> blobs = contourFinder.blobs;
		vector<ofxCvBlob>::iterator blobIt;
		
		for (blobIt = blobs.begin(); blobIt != blobs.end(); blobIt++) {
			ofLogNotice() << "centr: " << blobIt->centroid << " boundingrect: " << blobIt->boundingRect;
			
			oscMessageCentroids.addFloatArg(blobIt->centroid.x);
			oscMessageCentroids.addFloatArg(blobIt->centroid.y);
			
			oscMessageBoundingBoxes.addIntArg(blobIt->boundingRect.x);
			oscMessageBoundingBoxes.addIntArg(blobIt->boundingRect.y);
			oscMessageBoundingBoxes.addIntArg(blobIt->boundingRect.width);
			oscMessageBoundingBoxes.addIntArg(blobIt->boundingRect.height);
		}
		
		oscSender.sendMessage(oscMessageCentroids);
		oscSender.sendMessage(oscMessageBoundingBoxes);
	}
}

void ofApp::draw() {
	ofBackground(70, 70, 70);
	ofSetColor(255, 255, 255);

	kinect.draw(10, 10, 400, 300);
	
	kinect.drawDepth(420, 10, 400, 300);
	
	grayImage.draw(830, 10, 400, 300);
	contourFinder.draw(830, 10, 400, 300);
	
	gui.draw();
}

void ofApp::exit() {
	kinect.setCameraTiltAngle(0);
	kinect.close();
}

void ofApp::keyPressed(int key) {
	switch (key) {
		case 's':
			gui.saveToFile("settings.xml");
			break;

		case 'l':
			gui.loadFromFile("settings.xml");
			break;
			
		default:
			break;
	}
}

void ofApp::mousePressed(int x, int y, int button) {

}

void ofApp::farThresholdChanged(int &threshold) {
	this->farThreshold = threshold;
}

void ofApp::nearThresholdChanged(int &threshold) {
	this->nearThreshold = threshold;
}

void ofApp::angleChanged(int &angle) {
	ofLogNotice() << "setting angle to: " << angle;
	kinect.setCameraTiltAngle(angle);
}
