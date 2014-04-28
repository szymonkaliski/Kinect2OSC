#include "ofApp.h"
#define SSTR(x) dynamic_cast< std::ostringstream & >((std::ostringstream() << std::dec << x)).str()

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
	polylineSimplfy = 4.0f;
	
	ofSetFrameRate(30);
	timeThreshold = 100;
	lastTime = ofGetElapsedTimeMillis();
	
	nearThresholdSlider.addListener(this, &ofApp::nearThresholdChanged);
	farThresholdSlider.addListener(this, &ofApp::farThresholdChanged);
	angleSlider.addListener(this, &ofApp::angleChanged);
	simplifySlider.addListener(this, &ofApp::polylineSimplifyChanged);
	thresholdSlider.addListener(this, &ofApp::timeThresholdChanged);
	
	gui.setup();
	gui.setPosition(10, 320);
	gui.add(nearThresholdSlider.setup("near", nearThreshold, 0, 255));
	gui.add(farThresholdSlider.setup("far", farThreshold, 0, 255));
	gui.add(angleSlider.setup("angle", 0, -30, 30));
	gui.add(simplifySlider.setup("simplify", polylineSimplfy, 1.0f, 20.0f));
	gui.add(thresholdSlider.setup("threshold", timeThreshold, 10, 1000));
	gui.loadFromFile("settings.xml");
	
	oscSender.setup(HOST, PORT);
}

void ofApp::update() {
	kinect.update();

	if (kinect.isFrameNew() && ofGetElapsedTimeMillis() - lastTime > timeThreshold) {
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
		ofxOscMessage oscMessageCount;
		ofxOscBundle oscBundle;
		
		oscMessageCount.setAddress("/count");
		oscMessageCount.addIntArg(contourFinder.nBlobs);
		oscBundle.addMessage(oscMessageCount);
		
		blobPolylines.clear();
		int i = 0;

		for (ofxCvBlob blob : contourFinder.blobs) {
			ofxOscMessage oscMessageCentroids, oscMessageBoundingBoxes, oscMessageBlobs;
			
			string centroidAddr = "/centroid/";
			centroidAddr.append(SSTR(i));
			
			string boundingBoxAddr = "/boundingbox/";
			boundingBoxAddr.append(SSTR(i));
			
			string blobAddr = "/blob/";
			blobAddr.append(SSTR(i));
			
			oscMessageCentroids.setAddress(centroidAddr);
			oscMessageBoundingBoxes.setAddress(boundingBoxAddr);
			oscMessageBlobs.setAddress(blobAddr);
			
			ofPolyline blobPolyline(blob.pts);
			blobPolyline.simplify(polylineSimplfy);
			blobPolylines.push_back(blobPolyline);
			
			for (ofPoint blobPoint : blobPolyline.getVertices()) {
				oscMessageBlobs.addFloatArg(blobPoint.x / kinect.width);
				oscMessageBlobs.addFloatArg(1 - (blobPoint.y / kinect.height));
			}
			
//			oscMessageCentroids.addFloatArg(blob.centroid.x / kinect.width);
//			oscMessageCentroids.addFloatArg(1 - (blob.centroid.y / kinect.height));
			
			oscMessageCentroids.addFloatArg(blobPolyline.getCentroid2D().x / kinect.width);
			oscMessageCentroids.addFloatArg(1 - (blobPolyline.getCentroid2D().y / kinect.height));
			
			oscMessageBoundingBoxes.addIntArg(blob.boundingRect.x / kinect.width);
			oscMessageBoundingBoxes.addIntArg(1 - (blob.boundingRect.y / kinect.height));
			oscMessageBoundingBoxes.addIntArg(blob.boundingRect.width / kinect.width);
			oscMessageBoundingBoxes.addIntArg(blob.boundingRect.height / kinect.height);
			
			oscBundle.addMessage(oscMessageCentroids);
			oscBundle.addMessage(oscMessageBoundingBoxes);
			oscBundle.addMessage(oscMessageBlobs);

			i++;
		}
		
		oscSender.sendBundle(oscBundle);
		
		lastTime = ofGetElapsedTimeMillis();
	}
}

void ofApp::draw() {
	ofBackground(70, 70, 70);
	ofSetColor(255, 255, 255);

	kinect.draw(10, 10, 400, 300);
	
	kinect.drawDepth(420, 10, 400, 300);
	
	grayImage.draw(830, 10, 400, 300);
//	contourFinder.draw(830, 10, 400, 300);
	
	ofPushMatrix();
	ofTranslate(830, 10);
	ofScale(400.0 / kinect.width, 300.0 / kinect.height);
	for (ofPolyline polyline : blobPolylines) {
		ofSetColor(200, 20, 20);
		polyline.draw();
		
		ofSetColor(20, 200, 200);
		ofPoint centroidPos = polyline.getCentroid2D();
		ofDrawSphere(centroidPos.x, centroidPos.y, 3);
	}
	ofPopMatrix();
	
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
	kinect.setCameraTiltAngle(angle);
}

void ofApp::polylineSimplifyChanged(float &simplify) {
	this->polylineSimplfy = simplify;
}

void ofApp::timeThresholdChanged(int &timeThreshold) {
	this->timeThreshold = timeThreshold;
}
