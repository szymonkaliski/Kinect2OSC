#pragma once

#include "ofMain.h"
#include "ofxKinect.h"
#include "ofxCv.h"
#include "ofxUI.h"
#include "ofxOsc.h"

#define HOST "localhost"
#define PORT 3333

class ofApp : public ofBaseApp {
public:
	void setup();
	void update();
	void draw();
	void exit();

	void keyPressed(int key);
	void guiEvent(ofxUIEventArgs &event);

	ofxKinect kinect;

	ofxCv::FlowPyrLK flowPyrLK;
	ofxCv::ContourFinder contourFinder;
	ofImage kinectImage, nearImage, farImage;

	vector<ofPolyline> blobPolylines;

	int nearThreshold, farThreshold, timeThreshold, kinectAngle;
	float polylineSimplfy;
	bool flipX, flipY, flowEnabled;
	unsigned long long lastTime;

	int flowMaxFeatures, flowMaxLevel, flowMinDistance;
	float flowQualityLevel;

	ofxUICanvas *gui;

	ofxOscSender oscSender;
};
