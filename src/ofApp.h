#pragma once

//#include "ofMain.h"
#include "ofxKinect.h"
#include "ofxCv.h"
#include "ofxGui.h"
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

	ofxKinect kinect;

	ofxCv::FlowPyrLK flowPyrLK;
	ofxCv::ContourFinder contourFinder;
	ofImage kinectImage, nearImage, farImage;

	vector<ofPolyline> blobPolylines;

	int nearThreshold, farThreshold, timeThreshold, resetTimeThreshold, kinectAngle;
	float polylineSimplfy;
//	bool flipX, flipY, flowEnabled;
	unsigned long long lastTime, lastResetTime;

	int flowMaxFeatures, flowMaxLevel, flowMinDistance;
	float flowQualityLevel;

  ofxPanel gui;
  ofParameter<bool> flipX, flipY, flowEnabled;

	ofxOscSender oscSender;
};
