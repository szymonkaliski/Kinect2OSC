#pragma once

#include "ofMain.h"
#include "ofxKinect.h"
#include "ofxOpenCv.h"
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
	void mousePressed(int x, int y, int button);

	void farThresholdChanged(int &threshold);
	void nearThresholdChanged(int &threshold);
	void angleChanged(int &angle);
	void polylineSimplifyChanged(float &simplify);
	void timeThresholdChanged(int &timeThreshold);
	void flipXChanged(bool &flipX);

	ofxKinect kinect;

	ofxCv::FlowPyrLK flowPyrLK;
	ofxCv::Flow *flowCurrent;

	ofxCvColorImage colorImg;

	ofxCvGrayscaleImage grayImage;			// grayscale depth image
	ofxCvGrayscaleImage grayThreshNear;	// the near thresholded image
	ofxCvGrayscaleImage grayThreshFar;	// the far thresholded image

	ofxCvContourFinder contourFinder;

	float polylineSimplfy;
	vector<ofPolyline> blobPolylines;

	int nearThreshold, farThreshold, timeThreshold;
	unsigned long long lastTime;
	bool flipX;

	ofxPanel gui;
	ofxIntSlider nearThresholdSlider, farThresholdSlider, angleSlider, thresholdSlider;
	ofxFloatSlider simplifySlider;
	ofxToggle flipXToggle;

	ofxOscSender oscSender;
};
