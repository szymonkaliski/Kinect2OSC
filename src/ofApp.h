#pragma once

#include "ofMain.h"
#include "ofxKinect.h"
#include "ofxOpenCv.h"
#include "ofxGui.h"

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
	
	ofxKinect kinect;
	
	ofxCvColorImage colorImg;
	
	ofxCvGrayscaleImage grayImage;			// grayscale depth image
	ofxCvGrayscaleImage grayThreshNear;	// the near thresholded image
	ofxCvGrayscaleImage grayThreshFar;	// the far thresholded image
	
	ofxCvContourFinder contourFinder;
	
	int nearThreshold, farThreshold;
	
	ofxPanel gui;
	ofxIntSlider nearThresholdSlider, farThresholdSlider, angleSlider;
};
