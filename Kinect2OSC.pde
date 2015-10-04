import controlP5.*;

import netP5.*;
import oscP5.*;

import gab.opencv.*;

import org.openkinect.freenect.*;
import org.openkinect.processing.*;

ControlP5 cp5;
Kinect kinect;
OpenCV opencv;
Flow flow;
OscP5 oscP5;
NetAddress oscAddress;

int flowRegionSize = 10;
float flowThreshold = 0.005;
float flowPyramidScale = 0.5;
float flowMultiplier = 50.0;
int flowNLevels = 4;
int flowWinSize = 8;
int flowNIterations = 2;
int flowPolyN = 7;
float flowPolySigma = 1.5;

boolean flipX = false;
boolean flipY = false;

float kinectTilt = 2;

float kinectMin = 0;
float kinectMax = 255;

void setup() {
  size(640, 480);

  cp5 = new ControlP5(this);
  kinect = new Kinect(this);
  opencv = new OpenCV(this, 640, 480);
  flow = new Flow(this);
  oscP5 = new OscP5(this, 3000);

  oscAddress = new NetAddress("127.0.0.1", 3333);

  cp5.addToggle("flipX")
    .setPosition(10, 10)
    .setSize(20, 20);

  cp5.addToggle("flipY")
    .setPosition(40, 10)
    .setSize(20, 20);

  cp5.addRange("kinectThreshold")
    .setBroadcast(false)
    .setPosition(10, 50)
    .setSize(200, 10)
    .setRange(0, 255)
    .setRangeValues(kinectMin, kinectMax)
    .setBroadcast(true);

  cp5.addSlider("kinectTilt")
    .setPosition(10, 70)
    .setSize(200, 10)
    .setRange(0, 10);

  cp5.addSlider("flowRegionSize")
    .setPosition(10, 90)
    .setSize(200, 10)
    .setRange(1, 20);

  cp5.addSlider("flowThreshold")
    .setPosition(10, 110)
    .setSize(200, 10)
    .setRange(0.001, 0.01);

  cp5.addSlider("flowMultiplier")
    .setPosition(10, 130)
    .setSize(200, 10)
    .setRange(1.0, 100.0);

  cp5.addSlider("flowPyramidScale")
    .setPosition(10, 150)
    .setSize(200, 10)
    .setRange(0.0, 1.0);

  cp5.addSlider("flowNLevels")
    .setPosition(10, 170)
    .setSize(200, 10)
    .setRange(0, 10);

  cp5.addSlider("flowWinSize")
    .setPosition(10, 190)
    .setSize(200, 10)
    .setRange(1, 20);

  cp5.addSlider("flowNIterations")
    .setPosition(10, 210)
    .setSize(200, 10)
    .setRange(1, 10);

  cp5.addSlider("flowPolyN")
    .setPosition(10, 230)
    .setSize(200, 10)
    .setRange(1, 20);

  cp5.addSlider("flowPolySigma")
    .setPosition(10, 250)
    .setSize(200, 10)
    .setRange(1.0, 4.0);

  cp5.addButton("saveSettings")
    .setPosition(10, 270)
    .setSize(20, 20);

  cp5.addButton("loadSettings")
    .setPosition(40, 270)
    .setSize(20, 20);

  kinect.startDepth();
}

void draw() {
  background(0);

  kinect.tilt(kinectTilt);

  flow.setIterations(flowNIterations);
  flow.setIterations(flowNIterations);
  flow.setLevels(flowNLevels);
  flow.setPolyN(flowPolyN);
  flow.setPolySigma(flowPolySigma);
  flow.setPyramidScale(flowPyramidScale);
  flow.setWindowSize(flowWinSize);

  PImage frame = kinect.getDepthImage();

  frame = flipImage(frame, flipX, flipY);
  frame = thresholdDepth(frame, kinectMin, kinectMax);

  opencv.loadImage(frame);
  opencv.calculateOpticalFlow();

  image(frame, 0, 0);

  fill(255);

  stroke(255, 0, 0);
  strokeWeight(3);

  // required from previous version - count of blobs, not yet implemented
  OscMessage oscCountMessage = new OscMessage("/count");
  oscCountMessage.add(1);
  oscP5.send(oscCountMessage, oscAddress);

  OscMessage oscFlowMessage = new OscMessage("/flow");

  for (int i = 0; i < width / flowRegionSize; ++i) {
    for (int j = 0; j < height / flowRegionSize; ++j) {
      int x = i * flowRegionSize;
      int y = j * flowRegionSize;

      PVector flow = opencv.getAverageFlowInRegion(x, y, flowRegionSize, flowRegionSize);

      if (abs(flow.x) > flowThreshold || abs(flow.y) > flowThreshold) {
        float flowPointX = x + flowRegionSize / 2 + flow.x * flowRegionSize * flowMultiplier;
        float flowPointY = y + flowRegionSize / 2 + flow.y * flowRegionSize * flowMultiplier;

        line(
          x + flowRegionSize / 2,
          y + flowRegionSize / 2,
          flowPointX,
          flowPointY
        );

        oscFlowMessage.add(flowPointX / frame.width);
        oscFlowMessage.add(flowPointY / frame.height);
      }
    }
  }

  oscP5.send(oscFlowMessage, oscAddress);
}

void controlEvent(ControlEvent event) {
  if (event.isFrom("kinectThreshold")) {
    kinectMin = event.getController().getArrayValue(0);
    kinectMax = event.getController().getArrayValue(1);
  }
}

PImage thresholdDepth(PImage image, float minDepth, float maxDepth) {
  PImage thresholded = createImage(image.width, image.height, ARGB);

  for (int i = 0; i < image.pixels.length; i++) {
    float value = red(image.pixels[i]);

    if (value >= minDepth && value <= maxDepth) {
      thresholded.pixels[i] = image.pixels[i];
    }
    else {
      thresholded.pixels[i] = color(0);
    }
  }

  return thresholded;
}

PImage flipImage(PImage image, boolean flipX, boolean flipY) {
  PImage flipped = createImage(image.width, image.height, ARGB);

  for (int i = 0; i < image.width; i++) {
    for (int j = 0; j < image.height; j++) {
      int xPixel, yPixel;

      xPixel = flipX ? image.width - 1 - i : i;
      yPixel = flipY ? image.height - 1 - j : j;

      flipped.pixels[yPixel * image.width + xPixel] = image.pixels[j * image.width + i];
    }
  }

  return flipped;
}

public void saveSettings(int value) {
  cp5.saveProperties();
}

public void loadSettings(int value) {
  cp5.loadProperties();
}
