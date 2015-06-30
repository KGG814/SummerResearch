#!/usr/bin/env python

'''
ESC   - exit
SPACE - start tracking
r     - toggle RANSAC
'''

import numpy as np
import cv2
from common import draw_str
import math
import sys

lk_params = dict( winSize  = (19, 19),
                  maxLevel = 2,
                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

feature_params = dict( maxCorners = 1000,       # Max total corners
                       qualityLevel = 0.01,     # Ratio of best to worst corner
                       minDistance = 3,         # Min distance between points
                       blockSize = 19 )

def checkedTrace(img0, img1, p0, back_threshold = 1.0):
    p1, st, err = cv2.calcOpticalFlowPyrLK(img0, img1, p0, None, **lk_params)
    p0r, st, err = cv2.calcOpticalFlowPyrLK(img1, img0, p1, None, **lk_params)
    d = abs(p0-p0r).reshape(-1, 2).max(-1)
    status = d < back_threshold
    return p1, status, err

green = (0, 255, 0)
red = (0, 0, 255)
     
class App:
    def __init__(self):
        self.cam = cv2.VideoCapture(0)
        self.features = None
        self.use_ransac = True
        self.prevPosition = (0.0,0.0)
        self.resetCount = 0

    def run(self):
        while True:
            ret, frame = self.cam.read()
            frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            vis = frame.copy()
            if self.features is not None:
                tracePoints, trace_status, err = checkedTrace(self.gray1, frame_gray, self.prevFeatures)
                # Get the valid features in the previous and current point set
                self.prevFeatures = tracePoints[trace_status].copy()
                self.features = self.features[trace_status].copy()
                self.err = err[trace_status].copy()
                self.gray1 = frame_gray
                # If the number of features falls below a threshold, end tracking
                if len(self.features) < 30 or (self.resetCount >= 30 and len(self.features) < 200):
                    self.prevPosition = (0.0,0.0)
                    self.startTracking(frame, frame_gray)
                    self.resetCount = 0
                # Find a homography between the two feature sets
                H, status = cv2.findHomography(self.features, self.prevFeatures, (0, cv2.RANSAC)[self.use_ransac], 10.0)
                # Get the resolution, ignoring RGB channels
                h, w = frame.shape[:2]
                
                # Display an overlay of the original perspective (keyFrame)
                # Warp Perspective fails occasionally at the edge
                # Keep going when this happens
                try:
                  overlay = cv2.warpPerspective(self.keyFrame, H, (w, h))
                except cv2.error as e:
                  continue
                #vis = cv2.addWeighted(vis, 0.5, overlay, 0.5, 0.0)
                self.printVelocity(status)
                # Calculate dots and traces to draw for features
                for (x0, y0), (x1, y1), good, currErr in zip(self.features[:,0], self.prevFeatures[:,0], status[:,0], err):
                    if good:
                        cv2.line(vis, (x0, y0), (x1, y1), (0, 128, 0))
                    if math.isnan(currErr):
                      currErr = 6
                    currErr = currErr / 3
                    if currErr > 20:
                      currErr = 20
                    if currErr < 2:
                      currErr = 2
                    cv2.circle(vis, (x1, y1), currErr, (red, green)[good], -1)

                # Draw the traces
                draw_str(vis, (20, 20), 'track count: %d' % len(self.prevFeatures))

                if self.use_ransac:
                    draw_str(vis, (20, 40), 'RANSAC')
                if self.resetCount < 100:
                  self.resetCount += 1
            # If we have no features, find features for this frame
            else:
                goodFeatures = cv2.goodFeaturesToTrack(frame_gray, **feature_params)
                if goodFeatures is not None:
                    for x, y in goodFeatures[:,0]:
                        cv2.circle(vis, (x, y), 2, green, -1)
                    draw_str(vis, (20, 20), 'feature count: %d' % len(goodFeatures))

            cv2.imshow('lk_homography', vis)

            ch = 0xFF & cv2.waitKey(1)
            if ch == 27:
                break
            if ch == ord(' '):
                self.startTracking(frame, frame_gray)
            if ch == ord('r'):
                self.use_ransac = not self.use_ransac

    def printVelocity(self, status):
      currPosition = self.features[:,0] - self.prevFeatures[:,0]
      truePosition = []
      
      for x in np.nonzero(currPosition)[0]:
        truePosition.append(currPosition[x])
      currPosition = np.asarray(truePosition)
      meanPosition = np.mean(currPosition, axis=0)
      
      velocity = np.subtract(meanPosition,self.prevPosition)
      currStd = (np.std(currPosition, axis = 0))
      if currPosition.any():
        print "Velocity: ", velocity
        print "Std Dev :", currStd
      self.prevPosition = meanPosition
      

    def startTracking(self, frame, frame_gray):
      # Once we start tracking, save the features in this image for tracking
      self.keyFrame = frame.copy()
      self.features = cv2.goodFeaturesToTrack(frame_gray, **feature_params)
      if self.features is not None:
          self.prevFeatures = self.features
          self.gray0 = frame_gray
          self.gray1 = frame_gray

def main():
    print __doc__
    App().run()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
