#!/usr/bin/env python

import numpy as np
import cv2
from common import anorm, getsize
import time
import math

FLANN_INDEX_KDTREE = 1  # bug: flann enums are missing
FLANN_INDEX_LSH    = 6


def init_feature(name):
  chunks = name.split('-')
  if chunks[0] == 'sift':
    detector = cv2.xfeatures2d.SIFT_create()
    norm = cv2.NORM_L2
  elif chunks[0] == 'orb':
    detector = cv2.ORB(400)
    norm = cv2.NORM_HAMMING
  elif chunks[0] == 'surf':
    detector = cv2.xfeatures2d.SURF_create(800)
    norm = cv2.NORM_L2
  else:
    print "Error"
  if norm == cv2.NORM_L2:
      flann_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
  else:
      flann_params= dict(algorithm = FLANN_INDEX_LSH,
                         table_number = 6, # 12
                         key_size = 12,     # 20
                         multi_probe_level = 1) #2
  matcher = cv2.FlannBasedMatcher(flann_params, {})  # bug : need to pass empty dict (#1329)
  
  return detector, matcher


def filter_matches(kp1, kp2, matches, ratio = 0.75):
    mkp1, mkp2 = [], []
    for m in matches:
      if len(m) == 2 and m[0].distance < m[1].distance * ratio:
        m = m[0]
        mkp1.append(kp1[m.queryIdx])
        mkp2.append(kp2[m.trainIdx])
    p1 = np.float32([kp.pt for kp in mkp1])
    p2 = np.float32([kp.pt for kp in mkp2])
    return p1, p2
    
def match_and_draw(desc1, desc2, kp1, kp2):
  raw_matches = matcher.knnMatch(desc1, trainDescriptors = desc2, k = 2) #2
  p1, p2 = filter_matches(kp1, kp2, raw_matches)
  if len(p1) >= 4:
    M, mask = cv2.findHomography(p1, p2)
    matchesMask = mask.ravel().tolist()
    print '%d / %d  inliers/matched' % (np.sum(mask), len(mask))
  else:
    M, mask = None, None
    print '%d matches found, not enough for homography estimation' % len(p1)
  z = np.zeros((np.shape(p1)[0],1))
  p1 =  np.asmatrix(np.hstack((p1,z)))
  p2 =  np.asmatrix(np.hstack((p2,z)))
  return rigid_transform_3D(p1,p2)
  
def getComponents(H):
  '''((translationx, translationy), rotation, (scalex, scaley), shear)'''
  a = H[0,0]
  b = H[0,1]
  c = H[0,2]
  d = H[1,0]
  e = H[1,1]
  f = H[1,2]

  p = math.sqrt(a*a + b*b)
  r = (a*e - b*d)/(p)
  q = (a*d+b*e)/(a*e - b*d)

  translation = (c,f)
  scale = (p,r)
  shear = q
  theta = math.atan2(b,a)

  return (translation, theta, scale, shear)
  
def rigid_transform_3D(A, B):
  assert len(A) == len(B)
  N = A.shape[0]; # total points
  centroid_A = np.mean(A, axis=0)
  centroid_B = np.mean(B, axis=0)
  
  # centre the points
  AA = A - np.tile(centroid_A, (N, 1))
  BB = B - np.tile(centroid_B, (N, 1))

  # dot is matrix multiplication for array
  H = np.transpose(AA) * BB

  U, S, Vt = np.linalg.svd(H)

  R = Vt.T * U.T

  # special reflection case
  if np.linalg.det(R) < 0:
     print "Reflection detected"
     Vt[2,:] *= -1
     R = Vt.T * U.T
  centre1 = centroid_A.T
  centre1 = centre1 - 500
  centre1[2,0] += 500
  centre2 = centroid_B.T
  centre2 = centre2 - 500
  centre2[2,0] += 500
  
  t = R*centre1 - centre2
  R = -math.degrees(math.asin(R[0,1]))

  return R, t, centroid_A.T, centroid_B.T

if __name__ == '__main__':

    import sys, getopt
    opts, args = getopt.getopt(sys.argv[1:], '', ['feature='])
    opts = dict(opts)
    feature_name = opts.get('--feature', 'surf')
    try: fn1, fn2 = args
    except:
        fn1 = '../c/box.png'
        fn2 = '../c/box_in_scene.png'

    img1 = cv2.imread(fn1, 0)
    img2 = cv2.imread(fn2, 0)
    detector, matcher = init_feature(feature_name)
    if detector != None:
      #print 'using', feature_name
      pass
    else:
      print 'unknown feature:', feature_name
      sys.exit(1)

    green = (255, 255, 0)
    kp1, desc1 = detector.detectAndCompute(img1, None)
    kp2, desc2 = detector.detectAndCompute(img2, None)
    
    print 'img1 - %d features, img2 - %d features' % (len(kp1), len(kp2))  
    R, t, centre1, centre2 = match_and_draw(desc1, desc2, kp1, kp2)
    print int(centre1[0,0]), int(centre1[1,0])  
    print R
    print t
    print "test"
    cv2.circle(img1, (500, 500), 10, green, 2)
    cv2.circle(img1, (int(centre1[0,0]), int(centre1[1,0])), 10, green, 2)
    cv2.circle(img2, (500, 500), 10, green, 2)
    cv2.circle(img2, (int(centre2[0,0]), int(centre2[1,0])), 10, green, 2)
    cv2.imshow('image2', img2)
    cv2.imshow('image1', img1)
    
    
    cv2.waitKey(0)
    #cv2.destroyAllWindows()