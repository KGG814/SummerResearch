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
    kp_pairs = zip(mkp1, mkp2)
    return p1, p2, kp_pairs
    
def getComponents(H):
  '''((translationx, translationy), rotation, (scalex, scaley), shear)'''
  print "Test"
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

def explore_match(win, img1, img2, kp_pairs, status = None, H = None):
    h1, w1 = img1.shape[:2]
    h2, w2 = img2.shape[:2]
    vis = np.zeros((max(h1, h2), w1+w2), np.uint8)
    vis[:h1, :w1] = img1
    vis[:h2, w1:w1+w2] = img2
    vis = cv2.cvtColor(vis, cv2.COLOR_GRAY2BGR)
    translation, theta, scale, shear =  getComponents(H)
    print "Translation:", translation
    print "Theta:", theta
    print "Scale:", scale
    print "Shear:", shear
    if H is not None:
        corners = np.float32([[0, 0], [w1, 0], [w1, h1], [0, h1]])
        corners = np.int32(cv2.perspectiveTransform(corners.reshape(1,-1,2),
                           H).reshape(-1,2)+(w1,0))
        cv2.polylines(vis, [corners], True, (255, 255, 255))

    if status is None:
        status = np.ones(len(kp_pairs), np.bool_)
    p1 = np.int32([kpp[0].pt for kpp in kp_pairs])
    p2 = np.int32([kpp[1].pt for kpp in kp_pairs]) + (w1, 0)

    green = (0, 255, 0)
    red = (0, 0, 255)
    white = (255, 255, 255)
    kp_color = (51, 103, 236)
    '''for (x1, y1), (x2, y2), inlier in zip(p1, p2, status):
        if inlier:
            col = green
            cv2.circle(vis, (x1, y1), 2, col, -1)
            cv2.circle(vis, (x2, y2), 2, col, -1)
        else:
            col = red
            r = 2
            thickness = 3
            cv2.line(vis, (x1-r, y1-r), (x1+r, y1+r), col, thickness)
            cv2.line(vis, (x1-r, y1+r), (x1+r, y1-r), col, thickness)
            cv2.line(vis, (x2-r, y2-r), (x2+r, y2+r), col, thickness)
            cv2.line(vis, (x2-r, y2+r), (x2+r, y2-r), col, thickness)
    vis0 = vis.copy()
    for (x1, y1), (x2, y2), inlier in zip(p1, p2, status):
        if inlier:
            cv2.line(vis, (x1, y1), (x2, y2), green)'''

    cv2.imshow(win, vis)
    def onmouse(event, x, y, flags, param):
        cur_vis = vis
        if flags & cv2.EVENT_FLAG_LBUTTON:
            cur_vis = vis0.copy()
            r = 8
            m = (anorm(p1 - (x, y)) < r) | (anorm(p2 - (x, y)) < r)
            idxs = np.where(m)[0]
            kp1s, kp2s = [], []
            for i in idxs:
                 (x1, y1), (x2, y2) = p1[i], p2[i]
                 col = (red, green)[status[i]]
                 cv2.line(cur_vis, (x1, y1), (x2, y2), col)
                 kp1, kp2 = kp_pairs[i]
                 kp1s.append(kp1)
                 kp2s.append(kp2)
            cur_vis = cv2.drawKeypoints(cur_vis, kp1s, flags=4, color=kp_color)
            cur_vis[:,w1:] = cv2.drawKeypoints(cur_vis[:,w1:], kp2s, flags=4, color=kp_color)

        cv2.imshow(win, cur_vis)
    cv2.setMouseCallback(win, onmouse)
    return vis


if __name__ == '__main__':
    print __doc__

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
      print 'using', feature_name
    else:
      print 'unknown feature:', feature_name
      sys.exit(1)

    
    kp1, desc1 = detector.detectAndCompute(img1, None)
    kp2, desc2 = detector.detectAndCompute(img2, None)
    print 'img1 - %d features, img2 - %d features' % (len(kp1), len(kp2))
     
    def match_and_draw(win):
      print 'matching...'
      raw_matches = matcher.knnMatch(desc1, trainDescriptors = desc2, k = 2) #2
      #TODO
      p1, p2, kp_pairs = filter_matches(kp1, kp2, raw_matches)
      if len(p1) >= 4:
        M, mask = cv2.findHomography(p1, p2)
        matchesMask = mask.ravel().tolist()
        print '%d / %d  inliers/matched' % (np.sum(mask), len(mask))
      else:
        M, mask = None, None
        print '%d matches found, not enough for homography estimation' % len(p1)
      vis = explore_match(win, img1, img2, kp_pairs, mask, M)
      
    match_and_draw('find_obj')
    cv2.waitKey()
    cv2.destroyAllWindows()
