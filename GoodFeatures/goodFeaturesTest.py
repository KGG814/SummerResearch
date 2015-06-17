import cv2

feature_params = dict( maxCorners = 100,       # Max total corners
                       qualityLevel = 0.01,     # Ratio of best to worst corner
                       minDistance = 10,         # Min distance between points
                       blockSize = 10 )
green = (0, 255, 0)
image = cv2.imread("test.jpg", cv2.IMREAD_GRAYSCALE)
goodFeatures = cv2.goodFeaturesToTrack(image, **feature_params)
featuresList = []
if goodFeatures is not None:
    for x, y in goodFeatures[:,0]:
        cv2.circle(image, (x, y), 2, green, -1)
        featuresList.append((int(x),int(y)))

print featuresList
cv2.imshow('image', image)
cv2.waitKey(0)
