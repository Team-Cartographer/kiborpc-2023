import cv2
import numpy as np
import pupil_apriltags as p

#TAG = 'exampleTag.png'
TAG = 'tagformats_web.png'
tag_family = 'tag16h5'
detector = p.Detector(families=tag_family, nthreads=10)
tags = detector.detect(cv2.cvtColor(cv2.imread(TAG), cv2.COLOR_BGR2GRAY))
img = cv2.polylines(cv2.imread(TAG), 
                    [np.array(tag.corners, dtype=np.int32).reshape((-1, 1, 2)) for tag in tags], 
                    True, (255, 0, 0), 2
                    )
cv2.imshow('AprilTags', img)
cv2.waitKey(0)