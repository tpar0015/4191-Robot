import numpy as np
import cv2


def ceiling_base_localization(image_1, image_2, camera_matrix):

    # Load our image
    img1 = cv2.cvtColor(cv2.imread(image_1), cv2.COLOR_BGR2GRAY)    
    img2 = cv2.cvtColor(cv2.imread(image_2), cv2.COLOR_BGR2GRAY)    

    img1 = cv2.blur(img1,(10,10))
    img2 = cv2.blur(img2,(10,10))

    # Initiate ORB detector
    orb = cv2.ORB_create(patchSize=50)

    # Find the keypoints and descriptors
    kp1, des1 = orb.detectAndCompute(img1,None)
    kp2, des2 = orb.detectAndCompute(img2,None)

    # Match keypoints
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(des1,des2)

    # Sort matches
    matches = sorted(matches, key = lambda x:x.distance)

    # Find keypoint correspondences
    X1 = np.vstack([kp1[match.queryIdx].pt for match in matches])
    X2 = np.vstack([kp2[match.trainIdx].pt for match in matches])

    # Estimate homography using opencv - 
    H, mask = cv2.findHomography(X1, X2, cv2.RANSAC, 1.0)

    translation_matrix = np.matmul(np.linalg.inv(camera_matrix), H)

    rotation = np.arctan2(translation_matrix[0,1],translation_matrix[0,0])*180/np.pi
    x_translation = translation_matrix[0,2]
    y_translation = translation_matrix[1,2]

# need to convert pixel to meter (measure the length of one ceiling tile and the number of pixel for one tile)
    return x_translation, y_translation, rotation
