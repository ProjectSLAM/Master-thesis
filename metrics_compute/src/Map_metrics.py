###############################################################################################################################################################################################################
# this code is from the paper: "Xuan Sang LE, Luc Fabresse, Noury Bouraqadi, & Guillaume Lozenguez. (2018). Evaluation of Out-of-the-box ROS 2D SLAMs for Autonomous Exploration of Unknown Indoor #Environments. In The 11th International Conference on Intelligent Robotics and Applications. Newcastle, Australia, August 9-11, 2018"
# And can been found on : " https://blog.lxsang.me/post/id/21 " 
# Except for the first step,i.e the IMAGE REGISTRATION, which is from  https://learnopencv.com/image-#alignment-feature-based-using-opencv-c-python/?ref=https://githubhelp.com
###############################################################################################################################################################################################################

import cv2
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import sys
from skimage.metrics import structural_similarity as ssim
from numpy import float32, matrix
# using normalize neareast distance of all pixel
def nearest_error(groundtruth, slammap):

    cnt = 0
    sl_occupancies = None
    # build the training set from the aligned slam map
    print(groundtruth.shape[0])
    for i in range(slammap.shape[0]):
        for j in range(slammap.shape[1]):

            pixel = slammap.item(i, j)
            if(pixel < 50):
                if(sl_occupancies is None):
                    sl_occupancies = [i,j]
                else:
                    sl_occupancies = np.vstack((sl_occupancies, [i,j]))
 

   

    sl_occupancies = sl_occupancies.astype(np.float32)
    # label them
    gr_labels = np.full((sl_occupancies.shape[0], 1), 1).astype(np.float32)
 
 
    # CvKNearest instance
    knn = cv2.ml.KNearest_create()

    # trains the model
    knn.train(sl_occupancies, cv2.ml.ROW_SAMPLE,gr_labels)

    # New sample : (x,y)
    #newcomer = np.array([[500,500],[100,100]]).astype(np.float32)
    #plt.scatter(newcomer[:,0],newcomer[:,1],80,'g','o')

    # find nearest cell on the slam map for each occupancy cell in the groundtruth map
    gr_occupancies = None
    ncells = 0
    for i in range(groundtruth.shape[0]):
        for j in range(groundtruth.shape[1]):
            pixel = groundtruth.item(i, j)
            if(pixel < 50):
                ncells = ncells+1
                if(gr_occupancies is None):
                    gr_occupancies = [i,j]
                else:
                    gr_occupancies = np.vstack((gr_occupancies, [i,j]))
    gr_occupancies = gr_occupancies.astype(np.float32)
    ret, results, neighbours, dist = knn.findNearest(gr_occupancies, 1)


    sume = np.sum(dist)
    print ("sum of error: ", sume)
    print ("normalize error (NE):", (sume/ncells), "\n")
    return  (sume/ncells)
    #plt.show()
    #cv2.waitKey(0)



# using structure similarity index
def issim(imageA,imageB):
    e = ssim(imageA,imageB)
    print ("Structure similarity index: (SSIM)", e)
    return e

############################MAIN PROGRAM###################################
# 
# 
# I. FIRST STEP: IMAGE REGISTRATION
# the SLAM map will be aligned to the groundtruth map using cv2.estimateAffinePartial2D

# Read the images to be aligned as grayscale
groundtruth =  cv2.imread(sys.argv[1],0) # Image is open in grayscale 
slammap_unaligned =  cv2.imread(sys.argv[2],0) # Image is open in grayscale 

align = True

if align is True: 
    print ('Align images', "\n")
    # Find size of the groundtruth
    height, width = groundtruth.shape
    # Create ORB detector with 5000 features.
    orb_detector = cv2.ORB_create(50000)
    # Find keypoints and descriptors.
    # The first arg is the image, second arg is the mask
    #  (which is not required in this case).
    kp1, d1 = orb_detector.detectAndCompute(groundtruth, None)
    kp2, d2 = orb_detector.detectAndCompute(slammap_unaligned, None)
    # Match features between the two images.
    # We create a Brute Force matcher with
    # Hamming distance as measurement mode.
    matcher = cv2.DescriptorMatcher_create(cv2.DESCRIPTOR_MATCHER_BRUTEFORCE_HAMMING)
    matches = matcher.match(d2, d1, None)
    # Sort matches on the basis of their Hamming distance.
    matches.sort(key = lambda x: x.distance, reverse=False)
    # Take the top 30 % matches forward.
    matches = matches[:int(len(matches)*0.30)]
    no_of_matches = len(matches)
    # Draw top matches
    imMatches = cv2.drawMatches(slammap_unaligned, kp2, groundtruth, kp1, matches, None)
    # Extract location of good matches
    points1 = np.zeros((len(matches), 2), dtype=np.float32)
    points2 = np.zeros((len(matches), 2), dtype=np.float32)
    # Define empty matrices of shape no_of_matches * 2.
    p1 = np.zeros((no_of_matches, 2))
    p2 = np.zeros((no_of_matches, 2))
 
    for i, match in enumerate(matches):
    	p1[i, :] = kp2[match.queryIdx].pt
    	p2[i, :] = kp1[match.trainIdx].pt
    # Find the homography matrix.
    homography, mask = cv2.estimateAffinePartial2D(p1, p2, cv2.RANSAC )
    # Use this matrix to transform the
    # colored image wrt the reference image.
    slammap = cv2.warpAffine(slammap_unaligned, homography, (height, width))

  
    # Show the images
             # Values >0.0 zoom out
    cv2.imshow("Groundtruth", groundtruth)
    cv2.imshow("SLAM", slammap_unaligned)
    cv2.imshow("Aligned slam", slammap)
    cv2.waitKey(0)
 
else:
    slammap = slammap_unaligned
    print("no alignement")
#groundtruth =  cv2.imread(sys.argv[1],0)
#slammap =  cv2.imread(sys.argv[2],0)

# II. STEP 2, measure the error between the groundtruth amd the aligned slam map using knearest neighbour searvh, k = 1
output=[]
options = {
    0: issim,
    1: nearest_error
}
 
for attr, value in options.items():
    output.append(value(groundtruth,slammap))
    print(output)

data = matrix([output[0], output[1]])
header = "nearest_error issim"
f = open("/home/marin/catkin_ws/src/metrics_compute/map_quality.txt", 'w')
save = np.savetxt(f, data,delimiter=',', header=header,fmt='%5.4g')
