
import rclpy
import rclpy.node
import numpy as np
import cv2
import imutils


class PanoramaNode(rclpy.node.Node):
    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        for cap in self.video_captures:
            cap.release()

    def __init__(self):
        super().__init__("aruco_node")
        self.isv3 = imutils.is_cv3(or_better=True)
        self.results = []

        self.video_captures = [cv2.VideoCapture(4), cv2.VideoCapture(2)]
        for cap in self.video_captures:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

        # Setup timer for attempting image stitch
        self.detect_timer = self.create_timer(0.5, self.image_callback)


    def image_callback(self):
        frames = [cap.read()[1] for cap in self.video_captures]

        result = self.stitch(frames)
        if result is None:
            self.get_logger().warn("Could not stitch")
        else:
            cv2.imwrite("img.png", result)

    def stitch(self, images, ratio=0.75, reprojThresh=4.0, showMatches=False):
        (imageB, imageA) = images # unpack (assume two for now)
        (kpsA, featuresA) = self.detectAndDescribe(imageA)
        (kpsB, featuresB) = self.detectAndDescribe(imageB)

        # Match features
        M = self.match_keypoints(kpsA, kpsB, featuresA, featuresB, ratio, reprojThresh)

        # If this is none, not enough matched features
        if M is None:
            return None
        # Else, apply a perspective transform
        (matches, H, status) = M
        result = cv2.warpPerspective(imageA, H, (imageA.shape[1] + imageB.shape[1], imageA.shape[0]))
        result[0:imageB.shape[0], 0:imageB.shape[1]] = imageB

        # Visualize matched points, if requested
        # if showMatches:
        #     vis = self.drawMatches(imageA, imageB, kpsA, kpsB, matches, status)
        #     return (result, vis) # Return both stitched image, and visualization
        
        return result # return stitched image
    

    def detectAndDescribe(self, image):
        grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        if self.isv3:
            # Get sift features
            descriptor = cv2.xfeatures2d.SIFT_create()
            (kps, features) = descriptor.detectAndCompute(image, None)

        else:
            # Get sift features
            detector = cv2.FeatureDetector_create("SIFT")
            kps = detector.detect(grey)

            # extract features from the image
            extractor = cv2.DescriptorExtractor_create("SIFT")
            (kps, features) = extractor.compute(grey, kps)

        # Convert to numpy array
        kps = np.float32([kp.pt for kp in kps])

        return (kps, features)

    def match_keypoints(self, kpsA, kpsB, featuresA, featuresB, ratio, reprojThresh):
        # Find how to map one image onto another with the given 
        # features and points
        matcher = cv2.DescriptorMatcher_create("BruteForce")
        # For all points, find two best matches for it in the other image
        rawMatches = matcher.knnMatch(featuresA, featuresB, 2)
        matches = []

        # Keep only the best matches
        for m in rawMatches:
            # Use Lowe's ratio test to prune false positives 
            # (ie if top two patches are close to each other, prob false positive)
            if len(m) == 2 and m[0].distance < m[1].distance * ratio:
                matches.append((m[0].trainIdx, m[0].queryIdx))
        
        # Compute homography; need bare min of 4 matches
        if len(matches) > 4:
            # Construct point sets
            ptsA = np.float32([kpsA[i] for (_, i) in matches])
            ptsB = np.float32([kpsB[i] for (i, _) in matches])

            (H, status) = cv2.findHomography(ptsA, ptsB, cv2.RANSAC, reprojThresh)

            return (matches, H, status)
        else:
            return None



        

def main():
    rclpy.init()

    # This node uses other resources (opencv camera, process for ffmpeg)
    # So cleanup (outside of destroy_node) is needed.
    with PanoramaNode() as node:
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

## INSTALL imutils AND opencv-contrib-python##
