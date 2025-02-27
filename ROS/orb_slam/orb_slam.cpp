#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <limits>
#include <cmath>
#include <thread>
#include <chrono>

void detectHarrisCorners(const cv::Mat& image, std::vector<cv::Point>& corners, int blockSize = 2, int ksize = 3, double k = 0.04) {
    cv::Mat dst = cv::Mat::zeros(image.size(), CV_32FC1);
    cv::cornerHarris(image, dst, blockSize, ksize, k);
    double minVal, maxVal;
    cv::Point minLoc, maxLoc;
    cv::minMaxLoc(dst, &minVal, &maxVal, &minLoc, &maxLoc);
    double threshold = 0.01 * maxVal;
    for (int y = 0; y < dst.rows; y++) {
        for (int x = 0; x < dst.cols; x++) {
            if (dst.at<float>(y, x) > threshold) {
                corners.push_back(cv::Point(x, y));
            }
        }
    }
}

void computeSimpleDescriptors(const cv::Mat& image, const std::vector<cv::Point>& corners, cv::Mat& descriptors, int patchSize = 5) {
    int halfPatch = patchSize / 2;
    descriptors = cv::Mat::zeros(static_cast<int>(corners.size()), patchSize * patchSize, CV_32FC1);
    for (size_t i = 0; i < corners.size(); i++) {
        int x = corners[i].x;
        int y = corners[i].y;
        int idx = 0;
        for (int dy = -halfPatch; dy <= halfPatch; dy++) {
            for (int dx = -halfPatch; dx <= halfPatch; dx++) {
                int px = x + dx;
                int py = y + dy;
                if (px >= 0 && px < image.cols && py >= 0 && py < image.rows) {
                    descriptors.at<float>(i, idx) = static_cast<float>(image.at<uchar>(py, px));
                } else {
                    descriptors.at<float>(i, idx) = 0.0f;
                }
                idx++;
            }
        }
    }
}

class DescriptorMatcher {
public:
    DescriptorMatcher() {}
    float computeEuclideanDistance(const cv::Mat& desc1, const cv::Mat& desc2) {
        float distance = 0.0f;
        for (int i = 0; i < desc1.cols; ++i) {
            float diff = desc1.at<float>(0, i) - desc2.at<float>(0, i);
            distance += diff * diff;
        }
        return std::sqrt(distance);
    }

    void match(const cv::Mat& descriptors1, const cv::Mat& descriptors2, 
               std::vector<cv::DMatch>& matches, 
               const std::vector<cv::Point>& corners1, 
               const std::vector<cv::Point>& corners2) {
        matches.clear();
        for (int i = 0; i < descriptors1.rows; ++i) {
            float minDist = std::numeric_limits<float>::max();
            int minIdx = -1;
            for (int j = 0; j < descriptors2.rows; ++j) {
                float dist = computeEuclideanDistance(descriptors1.row(i), descriptors2.row(j));
                if (dist < minDist) {
                    minDist = dist;
                    minIdx = j;
                }
            }
            if (minIdx != -1) {
                matches.push_back(cv::DMatch(i, minIdx, minDist));
            }
        }
    }
};

int main() {
    cv::Mat image1 = cv::imread("1.png", cv::IMREAD_GRAYSCALE);
    cv::Mat image2 = cv::imread("2.png", cv::IMREAD_GRAYSCALE);
    if (image1.empty() || image2.empty()) {
        std::cerr << "Error: Could not load images." << std::endl;
        return 1;
    }
    cv::imshow("show1", image1);
    cv::imshow("show2", image2);
    cv::waitKey(1000);
    //std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    //detect corners
    std::vector<cv::Point> corners1, corners2;
    detectHarrisCorners(image1, corners1);
    detectHarrisCorners(image2, corners2);
    // Descripter
    cv::Mat descriptors1, descriptors2;
    computeSimpleDescriptors(image1, corners1, descriptors1);
    computeSimpleDescriptors(image2, corners2, descriptors2);
    cv::Mat output1 = image1.clone();  // 원본 이미지를 복사하여 시각화용 이미지 생성
    cv::Mat output2 = image2.clone();  // 원본 이미지를 복사하여 시각화용 이미지 생성
    for (const auto& corner1 : corners1) {
        cv::circle(output1, corner1, 5, cv::Scalar(255), 2);
    }
    cv::imshow("Detected Corners1", output1);
    for (const auto& corner2 : corners2) {
        cv::circle(output2, corner2, 5, cv::Scalar(255), 2);
    }
    cv::imshow("Detected Corners2", output2);
    cv::waitKey(1000);
    //std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // Descriptor Matching
    DescriptorMatcher matcher;
    std::vector<cv::DMatch> matches;
    matcher.match(descriptors1, descriptors2, matches, corners1, corners2);
    std::cout << "Number of matches: " << matches.size() << std::endl;
    std::vector<cv::KeyPoint> keypoints1, keypoints2;
    for (const auto& corner : corners1) {
        keypoints1.push_back(cv::KeyPoint(corner.x, corner.y, 5.0));
    }
    for (const auto& corner : corners2) {
        keypoints2.push_back(cv::KeyPoint(corner.x, corner.y, 5.0));
    }
    cv::Mat output;
    cv::drawMatches(image1, keypoints1, image2, keypoints2, matches, output);
    cv::imshow("All Matches", output);
    double min_distance = 100000, max_distance = 0;
    for (const auto& match : matches) {
        if (match.distance > max_distance)
            max_distance = match.distance;
        if (match.distance < min_distance)
            min_distance = match.distance;
    }
    std::vector<cv::DMatch> good_matches1;
    for (const auto& match : matches) {
        if (match.distance <= std::max(min_distance * 1.6, 25.0)) {
            good_matches1.push_back(match);
        }
    }
    std::cout << "Good matches (Method 1 - Min Distance Threshold): " << good_matches1.size() << std::endl;
    cv::Mat good_matches_img1;
    cv::drawMatches(image1, keypoints1, image2, keypoints2, good_matches1, good_matches_img1);
    cv::imshow("Good Matches (Method 1)", good_matches_img1);
    cv::Point2d principal_point(325.1, 249.7); // Provided principal point
    double focal_length = 521;                 // Provided focal length
    if (good_matches1.size() >= 5) {
        std::vector<cv::Point2f> points1, points2;
        for (const auto& match : good_matches1) {
            points1.push_back(keypoints1[match.queryIdx].pt);
            points2.push_back(keypoints2[match.trainIdx].pt);
        }
        // Compute the essential matrix
        cv::Mat essential_matrix = cv::findEssentialMat(points1, points2, focal_length, principal_point);
        if (!essential_matrix.empty()) {
            // Recover the rotation matrix R and translation vector t
            cv::Mat R, t;
            int inliers = cv::recoverPose(essential_matrix, points1, points2, R, t, focal_length, principal_point);
            // Output the results
            std::cout << "Number of inliers from pose recovery: " << inliers << std::endl;
            std::cout << "Essential Matrix:" << std::endl << essential_matrix << std::endl;
            std::cout << "Rotation Matrix (R):" << std::endl << R << std::endl;
            std::cout << "Translation Vector (t):" << std::endl << t << std::endl;
        } else {
            std::cerr << "Failed to compute essential matrix." << std::endl;
        }
    } else {
        std::cerr << "Not enough good matches to compute essential matrix (need at least 5)." << std::endl;
    }
    cv::waitKey(0);
    return 0;
}