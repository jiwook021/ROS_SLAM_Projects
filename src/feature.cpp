#include <iostream>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>



int main(int argc, char const *argv[]){
    

    std::string path1 = "/home/jiwook/catkin_ws/src/src/data/1.png";
    std::string path2 = "/home/jiwook/catkin_ws/src/src/data/2.png";
  
    cv::Mat img1 = cv::imread(path1, cv::IMREAD_COLOR);
    cv::Mat img2 = cv::imread(path2, cv::IMREAD_COLOR);
  
    cv::imshow("show1",img1);
    cv::imshow("show2",img2);
    cv::waitKey(0);

    //orb feature extractor 
    std::vector<cv::KeyPoint> keypoint_1, keypoint_2;
    cv::Mat descriptors_1, descriptors_2;

    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    
    //fast corner extraction
    detector -> detect(img1, keypoint_1);
    detector -> detect(img2, keypoint_2);

    //Descriptor ID 

    descriptor -> compute(img1, keypoint_1, descriptors_1);
    descriptor -> compute(img2, keypoint_2, descriptors_2);

    cv::Mat detect_result_img1;
    cv::drawKeypoints(img1,keypoint_1, detect_result_img1, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
    cv::imshow("draw result image 1", detect_result_img1);

    cv::Mat detect_result_img2;
    cv::drawKeypoints(img2,keypoint_2, detect_result_img2, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
    cv::imshow("draw result image 2", detect_result_img2);
 
    cv::waitKey(0);

    std::vector<cv::DMatch> matches;
    matcher -> match(descriptors_1,descriptors_2,matches);

    cv::Mat result_match;
    cv::drawMatches(img1,keypoint_1,img2,keypoint_2,matches, result_match);
    cv::imshow("result match", result_match);
    cv::waitKey(0);

    double min_distance = 100000, max_distance = 0; 
    for(int i=0; i<descriptors_1.rows;i++)
    {
        if (matches[i].distance > max_distance) 
            max_distance = matches[i].distance;
        if (matches[i].distance < min_distance) 
            min_distance = matches[i].distance;
    }

    std::vector<cv::DMatch> good_matches;

    for(int i = 0; i < descriptors_1.rows; i++){
        if(matches[i].distance <= std::max(min_distance*2,30.0))
        {
            good_matches.push_back(matches[i]);
        }
        
    }
    cv::Mat good_matches_img;
    
    cv::drawMatches(img1,keypoint_1, img2, keypoint_2, good_matches, good_matches_img);
    cv::imshow("Good result Match", good_matches_img);
    cv::waitKey(0);

    cv::Point2d principal_point(325.1,249.7); 
    double focal_length =521; 
    cv::Mat essential_matrix; 
    std::vector<cv::Point2f> points1; 
    std::vector<cv::Point2f> points2; 
    
    for(int i = 0; i <(int)good_matches.size();i++)
    {
        points1.push_back(keypoint_1[good_matches[i].queryIdx].pt);
        points2.push_back(keypoint_2[good_matches[i].trainIdx].pt);
    }

    essential_matrix = cv::findEssentialMat(points1, points2, focal_length, principal_point);

    std::cout<<"essential_matrix: " << std::endl << essential_matrix << std::endl; 

    cv::Mat R,t;
    cv::recoverPose(essential_matrix, points1, points2, R,t, focal_length, principal_point);
    std::cout << "R: " << std::endl << R << std::endl;
    std::cout << "t: " << std::endl << t << std::endl;
    
    return 0;
}

