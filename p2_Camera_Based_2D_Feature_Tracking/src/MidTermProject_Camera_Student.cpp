/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"

using namespace std;

int run( string detectorType, string descriptorType);

int main(int argc, const char *argv[])
{
    //for test easy----
    string detectorType_list [] = {"SHITOMASI", "HARRIS", "FAST", "BRISK", "ORB", "AKAZE", "SIFT"};
    string descriptorType_list [] = {"BRISK", "BRIEF", "ORB", "FREAK",};//AKAZE, SIFT out

    // set input params
    string detectorType;
    string descriptorType;
    
    bool flag_arg = true;
    bool flag_test = false;
    if(argc == 1){
        detectorType = "SHITOMASI";
        descriptorType = "BRISK";
    }
    else if(argc == 2){
         if(std::string(argv[1]) == "-h"){
            cout << "detectorType: SHITOMASI, HARRIS, FAST, BRISK, ORB, AKAZE, SIFT" << endl;
            cout << "descriptorType: BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT" << endl;
            cout << "-test: All detectors + all descriptors" << endl;
            flag_arg = false;
        }
        else  if(std::string(argv[1]) == "-test"){
            flag_test = true;
        }
        else{
            cout << "bad parameter -> introduce -h for info.";
            flag_arg = false;
        }
    }
    else if(argc == 3){
        detectorType = argv[1];
        descriptorType = argv[2];
    }
    else{
        cout << "Need 0 or 2 parameters";
        flag_arg = false;
    }

    //for test easy----
  std::ofstream myfile;
  myfile.open ("../output/statistics.csv");
  myfile << "detector+descriptor,kp,matched_kp,time,\n";
  myfile.close();
  
    if(flag_test){
        for(const string &detector : detectorType_list){
            for(const string &descriptor : descriptorType_list){
                run(detector, descriptor);
            }
        }
    }
    else if(flag_arg){
        run(detectorType, descriptorType);
    }
}



/* MAIN PROGRAM */
int run( string detectorType, string descriptorType)
{
    string descriptorType_BIN_HOG;
    // get detector and descriptor type from input args
    if(descriptorType == "SIFT"){
        descriptorType_BIN_HOG = "DES_HOG";
    }
    else{
        descriptorType_BIN_HOG = "DES_BINARY";
    }

    /* INIT VARIABLES AND DATA STRUCTURES */
    // data location
    string dataPath = "../";

    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 9;   // last file index to load
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // misc
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
    bool bVis = false;            // visualize results


    // statistics (added)
    int all_kpts_cnt = 0;
    int all_match_kpts_cnt = 0;
  
  	cv::TickMeter tm;
  	tm.start();

    /* MAIN LOOP OVER ALL IMAGES */

    for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
    {
        /* LOAD IMAGE INTO BUFFER */

        // assemble filenames for current index
        ostringstream imgNumber;
        imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
        string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

        // load image from file and convert to grayscale
        cv::Mat img, imgGray;
        img = cv::imread(imgFullFilename);
        cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

        //// STUDENT ASSIGNMENT
        //// TASK MP.1 -> replace the following code with ring buffer of size dataBufferSize

        // push image into data frame buffer
        DataFrame frame;
        frame.cameraImg = imgGray;
        dataBuffer.push_back(frame);

        // erase first data in order to keep fix dataBufferSize size
        if(dataBuffer.size() > dataBufferSize){
            dataBuffer.erase(dataBuffer.begin());
        }

        //// EOF STUDENT ASSIGNMENT
        //cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;

        /* DETECT IMAGE KEYPOINTS */

        // extract 2D keypoints from current image
        vector<cv::KeyPoint> keypoints; // create empty feature list for current image

        //string detectorType = "SHITOMASI";

        //// STUDENT ASSIGNMENT
        //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
        //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT

        if (detectorType.compare("SHITOMASI") == 0){
            detKeypointsShiTomasi(keypoints, imgGray, false);
        }
        else if(detectorType.compare("HARRIS") == 0){
            detKeypointsHarris(keypoints, imgGray, false);
        }
        else{
            detKeypointsModern(keypoints, imgGray, detectorType, false);
        }
        //// EOF STUDENT ASSIGNMENT

        //// STUDENT ASSIGNMENT
        //// TASK MP.3 -> only keep keypoints on the preceding vehicle

        // only keep keypoints on the preceding vehicle
        bool bFocusOnVehicle = true;
        cv::Rect vehicleRect(535, 180, 180, 150);
        if(bFocusOnVehicle)
        {
            vector<cv::KeyPoint> keypoints_roi;
            for(auto itr = keypoints.begin(); itr < keypoints.end(); itr++){
                if (vehicleRect.contains((*itr).pt)){
                    cv::KeyPoint newKeyPoint;
                    newKeyPoint.pt = cv::Point2f(itr->pt);
                    keypoints_roi.push_back(newKeyPoint);
                }
            }
            keypoints = keypoints_roi;
        }
        all_kpts_cnt += keypoints.size();

        //// EOF STUDENT ASSIGNMENT

        // optional : limit number of keypoints (helpful for debugging and learning)
        bool bLimitKpts = false;
        if (bLimitKpts)
        {
            int maxKeypoints = 50;

            if (detectorType.compare("SHITOMASI") == 0)
            { // there is no response info, so keep the first 50 as they are sorted in descending quality order
                keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
            }
            cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
            //cout << " NOTE: Keypoints have been limited!" << endl;
        }

        // push keypoints and descriptor for current frame to end of data buffer
        (dataBuffer.end() - 1)->keypoints = keypoints;
        //cout << "#2 : DETECT KEYPOINTS done" << endl;

        /* EXTRACT KEYPOINT DESCRIPTORS */

        //// STUDENT ASSIGNMENT
        //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
        //// -> BRIEF, ORB, FREAK, AKAZE, SIFT

        cv::Mat descriptors;
        //string descriptorType = "BRISK"; // BRIEF, ORB, FREAK, AKAZE, SIFT
        descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType);
        //// EOF STUDENT ASSIGNMENT

        // push descriptors for current frame to end of data buffer
        (dataBuffer.end() - 1)->descriptors = descriptors;

        //cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

        if (dataBuffer.size() > 1) // wait until at least two images have been processed
        {

            /* MATCH KEYPOINT DESCRIPTORS */

            vector<cv::DMatch> matches;
            string matcherType = "MAT_BF";        // MAT_BF, MAT_FLANN
            //string descriptorType = "DES_BINARY"; // DES_BINARY, DES_HOG
            string selectorType = "SEL_KNN";       // SEL_NN, SEL_KNN -> test change

            //// STUDENT ASSIGNMENT
            //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
            //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp

            matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                             (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                             matches, descriptorType_BIN_HOG, matcherType, selectorType);

            //// EOF STUDENT ASSIGNMENT

            // store matches in current data frame
            (dataBuffer.end() - 1)->kptMatches = matches;

            all_match_kpts_cnt += matches.size();

            //cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;

            // visualize matches between current and previous image
            if (bVis)
            {
                cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
                cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
                                (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints,
                                matches, matchImg,
                                cv::Scalar::all(-1), cv::Scalar::all(-1),
                                vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                string windowName = "Matching keypoints between two camera images";
                cv::namedWindow(windowName, 7);
                cv::imshow(windowName, matchImg);
                cout << "Press key to continue to next image" << endl;
                cv::waitKey(0); // wait for key to be pressed
            }
        }

    } // eof loop over all images
  	tm.stop();
  
    // output statistics
    std::ofstream myfile;
  	myfile.open ("../output/statistics.csv", std::ofstream::app);
    cout << detectorType << " + " << descriptorType << endl;
    myfile << detectorType<< " + " << descriptorType << ",";
    cout << "keypoints: " << all_kpts_cnt << endl;
  	myfile << all_kpts_cnt << ",";
    cout << "matched keypoints: " << all_match_kpts_cnt << endl;
  	myfile << all_match_kpts_cnt << ",";
  	cout << "tm: " <<  tm.getTimeMilli() << " ms" << endl;
  	myfile << tm.getTimeMilli() << ",\n";
	myfile.close();
    return 0;
}