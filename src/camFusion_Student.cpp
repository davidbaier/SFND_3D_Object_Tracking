
#include <iostream>
#include <algorithm>
#include <math.h>
#include <limits>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;

// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2); // pixel coordinates
        pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}


void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);  
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    std::vector<cv::DMatch> tmp_matches;
    for(auto match {kptMatches.begin()}; match != kptMatches.end(); ++match){
        float shrinkFactor {0.1};

        cv::Rect smallerBox;
        smallerBox.x = boundingBox.roi.x + shrinkFactor * boundingBox.roi.width / 2.0;
        smallerBox.y = boundingBox.roi.y + shrinkFactor * boundingBox.roi.height / 2.0;
        smallerBox.width = boundingBox.roi.width * (1 - shrinkFactor);
        smallerBox.height = boundingBox.roi.height * (1 - shrinkFactor);

        if(smallerBox.contains(kptsCurr[match->trainIdx].pt) && smallerBox.contains(kptsPrev[match->queryIdx].pt)){
            tmp_matches.push_back(*match);
        }
    }

    float mean_euc_dist {0.0};
    for (auto match {tmp_matches.begin()}; match != tmp_matches.end(); ++match){
        mean_euc_dist += match->distance;
    }

    if(tmp_matches.size() > 0) {
        mean_euc_dist = mean_euc_dist/tmp_matches.size();
    }

    for(auto it {tmp_matches.begin()}; it != tmp_matches.end(); ++it){
            if(it->distance <= mean_euc_dist + (mean_euc_dist*0.1) && it->distance >= mean_euc_dist - (mean_euc_dist*0.75)){
                boundingBox.kptMatches.push_back(*it);
            }
        }
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    double dT {1/frameRate};

    std::vector<double> ratios;

    double minDist = 100.0;
    for(auto match {kptMatches.begin()}; match != kptMatches.end(); ++match){
        cv::KeyPoint currPts {kptsCurr.at(match->trainIdx)};
        cv::KeyPoint prevPts {kptsPrev.at(match->queryIdx)};

        for(auto it {kptMatches.begin()}; it != kptMatches.end(); ++it){
            if(match == it ){
                continue;
            }
            cv::KeyPoint currItPts {kptsCurr.at(it->trainIdx)};
            cv::KeyPoint prevItPts {kptsPrev.at(it->queryIdx)};

            double currDist = cv::norm(currPts.pt - currItPts.pt);
            double prevDist = cv::norm(prevPts.pt - prevItPts.pt);

            if(prevDist > std::numeric_limits<double>::epsilon() && currDist >= minDist){
                ratios.push_back(currDist/prevDist);
            }
        }
    }


    if(ratios.size() == 0){
        TTC = NAN;
        return;
    }

    std::sort(ratios.begin(), ratios.end());
    long index {static_cast<long>(std::floor(ratios.size()/2.0))};
    double median_ratio = ratios.size() % 2 == 0 ? (ratios[index] + ratios[index-1])/2 :ratios[index];

    TTC = -dT/(1 - median_ratio);
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    if(lidarPointsCurr.size() == 0 || lidarPointsPrev.size() == 0){
        TTC = NAN;
        return;
    }

    float median_prev_x {0.0};
    float median_curr_x {0.0};
    for(auto lidarPrev {lidarPointsPrev.begin()}; lidarPrev != lidarPointsPrev.end(); ++lidarPrev){
        median_prev_x += lidarPrev->x;
    }

    for(auto lidarCurr {lidarPointsCurr.begin()}; lidarCurr != lidarPointsCurr.end(); ++lidarCurr){
        median_curr_x += lidarCurr->x;
    }

    median_prev_x /= lidarPointsPrev.size();
    median_curr_x /= lidarPointsCurr.size();

    float diff {median_prev_x - median_curr_x};
    double dT {1/frameRate};

    TTC = (median_curr_x/diff) * dT;
}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    std::map<int, std::map<int, int>> hitCounter;
    for(auto bb {prevFrame.boundingBoxes.begin()}; bb != prevFrame.boundingBoxes.end(); ++bb){
        for(auto match {matches.begin()}; match != matches.end(); ++match){
            cv::KeyPoint tmp {currFrame.keypoints[match->trainIdx]};
            if(bb->roi.contains(tmp.pt)){
                bb->kptMatches.push_back(*match);
            }
        }
        std::map<int, int> tmpCounter;
        for(auto currBB {currFrame.boundingBoxes.begin()}; currBB != currFrame.boundingBoxes.end(); ++currBB){
            int count {0};
            for(auto bbMatch {bb->kptMatches.begin()}; bbMatch != bb->kptMatches.end(); ++bbMatch){
                cv::KeyPoint tmp {prevFrame.keypoints[bbMatch->queryIdx]};
                if(currBB->roi.contains(tmp.pt)){
                    count++;
                }
            }
            tmpCounter[currBB->boxID] = count;
        }
        int tmp_box_id {-1};
        int max {0};
        for(auto val {tmpCounter.begin()}; val != tmpCounter.end(); ++val){
            if(val->second > max){
                tmp_box_id = val->first;
                max = val->second;
            }
        }
        if(tmp_box_id >= 0){
            bbBestMatches[bb->boxID] = tmp_box_id;
        }
    }
}
