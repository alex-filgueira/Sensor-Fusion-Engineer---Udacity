# Radar_Target_Generation_and_Detection

Click in the image above for see the video project overview!
[![Project Overview](https://img.youtube.com/vi/DIVmHps0G8M/maxresdefault.jpg)](https://youtu.be/DIVmHps0G8M)


![Animation_neural](output/Animation_neural.gif)

![Animation_ttc](output/Animation_ttc.gif)

## Project Rubric Points

#### 1. Implementation steps for the 2D CFAR process. 
In this task, please implement the method "matchBoundingBoxes", which takes as input both the previous and the current data frames and provides as output the ids of the matched regions of interest (i.e. the boxID property)“. Matches must be the ones with the highest number of keypoint correspondences.

#### Result:
```
void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    // ...
    int prevKpIdx, currKpIdx;
    cv::KeyPoint prevKp, currKp;

    int pSize = prevFrame.boundingBoxes.size();
    int cSize = currFrame.boundingBoxes.size();
    int counts[pSize][cSize] = {};

    vector<int> prevBoxesIds, currBoxesIds;

    for(auto it1=matches.begin(); it1!= matches.end(); ++it1){
        prevKpIdx = (*it1).queryIdx;
        currKpIdx = (*it1).trainIdx;

        prevKp = prevFrame.keypoints[prevKpIdx];
        currKp = currFrame.keypoints[currKpIdx];

        prevBoxesIds.clear();
        currBoxesIds.clear();

        for(auto it2 = prevFrame.boundingBoxes.begin(); it2!= prevFrame.boundingBoxes.end(); ++it2){
            if((*it2).roi.contains(prevKp.pt)){
                prevBoxesIds.push_back((*it2).boxID);
            }
        }

        for(auto it2 = currFrame.boundingBoxes.begin(); it2!= currFrame.boundingBoxes.end(); ++it2){
            if((*it2).roi.contains(prevKp.pt)){
                currBoxesIds.push_back((*it2).boxID);
            }
        }

        for(auto prevId:prevBoxesIds){
            for(auto currId:currBoxesIds){
                counts[prevId][currId]++;
            }
        }
    }

    int maxCount=0, maxId;
    for(int prevId=0; prevId<pSize; prevId++){
        maxCount = 0;
        for(int currId=0; currId<cSize; currId++){
            if (counts[prevId][currId] > maxCount){
                maxCount = counts[prevId][currId];
                maxId = currId;
            }
        }
        bbBestMatches[prevId] = maxId;
    }
}
```

#### 2. Selection of Training, Guard cells and offset.
In this part of the final project, your task is to compute the time-to-collision for all matched 3D objects based on Lidar measurements alone. Please take a look at the "Lesson 3: Engineering a Collision Detection System" of this course to revisit the theory behind TTC estimation. Also, please implement the estimation in a way that makes it robust against outliers which might be way too close and thus lead to faulty estimates of the TTC. Please return your TCC to the main function at the end of computeTTCLidar.

### Result:

```
void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    // ...
    double prevX=0, currX=0, speed;
    if(!lidarPointsPrev.size() || !lidarPointsCurr.size()){
        TTC = NAN;
        return;
    }
    for(auto it = lidarPointsPrev.begin(); it!= lidarPointsPrev.end(); ++it){
        prevX += (*it).x;
    }
    prevX/=lidarPointsPrev.size();

    for(auto it = lidarPointsCurr.begin(); it!= lidarPointsCurr.end(); ++it){
        currX += (*it).x;
    }
    currX/=lidarPointsCurr.size();

    speed = (prevX - currX) / (1/frameRate);
    if(speed<0){
        TTC = NAN;
        return;
    }
    TTC = currX/speed;
}
```

#### 3. Steps taken to suppress the non-thresholded cells at the edges.
