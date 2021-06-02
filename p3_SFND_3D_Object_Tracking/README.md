# SFND 3D Object Tracking


## Project Rubric Points

#### 1. FP.1 Match 3D Objects

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

#### 2. FP.2 Compute Lidar-based TTC

#### 3. FP.3 Associate Keypoint Correspondences with Bounding Boxes

#### 4. FP.4 Compute Camera-based TTC

#### 5. FP.5 Performance Evaluation 1

#### 6. FP.6 Performance Evaluation 2


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level project directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./3D_object_tracking`.