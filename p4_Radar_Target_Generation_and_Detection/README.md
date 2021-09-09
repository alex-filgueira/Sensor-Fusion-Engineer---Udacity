# Radar_Target_Generation_and_Detection

#### Click in the image above for see the video project overview!
[![Project Overview](https://img.youtube.com/vi/DIVmHps0G8M/maxresdefault.jpg)](https://youtu.be/DIVmHps0G8M)



## Project Rubric Points

#### 1. FMCW Waveform Design
Using the given system requirements, design
a FMCW waveform. Find its Bandwidth (B), chirp time (Tchirp) and slope of the chirp.

#### 2. Simulation Loop
Simulate Target movement and calculate the beat or mixed signal for every timestamp.

#### 3. Range FFT (1st FFT)
Implement the Range FFT on the Beat or Mixed Signal and plot the result.

#### 4. 2D CFAR
Implement the 2D CFAR process on the output of 2D FFT operation, i.e the Range Doppler Map.


## Radar Specifications
```
% Frequency of operation = 77GHz
% Max Range = 200m
% Range Resolution = 1 m
% Max Velocity = 100 m/s
%%%%%%%%%%%%%%%%%%%%%%%%%%%

%speed of light = 3e8
```

## User Defined Range and Velocity of target
```
Velocity = 20; %velocity can be any value in the range of -70 to + 70 m/s.
InitialRange = 70; %Range cannot exceed the max value of 200m
```

## FFTP
FFTP output below
![FFTP output](images/fig1.jpg)
```
%% Signal generation and Moving Target simulation
% Running the radar scenario over the time. 

for i=1:length(t)         
    
    
    % *%TODO* :
    %For each time stamp update the Range of the Target for constant velocity. 
    r_t(i) = InitialRange + Velocity*t(i);
    td(i) = 2 * r_t(i)/c;
    
    % *%TODO* :
    %For each time sample we need update the transmitted and
    %received signal. 
    Tx(i) = cos(2*pi*(fc*t(i) + slope*(t(i)^2)/2 ));
    delay = t(i) - td(i);
    Rx (i)  = cos(2*pi*( fc*delay + slope * (delay^2)/2 )) ;
    
    % *%TODO* :
    %Now by mixing the Transmit and Receive generate the beat signal
    %This is done by element wise matrix multiplication of Transmit and
    %Receiver Signal
    Mix(i) = Tx(i)*Rx(i);
    
end

%% RANGE MEASUREMENT


 % *%TODO* :
%reshape the vector into Nr*Nd array. Nr and Nd here would also define the size of
%Range and Doppler FFT respectively.
Mix = reshape(Mix, [Nr, Nd]);

 % *%TODO* :
%run the FFT on the beat signal along the range bins dimension (Nr) and
%normalize.
signal_fft = fft(Mix, Nr); 
L = Tchirp * Bandwidth;

 % *%TODO* :
% Take the absolute value of FFT output
signal_fft = abs(signal_fft/L);

 % *%TODO* :
% Output of FFT is double sided signal, but we are interested in only one side of the spectrum.
% Hence we throw out half of the samples.
signal_fft = signal_fft(1:L/2+1);
```


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
