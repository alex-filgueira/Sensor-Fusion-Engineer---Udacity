# SFND 2D Feature Tracking

## Example: HARRIS + BRIEF:
![HARRIS + BRIEF](output/harris_BRIEF.gif)


# Result test using Selector Type SEL_KNN:

## Results using Matcher Type MAT_BF:
![Table data test BF](output/table_BF.PNG)


## Results using Matcher Type MAT_FLANN:
![Table data test FLANN](output/table_FLANN.PNG)

# Top 3+1 pairs:
How we are working in the ambit of autonomous driving, the "real time" is very important. For this I selected the next TOP3+1 pair, because the resuts are robust and quickly.

## MAT_BF:
![Table data test BF](output/top3_BF.PNG)


## MAT_FLANN:
![Table data test FLANN](output/top3_FLANN.PNG)





## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./2D_feature_tracking`.
