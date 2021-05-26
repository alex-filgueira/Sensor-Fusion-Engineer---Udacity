+ The project code must compile without errors using cmake and make.

+ Bounding boxes enclose vehicles, and the pole on the right side of the vehicle. There is one box per detected object.

+ Most bounding boxes can be followed through the lidar stream, and major objects don't lose or gain bounding boxes in the middle of the lidar stream.

+ The code used for segmentation uses the 3D RANSAC algorithm developed in the course lesson.

+ The code used for clustering uses the Euclidean clustering algorithm along with the KD-Tree developed in the course lesson.



Here are some things to avoid. This is not a complete list, but there are a few examples of inefficiencies.

    # Running the exact same calculation repeatedly when you can run it once, store the value and then reuse the value later.
    # Loops that run too many times.
    # Creating unnecessarily complex data structures when simpler structures work equivalently.
    # Unnecessary control flow checks.
