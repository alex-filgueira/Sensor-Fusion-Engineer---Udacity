/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false; //change to false
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
  	Lidar* lidar = new Lidar(cars, 0);
	pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar ->scan();
  	//renderRays(viewer, lidar->position, inputCloud);
  	renderPointCloud(viewer, inputCloud, "inputCloud");
  
    // TODO:: Create point processor
    //Planes
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(inputCloud, 100, 0.2);

    //Cluster
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 1.0, 3, 30);

    //boundingBox
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessor.numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]); //change name -> "obstCloud"+std::to_string(clusterId)
        Box box = pointProcessor.BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
        ++clusterId;
    }
}


void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessor, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud;

    //Filter
    filterCloud = pointProcessor->FilterCloud(inputCloud, 0.15 , Eigen::Vector4f (-20, -6, -2, 1), Eigen::Vector4f ( 30, 7, 5, 1));

    //Segment
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessor->SegmentPlane_RANSAC(filterCloud, 50, 0.2);
    renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));

    //Cluster
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessor->Clustering_EUCLIDEAN(segmentCloud.first, 0.3, 10, 1000);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
       std::cout << "cluster size ";
       pointProcessor->numPoints(cluster);
       renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId  % colors.size()]);

        //BoundigBox
       Box box = pointProcessor->BoundingBox(cluster);
       renderBox(viewer,box,clusterId);

       ++clusterId;
    }

}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;
  
  	std::cout << "argc:" <<argc<< std::endl;
    
  
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

    ProcessPointClouds<pcl::PointXYZI>* pointProcessor = new ProcessPointClouds<pcl::PointXYZI>();
  
  //You can select the cloud files folder here
  std::vector<boost::filesystem::path> stream;
    
  	if(argc > 1){
      if(std::string(argv[1]) == "1"){
        stream = pointProcessor->streamPcd("../src/sensors/data/pcd/data_1");
        std::cout << "selected data_1" << std::endl;
      }
      else{
      if(std::string(argv[1]) == "2"){
        stream = pointProcessor->streamPcd("../src/sensors/data/pcd/data_2");
        std::cout << "selected data_2" << std::endl;
      }
        else{
          stream = pointProcessor->streamPcd("../src/sensors/data/pcd/data_1"); 
        }
      }
    }
      else{
        stream = pointProcessor->streamPcd("../src/sensors/data/pcd/data_1"); 
        std::cout << "default data_1" << std::endl;
      }

    auto streamIterator = stream.begin();

    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud;

    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);

  
    while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloud = pointProcessor->loadPcd((*streamIterator).string());
        
        //Call cityBlock to identify the objects
        cityBlock(viewer, pointProcessor, inputCloud);

        // Increment to next scene and if the last scene is reached loop back to first scene.
        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();
    } 

}