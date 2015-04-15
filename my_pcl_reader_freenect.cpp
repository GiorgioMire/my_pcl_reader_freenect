
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "freenect_grabber.hpp"
#include "opencv2/highgui/highgui.hpp"
 #include "opencv2/imgproc/imgproc.hpp"
 #include "opencv2/core/core.hpp"
#include <string.h>
using namespace cv;
const int distance = 7000;
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud;
int contatore_save=0;
Mat rgbframe;






 




void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                        void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
  if (event.getKeySym () == "r" && event.keyDown ())
  {
    std::cout << "Hai premuto il tasto r; salvo la cloud " << std::endl;
    contatore_save++;
    pcl::io::savePCDFileASCII ("test_pcd"+std::to_string(contatore_save)+".pcd", *cloud);
}
}




int main( int argc, char** argv )
{




  freenectGrabber<pcl::PointXYZRGB> c;

  cloud = c.get_point_cloud(distance, true);

 namedWindow("RGB");


  cloud->sensor_orientation_.w () = 0.0;
  cloud->sensor_orientation_.x () = 1.0;
  cloud->sensor_orientation_.y () = 0.0;
  cloud->sensor_orientation_.z () = 0.0;









  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (255, 255, 255);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);
  //viewer->registerMouseCallback (mouseEventOccurred, (void*)&viewer);

  while (!viewer->wasStopped ()) {

//cout<<endl<<"Salvata"<<endl;
    viewer->spinOnce ();
    cloud = c.get_point_cloud(distance, true);
rgbframe=c.ottieniRGB();
imshow("RGB",rgbframe);
 waitKey(100);



    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->updatePointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud"); 

  
  }
  cloud->height = 1;
  cloud->width = cloud->points.size();

  return 0;
}



