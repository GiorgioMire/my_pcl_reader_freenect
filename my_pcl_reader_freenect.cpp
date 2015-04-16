
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "freenect_grabber.hpp"
#include "opencv2/highgui/highgui.hpp"
 #include "opencv2/imgproc/imgproc.hpp"
 #include "opencv2/core/core.hpp"
#include <string.h>
#include "omp.h"
const int distance = 7000;
 // 
//int contatore_save=0;
//Mat rgbframe;
//Mat depthframe;




void GestoreEvento_tastoR(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud){
  static int contatore_salvataggi=0;
  std::cout << "Hai premuto il tasto r; salvo la cloud " << std::endl;
    contatore_salvataggi++;
    pcl::io::savePCDFileASCII ("test_pcd"+std::to_string(contatore_salvataggi)+".pcd", *cloud);
}

 




void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* cloud_void)
{
  //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
   boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud = *static_cast<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>*> (cloud_void);

  if (event.getKeySym () == "r" && event.keyDown ())
    GestoreEvento_tastoR(cloud);
}




int main( int argc, char** argv )
{



boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud;
  freenectGrabber<pcl::PointXYZRGB> c;

  cloud = c.get_point_cloud(distance, true);




  cloud->sensor_orientation_.w () = 0.0;
  cloud->sensor_orientation_.x () = 1.0;
  cloud->sensor_orientation_.y () = 0.0;
  cloud->sensor_orientation_.z () = 0.0;









  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (255, 255, 255);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&cloud);
  //viewer->registerMouseCallback (mouseEventOccurred, (void*)&viewer);

 cv::namedWindow("RGB");
   cv::Mat rgbframe(cv::Size(640,480),CV_8UC3);
   cv::Mat depthframe(cv::Size(640,480),CV_16UC1);
  while (!viewer->wasStopped ()) {
   // cout<<endl<<c.rgb.size()<<endl;
      //cv::Mat depthframe=cv::Mat(c.depth_map);

    #pragma omp parallel for
   for(int i=0;i<rgbframe.rows;i++)
    for(int j=0;j<rgbframe.cols;j++){
      cv::Vec3b p;
                        p[2]= c.rgb[(i*rgbframe.cols + j)*3];
                        p[1] = c.rgb[(i*rgbframe.cols + j)*3 + 1];
                        p[0] = c.rgb[(i*rgbframe.cols+ j)*3 + 2];
                        rgbframe.at<cv::Vec3b>(i,j)=p;
                      }
  #pragma omp parallel for
   for(int i=0;i<rgbframe.rows;i++)
    for(int j=0;j<rgbframe.cols;j++){
   
                        
                      
                        depthframe.at<uint16_t>(i,j)= c.depth_map[(i*rgbframe.cols + j)];;
                      }




  //cv::namedWindow("Depth");
cv::imshow("Depth",depthframe);                      
cv::imshow("RGB",rgbframe);
cv::waitKey(1);

viewer->spinOnce ();
cloud = c.get_point_cloud(distance, true);
 




    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->updatePointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud"); 

  
  }
  cloud->height = 1;
  cloud->width = cloud->points.size();

  return 0;
}



