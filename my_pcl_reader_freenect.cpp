
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "freenect_grabber.hpp"
#include "opencv2/highgui/highgui.hpp"
 #include "opencv2/imgproc/imgproc.hpp"
 #include "opencv2/core/core.hpp"
#include "opencv2/contrib/contrib.hpp"
#include <string.h>
#include "boost/filesystem.hpp"



const int distance = 7000;
 // 
//int contatore_save=0;
//Mat rgbframe;
//Mat depthframe;

 struct cookiesStruct{
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud;
  cv::Mat rgb;
  cv::Mat depth;
  std::string percorso;
};


void GestoreEventoTastoR(void* cookiesVoid){
  cookiesStruct* cookies=(cookiesStruct*)cookiesVoid;
  static int contatoreSalvataggi=0;
  std::cout << "Hai premuto il tasto r; salvo la cloud " << std::endl;
    contatoreSalvataggi++;

    pcl::io::savePCDFileASCII ((cookies->percorso)+"PointCloud"+std::to_string(contatoreSalvataggi)+".pcd", *(cookies->cloud));
    cv::imwrite( (cookies->percorso)+"RGB"+std::to_string(contatoreSalvataggi)+".jpg", cookies->rgb);
    cv::imwrite( (cookies->percorso)+"DEPTH"+std::to_string(contatoreSalvataggi)+".jpg", cookies->depth);

}

 




void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* cookiesVoid)
{

  if (event.getKeySym () == "r" && event.keyDown ())
    GestoreEventoTastoR( cookiesVoid);
}


/*____________________________________________________________________________________________*/

int main( int argc, char** argv )
{
  /*Dichiarazioni*/
std::string nomeSoggetto;
boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud;
freenectGrabber<pcl::PointXYZRGB> c;
cookiesStruct cookies;

/* Fase iniziale*/

cout<<endl<<"PROGRAMMA ACQUISIZIONE DATI"<<endl;
cout<<endl<<"Digitare il nome del soggetto che si vuole acquisire e premere invio"<<endl;
cin>>nomeSoggetto;
const std::string nuovaCartellaString="./"+nomeSoggetto+"/";
boost::filesystem::path nuovaCartella=nuovaCartellaString;
boost::system::error_code returnedError;

boost::filesystem::create_directories( nuovaCartella, returnedError );

if ( returnedError )
  {cout<<endl<<"Nuova cartella creata"<<endl;}
   //did not successfully create directories
else 
  {cout<<endl<<"Nuova cartella non creata"<<endl;}




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
  viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&cookies);
  //viewer->registerMouseCallback (mouseEventOccurred, (void*)&viewer);

 cv::namedWindow("RGB");
 cv::namedWindow("Depth");

  cv::Mat rgbframe(cv::Size(640,480),CV_8UC3);
  cv::Mat depthframe(cv::Size(640,480),CV_16UC1);
  cv::Mat mapped;

  while (!viewer->wasStopped ()) {
memcpy(rgbframe.ptr(),(void*)&c.rgb.front(),640*480*3); 
memcpy(depthframe.ptr(),(void*)&c.depth_map.front(),640*480*2);






cv::cvtColor(rgbframe,rgbframe, cv::COLOR_BGR2RGB);
//cout<<depthframe;
depthframe.convertTo(mapped,CV_8U,256.0/4096.0);
viewer->spinOnce ();
cloud = c.get_point_cloud(distance, true);

cookies={
  cloud,
rgbframe,
mapped,
nuovaCartellaString
};
//cvtColor(depthframe,depthframe,CV_GRAY2BGR);
applyColorMap(mapped, mapped,cv::COLORMAP_HOT);
  //

cv::imshow("Depth",mapped);                      
cv::imshow("RGB",rgbframe);
cv::waitKey(1);






    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->updatePointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud"); 

  
  }
  cloud->height = 1;
  cloud->width = cloud->points.size();

  return 0;
}



