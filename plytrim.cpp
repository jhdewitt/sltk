// plytrim
// trim down ply files using bounding box
// by John De Witt

#include <opencv2/opencv.hpp>
//#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
//#include <opencv2/imgcodecs.hpp>

//#include <ImfRgbaFile.h>
//#include <ImfStringAttribute.h>
//#include <ImfMatrixAttribute.h>
//#include <ImfArray.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <map>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>

#include <assert.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>

//#include <stdio.h>
//#include <math.h>
//#include <string.h>
//#include <time.h>
//#include <stdint.h>

#include "util.h"
#include "sl_util.h"

#include <iostream>
#include <fstream>
#include <string>

using namespace cv;
using namespace std;
//using namespace Imf;
//using namespace Imath;

#define DBUG 0
#define THRESHOLD_VALUE 5

#define SUBSAMPLE_GRID 0
#define SUBSAMPLE_POINT 1

enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };

static void help()
{
   printf("This program accepts a ply point cloud file and outputs a copy with certain points removed.\n"
          "  Notable options include:\n"
          "   - Bounding box inclusion\n"

          "ex: ply_trim -bb 0 0 0 1 1 1 model_01.ply\n"
          "Usage: ply_trim\n"
          "    -bb                   # specify bounding box of points to keep min(x,y,z) max(x,y,z) xyz\n"
          "    [-s <file.ply>]       # subtraction cloud - reference pointcloud to subtract from input pointcloud\n"
          "    [-d 1.0]              # subtraction cloud - distance from any subtraction cloud points to cull\n"
          "    input_data            # input data\n"
          "\n");
}

template<typename ... Args>
string string_format( const std::string& format, Args ... args )
{
    size_t size = snprintf( nullptr, 0, format.c_str(), args ... ) + 1; // Extra space for '\0'
    unique_ptr<char[]> buf( new char[ size ] ); 
    snprintf( buf.get(), size, format.c_str(), args ... );
    return string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
}

Vec3b float2Color( float t ){
   
   float n = 4.f;
   float dt = 1.f/n;
   
   float r,g,b;
   if(t<=1.f*dt)
   {
      float c = n*(t-0.f*dt);
      r = 0.f;
      g = 0.f;
      b = c;
   }
   else if (t<=2.f*dt){
      float c = n*(t-1.f*dt);
      r = 0.f;
      g = c;
      b = 1.f;
   }
   else if (t<=3.f*dt){
      float c = n*(t-2.f*dt);
      r = c;
      g = 1.f;
      b = 1.f-c;
   }
   else if (t<=4.f*dt){
      float c = n*(t-3.f*dt);
      r = 1.f;
      g = 1.f-c;
      b = 0.f;
   }
   
   Vec3b col;
   col[0] = (uint8_t)(255*r);
   col[1] = (uint8_t)(255*g);
   col[2] = (uint8_t)(255*b);
   
   //    if (t<=1.f*dt)
   //    {   //black -> red
   //        float c = n*(t-0.f*dt);
   //        col[0] = c;     //0-255
   //        col[1] = 0.f;   //0
   //        col[2] = 0.f;   //0
   //    }
   //    else if (t<=2.f*dt)
   //    {   //red -> red,green
   //        float c = n*(t-1.f*dt);
   //        col[0] = 255.f; //255
   //        col[1] = c;     //0-255
   //        col[2] = 0.f;   //0
   //    }
   //    else if (t<=3.f*dt)
   //    {   //red,green -> green
   //        float c = n*(t-2.f*dt);
   //        col[0] = 255.f-c;   //255-0
   //        col[1] = 255.f;     //255
   //        col[2] = 0.f;       //0
   //    }
   //    else if (t<=4.f*dt)
   //    {   //green -> blue
   //        float c = n*(t-3.f*dt);
   //        col[0] = 0.f;       //0
   //        col[1] = 255.f-c;   //255-0
   //        col[2] = c;         //0-255
   //    }
   return col;
}

// always assume x images precede y images
int main( int argc, char** argv ) {
   
   if(argc<2)
   {
      printf("not enough arguments, exiting..\n");
      return -1;
   }
   
   printf("sizeof(float) = %d\n",(int)sizeof(float));
   printf("sizeof(uchar) = %d\n",(int)sizeof(uchar));
   
   
   // point lists
   vector< Vec3f > in_triPoints;
   vector< Vec3b > in_rgbPoints;

   vector< Vec3f > out_triPoints;
   vector< Vec3b > out_rgbPoints;
   
   // bounding box
//   Vec3f bb_cen(256,19,-442);
//   Vec3f bb_dim(62,51,64);
//   
//   Vec3f bb_cen(44,0,-59);
//   Vec3f bb_dim(17,17,17);

   Vec3f bb_cen(-2,-20,-255);
   Vec3f bb_dim(50,50,50);
    
   float sub_dist_thresh = 10;
   
   string dat_fname = "";
   string sub_fname = "";
   string out_fmt = "trimmed_%s";
   string fname_out = "";

   for(int i=1; i<argc; i++)
   {
      const char* s = argv[i];
      if( strcmp(s,"--help") == 0 )
      {
         help();
         return 1;
      }
      else if( strcmp(s,"-s") == 0 )
      {
          sub_fname = string(argv[++i]);
      }
      else if( strcmp(s,"-d") == 0 )
      {
         if( sscanf( argv[++i], "%f", &sub_dist_thresh ) != 1 || sub_dist_thresh <= 0 )
            return fprintf( stderr, "Invalid subtraction distance threshold (<=0)\n" ), -1;
      }
      else
      {
          dat_fname = string(s);
          fname_out = string_format(out_fmt,dat_fname.c_str());
          cout << "destination file [" << fname_out << "]" << endl;
      }
   }

    
    // dat cloud
    cout << "reading source cloud.." << endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_dat (new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(dat_fname, *cloud_dat) == -1) 
    {
        cout << "Error loading [" << dat_fname << "]" << endl;
        return -1;
    }
    std::cout << "Loaded "
            << cloud_dat->width * cloud_dat->height
            << " data points from file " << dat_fname << " with the following fields: "
            << std::endl;
    if(DBUG)
    for (size_t i = 0; i < fmin(20,cloud_dat->points.size ()); ++i)
    {
     std::cout << "    " << cloud_dat->points[i].x
              << " "    << cloud_dat->points[i].y
              << " "    << cloud_dat->points[i].z
              << " "    << cloud_dat->points[i].r
              << " "    << cloud_dat->points[i].g
              << " "    << cloud_dat->points[i].b << std::endl;
    }
    
    // sub cloud
    cout << "reading sub cloud.." << endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_sub_rgb (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sub (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(sub_fname, *cloud_sub_rgb) == -1) 
    {
        cout << "Error loading [" << sub_fname << "]" << endl;
        return -1;
    }
    std::cout << "Loaded "
            << cloud_sub_rgb->width * cloud_sub_rgb->height
            << " data points from file " << sub_fname << " with the following fields: "
            << std::endl;

    for (size_t i = 0; i < cloud_sub_rgb->points.size (); ++i)
    {
        pcl::PointXYZ xyz_pt( cloud_sub_rgb->points[i].x, cloud_sub_rgb->points[i].y, cloud_sub_rgb->points[i].z );
        cloud_sub->push_back(xyz_pt);
    }
    
    if(DBUG)
    for (size_t i = 0; i < fmin(20,cloud_sub_rgb->points.size ()); ++i)
    {
     std::cout << "    " << cloud_sub_rgb->points[i].x
              << " "    << cloud_sub_rgb->points[i].y
              << " "    << cloud_sub_rgb->points[i].z
              << " "    << cloud_sub_rgb->points[i].r
              << " "    << cloud_sub_rgb->points[i].g
              << " "    << cloud_sub_rgb->points[i].b << std::endl;
    }
    
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_sub;
    kdtree_sub.setInputCloud (cloud_sub);
    


    // cull points from source data

    for (size_t i = 0; i < cloud_dat->points.size (); ++i)
    {
        if(DBUG)
        std::cout << "    " << cloud_dat->points[i].x
              << " "    << cloud_dat->points[i].y
              << " "    << cloud_dat->points[i].z
              << " "    << cloud_dat->points[i].r
              << " "    << cloud_dat->points[i].g
              << " "    << cloud_dat->points[i].b << std::endl;


        float radius = sub_dist_thresh;
        pcl::PointXYZ searchPoint;
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        
        if(DBUG)
        std::cout << "Neighbors within radius search at (" << searchPoint.x 
                << " " << searchPoint.y 
                << " " << searchPoint.z
                << ") with radius=" << radius << std::endl;

        Vec3f xyz_pt(cloud_dat->points[i].x,cloud_dat->points[i].y,cloud_dat->points[i].z);
        Vec3b rgb_pt(cloud_dat->points[i].r,cloud_dat->points[i].g,cloud_dat->points[i].b);
        searchPoint.x = xyz_pt[0];
        searchPoint.y = xyz_pt[1];
        searchPoint.z = xyz_pt[2];

        if ( kdtree_sub.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) == 0 )
        {
            
            out_triPoints.push_back(xyz_pt);
            out_rgbPoints.push_back(rgb_pt);
            
        }

    }


   // save PLY using procam calibration
   if( true ){
      printf("saving ply file..\n");
      
      //         triPoints.push_back(Vec3f(x,y,z));
      //         rgbPoints.push_back(Vec3b(r,g,b));
      
      // write PLY file
      ofstream myfile;
      myfile.open (fname_out, ios::out | ios::binary);
      
      if(myfile.is_open()){
         printf("writing PLY file: [%s] [%d points]\n", fname_out.c_str(), (int)out_triPoints.size() );
         char buf[50];
         sprintf(buf, "%d", (int)out_triPoints.size());
         
         // write header
         myfile << "ply\n";
         myfile << "format binary_little_endian 1.0\n";
         myfile << "comment author: john dewitt\n";
         myfile << "element vertex " << buf << "\n";
         myfile << "property float x\n";
         myfile << "property float y\n";
         myfile << "property float z\n";
         myfile << "property uchar red\n";
         myfile << "property uchar green\n";
         myfile << "property uchar blue\n";
         myfile << "end_header\n";
         
         // write binary data
         for(int i=0;i<out_triPoints.size();i++){
            myfile.write( (char*)(&out_triPoints[i]), sizeof(float)*3 );
            myfile.write( (char*)(&out_rgbPoints[i]), sizeof(uchar)*3 );
         }
      }
      
   }
   
}
