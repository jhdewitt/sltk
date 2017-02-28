// plymerge
// merge multiple pointclouds together and subsample to remove duplicates
// only XYZ data is preser
// intended to be used to estimate background for automatic pointcloud cleanup
// by John De Witt

#include <Eigen/Dense>

#include <opencv2/opencv.hpp>
//#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/eigen.hpp>
//#include <opencv2/imgcodecs.hpp>

//#include <ImfRgbaFile.h>
//#include <ImfStringAttribute.h>
//#include <ImfMatrixAttribute.h>
//#include <ImfArray.h>

#include <map>
#include <math.h>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <fstream>
#include <string>

#include <assert.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>

//#include <math.h>
//#include <string.h>
//#include <time.h>
//#include <stdint.h>

#include "util.h"
#include "sl_util.h"

#include <Eigen/Core>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>


using namespace cv;
using namespace std;
//using namespace Imf;
//using namespace Imath;

#define THRESHOLD_VALUE 5

#define SUBSAMPLE_GRID 0
#define SUBSAMPLE_POINT 1

// Types
typedef pcl::PointNormal PointNT;
typedef pcl::PointXYZ PointXYZT;
typedef pcl::PointCloud<PointXYZT> PointCloudT;

static void help()
{
   printf("This program accepts two or more ply point cloud files and outputs a merged and downsampled version. No registration is applied.\n"
          "ex: plymerge model_01.ply model_02.ply ..\n"
          "\n");
}

int main( int argc, char** argv ) {
   
   if(argc<2)
   {
      printf("not enough arguments, exiting..\n");
      return -1;
   }
   
   vector<string> fnames;
   vector<PointCloudT::Ptr> clouds;
   PointCloudT::Ptr merged_cloud (new PointCloudT);
   string fname_out = "merged_cloud.ply";

   // point lists
   vector< Vec3f > in_triPoints;
   vector< Vec3b > in_rgbPoints;

   vector< Vec3f > out_triPoints;
   vector< Vec3b > out_rgbPoints;
   
   float leaf_scl = 1.0;

   for(int i=1; i<argc; i++)
   {
        char* s=argv[i];
        if( strcmp(s,"-s") == 0 )
        {
            float v = 1;
            if( i+1 >= argc )
               return fprintf(stderr,"Leaf scale not provided\n"), -1;
            if( sscanf( argv[++i], "%f", &v) != 1 || v<=0 )
               return fprintf(stderr,"Invalid leaf scale value\n"), -1;
            leaf_scl = v;
            printf("leaf_scl set to %0.2f\n",leaf_scl);
        }
        else
        {
            string fn = s;
            fnames.push_back(fn);
            printf("argv[%d] : %s\n",i,fn.c_str());
        }
   }
   
   for (std::vector<string>::iterator it = fnames.begin() ; it != fnames.end(); ++it)
   {
       PointCloudT::Ptr tmp_cloud (new PointCloudT);
       if (pcl::io::loadPLYFile<PointXYZT>(*it, *tmp_cloud) == -1) 
       {
           cout << "\tError loading [" << *it << "]" << endl;
           return -1;
       }
       else
       {
           printf("loaded [%s].. %8d points\n",it->c_str(),(int)tmp_cloud->points.size());
           for (size_t i = 0; i < tmp_cloud->points.size(); ++i)
           {
               PointXYZT tmp_pt(tmp_cloud->points[i]);
               merged_cloud->push_back(tmp_pt);
               if(i<0)
               std::cout << "    " << tmp_cloud->points[i].x
                         << " "    << tmp_cloud->points[i].y
                         << " "    << tmp_cloud->points[i].z << std::endl;
           }
       }
   }
   
   int pt_before = merged_cloud->points.size();

   printf("downsampling to %0.2fmm grid..\n",leaf_scl);
   pcl::VoxelGrid<PointXYZT> grid;
   const float leaf = leaf_scl;
   grid.setLeafSize(leaf, leaf, leaf);
   grid.setInputCloud(merged_cloud);
   grid.filter(*merged_cloud);

   int pt_after = merged_cloud->points.size();
   
   printf("in_size  = %d\n",pt_before);
   printf("out_size = %d\n",pt_after);
   
    for (size_t i = 0; i < merged_cloud->points.size(); ++i)
    {
       Vec3f tmp_pt(merged_cloud->points[i].x,merged_cloud->points[i].y,merged_cloud->points[i].z);
       out_triPoints.push_back(tmp_pt);
    }

   // PLY output
   if( true ){
      printf("saving ply file..\n");

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
         //myfile << "property uchar red\n";
         //myfile << "property uchar green\n";
         //myfile << "property uchar blue\n";
         myfile << "end_header\n";
         
         // write binary data
         for(int i=0;i<out_triPoints.size();i++){
            myfile.write( (char*)(&out_triPoints[i]), sizeof(float)*3 );
            //myfile.write( (char*)(&out_rgbPoints[i]), sizeof(uchar)*3 );
         }
      }
      
   }
   
   return 0;
}

