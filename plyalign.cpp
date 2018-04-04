// plytrim
// trim down ply files using bounding box
// by John De Witt

#include <Eigen/Dense>

#include <opencv2/opencv.hpp>
//#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
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
#include <pcl/common/transforms.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>


using namespace cv;
using namespace std;
//using namespace Imf;
//using namespace Imath;

#define DBUG 0
#define THRESHOLD_VALUE 5

#define SUBSAMPLE_GRID 0
#define SUBSAMPLE_POINT 1

// Types
typedef pcl::PointXYZ PointT;
typedef pcl::PointXYZRGB PointCT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointXYZRGBNormal PointCNT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointCT> PointCloudCT;
typedef pcl::PointCloud<PointNT> PointCloudNT;
typedef pcl::PointCloud<PointCNT> PointCloudCNT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
//typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;

static void help()
{
   printf("This program accepts two or more ply point cloud files and outputs a combined co-aligned point cloud.\n"
          "  Notable options include:\n"
          "   - Bounding box inclusion\n"

          //"ex: ply_trim -bb 0 0 0 1 1 1 model_01.ply\n"
          "ex: plyalign model_01.ply model_02.ply\n"
          "Usage: plyalign\n"
          "    input_data            # XYZ RGB PLY file\n"
          "\n");
}

// align B to A
//double AlignRANSAC(string& fname_ref, string& fname_dat)
// cloud_ref is reference
// cloud_dat is aligned to reference
// final_transform maps data to reference
double AlignRANSAC (const PointCloudT::Ptr cloud_ref, const PointCloudT::Ptr cloud_dat, PointCloudT::Ptr cloud_output, Eigen::Matrix4f &final_transform, bool downsample = false, float leafscl = 1.0f, float inlier_t = 10.0f)
{
    PointCloudT::Ptr ref (new PointCloudT);
    PointCloudT::Ptr dat (new PointCloudT);
    PointCloudNT::Ptr ref_n (new PointCloudNT);
    PointCloudNT::Ptr dat_n (new PointCloudNT);

    PointCloudNT::Ptr registration_res (new PointCloudNT);
    
    if(downsample)
    {
        //pcl::copyPointCloud(*cloud_ref, *ref);
        //pcl::copyPointCloud(*cloud_dat, *dat);
        
        pcl::console::print_highlight ("Downsampling...\n");
        pcl::VoxelGrid<PointT> grid;
        grid.setLeafSize (leafscl, leafscl, leafscl);
        
        grid.setInputCloud (cloud_ref);
        grid.filter (*ref);
        grid.setInputCloud (cloud_dat);
        grid.filter (*dat);
                
        // convert xyz to xyznorm
        for (size_t i = 0; i < ref->points.size (); ++i)
        {
            PointNT tmp_nt;
            tmp_nt.x = ref->points[i].x;
            tmp_nt.y = ref->points[i].y;
            tmp_nt.z = ref->points[i].z;
            ref_n->push_back(tmp_nt);
        }
        for (size_t i = 0; i < dat->points.size (); ++i)
        {
            PointNT tmp_nt;
            tmp_nt.x = dat->points[i].x;
            tmp_nt.y = dat->points[i].y;
            tmp_nt.z = dat->points[i].z;
            dat_n->push_back(tmp_nt);
        }

    }
    else
    {
        pcl::copyPointCloud(*cloud_ref, *ref);
        pcl::copyPointCloud(*cloud_dat, *dat);
        //ref = cloud_ref;
        //dat = cloud_dat;
        
        // convert xyz to xyznorm
        for (size_t i = 0; i < ref->points.size (); ++i)
        {
            PointNT tmp_nt;
            tmp_nt.x = ref->points[i].x;
            tmp_nt.y = ref->points[i].y;
            tmp_nt.z = ref->points[i].z;
            ref_n->push_back(tmp_nt);
        }
        for (size_t i = 0; i < dat->points.size (); ++i)
        {
            PointNT tmp_nt;
            tmp_nt.x = dat->points[i].x;
            tmp_nt.y = dat->points[i].y;
            tmp_nt.z = dat->points[i].z;
            dat_n->push_back(tmp_nt);
        }

    }
    
    cout << "scene : "  << ref->width * ref->height << endl;
    cout << "object : " << dat->width * dat->height << endl;
    
    // Estimate normals for scene and object
    //pcl::NormalEstimationOMP<PointNT,PointNT> nest;
    //nest.setRadiusSearch (10);

    pcl::NormalEstimation<PointT, PointNT> nest;
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
    nest.setSearchMethod (tree);
    nest.setKSearch (11);

    pcl::console::print_highlight ("Estimating ref normals...\n");
    nest.setInputCloud (ref);
    nest.compute (*ref_n);
    pcl::console::print_highlight ("Estimating dat normals...\n");
    nest.setInputCloud (dat);
    nest.compute (*dat_n);
    
    // Estimate features for scene and object
    FeatureCloudT::Ptr ref_features (new FeatureCloudT);
    FeatureCloudT::Ptr dat_features (new FeatureCloudT);
    pcl::console::print_highlight ("Estimating features...\n");
    FeatureEstimationT fest;
    fest.setRadiusSearch (leafscl*10);
    
    fest.setInputCloud (ref_n);
    fest.setInputNormals (ref_n);
    fest.compute (*ref_features);

    fest.setInputCloud (dat_n);
    fest.setInputNormals (dat_n);
    fest.compute (*dat_features);

    // Perform alignment
    pcl::console::print_highlight ("Starting alignment...\n");
    pcl::SampleConsensusPrerejective<PointNT,PointNT,FeatureT> align;
    align.setInputSource (dat_n);
    align.setSourceFeatures (dat_features);
    align.setInputTarget (ref_n);
    align.setTargetFeatures (ref_features);
    align.setMaximumIterations (10000);    // Number of RANSAC iterations
    align.setNumberOfSamples (11);         // Number of points to sample for generating/prerejecting a pose
    align.setEuclideanFitnessEpsilon(25.0f);
    align.setCorrespondenceRandomness (3); // Number of nearest features to use
    align.setSimilarityThreshold (0.9f);  // Polygonal edge length similarity threshold
    align.setInlierFraction (0.8f);       // Required inlier fraction for accepting a pose hypothesis
    align.setRANSACOutlierRejectionThreshold(30);
    align.setMaxCorrespondenceDistance (inlier_t); //10mm 2.5f * leaf); // Inlier threshold
    
    {
        pcl::ScopeTime t("Alignment");
        align.align (*registration_res);
    }

    if (align.hasConverged ())
    {
        // Print results
        printf ("\n");
        Eigen::Matrix4f t = align.getFinalTransformation ();
        final_transform = t;
        Eigen::Matrix4f final_inverse = final_transform.inverse();
        pcl::transformPointCloud(*cloud_dat, *cloud_output, final_inverse);
        
        pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", t (0,0), t (0,1), t (0,2));
        pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", t (1,0), t (1,1), t (1,2));
        pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", t (2,0), t (2,1), t (2,2));
        pcl::console::print_info ("\n");
        pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", t (0,3), t (1,3), t (2,3));
        pcl::console::print_info ("\n");
        pcl::console::print_info ("Inliers: (%0.2f%%) %i/%i\n", 100*align.getInliers().size()/ (float)dat->size(), align.getInliers ().size (), dat->size ());
        pcl::console::print_info ("Final score: %f\n", align.getFitnessScore() );
        
        cout << "-------------------------------" << endl;
        // Show alignment
        //pcl::visualization::PCLVisualizer visu("Alignment");
        //visu.addPointCloud (scene, ColorHandlerT (scene, 0.0, 255.0, 0.0), "scene");
        //visu.addPointCloud (object_aligned, ColorHandlerT (object_aligned, 0.0, 0.0, 255.0), "object_aligned");
        //visu.spin ();
        return align.getFitnessScore();
    }
    else
    {
        pcl::console::print_error ("Alignment failed!\n");
        return (-1);
    }
}

// align B to A
//double AlignICP(string& fname_ref, string& fname_dat, Eigen::Matrix4f& final_transform)
double AlignICP(const PointCloudT::Ptr cloud_ref, const PointCloudT::Ptr cloud_dat, PointCloudNT::Ptr cloud_output, Eigen::Matrix4f &final_transform, bool downsample = false, float leafscl = 3.0f, float inlier_t = 10.0f)
{
    PointCloudT::Ptr ref (new PointCloudT());
    PointCloudT::Ptr dat (new PointCloudT());
    PointCloudNT::Ptr ref_n (new PointCloudNT());
    PointCloudNT::Ptr dat_n (new PointCloudNT());

    if(downsample)
    {
        //pcl::copyPointCloud(*cloud_ref, *ref);
        //pcl::copyPointCloud(*cloud_dat, *dat);
        
        pcl::console::print_highlight ("Downsampling...\n");
        pcl::VoxelGrid<PointT> grid;
        grid.setLeafSize (leafscl, leafscl, leafscl);
        
        grid.setInputCloud (cloud_ref);
        grid.filter (*ref);
        grid.setInputCloud (cloud_dat);
        grid.filter (*dat);
                
        pcl::copyPointCloud(*ref, *ref_n);
        pcl::copyPointCloud(*dat, *dat_n);
        
    }
    else
    {
        //ref = cloud_ref;
        //dat = cloud_dat;
        
        dat->reserve(cloud_ref->points.size());
        ref->reserve(cloud_dat->points.size());
        dat_n->reserve(cloud_ref->points.size());
        ref_n->reserve(cloud_dat->points.size());
        
        pcl::copyPointCloud(*cloud_ref, *ref);
        pcl::copyPointCloud(*cloud_dat, *dat);
        pcl::copyPointCloud(*cloud_ref, *ref_n);
        pcl::copyPointCloud(*cloud_dat, *dat_n);
        
    }

    pcl::NormalEstimation<PointT, PointNT> nest;
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
    nest.setSearchMethod (tree);
    nest.setKSearch (11);

    pcl::console::print_highlight ("Estimating ref normals...\n");
    nest.setInputCloud (ref);
    nest.compute (*ref_n);
    pcl::console::print_highlight ("Estimating dat normals...\n");
    nest.setInputCloud (dat);
    nest.compute (*dat_n);


    //pcl::registration::DefaultConvergenceCriteria<Scalar>::setRotationThreshold(1e-9);
    
    //pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    pcl::IterativeClosestPointWithNormals<PointNT, PointNT> icp;
    
    icp.setInputTarget(ref_n);
    icp.setInputSource(dat_n);
    
    icp.setRANSACIterations(150);
    icp.setMaximumIterations(1000);
    icp.setTransformationEpsilon(1e-3);
    icp.setEuclideanFitnessEpsilon(5e-3);
    
    icp.setMaxCorrespondenceDistance(inlier_t);
    icp.setRANSACOutlierRejectionThreshold(10);
    
    // pcl::registration::CorrespondenceRejector::CorrespondenceRejector::Ptr rej_def(new pcl::registration::CorrespondenceRejectorTrimmed);
    // pcl::registration::CorrespondenceRejectorTrimmed CRT;

    //CRT.setOverlapRatio(0.9);
    //cout << "CRT.getOverlapRatio " << CRT.getOverlapRatio() << endl;
    //icp.clearCorrespondenceRejectors();
    //icp.addCorrespondenceRejector(CRT);

    //pcl::PointCloud<pcl::PointXYZ> Final;
    //PointCloudT Final;
    PointCloudNT Final;
    icp.align(Final);
    printf("converged: %d score: >>>>>>> %0.1e (%0.6f) <<<<<<<\n",icp.hasConverged(), icp.getFitnessScore(), icp.getFitnessScore() );
    //std::cout << "has converged:" << icp.hasConverged() << " score: >>>>>>> " << icp.getFitnessScore() << " <<<<<<<" << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;
    
    Eigen::Matrix4f transformation = icp.getFinalTransformation();
    final_transform = transformation;
    Eigen::Matrix4f final_inverse = final_transform.inverse();

    Mat rotmat44,rotvec;
    eigen2cv(transformation, rotmat44);
    Mat rotmat33(rotmat44,Rect(0,0,3,3));
    cout << "ROWS = " << rotmat33.rows << endl;
    cv::Rodrigues(rotmat33,rotvec);
    cout << "ROTVEC: " << rotvec << endl;
   
    Mat mtxR,mtxQ;
    cv::Vec3d rotvec3d = cv::RQDecomp3x3(rotmat33,mtxR,mtxQ);
    cout << "ROTVEC3D: " << rotvec3d << endl;
    cout << "-------------------------------" << endl;

    if(icp.hasConverged())
    {
        pcl::transformPointCloudWithNormals(*ref_n, *cloud_output, final_inverse);
        return icp.getFitnessScore();
    }
    else
    {
        return -1;
    }
    //icp.setUseReciprocalCorrespondences(true);
    //pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Ptr\
        trans_svd (new pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>);
    //icp.setTransformationEstimation(trans_svd);
}

// ASSUMPTION: scans are captured by static camera w/ object on turntable
// ASSUMPTION: successive rotations are roughly the same distance
int main( int argc, char** argv ) {
   
   if(argc<3)
   {
      printf("not enough arguments, exiting..\n");
      return -1;
   }
   
   bool use_downscale = false;
   vector<string> fnames;
   vector<PointCloudT::Ptr> clouds;
   vector<float> pair_scores_ransac;
   vector<float> pair_scores_icp_rough;
   vector<float> pair_scores_icp_fine;
   vector<Eigen::Matrix4f> pair_transforms;
   PointCloudT::Ptr aligned_cloud (new PointCloudT);
   Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), PairTransform;
   PointCloudNT::Ptr cloud_union (new PointCloudNT);
   string fname_out = "aligned_cloud.ply";

   // point lists
   vector< Vec3f > in_triPoints;
   vector< Vec3b > in_rgbPoints;

   vector< Vec3f > out_triPoints;
   vector< Vec3b > out_rgbPoints;
   
   float leaf_scl = 1.0;

   // parse args
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
        else if( strcmp(s,"-d") == 0 )
        {
            use_downscale = true;
            printf("enabled downscaling\n");
        }
        else
        {
            string fn = s;
            fnames.push_back(fn);
            printf("argv[%d] : %s\n",i,fn.c_str());
        }
   }
   
   // load all pointclouds
   for (std::vector<string>::iterator it = fnames.begin() ; it != fnames.end(); ++it)
   {
       PointCloudT::Ptr tmp_cloud (new PointCloudT);
       if (pcl::io::loadPLYFile<PointT>(*it, *tmp_cloud) == -1) 
       {
           cout << "\tError loading [" << *it << "]" << endl;
           return -1;
       }
       else
       {
           clouds.push_back(tmp_cloud);
           //if(clouds.size()>0)
           {
                Eigen::Matrix4f tmp_mat;
                pair_transforms.push_back(tmp_mat);
                pair_scores_ransac.push_back(0);
                pair_scores_icp_rough.push_back(0);
                pair_scores_icp_fine.push_back(0);
           }
           printf("loaded [%s].. %8d points\n",it->c_str(),(int)tmp_cloud->points.size());
           /*
           for (size_t i = 0; i < tmp_cloud->points.size(); ++i)
           {
               PointT tmp_pt(tmp_cloud->points[i]);
               aligned_cloud->push_back(tmp_pt);
               if(i<0)
               std::cout << "    " << tmp_cloud->points[i].x
                         << " "    << tmp_cloud->points[i].y
                         << " "    << tmp_cloud->points[i].z << std::endl;
           }
           */
       }
   }

   // pairwise align all clouds to first one (2->1) (3->2->1) (4->3->2->1)
   for(int i=1; i<clouds.size(); i++)
   {   
       Eigen::Matrix4f t_pair,t_ransac,t_icp_rough,t_icp_fine;
       PointCloudT::Ptr cloud_ransac_rough (new PointCloudT);
       PointCloudT::Ptr cloud_ransac_fine (new PointCloudT);
       PointCloudT::Ptr cloud_icp (new PointCloudT);
       
       PointCloudT::Ptr ref (new PointCloudT);
       ref->reserve(clouds[i]->points.size());
       PointCloudT::Ptr dat (new PointCloudT);
       dat->reserve(clouds[i-1]->points.size());

       for(size_t j=0; j<clouds[i]->points.size(); ++j)
       {
            PointT tmp_t;
            tmp_t.x = clouds[i]->points[j].x;
            tmp_t.y = clouds[i]->points[j].y;
            tmp_t.z = clouds[i]->points[j].z;
            ref->push_back(tmp_t);
       }
       
       for(size_t j=0; j<clouds[i-1]->points.size(); ++j)
       {
            PointT tmp_t;
            tmp_t.x = clouds[i-1]->points[j].x;
            tmp_t.y = clouds[i-1]->points[j].y;
            tmp_t.z = clouds[i-1]->points[j].z;
            dat->push_back(tmp_t);
       }
       //pcl::copyPointCloud(clouds[i-1],ref);
       
       //pcl::copyPointCloud(clouds[i],dat);
        
        //pcl::transformPointCloud(*clouds[i], *ref, GlobalTransform);
        //pcl::transformPointCloud(*clouds[i-1],   *dat, GlobalTransform);
       
       double err = AlignRANSAC (ref, dat, cloud_ransac_rough, t_ransac, use_downscale, leaf_scl, 7.5f); 
       pair_scores_ransac[i] = err;

       if( err >= 0 )
       {
           pair_transforms[i] = t_ransac;
           PointCloudT::Ptr  cloud_ransac (new PointCloudT);
           cloud_ransac->reserve(clouds[i]->points.size());
           
           // transform full cloud based on downscaled alignment
           for (size_t j = 0; j < dat->points.size(); ++j)
            {
                float x,y,z;
                x = dat->points[j].x;
                y = dat->points[j].y;
                z = dat->points[j].z;
                Eigen::Vector4f ptin(x,y,z,1);
                Eigen::Vector4f ptout = t_ransac * ptin;
                if(DBUG)
                {
                    cout << "XYZ " << x << ", " << y << ", " << z << endl;
                    cout << "ptin : " <<  ptin << endl;
                    cout << "ptout : " <<  ptout << endl;
                    cout << "t_ransac : " << t_ransac << endl;
                }
                
                PointT tmp_t;
                tmp_t.x = ptout[0]/ptout[3];
                tmp_t.y = ptout[1]/ptout[3];
                tmp_t.z = ptout[2]/ptout[3];
                cloud_ransac->push_back(tmp_t);

                if(DBUG)
                    printf("clouds[%d][%d] = (%0.3f, %0.3f, %0.3f)\n",(int)i,(int)j,tmp_t.x,tmp_t.y,tmp_t.z);
            }
           

            //for (size_t j = 0; j < cloud_ransac->points.size(); ++j)
            if(DBUG)
            for (size_t j = 0; j < 30; ++j)
            {
                float x,y,z;
                x = dat->points[j].x;
                y = dat->points[j].y;
                z = dat->points[j].z;
                printf("clouds[%d][%d] = (%0.3f, %0.3f, %0.3f)\n",(int)i,(int)j,x,y,z);
            }
            if(DBUG)
            for (size_t j = 0; j < 30; ++j)
            {
                float x,y,z;
                x = cloud_ransac->points[j].x;
                y = cloud_ransac->points[j].y;
                z = cloud_ransac->points[j].z;
                printf("cloud_ransac[%d] = (%0.3f, %0.3f, %0.3f)\n",(int)j,x,y,z);
            }

           PointCloudNT::Ptr cloud_aligned_rough  (new PointCloudNT());
           PointCloudNT::Ptr cloud_aligned_fine   (new PointCloudNT());
           PointCloudNT::Ptr cloud_aligned_global (new PointCloudNT());
           double erricp_rough = AlignICP(ref, cloud_ransac, cloud_aligned_rough, t_icp_rough, false, 1, 0.15f);
           pair_scores_icp_rough[i] = erricp_rough;
           
           if( erricp_rough >= 0 )
           {
               PointCloudT::Ptr  cloud_icp_rough (new PointCloudT);
               cloud_icp_rough->reserve(cloud_ransac->points.size());
           
               // transform full cloud based on downscaled alignment
               for (size_t j = 0; j < cloud_ransac->points.size(); ++j)
               {
                   float x,y,z;
                    x = cloud_ransac->points[j].x;
                    y = cloud_ransac->points[j].y;
                    z = cloud_ransac->points[j].z;
                    Eigen::Vector4f ptin(x,y,z,1);
                    Eigen::Vector4f ptout = t_icp_rough * ptin;
                    
                    PointT tmp_t;
                    tmp_t.x = ptout[0]/ptout[3];
                    tmp_t.y = ptout[1]/ptout[3];
                    tmp_t.z = ptout[2]/ptout[3];
                    cloud_icp_rough->push_back(tmp_t);
                }


               double erricp_fine = AlignICP(ref, cloud_icp_rough, cloud_aligned_fine, t_icp_fine, false, 1, 0.15f);
               pair_scores_icp_fine[i] = erricp_fine;
               
               if( erricp_fine < 0.75f )
               {
                   pair_transforms[i] = t_ransac * t_icp_rough * t_icp_fine;
                   GlobalTransform = GlobalTransform * pair_transforms[i];
                   pcl::transformPointCloudWithNormals(*cloud_aligned_fine,*cloud_aligned_global,GlobalTransform.inverse());
                   for( size_t j=0; j<cloud_aligned_global->points.size(); ++j)
                   {   
                       PointNT tmp_nt1 = cloud_aligned_global->points[j];
                       cloud_union->push_back(tmp_nt1);
                   }
               }

           }
           
           if(false && i==clouds.size()-1)
           {
               PointCloudNT::Ptr ref_n (new PointCloudNT());
               
               pcl::NormalEstimation<PointT, PointNT> nest;
               pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
               nest.setSearchMethod (tree);
               nest.setKSearch (11);
               
               pcl::console::print_highlight ("Estimating ref normals...\n");
               nest.setInputCloud (ref);
               nest.compute (*ref_n);
               pcl::transformPointCloudWithNormals(*ref_n,*ref_n,GlobalTransform);

               for( size_t j=0; j<ref_n->points.size(); ++j)
               {   
                   PointNT tmp_nt2 = ref_n->points[j];
                   cloud_union->push_back(tmp_nt2);
               }
           }

       }
       
   }
   
   printf("pair_scores_ransac.size = %d\n",(int)pair_scores_ransac.size());

   // RANSAC stats
   float RAN_mean=0, RAN_stddev=0;
   int RAN_N=pair_scores_ransac.size();
   for( size_t i=0; i<RAN_N; i++)
   {
        RAN_mean = RAN_mean + pair_scores_ransac[i];
   }
   RAN_mean /= RAN_N;
   for( size_t i=0; i<RAN_N; i++)
   {
        RAN_stddev += pow(pair_scores_ransac[i]-RAN_mean,2.0f);
   }
   RAN_stddev /= RAN_N;
   RAN_stddev = pow(RAN_stddev,0.5f);

   // ICP stats rough
   float ICP_meanR=0, ICP_stddevR=0;
   int ICP_NR=pair_scores_ransac.size();
   for( size_t i=0; i<ICP_NR; i++)
   {
        ICP_meanR += pair_scores_icp_rough[i];
   }
   ICP_meanR /= ICP_NR;
   for( size_t i=0; i<ICP_NR; i++)
   {
        ICP_stddevR += pow(pair_scores_icp_rough[i]-ICP_meanR,2.0f);
   }
   ICP_stddevR /= ICP_NR;
   ICP_stddevR = pow(ICP_stddevR,0.5f);
   
   // ICP stats fine
   float ICP_meanF=0, ICP_stddevF=0;
   int ICP_NF=pair_scores_ransac.size();
   for( size_t i=0; i<ICP_NF; i++)
   {
        ICP_meanF += pair_scores_icp_fine[i];
   }
   ICP_meanF /= ICP_NF;
   for( size_t i=0; i<ICP_NF; i++)
   {
        ICP_stddevF += pow(pair_scores_icp_fine[i]-ICP_meanF,2.0f);
   }
   ICP_stddevF /= ICP_NF;
   ICP_stddevF = pow(ICP_stddevF,0.5f);

   
   // clipped ICP stats
   float ICP_meanC=0, ICP_stddevC=0;
   int ICP_NC=0;
   for( size_t i=0; i<pair_scores_icp_fine.size(); i++)
   {
       if(pair_scores_icp_fine[i]<1.0f)
       {
        ICP_NC++;
        ICP_meanC += pair_scores_icp_fine[i];
       }
   }
   ICP_meanC /= ICP_NC;
   for( size_t i=0; i<pair_scores_icp_fine.size(); i++)
   {
       if(pair_scores_icp_fine[i]<1.0f)
        ICP_stddevC += pow(pair_scores_icp_fine[i]-ICP_meanC,2.0f);
   }
   ICP_stddevC /= ICP_NC;
   ICP_stddevC = pow(ICP_stddevC,0.5f);


   printf("     RANSAC[%d] mean (%0.5f), stddev (%0.5f)\n", RAN_N, RAN_mean,  RAN_stddev);
   printf("  rough ICP[%d] mean (%0.5f), stddev (%0.5f)\n", ICP_NR, ICP_meanR,  ICP_stddevR);
   printf("   fine ICP[%d] mean (%0.5f), stddev (%0.5f)\n", ICP_NF, ICP_meanF,  ICP_stddevF);
   printf("clipped ICP[%d] mean (%0.5f), stddev (%0.5f)\n", ICP_NC, ICP_meanC, ICP_stddevC);

   for( size_t i=0; i<pair_scores_ransac.size(); i++)
   {
        printf("    [%2d] : %2.4f -> %2.4f -> %2.4f (%0.1f, %0.3f x better)\n",(int)i,pair_scores_ransac[i],pair_scores_icp_rough[i],pair_scores_icp_fine[i]
                ,pair_scores_ransac[i]/pair_scores_icp_rough[i],pair_scores_icp_rough[i]/pair_scores_icp_fine[i]);
   }
   printf("pair_scores_icp_rough.size = %d\n",(int)pair_scores_icp_rough.size());
   printf("pair_scores_icp_fine.size = %d\n",(int)pair_scores_icp_fine.size());
   
    pcl::PLYWriter pw;
    pw.write("cloud_union_OUT.ply", *cloud_union, false);

   // AlignRANSAC(fname_ref,fname_dat);


   //printf("sizeof(float) = %d\n",(int)sizeof(float));
   //printf("sizeof(uchar) = %d\n",(int)sizeof(uchar));

   //AlignICP(fname_ref,fname_dat);
   
   /*
   string fname_out = "trimmed_"+fname;
   
   // point lists
   vector< Vec3f > in_triPoints;
   vector< Vec3b > in_rgbPoints;

   vector< Vec3f > out_triPoints;
   vector< Vec3b > out_rgbPoints;
   
   // bounding box
//   Vec3f bb_cen(256,19,-442);
//   Vec3f bb_dim(62,51,64);
//   
   Vec3f bb_cen(44,0,-59);
   Vec3f bb_dim(17,17,17);
   
   
   ifstream myfile (fname.c_str(), ios::in | ios::binary);
   if (myfile.is_open())
   {
      bool alpha = false;
      bool onbinary = false;
      bool done = false;
      char buf[15]; // 3x4 bytes floats (xyz), 3x1 bytes uchar (rgb)
      string line;

      int i=0;
      int ib = -1;
      
      while( !done && !myfile.eof() )
      {
         if( !onbinary )
         {
            getline (myfile,line);
            if( line.find("ply") != string::npos && i!=0 )
               done = true;
            if( line.find("format") != string::npos && i!=1 )
               done = true;
            if( line.find("element vertex") != string::npos )
            {
               int d=0;
               sscanf(line.c_str(),"element vertex %d",&d);
               printf("VERTEX COUNT = [%d]\n",d);
            }
            if( line.find("property uchar alpha") != string::npos )
            {
               alpha = true;
            }
            if( line.find("end_header") != string::npos )
            {
               ib = i;
               onbinary = true;
            }
            cout << "[" << i << "] :" <<  line << endl;
         }
         else
         {
            float xyz[3];
            uchar rgb[3];
            uchar a;
            
            myfile.read((char*)&(xyz[0]),sizeof(float));
            myfile.read((char*)&(xyz[1]),sizeof(float));
            myfile.read((char*)&(xyz[2]),sizeof(float));

            myfile.read((char*)&rgb[0],sizeof(char));
            myfile.read((char*)&rgb[1],sizeof(char));
            myfile.read((char*)&rgb[2],sizeof(char));
            if(alpha)
               myfile.read((char*)&a,sizeof(char));

            in_triPoints.push_back(Vec3f(xyz[0],xyz[1],xyz[2]));
            in_rgbPoints.push_back(Vec3b(rgb[0],rgb[1],rgb[2]));
            
         }
         
         
//         if(onbinary && ib>=0 && i>ib+5)
//            done = true;
         
//         if(done)
//            printf("stopping..\n");
         i++;
      }
      myfile.close();
   }
   
   for(int i=0; i<in_triPoints.size(); i++)
   {
      Vec3f pos = in_triPoints[i];
      Vec3b col = in_rgbPoints[i];
      
      bool add = true;
      for(int k=0;k<3;k++)
      {
         if(pos[k]<bb_cen[k]-bb_dim[k]*0.5 || pos[k]>bb_cen[k]+bb_dim[k]*0.5)
            add = false;
      }
      if(add)
      {
         out_triPoints.push_back(Vec3f(pos[0],pos[1],pos[2]));
         out_rgbPoints.push_back(Vec3b(col[0],col[1],col[2]));
      }
   }
   
   
   printf("in_size  = %d\n",(int)in_triPoints.size());
   printf("out_size = %d\n",(int)out_triPoints.size());
   cout << fname_out << endl;
   
//   for( int i=0; i<out_triPoints.size(); i++)
   printf("point sample:\n");
   for( int i=0; i<6; i++)
   {
      Vec3f pos = in_triPoints[i];
      Vec3b col = in_rgbPoints[i];
      printf("x,y,z = (%3.3f, %3.3f, %3.3f)\n",pos[0],pos[1],pos[2]);
      printf("r,g,b = (%d, %d, %d)\n",col[0],col[1],col[2]);
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
   */
   return 0;
}

