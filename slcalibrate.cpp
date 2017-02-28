// slcalibrate.cpp
// calculate camera and projector-camera calibration from input points
// by John De Witt

#include <iostream>
#include <vector>
#include <string>

#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "util.h"

using namespace std;
using namespace cv;

static void help()
{
   printf("This program calibrates a camera using the image-object point\n"
          "correspondences from a structured light scan, such as from slcrunch\n"
          "Usage: slcalibrate [option]... input_data\n"
          "Options:\n"
          "   [--help]              # shows this prompt as well as information about the input file format\n"
          "   [-input <calib_file>] # specify an existing calibration file to use as a starting point\n"
          "                         # use multiple -input <arg> to use separate cam/projector calibration data\n"
          "   [-prosize <NxM>]      # specify the projector space resolution used\n"
          "   [-chess]              # indicates input files contain chess corner data\n"
          "   [-rational]           # enable rational lens distortion model (use for fisheye)\n"
          "   [-zerotan]\n"
          "   [-f]                  # only try to find the fundamental matrix (cam-pro)\n"
          "   [-s <squareSize>]     # square size in some user-defined units (1 by default)\n"
          "                         # this value might be equal to pixel pitch of the computer monitor\n"
          "   [-o <out_cam_params>] # the output filename for the intrinsic [and extrinsic] parameters\n"
          "   input_data            # text file with a list of the scans (e.g. of a computer monitor)\n"
          "                         # the text file can be generated with imagelist_creator\n"
          "                         # must be xml/yaml opencv-compatible format; use --help for more info\n"
          "   [-camguess]           # use input camera calibration as guess for further optimization\n"
          "   [-proguess]           # use input projector calibration as guess for further optimization\n"
          "   [-nocam]              # do not load camera calibration from input file\n"
          "   [-nopro]              # do not load projector calibration from input file\n"
          "\n"
          "example command line of calibration from a list of stored image-object correspondence files:\n"
          "   imagelist_creator scanlist.yaml slscan_*.yaml\n"
          "   slcalibrate scanlist.yaml\n"
          "   slcalibrate -chess -prosize 1280x800 scanlist.yaml\n"
          );
}

static void help_inputfile()
{
   printf("\n"
          "Input files from the list are expected to contain the size of the image\n"
          "used to capture the structured light patterns as well as the\n"
          "list of corresponding points in the form of two matrices:\n"
          "one containing xy image points, and one containing xyz object points.\n"
          "*See OpenCV calibrateCamera documentation for more details*\n"
          "\n"
          "The data is expected in this form: image_size (Size2i)\n"
          "                                   image_points (Mat-2f)\n"
          "                                   object_points (Mat-3f)\n"
          );
}

static bool readStringList( const string& filename, vector<string>& l )
{
   l.resize(0);
   FileStorage fs(filename, FileStorage::READ);
   if( !fs.isOpened() )
      return false;
   FileNode n = fs.getFirstTopLevelNode();
   if( n.type() != FileNode::SEQ )
      return false;
   FileNodeIterator it = n.begin(), it_end = n.end();
   for( ; it != it_end; ++it )
      l.push_back((string)*it);
   return true;
}

int main( int argc, char** argv ){
   
   int n = 0;
   
   float squareSize = 1.f;
   float pixSize = -1.f;
   const char* outCalibFN = findNextName( "calibration_%04d.yaml", &n );
   const char* inCalibFN = 0;
   bool load_calib = false;
   bool enable_prosize_input = false;
   bool enable_rational_model = false;
   bool enable_chess = false;
   bool enable_stereo = false;
   bool enable_zerotan = false;
   bool only_fundamental = false;

   bool use_cam_guess = false;
   bool use_pro_guess = false;
   bool no_cam_load = false;
   bool no_pro_load = false;

   Size2i base_cam_size;
   Size2i base_pro_size;
   Size2i pro_size_input;
   vector< string > calibList;
   vector< string > scanList;
   vector< string > fileList;
   
   // exit if no args
   if (argc < 2)
   {
      help();
      return 0;
   }
   
   // parse arguments
   for(int i=1; i<argc; i++)
   {
      const char* s = argv[i];
      if( strcmp(s,"--help") == 0 )
      {
         help();
         help_inputfile();
         return 1;
      }
      else if( strcmp(s,"-s") == 0 )
      {
         if( sscanf( argv[++i], "%f", &squareSize ) != 1 || squareSize <= 0 )
            return fprintf( stderr, "Invalid pattern square width\n" ), -1;
      }
      else if( strcmp(s,"-rational") == 0 )
      {
         enable_rational_model = true;
      }
      else if( strcmp(s,"-camguess") == 0 )
      {
         use_cam_guess = true;
      }
      else if( strcmp(s,"-proguess") == 0 )
      {
         use_pro_guess = true;
      }
      else if( strcmp(s,"-nocam") == 0 )
      {
         no_cam_load = true;
      }
      else if( strcmp(s,"-nopro") == 0 )
      {
         no_pro_load = true;
      }

      else if( strcmp(s,"-pixscl") == 0 )
      {
         if( sscanf( argv[++i], "%f", &pixSize ) != 1 || pixSize <= 0 )
            return fprintf( stderr, "Invalid pixel size\n" ), -1;
      }
      else if( strcmp(s,"-chess") == 0 )
      {
         enable_chess = true;
         enable_stereo = true;
      }
      else if( strcmp(s,"-f") == 0 )
      {
         only_fundamental = true;
      }
      else if( strcmp(s,"-nostereo") == 0 )
      {
         enable_stereo = false;
      }
      else if( strcmp(s,"-zerotan") == 0 )
      {
         enable_zerotan = true;
      }
      else if( strcmp(s,"-input") == 0 )
      {
         if(i+1<argc){
            load_calib = true;
            calibList.push_back( string(argv[++i]) );
         } else {
            printf("error: no saved calibration file specified\n");
         }
         //            if(i+1<argc){
         //                load_calib = true;
         //                inCalibFN = argv[++i];
         //            } else {
         //                printf("error: no saved calibration file specified\n");
         //            }
      }
      else if( strcmp(s,"-prosize") == 0 )
      {
         if(i+1<argc){
            int w,h;
            if( sscanf( argv[++i], "%dx%d", &w, &h) != 2 || w<1 || h<1 )
               return fprintf( stderr, "Invalid chessboard size\n"), -1;
            enable_prosize_input = true;
            pro_size_input.width = w;
            pro_size_input.height = h;
            base_pro_size = pro_size_input;
            printf("projector size : %d x %d\n",w,h);
         } else {
            return fprintf( stderr, "Invalid projector size input\n" ), -1;
         }
      }
      else if( strcmp( s, "-o" ) == 0 )
      {
         outCalibFN = argv[++i];
      }
      else if( s[0] != '-' )
      {
         fileList.push_back( string(s) );
      }
      else
         return fprintf( stderr, "Unknown option %s", s ), -1;
   }
   // ----------------------------------------------------------------------------
   // fileList -> scanList
   // INPUT : concatenate input correspondence files
   if( fileList.size() > 0 )
   {
      scanList.clear();
      for(int i=0; i<fileList.size(); i++)
      {
         // concatenate any string lists
         vector<string> fvec;
         if( readStringList(fileList[i],fvec) && fvec.size()>0 )
         {
            for(int j=0; j<fvec.size(); j++)
            {
               scanList.push_back(fvec[j]);
            }
         }
         else
         {
            scanList.push_back(fileList[i]);
         }
         
      }
   } else {
      return fprintf( stderr, "No input files specified\n"), -1;
   }
   
   printf("Output file : [%s]\n", outCalibFN);
   
   // camera<->output correspondence data
   vector< vector<Point2f> > cam_point_list;
   vector< vector<Point3f> > pro_point_list;
   
   // chessboard corner correspondence data
   vector< vector< Point2f > > camera_corners;
   vector< vector< Point2f > > projector_corners;
   vector< vector< Point3f > > object_corners;

   // concatenated correspondence data
   vector< Point2f > vec_cam_matches;
   vector< Point2f > vec_obj_matches2d;
   vector< Point3f > vec_obj_matches;

   // load points from each file
   // scanList
   for (int i=0; i<scanList.size(); i++)
   {
      // PER FILE
      // printf("loading file [%d]\n",i);
      Mat obj_corners, cam_corners, pro_corners;
      Mat obj_points, cam_points;
      Mat cam_dims;// = Mat::zeros(2,1,CV_32S);
      Mat pro_dims;// = Mat::zeros(2,1,CV_32S);
      String cam_make="", cam_model="", cam_lens="", cam_serial="", cam_intserial="";
      
      vector< Point2f > tmp_cam_matches;
      vector< Point2f > tmp_obj_matches2d;
      vector< Point3f > tmp_obj_matches;

      // reading file
      {
         FileStorage fs;
         fs.open(scanList[i], FileStorage::READ);
         
         if (!fs.isOpened())
            return fprintf(stderr,"failed to open [%s]",scanList[i].c_str()), -1;
         
         fs["pro_size"] >> pro_dims;
         fs["cam_size"] >> cam_dims;
         
         fs["cam_make"] >> cam_make;
         fs["cam_model"] >> cam_model;
         fs["cam_lens_str"] >> cam_lens;
         fs["cam_serial"] >> cam_serial;
         fs["cam_intserial"] >> cam_intserial;
         
         // printf("loaded size,make,model,lens_str,pro_size\n");
         
         // load chess corner data
         if( enable_chess )
         {
            fs["obj_chess_corners"] >> obj_corners;
            fs["cam_chess_corners"] >> cam_corners;
            fs["pro_chess_corners"] >> pro_corners;
            if(!obj_corners.data || !cam_corners.data || !pro_corners.data)
            {
               printf("error reading [%s], exiting..\n",scanList[i].c_str());
               return -1;
            }
         }
         // load correspondence data
         else
         {
            fs["cam_points"] >> cam_points;
            fs["pro_points"] >> obj_points;
            if(!cam_points.data || !obj_points.data)
            {
               printf("error reading [%s], exiting..\n",scanList[i].c_str());
               return -1;
            }
         }
         
         fs.release();
      }
      // done reading file
      // ----------------------------------------------------------------------
      // now process file data
      
      // confirm all input image sizes are the same
      Size2i cam_size;
      if( cam_dims.data )
      {
         cam_size = Size2i( cam_dims.ptr<int>(0)[0], cam_dims.ptr<int>(0)[1] );
         
         if(i==0)
         {
            base_cam_size = cam_size;
            printf("camera size : %d x %d\n",base_cam_size.width,base_cam_size.height);
         } else
         {
            CV_Assert( cam_size.width == base_cam_size.width );
            CV_Assert( cam_size.height == base_cam_size.height );
         }
      }
      
      // confirm all projector sizes are the same
      Size2i pro_size;
      printf("confirm all projector sizes are the same..");
      fflush(stdout);
      if( pro_dims.data )
      {
         pro_size = Size2i(pro_dims.ptr<int>(0)[0], pro_dims.ptr<int>(0)[1]);
         
         // initialize base_pro_size if needed
         if(i==0 && base_pro_size.width == 0)
         {
            base_pro_size = pro_size;
         } else
            // confirm size matches
         {
            CV_Assert( pro_size.width == base_pro_size.width );
            CV_Assert( pro_size.height == base_pro_size.height );
         }
      }
      printf("done\n");
      
      // copy 2d correspondence mat to 3d vector
      //vector< Point2f > vec_cam_matches; //(cam_points.begin<Point2f>(), cam_points.end<Point2f>());
      
      // copy mat to vector of points
      for(int r=0; r<cam_points.rows; r++)
      {
         //float* ptr = cam_points.ptr<float>(r);
         Vec2f* ptr = cam_points.ptr<Vec2f>(r);
         for(int c=0; c<cam_points.cols; c++)
         {
            //float x = ptr[c*2+0];
            //float y = ptr[c*2+1];
            float x = ptr[c][0];
            float y = ptr[c][1];
            tmp_cam_matches.push_back(Point2f(x,y));
            vec_cam_matches.push_back(Point2f(x,y));
         }
      }
      
      //vector< Point3f > vec_obj_matches;
      //vector< Point2f > vec_obj_matches2d;
      printf("copying 2d decoded points into 3d vector");
      fflush(stdout);
      if( !enable_chess )
      {
         // copy mat to vector of points
         for(int r=0; r<obj_points.rows; r++)
         {
            //float* ptr = obj_points.ptr<float>(r);
            Vec2f* ptr = obj_points.ptr<Vec2f>(r);
            for(int c=0; c<obj_points.cols; c++)
            {
               //float x = ptr[c*2+0]*squareSize;
               //float y = ptr[c*2+1]*squareSize;
               float x = ptr[c][0]*squareSize;
               float y = ptr[c][1]*squareSize;
               tmp_obj_matches.push_back(Point3f(x,y,0));
               tmp_obj_matches2d.push_back(Point2f(x,y));
               
               vec_obj_matches.push_back(Point3f(x,y,0));
               vec_obj_matches2d.push_back(Point2f(x,y));
            }
         }
      }
      printf("done\n");
      CV_Assert( vec_obj_matches.size() == vec_cam_matches.size() );
      
      // add onto vector
      if( enable_chess )
      {
         object_corners.push_back(   vector<Point3f>(obj_corners));
         camera_corners.push_back(   vector<Point2f>(cam_corners));
         projector_corners.push_back(vector<Point2f>(pro_corners));
      } else {
         pro_point_list.push_back( tmp_obj_matches );
         cam_point_list.push_back( tmp_cam_matches );
      }
      
      /*
       int cur_idx = pro_point_list.size()-1;
       cur_idx = cam_point_list.size()-1;
       printf("Mat pro_point_list[%d] is %dx%d\n",cur_idx,obj_points.rows,obj_points.cols);
       printf("Mat cam_point_list[%d] is %dx%d\n",cur_idx,cam_points.rows,cam_points.cols);
       printf("vec objPoints: %d\n", (int)vec_obj_matches.size() );
       printf("vec imgPoints: %d\n", (int)vec_cam_matches.size() );
       printf("----\n");
       printf("cam_pts: ch=%d, %d rows, %d cols\n",cam_points.channels(),cam_points.rows,cam_points.cols);
       printf("pro_pts: ch=%d, %d rows, %d cols\n",obj_points.channels(),obj_points.rows,obj_points.cols);
       */
      
      int used_points = 0;
      if( enable_chess )
      {
         used_points = obj_corners.rows;
      } else {
         used_points = obj_points.rows;
      }
      
      printf("loaded : (%4d x %4d) [%s; %s; %s; %s/%s] %8d points [%s]\n",
             (int)cam_size.width,(int)cam_size.height,
             cam_make.c_str(), cam_model.c_str(), cam_lens.c_str(),
             cam_serial.c_str(), cam_intserial.c_str(),
             used_points,  scanList[i].c_str());
      
      // end data loading
   }
   
   cout << "pro_size = " << base_pro_size << endl;
   
   
   // count total points of input(s)
   vector<int> list_sizes;
   if( enable_chess )
   {
      for(int i=0; i<object_corners.size(); i++)
      {
         list_sizes.push_back(object_corners[i].size());
      }
   } else {
      for(int i=0; i<pro_point_list.size(); i++)
      {
         list_sizes.push_back(pro_point_list[i].size());
      }
   }
   
   printf("* finished loading points *\n");
   printf("---------------------------\n");
   printf("total scans: %d\n", (int)pro_point_list.size() );
   // ----------------------------------------------------------------------
   
   // begin calibration
   Mat def_K  = Mat::eye(3,3,CV_64F);
   Mat def_kc = Mat::zeros(8,1,CV_64F);
   Mat def_size = Mat::zeros(2,1,CV_32S);
   
   // intrinsics
   Mat cam_K, cam_kc, cam_dims;
   Mat pro_K, pro_kc, pro_dims;
   
   float cam_rms = -1;
   float pro_rms = -1;
   
   bool cam_calib_loaded = false;
   bool pro_calib_loaded = false;
   
   // -----------------------------------
   // load pre-existing calibration data
   if( load_calib )
   {
      printf("loading calibration files..\n");
      for (std::vector<string>::iterator it = calibList.begin() ; it != calibList.end(); ++it)
      {
         cout << "loading [" << *it << "]" << endl;
         
         FileStorage fs;
         fs.open(*it, FileStorage::READ);
         if (!fs.isOpened())
            return fprintf(stderr,"failed to open [%s]",inCalibFN), -1;
         printf("\treading data from %s..\n",inCalibFN);
         
         float tmp_cam_rms = 0;
         Mat tmp_cam_K, tmp_cam_kc, tmp_cam_dims;
         fs["cam_K"]  >> tmp_cam_K;
         fs["cam_kc"] >> tmp_cam_kc;
         fs["cam_size"] >> tmp_cam_dims;
         fs["cam_rms"] >> tmp_cam_rms;
         
         if( tmp_cam_K.data && tmp_cam_kc.data && tmp_cam_dims.data )
         {
            cam_calib_loaded = true;
            cam_K    = tmp_cam_K;
            cam_kc   = tmp_cam_kc;
            cam_dims = tmp_cam_dims;
            cam_rms  = tmp_cam_rms;
         }
         if( no_cam_load )
         {
             cam_calib_loaded = false;
             cam_K = def_K.clone();
             cam_kc = def_kc.clone();
             cam_dims = def_size.clone();
             cam_rms = -1;
         }
         
         float tmp_pro_rms = 0;
         Mat tmp_pro_K, tmp_pro_kc, tmp_pro_dims;
         fs["pro_K"]  >> tmp_pro_K;
         fs["pro_kc"] >> tmp_pro_kc;
         fs["pro_size"] >> tmp_pro_dims;
         fs["pro_rms"] >> tmp_pro_rms;
         
         if( tmp_pro_K.data && tmp_pro_kc.data && tmp_cam_dims.data )
         {
            pro_calib_loaded = true;
            pro_K    = tmp_pro_K;
            pro_kc   = tmp_pro_kc;
            pro_dims = tmp_pro_dims;
            pro_rms  = tmp_pro_rms;
         }
         if( no_pro_load )
         {
             pro_calib_loaded = false;
             pro_K = def_K.clone();
             pro_kc = def_kc.clone();
             pro_dims = def_size.clone();
             pro_rms = -1;
         }
         
      }
      
   }
   
   // set sizes from Mat input
   //    if(cam_dims.data) cam_size = Size2i(cam_dims.ptr<int>(0)[0],cam_dims.ptr<int>(0)[1]);
   //    printf("cam_dims = %d, %d\n",cam_size.width,cam_size.height);
   //    if(pro_dims.data) pro_size = Size2i(pro_dims.ptr<int>(0)[0],pro_dims.ptr<int>(0)[1]);
   //    printf("pro_dims = %d, %d\n",pro_size.width,pro_size.height);
   
   if(base_cam_size.width==0||base_cam_size.height==0||
      ( enable_chess && (base_pro_size.width==0||base_pro_size.height==0)))
   {
      return fprintf( stderr, "Invalid camera or projector size\n" ), -1;
   }
   
   printf("cam calib loaded? %s\n",cam_calib_loaded?"YES":"NO");
   printf("pro calib loaded? %s\n",pro_calib_loaded?"YES":"NO");
   
   //    cout << "cam_K:\n"    << cam_K << endl;
   //    cout << "cam_kc:\n"   << cam_kc << endl;
   //    cout << "cam_size = " << base_cam_size << endl;
   //    cout << "pro_K:\n"    << pro_K << endl;
   //    cout << "pro_kc:\n"   << pro_kc << endl;
   
   // calibration calls
   int def_flags = 0;
   TermCriteria def_tc = TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 50, DBL_EPSILON);
   
   vector<Mat> cam_rvecs, cam_tvecs;
   vector<Mat> pro_rvecs, pro_tvecs;
   
   Mat E,F,F2,R,T,H,H2;
   Mat R1,R2,P1,P2,Q;
   
   Mat def_3x1(3, 1, CV_64FC1, Scalar::all(0));
   Mat def_3x3(3, 3, CV_64FC1, Scalar::all(0));
   Mat def_3x4(3, 4, CV_64FC1, Scalar::all(0));
   Mat def_4x4(4, 4, CV_64FC1, Scalar::all(0));
   
   R = def_3x3.clone();
   T = def_3x1.clone();

   F  = def_3x3.clone();
   F2 = def_3x3.clone();
   H  = def_3x3.clone();
   H2 = def_3x3.clone();

   R1 = def_3x3.clone();
   R2 = def_3x3.clone();
   P1 = def_3x4.clone();
   P2 = def_3x4.clone();
   Q  = def_4x4.clone();

   double cam_error, pro_error, stereo_error;
   
   // set defaults
//   if( !cam_calib_loaded ){
//      cam_K  = def_K.clone();
//      cam_kc = def_kc.clone();
//      cam_dims = def_size.clone();
//   } else {
//      
//   }
//   if( !pro_calib_loaded ){
//      pro_K  = def_K.clone();
//      pro_kc = def_kc.clone();
//      pro_dims = def_size.clone();
//   }
   
   // compute camera-projector stereo calibration
   if( only_fundamental)
   {
      vector<Point2f> cam_pt0(vec_cam_matches.begin(),vec_cam_matches.end());//(vec_cam_matches);
      vector<Point2f> obj_pt0(vec_obj_matches2d.begin(),vec_obj_matches2d.end());//(vec_obj_matches2d);
      vector<Point2f> cam_pt1(vec_cam_matches.begin(),vec_cam_matches.end());//(vec_cam_matches);
      vector<Point2f> obj_pt1(vec_obj_matches2d.begin(),vec_obj_matches2d.end());//(vec_obj_matches2d);
      F  = findFundamentalMat( cam_pt0, obj_pt0, CV_FM_RANSAC, 1., 0.99 );
      F2 = findFundamentalMat( obj_pt1, cam_pt1, CV_FM_RANSAC, 1., 0.99 );
      H  = findHomography( vec_cam_matches, vec_obj_matches2d, CV_FM_RANSAC, 3 );
      H2 = findHomography( vec_obj_matches2d, vec_cam_matches, CV_FM_RANSAC, 3 );
        

      double avg_dx=0,avg_dy=0;
      double rms_dx=0,rms_dy=0;
      int avg_n=0;
      for(int i=0;i<vec_cam_matches.size();i++)
      {
          Point2f cam_pt = Point2f(vec_cam_matches[i]);
          Point2f obj_pt = Point2f(vec_obj_matches2d[i]);
          
          Mat_<double> cam_h = (Mat_<double>(3,1) << (double)cam_pt.x, (double)cam_pt.y, 1.0);
          Mat_<double> obj_h = (Mat_<double>(3,1) << (double)obj_pt.x, (double)obj_pt.y, 1.0);
          Mat_<double> hom_pt = H*cam_h;
          
          Point3d pt(hom_pt(0,0),hom_pt(0,1),hom_pt(0,2));
          //pt *= 1/cv::norm(pt);
          pt *= 1/pt.z;
          
          double dx = pt.x-obj_pt.x;
          double dy = pt.y-obj_pt.y;

          avg_dx += dx;
          avg_dy += dy;
          rms_dx += pow(dx,2);
          rms_dy += pow(dy,2);
          avg_n ++;

          //printf("(%0.2f, %0.2f) -> (%0.2f, %0.2f) vs (%0.6f,%0.6f,%0.6f) d(%0.6f,%0.6f)\n",\
                  cam_pt.x,cam_pt.y, obj_pt.x,obj_pt.y, pt.x,pt.y,pt.z, pt.x-obj_pt.x, pt.y-obj_pt.y);
      }
      avg_dx /= avg_n;
      avg_dy /= avg_n;
      rms_dx /= avg_n;
      rms_dy /= avg_n;
      rms_dx = pow(rms_dx,0.5);
      rms_dy = pow(rms_dy,0.5);

      printf("avg d(%0.6f,%0.6f) rms(%0.6f,%0.6f) over %d points\n",\
              avg_dx,avg_dy, rms_dx,rms_dy, avg_n);

   }
   else if( enable_chess )
   {
      // if no camera calibration, calculate it
      if( cam_calib_loaded && !use_cam_guess )
      {
         cam_error = cam_rms;
      }
      else
      {
         int cam_flags = def_flags;
         if( use_cam_guess )
         {
            cam_flags |= CALIB_USE_INTRINSIC_GUESS;
         }
         else
         {
            cam_K  = def_K.clone();
            cam_kc = def_kc.clone();
            cam_dims = def_size.clone();
         }

         if( enable_rational_model )
            cam_flags |= CALIB_RATIONAL_MODEL;

         cam_error = calibrateCamera(object_corners, camera_corners,
                                     base_cam_size, cam_K, cam_kc,
                                     cam_rvecs, cam_tvecs,
                                     cam_flags, def_tc);
         
         printf("cam rms = %f\n",cam_error);
         
      }
      
      // if no projector calibration, calculate it
      if( pro_calib_loaded && !use_pro_guess )
      {
         pro_error = pro_rms;
      }
      else
      {
         int pro_flags = def_flags;
         if( use_pro_guess )
         {
            pro_flags |= CALIB_USE_INTRINSIC_GUESS;
         }
         else
         {
            pro_K  = def_K.clone();
            pro_kc = def_kc.clone();
            pro_dims = def_size.clone();
         }
         
         if( enable_rational_model )
            pro_flags |= CALIB_RATIONAL_MODEL;
         pro_error = calibrateCamera(object_corners, projector_corners,
                                     base_pro_size, pro_K, pro_kc,
                                     pro_rvecs, pro_tvecs,
                                     pro_flags, def_tc);
         
         printf("pro rms = %f\n",pro_error);
      }
      
      if( enable_stereo )
      {
         // calculate stereo calibration
         int stereo_flags = CALIB_FIX_INTRINSIC | CALIB_USE_INTRINSIC_GUESS;
         
         stereo_error = stereoCalibrate( object_corners, camera_corners, projector_corners,
                                        cam_K, cam_kc, pro_K, pro_kc, base_cam_size,
                                        R,T,E,F, def_tc,stereo_flags);
                                        
         printf("stereo rms = %f\n",stereo_error);
         printf("HELP!\n");
         // calculate stereo rectification
         int rect_flags = CALIB_ZERO_DISPARITY;
         stereoRectify(cam_K,cam_kc, pro_K,pro_kc, base_cam_size, R,T,
                       R1,R2,P1,P2,Q, rect_flags);
         printf("HELP2!\n");
      }

   }
   // compute just camera calibration
   else
   {
      int cam_flags = def_flags;
      if( enable_rational_model )
        cam_flags |= CALIB_RATIONAL_MODEL;
      if(enable_zerotan)
         cam_flags |= CV_CALIB_ZERO_TANGENT_DIST;
      cam_error = calibrateCamera( pro_point_list, cam_point_list,
                                  base_cam_size, cam_K, cam_kc,
                                  cam_rvecs, cam_tvecs,
                                  cam_flags);

      F = findFundamentalMat( vec_cam_matches, vec_obj_matches2d, CV_FM_RANSAC, 3., 0.99 );
      F2 = findFundamentalMat( vec_obj_matches2d, vec_cam_matches, CV_FM_RANSAC, 3., 0.99 );

   }
   
   // save calibration results
   if(cam_error >= 0)
   {
      printf("writing calibration data to [%s]..\n",outCalibFN);
      cv::FileStorage fs(outCalibFN, cv::FileStorage::WRITE);
      
      cout << "cam_size: " << Mat(Point2i(base_cam_size)) << endl;

      if( only_fundamental)
      {
          fs << "F" << F;
          fs << "F2" << F2;
          fs << "H" << H;
          fs << "H2" << H2;
      }
      else
      {
          cout << "cam_K: " << cam_K << endl;
          cout << "cam_kc: " << cam_kc << endl;
          cout << "cam_rms: " << cam_error << endl;

          if( pixSize > -1 )
          {
            double fovx,fovy,FL,aspectRatio;
            Point2d pp;
            calibrationMatrixValues(cam_K, base_cam_size, pixSize*base_cam_size.width, pixSize*base_cam_size.height,\
                  fovx, fovy, FL, pp, aspectRatio);
            fs << "cam_pixel_size" << pixSize;
            fs << "cam_fovx" << fovx;
            fs << "cam_fovy" << fovy;
            fs << "cam_focal_length" << FL;
            fs << "cam_aspect_ratio" << aspectRatio;
          }
          
          fs << "cam_size" << Mat(Point2i(base_cam_size));
          fs << "cam_K"  << cam_K;
          fs << "cam_kc" << cam_kc;
          fs << "cam_rms" << cam_error;


          //fs << "F" << F;
          //fs << "F2" << F2;
          
          if(enable_chess)
          {
             cout << "pro_size: " << Mat(Point2i(base_pro_size)) << endl;
             cout << "pro_K: "   << pro_K << endl;
             cout << "pro_kc: "  << pro_kc << endl;
             cout << "pro_rms: " << pro_error << endl;
            
             fs << "pro_size" << Mat(Point2i(base_pro_size));
             fs << "pro_K"   << pro_K;
             fs << "pro_kc"  << pro_kc;
             fs << "pro_rms" << pro_error;
             
             if(enable_stereo)
             {
                fs << "stereo_rms" << stereo_error;
                fs << "R" << R;
                fs << "T" << T;
                fs << "E" << E;
                //fs << "F" << F;
                fs << "R1" << R1;
                fs << "R2" << R2;
                fs << "P1" << P1;
                fs << "P2" << P2;
                fs << "Q"  << Q;
             }
          }
          
          fs << "cam_rvecs" << cam_rvecs;
          fs << "cam_tvecs" << cam_tvecs;
          
          fs << "square_size" << squareSize;
          fs << "input_files" << scanList;
          fs << "input_counts" << list_sizes;
      }
   }
   
   return 0;
}













