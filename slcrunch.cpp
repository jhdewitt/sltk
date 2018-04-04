// slcrunch.cpp
// parse images from gray code binary image sequence(s)
// by John De Witt

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

#include "util.h"
#include "sl_util.h"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/core/core.hpp>
//#include <opencv2/imgcodecs.hpp>

//#include <stdio.h>
//#include <math.h>
//#include <string.h>
//#include <time.h>
//#include <stdint.h>

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
   printf("This program parses an image sequence of structured light patterns to extract pixel locations.\n"
          "  Notable options include:\n"
          "   - subsampling of dense point correspondence data\n"
          "   - find chessboard corners in camera and projector coordinate spaces\n"
          //           "ex: lsnum -d folder/ | xargs slcrunch\n"
          "ex: slcrunch.out -sf 16 -st grid -chess 5 3 calibset_01.yaml\n"
          "Usage: slcrunch\n"
          //         "      image [..]          # sequence of images to parse\n"
          "    input_data            # input data\n"
          "                          #  - text file with a list of the images in the sequence\n"
          "    [-red]                # process only the red   channel of input images\n"
          "    [-green]              # process only the green channel of input images\n"
          "    [-blue]               # process only the blue  channel of input images\n"
          "    [-m <msg>]            # message/note to attach to scan for identification\n"
          "    [-ply]                # configure ply 3d point output (disables yaml match output)\n"
          "    [-prosize <NxM>]      # specify the projector space resolution used\n"
          "    [-procam <file.yaml>] # configure camera-projector calibration input (required for PLY output)\n"
          "    [-inv|-noinv]         # specify whether inverses are shown in sequence\n"
          "                          # default = inv\n"
          "    [-x|-nox]             # specify whether x axis is encoded in sequence\n"
          "                          # default = x\n"
          "    [-y|-noy]             # specify whether y axis is encoded in sequence\n"
          "                          # default = y\n"
          "    [-chess <w> <h>]      # specify that there is a chessboard in view of the camera\n"
          "                          #   - w and h refer to the number of inner corners on each axis\n"
          "                          #   - corners are found in camera and projector coordinate spaces\n"
          "                          #   - corners in projector space are only found if -x -y enabled\n"
          "    [-s <scale>]          # set chessboard scale\n"
          "    [-chessimg <fname>]   # load a full-white image for finding the chessboard\n"
          "    [-chessidx <number>]  # load a full-white image for finding the chessboard from the input sequence\n"
          "    [-t <thresh_value>]   # specify black/white threshold parameter\n"
          "    [-sf <factor>]        # set factor for subsampling output points\n"
          "    [-st <grid|point>]    # set type of subsampling (grid or point)\n"
          "                          # default = grid\n"
          "    [-c]                  # coalesce each object space point into a single image space point\n"
          "    [-f <filter>]         # apply filter for output point RGB values (median)\n"
          "    [-b <bits>]           # truncation: number of LSB in the pattern images that will be ignored\n"
          "    [-noc]                # disable point coalescing\n"
          "    [-prefix <prefix>]    # the output filename prefix; it is appended to every output file path\n"
          "    [-o <out_map>]        # the output filename for the image/object correspondence map\n"
          "                          #     the text file can be generated with imagelist_creator\n"
          "    [-yaml|-noyaml]       # configure yaml output\n"
          "    [-nomatch]            # disable saving camera-projector correspondences"
          "    [-vis]                # save visualization images\n"
          "                          # default = yaml\n"
          "    [-vscl]               # set color mapping visualization scale\n"
          "    [-vsize]              # supersample the output correspondence visualization output\n"
          "\n");
}

// from opencv
static bool readStringList( const string& filename, vector<string>& l )
{
//   if( !file_exists(filename) )
//   {
//      return false;
//   }
   
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

// parse a list file and add pathprefix to each file if needed
// get GCB images
// get RGB images
static bool parseStringList( const string& filename, vector<string>& gcb_l, vector<string>& rgb_l )
{
   // fail if dir
   if( is_dir( filename.c_str() ) )
   {
      cout << "Can't parse directory as string list.." << endl;
      return false;
   }
   
   // fail if file doesn't exist
   if( !file_exists(filename.c_str()) )
   {
      return false;
   }

   string prefix;
   string fname;
   
   size_t found = filename.find_last_of("/");
   if( found == string::npos )
   {
      prefix = "";
      fname = filename;
   }
   else
   {
      prefix = filename.substr(0,found);
      fname  = filename.substr(found+1);
   }
   printf("\tprefix : [%s]\n",prefix.c_str());
   printf("\tfname  : [%s]\n",fname.c_str());
   
   // open the file
   FileStorage fs(filename, FileStorage::READ);
   gcb_l.resize(0);
   rgb_l.resize(0);
   if( !fs.isOpened() )
      return false;
   
   cout << "parsing [" << filename << "]\n";

   // first load all GRAY CODE BINARY images
   FileNode gcb_images = fs["gcb_images"];
   if( gcb_images.type() == FileNode::SEQ )
   {
       FileNodeIterator gcb_it = gcb_images.begin(), gcb_it_end = gcb_images.end();
       for( ; gcb_it != gcb_it_end; ++gcb_it )
       {
          // add prefix if no slash present in path
          if( ((string)*gcb_it).find("/") == string::npos )
          {
             if( prefix != "" )
                gcb_l.push_back(prefix +"/"+ (string)*gcb_it);
             else
                gcb_l.push_back((string)*gcb_it);
             
          }
          // omit prefix if slash is present in path
          else
          {
             gcb_l.push_back((string)*gcb_it);
          }
          cout << "[" << gcb_l[gcb_l.size()-1] << "]" <<  endl;
       }
   }
   
   // next load all RGB MONOCHROME images
   FileNode rgb_images = fs["rgb_images"];
   if( rgb_images.type() == FileNode::SEQ )
   {
       FileNodeIterator rgb_it = rgb_images.begin(), rgb_it_end = rgb_images.end();
       for( ; rgb_it != rgb_it_end; ++rgb_it )
       {
          // add prefix if no slash present in path
          if( ((string)*rgb_it).find("/") == string::npos )
          {
             if( prefix != "" )
                rgb_l.push_back(prefix +"/"+ (string)*rgb_it);
             else
                rgb_l.push_back((string)*rgb_it);
             
          }
          // omit prefix if slash is present in path
          else
          {
             rgb_l.push_back((string)*rgb_it);
          }
          cout << "[" << rgb_l[rgb_l.size()-1] << "]" <<  endl;
       }
   }
   
   printf("gcb images : %d, rgb images: %d\n",(int)gcb_l.size(),(int)rgb_l.size());
   
   return true;
}

Vec3b float2Gray( float t ){
   t = fmin(fmax(t,0.f),1.f);
   Vec3b col;
   col[0] = (uint8_t)(255*t);
   col[1] = (uint8_t)(255*t);
   col[2] = (uint8_t)(255*t);
   return col;
}

Vec3b float2Color( float t ){
   float n = 4.f;
   float dt = 1.f/n;
   
   float r,g,b;
   if(t<=1.f*dt)
   {
      // red to orange
      float c = n*(t-0.f*dt);
      r = 1.0f;
      g = c;
      b = 0.f;
   }
   else if (t<=2.f*dt){
      // orange to green
      float c = n*(t-1.f*dt);
      r = 1.0f-c;
      g = 1.0f;
      b = 0.f;
   }
   else if (t<=3.f*dt){
      // green to cyan
      float c = n*(t-2.f*dt);
      r = 0.f;
      g = 1.0f;
      b = c;
   }
   else if (t<=4.f*dt){
      // cyan to blue
      float c = n*(t-3.f*dt);
      r = 0.f;
      g = 1.f-c;
      b = 1.f;
   }

   /*
   if(t<=1.f*dt)
   {
      float c = n*(t-0.f*dt);
      r = c;
      g = 0.f;
      b = 0.f;
   }
   else if (t<=2.f*dt){
      float c = n*(t-1.f*dt);
      r = 1.0f;
      g = c;
      b = 0.f;
   }
   else if (t<=3.f*dt){
      float c = n*(t-2.f*dt);
      r = 1.0f-c;
      g = 1.0f;
      b = c;
   }
   else if (t<=4.f*dt){
      float c = n*(t-3.f*dt);
      r = 0.f;
      g = 1.f-c;
      b = 1.f-c*0.5;
   }
    */
   
   Vec3b col;
   col[0] = (uint8_t)(255*b);
   col[1] = (uint8_t)(255*g);
   col[2] = (uint8_t)(255*r);
   
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

/**
 From "Triangulation", Hartley, R.I. and Sturm, P., Computer vision and image understanding, 1997
 */
Mat LinearLSTriangulation(Point3d u,       //homogenous image point (u,v,1)
                          Matx34d P,       //camera 1 matrix
                          Point3d u1,      //homogenous image point in 2nd camera
                          Matx34d P1       //camera 2 matrix
)
{
   // build matrix A for homogenous equation system Ax = 0
   // assume X = (x,y,z,1), for Linear-LS method
   // which turns it into a AX = B system
   //    where A is 4x3, X is 3x1 and B is 4x1
   Matx43d A(u.x *  P(2,0)-  P(0,0), u.x *  P(2,1)-P(0,1),  u.x *  P(2,2)- P(0,2),
             u.y *  P(2,0)-  P(1,0), u.y *  P(2,1)-P(1,1),  u.y *  P(2,2)- P(1,2),
             u1.x* P1(2,0)- P1(0,0), u1.x* P1(2,1)-P1(0,1), u1.x* P1(2,2)-P1(0,2),
             u1.y* P1(2,0)- P1(1,0), u1.y* P1(2,1)-P1(1,1), u1.y* P1(2,2)-P1(1,2) );
   
   Mat B = (Mat_<double>(4,1) <<
            -(u.x * P(2,3) - P(0,3)),
            -(u.y * P(2,3) - P(1,3)),
            -(u1.x*P1(2,3) -P1(0,3)),
            -(u1.y*P1(2,3) -P1(1,3)));
   
   Mat X;
   solve(A,B,X,DECOMP_SVD);
   
   return X;
}

#define EPSILON 0.1
/**
 From "Triangulation", Hartley, R.I. and Sturm, P., Computer vision and image understanding, 1997
 */
Mat_<double> IterativeLinearLSTriangulation(Point3d u,    //homogenous image point (u,v,1)
                                            Matx34d P,          //camera 1 matrix
                                            Point3d u1,         //homogenous image point in 2nd camera
                                            Matx34d P1          //camera 2 matrix
) {
   double wi = 1, wi1 = 1;
   Mat_<double> X(4,1);
   for (int i=0; i<10; i++) { //Hartley suggests 10 iterations at most
      Mat_<double> X_ = LinearLSTriangulation(u,P,u1,P1);
      X(0) = X_(0); X(1) = X_(1); X(2) = X_(2); X_(3) = 1.0;
      
      //recalculate weights
      double p2x = Mat_<double>(Mat_<double>(P).row(2)*X)(0);
      double p2x1 = Mat_<double>(Mat_<double>(P1).row(2)*X)(0);
      
      //breaking point
      if(fabs(wi - p2x) <= EPSILON && fabs(wi1 - p2x1) <= EPSILON) break;
      
      wi = p2x;
      wi1 = p2x1;
      
      //reweight equations and solve
      Matx43d A((u.x*P(2,0)-P(0,0))/wi,       (u.x*P(2,1)-P(0,1))/wi,         (u.x*P(2,2)-P(0,2))/wi,
                (u.y*P(2,0)-P(1,0))/wi,       (u.y*P(2,1)-P(1,1))/wi,         (u.y*P(2,2)-P(1,2))/wi,
                (u1.x*P1(2,0)-P1(0,0))/wi1,   (u1.x*P1(2,1)-P1(0,1))/wi1,     (u1.x*P1(2,2)-P1(0,2))/wi1,
                (u1.y*P1(2,0)-P1(1,0))/wi1,   (u1.y*P1(2,1)-P1(1,1))/wi1,     (u1.y*P1(2,2)-P1(1,2))/wi1
                );
      Mat_<double> B = (Mat_<double>(4,1) <<    -(u.x*P(2,3)    -P(0,3))/wi,
                        -(u.y*P(2,3)  -P(1,3))/wi,
                        -(u1.x*P1(2,3)    -P1(0,3))/wi1,
                        -(u1.y*P1(2,3)    -P1(1,3))/wi1
                        );
      
      solve(A,B,X_,DECOMP_SVD);
      X(0) = X_(0); X(1) = X_(1); X(2) = X_(2); X_(3) = 1.0;
   }
   return X;
}

// always assume x images precede y images
int main( int argc, char** argv ) {
   
   //    Mat test(1050,1680,CV_8UC3,Scalar::all(0));
   //    for(int r=0; r<test.rows; r++){
   //        uint8_t* pix = test.ptr<uint8_t>(r);
   //        for(int c=0; c<test.cols; c++){
   //            Vec3b col = float2Color( c/(float)test.cols );
   //            pix[c*3+0]=col[0];
   //            pix[c*3+1]=col[1];
   //            pix[c*3+2]=col[2];
   //        }
   //    }
   //    imshow("test",test);
   //    waitKey(0);
   //    return 0;
   
   // string
   const char* inImgListFN = 0;
   const char* outputMapFilename = 0;
   const char* outputImgFilename = 0;
   const char* outPrefix = 0;
   //    const char* outputPlyFilename = 0;
   //    const char* outputDirectImgFilename = 0;
   //    const char* outputGlobalImgFilename = 0;
   //    
   int n=0;
   
   /*
   const char* outMapFN      = findNextName( "slscan_%04d.yaml",&n); n=0;
   const char* outViscFN     = findNextName( "slscan_%04d_rgb.png",&n); n=0;
   const char* outVisxFN     = findNextName( "slscan_%04d_x.png",&n); n=0;
   const char* outVisyFN     = findNextName( "slscan_%04d_y.png",&n); n=0;
   const char* outVisDirFN   = findNextName( "slscan_%04d_direct.png",&n); n=0;
   const char* outVisAmbFN   = findNextName( "slscan_%04d_indirect.png",&n); n=0;
   const char* outChessCamFN = findNextName( "slscan_%04d_chesscam.png",&n); n=0;
   const char* outChessProFN = findNextName( "slscan_%04d_chesspro.png",&n); n=0;
   const char* outPlyFN      = findNextName( "slscan_%04d.ply", &n); n=0;
    */
   //(char*)malloc(.
   const char* outMapFN      = findNextName( "slscan_%04d.yaml",&n); n=0;
   const char* outViscFN     = findNextName( "slscan_%04d_rgb.png",&n); n=0;
   const char* outVisxFN     = findNextName( "slscan_%04d_x.png",&n); n=0;
   const char* outVisyFN     = findNextName( "slscan_%04d_y.png",&n); n=0;
   const char* outVisDirFN   = findNextName( "slscan_%04d_direct.png",&n); n=0;
   const char* outVisAmbFN   = findNextName( "slscan_%04d_indirect.png",&n); n=0;
   const char* outChessCamFN = findNextName( "slscan_%04d_chesscam.png",&n); n=0;
   const char* outChessProFN = findNextName( "slscan_%04d_chesspro.png",&n); n=0;
   const char* outPlyFN      = findNextName( "slscan_%04d.ply", &n); n=0;

   const char* outFilter = 0;
   bool rgbFilter = false;

   bool chess_image_input = false;
   const char* inChessImageFN = 0;
   int chess_idx = 23;
   
   //outputMapFilename = outMapFN;
   
   string scanMessage;
   vector< stringmap > exifList;
   vector< string > gcbFileList;
   vector< string > rgbFileList;
   vector< Mat > imageList;
   
   //    vector< pair<string,stringmap> > image_list;
   
   // memory hog mode (load all images at once)
   bool load_all = false;
   int diff_threshold = 5;
   
   // procam calib input options
   bool enable_procam = false;
   const char* inCalibFN = 0;
   
   // projector info
   Size2i base_pro_size;
   Size2i pro_size_input;
   bool enable_prosize_input = false;

   // rgb pattern processing
   bool enable_rgb_pattern_processing = true;

   // color channel selection
   bool enable_monochrome_processing = false;
   int monochrome_channel_idx = 0;

   // chessboard options
   bool enable_chess = false;
   Size chess_size(0,0);
   
   // what to parse
   bool show_inverse = true;
   bool show_x = true;
   bool show_y = true;
   
   // what to load
   bool enable_mapload = false;
   const char* inMapFN = 0;
   
   // what to save
   bool save_yaml = true;
   bool save_ply = false;
   bool save_preview = false;
   
   // truncation
   bool truncate_enable = false;
   int truncate_value = 0; // can abandon last N bits in sequence // must get at least N bits
   
   // whether to output the result
   bool save_file = true;
   bool save_matches = true;
   
   // subsampling options
   bool enable_coalesce = false;
   bool enable_subsamp = false;
   int subsamp_factor = 1;
   int subsamp_type = SUBSAMPLE_GRID;
   
   // useful values
   float chess_scale = 1;
   int mode = DETECTION;
   float vis_scl = 1;
   float vis_supersample = 1;
   
   int mult = 1;
   int bits = 1;
   int offset = 1;
   
   if( argc < 2 ){
      help();
      return 0;
   }
   
   // setup
   {
      // parse arguments
      for(int i=1; i<argc; i++){
         
         const char* s = argv[i];
         // printf("%d : %s\n",i,s);
         
         // subsample parameters
         if( strcmp(s,"-sf") == 0 )
         {
            enable_subsamp = true;
            if( i+1 >= argc )
               return fprintf( stderr, "Subsample factor not provided\n"), -1;
            if( sscanf( argv[++i], "%d", &subsamp_factor ) != 1 || subsamp_factor < 1 )
               return fprintf( stderr, "Invalid subsample factor\n"), -1;
         }
         else if( strcmp(s,"-st") == 0 )
         {
            if( i+1 >= argc )
               return fprintf( stderr, "Subsample mode not provided\n"), -1;
            const char* s2 = argv[++i];
            if(strcmp(s2,"grid") == 0)
               subsamp_type = SUBSAMPLE_GRID;
            else if(strcmp(s2,"point") == 0)
               subsamp_type = SUBSAMPLE_POINT;
            else
               return fprintf( stderr, "Invalid subsample mode\n"), -1;
         }
         // RGB filter
         else if( strcmp(s,"-f") == 0 )
         {
            outFilter = argv[++i];
            printf("RGB FILTER ENGAGE [%s]\n",outFilter);
         }
         // output prefix
         else if( strcmp(s,"-prefix") == 0 )
         {
            outPrefix = argv[++i];
            printf("outPrefix [%s]\n",argv[i]);
         }
         // coalesce
         else if( strcmp(s,"-c") == 0 )
         {
            enable_coalesce = true;
            printf("enable_coalesce\n");
         }
         // enable rgb pattern processing 
         else if( strcmp(s,"-rgb") == 0 )
         {
            enable_rgb_pattern_processing = true;
            printf("enable_rgb_pattern_processing = true\n");
         }
         // input monochrome red mode
         else if( strcmp(s,"-red") == 0 )
         {
            enable_monochrome_processing = true;
            monochrome_channel_idx = 2;
            printf("enable_monochrome_processing = red\n");
         }
         // input monochrome green mode
         else if( strcmp(s,"-green") == 0 )
         {
            enable_monochrome_processing = true;
            monochrome_channel_idx = 1;
            printf("enable_monochrome_processing = green\n");
         }
         // input monochrome blue mode
         else if( strcmp(s,"-blue") == 0 )
         {
            enable_monochrome_processing = true;
            monochrome_channel_idx = 0;
            printf("enable_monochrome_processing = blue\n");
         }
         // no coalesce
         else if( strcmp(s,"-noc") == 0 )
         {
            enable_coalesce = false;
            printf("disable_coalesce\n");
         }
         // sequence parameters
         else if( strcmp(s,"-inv") == 0 )
         {
            show_inverse = true;
         }
         else if( strcmp(s,"-noinv") == 0 )
         {
            show_inverse = false;
         }
         else if( strcmp(s,"-x") == 0 )
         {
            show_x = true;
         }
         else if( strcmp(s,"-nox") == 0 )
         {
            show_x = false;
         }
         else if( strcmp(s,"-y") == 0 )
         {
            show_y = true;
         }
         else if( strcmp(s,"-noy") == 0 )
         {
            show_y = false;
         }
         
         // output format
         else if( strcmp(s,"-vis") == 0 )
         {
            save_preview = true;
         }
         else if( strcmp(s,"-ply") == 0 )
         {
            save_ply = true;
            // also turn off yaml output
            save_yaml = false;
            save_matches = false;
            enable_coalesce = true;
            printf("enable_coalesce\n");
         }
         else if( strcmp(s,"-noply") == 0 )
         {
            save_ply = false;
         }
         else if( strcmp(s,"-yaml") == 0 )
         {
            save_yaml = true;
         }
         else if( strcmp(s,"-noyaml") == 0 )
         {
            save_yaml = false;
         }
         // do not save correspondence data
         else if( strcmp(s,"-nomatch") == 0 )
         {
            save_matches = false;
            
         }
         else if( strcmp(s,"-nosave") == 0 )
         {
            //                save_file = false;
         }
         else if( strcmp( s, "-o" ) == 0 )
         {
            if( i+1 >= argc )
               return fprintf( stderr, "Output file name not provided\n"), -1;
            outPrefix = argv[++i];
         }
         else if( strcmp( s, "-m" ) == 0 )
         {
            if( i+1 >= argc )
               return fprintf( stderr, "Scan message not provided\n"), -1;
            scanMessage = string(argv[++i]);
         }
         // load all images into memory first
         else if( strcmp(s,"-loadall") == 0 )
         {
            load_all = true;
         }
         // set threshold value
         else if( strcmp(s,"-t") == 0 )
         {
            int v = 5;
            if( i+1 >= argc )
               return fprintf( stderr, "Threshold value not provided\n"), -1;
            if( sscanf( argv[++i], "%d", &v ) != 1 || v < 1 )
               return fprintf( stderr, "Invalid threshold value\n"), -1;
            diff_threshold = v;
         }
         // set truncate value
         else if( strcmp(s,"-b") == 0 )
         {
            int v = 0;
            if( i+1 >= argc )
               return fprintf( stderr, "Truncate value not provided\n"), -1;
            if( sscanf( argv[++i], "%d", &v ) != 1 || v < 0 )
               return fprintf( stderr, "Invalid truncate value\n"), -1;
            truncate_enable = true;
            truncate_value = v;
         }

         // enable chessboard detection
         else if( strcmp(s,"-chess") == 0 )
         {
            int w,h;
            if( i+2 >= argc )
               return fprintf( stderr, "Not enough chessboard parameters provided\n"), -1;
            if( sscanf( argv[++i], "%d", &w) != 1 || w<2 )
               return fprintf( stderr, "Invalid chessboard width\n"), -1;
            if( sscanf( argv[++i], "%d", &h) != 1 || h<2 )
               return fprintf( stderr, "Invalid chessboard height\n"), -1;
            enable_chess = true;
            chess_size.width = w;
            chess_size.height = h;
            printf("looking for chessboard [%dx%d]\n",w,h);
            chess_image_input = true;
         }
         // set chessboard scale
         else if( strcmp(s,"-s") == 0 )
         {
            float v = 1;
            if( i+1 >= argc )
               return fprintf(stderr,"Chessboard scale value not provided\n"), -1;
            if( sscanf( argv[++i], "%f", &v) != 1 || v<=0 )
               return fprintf(stderr,"Invalid chessboard scale value\n"), -1;
            chess_scale = v;
         }
         // set color mapping visualization scale
         else if( strcmp(s,"-vscl") == 0 )
         {
            float v = 1;
            if( i+1 >= argc )
               return fprintf(stderr,"Color mapping visualization scale value not provided\n"), -1;
            if( sscanf( argv[++i], "%f", &v) != 1 || v<=0 )
               return fprintf(stderr,"Invalid color mapping visualization scale value\n"), -1;
            vis_scl = v;
         }
         // set color mapping visualization scale
         else if( strcmp(s,"-vscale") == 0 )
         {
            float v = 1;
            if( i+1 >= argc )
               return fprintf(stderr,"Supersample visualization output scale value not provided\n"), -1;
            if( sscanf( argv[++i], "%f", &v) != 1 || v<=0 )
               return fprintf(stderr,"Invalid supersample scale value\n"), -1;
            vis_supersample = fmin(fmax(v,1.0f),10.0f);
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

         else if( strcmp(s,"-procam") == 0 )
         {
            if( i+1 >= argc )
               return fprintf(stderr,"Projector camera calibration filename not provided\n"),-1;
            enable_procam = true;
            inCalibFN = argv[++i];
         }
         else if( strcmp(s,"-chessimg") == 0 )
         {
            if( i+1 >= argc )
               return fprintf(stderr,"Chessboard image filename not provided\n"),-1;
            
            chess_image_input = true;
            inChessImageFN = argv[++i];
         }
         else if( strcmp(s,"-chessidx") == 0 )
         {
            if( i+1 >= argc )
               return fprintf(stderr,"Chessboard image filename not provided\n"),-1;
            
            chess_image_input = true;
            if( sscanf( argv[++i], "%d", &chess_idx ) != 1 )
            {
                printf("error: invalid chess image idx\n");
            }
         }
         else if( strcmp(s,"-load") == 0 )
         {
            if( i+1 >= argc )
               return fprintf(stderr,"Load point map filename not provided\n"),-1;
            enable_mapload = true;
            inMapFN = argv[++i];
         }
         // file input
         else if( s[0] != '-' ){
            inImgListFN = s;
         }
         else
            return fprintf( stderr, "Unknown option %s", s), -1;
      }
       // args done
      
       string filename(inImgListFN);
       string prefix;
       size_t found = filename.find_last_of("/");
       if( found == string::npos )
       {
          prefix = filename;
       }
       else
       {
          prefix = filename.substr(0,found);
       }

       printf("PREFIX FOR OUTPUT : [%s]\n",prefix.c_str());
       
       // generate names for all output files
        char* filepattern = (char*)calloc(100,sizeof(char));
        if( outPrefix )
        {
            sprintf(filepattern,"%s%%s%%s",outPrefix);
        } else {
            sprintf(filepattern,"%%s%%s");
        }
        char* outMapPath         = (char*)calloc(100,sizeof(char));
        char* outVisCPath        = (char*)calloc(100,sizeof(char));
        char* outVisXPath        = (char*)calloc(100,sizeof(char));
        char* outVisYPath        = (char*)calloc(100,sizeof(char));
        char* outVisDirPath      = (char*)calloc(100,sizeof(char));
        char* outVisAmbPath      = (char*)calloc(100,sizeof(char));
        char* outVisChessCamPath = (char*)calloc(100,sizeof(char));
        char* outVisChessProPath = (char*)calloc(100,sizeof(char));
        char* outPlyPath         = (char*)calloc(100,sizeof(char));

        sprintf(outMapPath,        filepattern, prefix.c_str(), ".yaml");
        sprintf(outVisCPath,       filepattern, prefix.c_str(), "_rgb.png");
        sprintf(outVisXPath,       filepattern, prefix.c_str(), "_x.png");
        sprintf(outVisYPath,       filepattern, prefix.c_str(), "_y.png");
        sprintf(outVisDirPath,     filepattern, prefix.c_str(), "_direct.png");
        sprintf(outVisAmbPath,     filepattern, prefix.c_str(), "_indirect.png");
        sprintf(outVisChessCamPath,filepattern, prefix.c_str(), "_chesscam.png");
        sprintf(outVisChessProPath,filepattern, prefix.c_str(), "_chesspro.png");
        sprintf(outPlyPath,        filepattern, prefix.c_str(), "_cloud.ply");

       cout << "outMapPath\t" << outMapPath << endl;
       cout << "outVisCPath\t" << outVisCPath << endl;
       cout << "outVisXPath\t" << outVisXPath << endl;
       cout << "outVisYPath\t" << outVisYPath << endl;
       cout << "outVisDirPath\t" << outVisDirPath << endl;
       cout << "outVisAmbPath\t" << outVisAmbPath << endl;
       cout << "outVisChessCamPath\t" << outVisChessCamPath << endl;
       cout << "outVisChessProPath\t" << outVisChessProPath << endl;
       cout << "outPlyPath\t" << outPlyPath << endl;

       if(true)
       {
            outMapFN      = outMapPath;        
            outViscFN     = outVisCPath;
            outVisxFN     = outVisXPath;
            outVisyFN     = outVisYPath;
            outVisDirFN   = outVisDirPath;
            outVisAmbFN   = outVisAmbPath;
            outChessCamFN = outVisChessCamPath;
            outChessProFN = outVisChessProPath;
            outPlyFN      = outPlyPath;
       }
       

       

      // touch files
      FILE *f_chesscam, *f_chesspro, *f_yaml, *f_ply;
      if( enable_chess )
      {
         f_chesscam = fopen(outChessCamFN,"w"); fclose(f_chesscam);
         f_chesspro = fopen(outChessProFN,"w"); fclose(f_chesspro);
      }
      
      if( save_yaml )
      {
         f_yaml     = fopen(outMapFN,"w"); fclose(f_yaml);
      }
      
      if( save_ply )
      {
         f_ply      = fopen(outPlyFN,"w"); fclose(f_ply);
      }
      
      
      // load file list
      if( inImgListFN ){
         
         string fn(inImgListFN);
         string fn_auto;
         
         // if passed a directory, check for a list.yaml automatically
         if( is_dir(inImgListFN) )
         {
            if( fn.find_last_of("/") == fn.length()-1 )
            {
               fn_auto = fn + "sequence.yaml";
            }
            else
            {
               fn_auto = fn + "/sequence.yaml";
            }
         }
         
         printf("checking image list: %s\n", inImgListFN);
         
         bool listLoaded = false;
         
         listLoaded = parseStringList(fn.c_str(), gcbFileList, rgbFileList);
         if( listLoaded && ( gcbFileList.size() > 0 || rgbFileList.size() > 0 ) )
         {
            mode = CAPTURING;
            printf("Using image list: %s\n", inImgListFN);
         }
         else
         {
             listLoaded = parseStringList( fn_auto.c_str(), gcbFileList, rgbFileList );
             if( listLoaded && ( gcbFileList.size() > 0 || rgbFileList.size() > 0 ) )
             {
                mode = CAPTURING;
                printf("Using image list: %s\n", inImgListFN);
                inImgListFN = fn_auto.c_str();
             }
             else
             {
                // give up
                printf("invalid image list specified, exiting..\n");
                return 1;
             }
         }

         /*
         // try parsing the actual path
         if( parseStringList(fn.c_str(), gcbFileList, rgbFileList) ){
            mode = CAPTURING;
            printf("Using image list: %s\n", inImgListFN);
            
         }
         // try parsing an auto-guess of path + "list.yaml"
         else if( parseStringList( fn_auto.c_str(), gcbFileList, rgbFileList ) )
         {
            inImgListFN = fn_auto.c_str();
            mode = CAPTURING;
            printf("Using image list: %s\n", inImgListFN);
         
         }
         // give up
         else
         {
            printf("invalid image list specified, exiting..\n");
            return 1;
         }
         */
      } else {
         printf("no file specified, exiting..\n");
         return 1;
      }
      
      // load exif data
      {
         printf("loading exif data..\n");
         getExifList( gcbFileList, exifList );
         printf("done!\n");
      }
      
      if( !scanMessage.empty() ){
         printf("scan message: \"%s\"\n",scanMessage.c_str());
      }
      
      // detect invalid options
      // exit if neither x or y data is selected
      if( !show_x && !show_y ){
         printf("neither x or y axis configured, exiting..\n");
         return 1;
      }
      
      // calculate # of bitfields
      {
         mult = 1;
         if(show_inverse)     mult *= 2;
         if(show_x && show_y) mult *= 2;
         bits = gcbFileList.size() / mult;
      }
      
      // fail if unexpected number of images
      if( mult * bits != gcbFileList.size() ){
         printf("invalid number of images, exiting..\n");
         return 1;
      }
      
      // fail on odd number of input images
      if( gcbFileList.size() % 2 == 1 ){
         if( show_inverse || (show_x && show_y) ){
            printf("odd number of images, exiting..\n");
            return 1;
         }
      }
      
   }
   
   // ------------- program start -------------
   
   // discover image info
   //    printf("checking image dimensions (%s).. ",gcbFileList[0].c_str());
   
   Mat thumb, thumb_i;
   Mat image = imread(gcbFileList[0]);
   
   if( load_all ){
      printf("loading all images..\n");
      for(int i=0; i<gcbFileList.size(); i++){
         Mat img = imread(gcbFileList[i]);
         printf("\t%s\n",gcbFileList[i].c_str());
         imageList.push_back(img);
      }
   }
   //    string cam_make = exifList[0].find("Make")->second;
   string cam_model;
   string lens_model;
   
   //    printf("make = %s\n",cam_make.c_str());
   
   Size image_size = image.size();
   //    image.release();
   
   // maps
   
   // camera space projector data
   //    Mat cam_proj_map_gray(image_size,CV_16UC2,Scalar::all(0));
   //    Mat cam_proj_map(image_size,CV_16UC2,Scalar::all(0));
   
   Mat row_map_gray(image_size,CV_16UC1,Scalar::all(0));
   Mat col_map_gray(image_size,CV_16UC1,Scalar::all(0));
   Mat row_map_bin(image_size,CV_16UC1,Scalar::all(0));
   Mat col_map_bin(image_size,CV_16UC1,Scalar::all(0));
   
   // projector space camera data
   Mat row_map_avg;
   Mat col_map_avg;
   Mat rgb_map_avg;
   Mat rgb_map_tmp(image_size,CV_32FC4,Scalar::all(0));
   Mat bit_map_tmp(image_size,CV_8UC1,Scalar::all(0));
   
   // <Vec4f>
   // camera pixel colors from x or y sequences
   Mat rgb_map_x(image_size,CV_32FC4,Scalar::all(0));
   Mat rgb_map_y(image_size,CV_32FC4,Scalar::all(0));
   
   Mat xdiff_map(1,bits,CV_64FC2,Scalar::all(0));
   Mat ydiff_map(1,bits,CV_64FC2,Scalar::all(0));
   
   // per pixel min/max
   Mat max_map(image_size,CV_8UC3,Scalar::all(0));
   Mat min_map(image_size,CV_8UC3,Scalar::all(255));
   
   Mat gray_chess;
   Mat color_chess;
   
   /////////////////
   // output vectors
   
   // decoded matches
   vector< Point2f > projector_points;  // decoded matches in projector space (xy)
   vector< Point2f > camera_points;     // decoded matches in camera space (xy)
   vector< Point3f > camera_points_rgb; // decoded matches in camera space (rgb)
   
   vector< Point2f > projector_points_avg;  // coalesced matches in projector space (xy)
   vector< Point2f > camera_points_avg;     // coalesced matches in camera space (xy)
   vector< Point3f > camera_points_rgb_avg; // coalesced matches in camera space (rgb)
   
   // chess corners
   vector< Point3f > object_corners;    // chess corners in world space (xyz)
   vector< Point2f > chess_corners;     // chess corners in camera space (xy)
   vector< Point2f > projector_corners; // chess corners in projector space (xy)
   
   printf("---------------------\n");
   printf("show inverse: %s\n", show_inverse ? "√" : "no" );
   printf("show x-axis:  %s\n", show_x ? "√" : "no" );
   printf("show y-axis:  %s\n", show_y ? "√" : "no" );
   printf("bits:   %2d (2^%d = %d)\n", bits, bits, (int)std::pow(2.0f,bits) );
   printf("mult:   %2d\n", mult);
   printf("images: %2d\n", (int)gcbFileList.size());
   printf("---------------------\n");
   printf("image size = %d x %d\n", image_size.width, image_size.height);
   printf("threshold = %d\n",diff_threshold);
   printf("---------------------\n");
   
   // -----------------------------------------------------------------
   // load projector camera calibration if needed
   
   Mat pro_K, pro_kc, pro_P, pro_R;
   Mat cam_K, cam_kc, cam_P, cam_R;
   Mat campro_R, campro_T;
   
   // check projector-camera calibration early
   if( enable_procam )
   {
      FileStorage fs;
      fs.open(inCalibFN, FileStorage::READ);
      
      if (!fs.isOpened())
         return fprintf(stderr,"failed to open procam file [%s]\n",inCalibFN), -1;
      
      fs["cam_K"] >> cam_K;
      fs["pro_K"] >> pro_K;
      
      fs["cam_kc"] >> cam_kc;
      fs["pro_kc"] >> pro_kc;
      
      fs["P1"] >> cam_P; // camera    projection matrix
      fs["P2"] >> pro_P; // projector projection matrix
      
      fs["R1"] >> cam_R; // camera    rectification matrix
      fs["R2"] >> pro_R; // projector rectification matrix
      
      fs["R"] >> campro_R;
      fs["T"] >> campro_T;
      
      if( !cam_P.data || !pro_P.data )
      {
         return fprintf(stderr,"failed to load procam values\n") -1;
      }
      
   }
   
   // CHESS ONLY
   // check chessboard detection early
   bool chess_found = false;
   if( chess_image_input ) {
      
//      color_chess = imread(inChessImageFN);
      color_chess = imread(gcbFileList[chess_idx]);
      if( !color_chess.data )
      {
         return fprintf(stderr,"error loading chessboard pattern image\n"), -1;
      }
      cvtColor(color_chess,gray_chess,COLOR_BGR2GRAY);
      
      
      Mat small_chess;
      Size target_size(2560,1600);
      float ratio = min((target_size.width )/(float)color_chess.cols,
                        (target_size.height)/(float)color_chess.rows);
      resize( gray_chess, small_chess, Size(), ratio, ratio, INTER_AREA );
      
//      imshow("chess",small_chess);
//      waitKey(0);
      cout << gcbFileList[chess_idx] << endl;
      
      printf("looking for chessboard...\n");
      chess_found = findChessboardCorners(small_chess,chess_size,chess_corners,
                                          CALIB_CB_ADAPTIVE_THRESH);// | CALIB_CB_NORMALIZE_IMAGE);
      
      if( chess_found ){
         printf("success\n");
         for(int i=0; i<chess_corners.size(); i++)
         {
            chess_corners[i].x /= ratio;
            chess_corners[i].y /= ratio;
         }
         
         int n = 0;
         float avg_edge_len = 0;
         int chx = chess_size.width;
         int chy = chess_size.height;
         for(int r=0; r<chy-1; r++)
         {
             for(int c=0; c<chx-1; c++)
             {
                 Point2f p1(chess_corners[r*chx+c]);
                 Point2f pX(chess_corners[r*chx+c+1]);
                 Point2f pY(chess_corners[(r+1)*chx+c]);
                 float d1 = norm(p1-pX);
                 float d2 = norm(p1-pY);
                 avg_edge_len += d1;
                 n++;
                 avg_edge_len += d2;
                 n++;
             }
         }
         avg_edge_len /= n;
         
         int subpix_window = static_cast<int>(fmax(avg_edge_len*0.85*0.5,5));
         printf("***** avg_edge_len = %0.4f ***** subpix_window = %d *****\n",avg_edge_len,subpix_window);

         cornerSubPix( gray_chess, chess_corners, Size(subpix_window,subpix_window), Size(-1,-1),
                      TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 500, 0.01 ));
         //cornerSubPix( gray_chess, chess_corners, Size(5,5), Size(1,1),
         //             TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 500, 0.025 ));
         
         drawChessboardCorners( color_chess, chess_size, Mat(chess_corners), chess_found );
         //            imshow("img",color_chess);
//         imwrite("chess_corners.png",color_chess);
         //            waitKey(0);
      } else {
         printf("trying full res..\n");
         chess_found = findChessboardCorners(gray_chess,chess_size,chess_corners,
                 CALIB_CB_ADAPTIVE_THRESH);
         //cornerSubPix( gray_chess, chess_corners, Size(9,9), Size(-1,-1),
          //            TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 30, 0.1 ));
         drawChessboardCorners( color_chess, chess_size, Mat(chess_corners), chess_found );

         if( chess_found ){
            printf("success\n");

         } else {
            printf("FAILED TO FIND CHESSBOARD, exiting..\n");
            return -1;
         }
      }
      
   }
   
   // -----------------------------------------------------------------
   // begin parsing images
   //
   
   // projector space x/y min/max values
   double x_min = -1, x_max = -1;
   double y_min = -1, y_max = -1;
   
   // counting decoded pixels
   int tot_pix = image_size.height*image_size.width;
   int tot_x_pix = 0;
   int tot_y_pix = 0;
   
   Mat tmp_img;
   
   // load patterns
   if(enable_mapload)
   {
      FileStorage fs(inMapFN, FileStorage::READ);
      if (!fs.isOpened())
         return fprintf(stderr,"failed to open [%s]",inMapFN), -1;
      
      Mat cam_dims;
      fs["cam_size"] >> cam_dims;
      
      Mat cam_points, pro_points;
      fs["cam_points"] >> cam_points;
      fs["pro_points"] >> pro_points;
      
      camera_points    = vector< Point2f >(cam_points);
      projector_points = vector< Point2f >(pro_points);
      
      enable_coalesce = false;
   }
   // decode patterns
   else
   {
      printf("completed bitfields: \n");
      fflush(stdout);
      
      // rgb and bit map tmp
      rgb_map_tmp.setTo(Scalar::all(0));
      bit_map_tmp.setTo(Scalar::all(0));

      
      // parse bitfields from image sequence into row_map_gray, col_map_gray
      // patterns are input in order MSB=0, LSB=N
      for( int bitfield=0; bitfield<bits; bitfield++ ){
         int x_idx0 = bitfield*2;
         int x_idx1 = bitfield*2+1;
         
         int y_idx0 = bits*2+bitfield*2;
         int y_idx1 = bits*2+bitfield*2+1;
         
         Mat x_pattern;
         Mat x_pattern_i;
         
         Mat y_pattern;
         Mat y_pattern_i;
         
         // load images
         {
            if( show_x ){
               if(load_all){
                  x_pattern   = imageList[x_idx0];
                  x_pattern_i = imageList[x_idx1];
               } else {
                  x_pattern   = imread(gcbFileList[x_idx0]);
                  x_pattern_i = imread(gcbFileList[x_idx1]);
               }
            }
            if( show_y ){
               if(load_all){
                  y_pattern   = imageList[y_idx0];
                  y_pattern_i = imageList[y_idx1];
               } else {
                  y_pattern   = imread(gcbFileList[y_idx0]);
                  y_pattern_i = imread(gcbFileList[y_idx1]);
               }
            }
         }
         
         bool new_way = true;
         if( !new_way )
         {
            //            parseImagePair expects MSB -> LSB bitfield order
            //            parseImagePair decodes and merges new bitfield data
            if( show_x )
               parseImagePair( x_pattern, x_pattern_i, col_map_gray, rgb_map_x, xdiff_map, bitfield, diff_threshold );
            
            if( show_y )
               parseImagePair( y_pattern, y_pattern_i, row_map_gray, rgb_map_y, ydiff_map, bitfield, diff_threshold );
            
            
         }
         // new way with truncation support
         else {
            // mergeNewBit expects bitfield parameter as LSB=0
            
            int minbit = truncate_value; // allow per-pixel decode failure beyond this number of successfully decoded bits
            
            if( show_x )
            {
               //    Mat row_map_gray(image_size,CV_16UC1,Scalar::all(0));
               //rgb_map_tmp Vec4f
               
               double ret = 0;
               // decode image pair into bit
               if( enable_monochrome_processing )
               {
                   vector<Mat> x_chs,xi_chs;
                   split(x_pattern,   x_chs);
                   split(x_pattern_i,xi_chs);
                   ret = decodeImagePair(x_chs[monochrome_channel_idx], xi_chs[monochrome_channel_idx], rgb_map_tmp, bit_map_tmp, diff_threshold);
               }
               else
               {
                   ret = decodeImagePair(x_pattern, x_pattern_i, rgb_map_tmp, bit_map_tmp, diff_threshold);
               }

               
               //printf("x bit [%d] : diff = %0.3f",bitfield, ret);
               //fflush(stdout);
               
               //            cvtColor(bit_map_tmp,tmp_img,CV_GRAY2BGR);
               //imshow("bit",bit_map_tmp);
//               waitKey(0);
               
               // merge new bit into x graycode map
					int bit_idx = bits-1-bitfield;
               mergeNewBit(bit_map_tmp, col_map_gray, bit_idx, minbit);
					printf("[%2d] MERGING NEW X BITFIELD : [bitfield %2d] [minbit %d] [diff = %0.3f]",bitfield, bit_idx, minbit, ret);
					if( show_y )
						printf(" ");

            }
            
            if( show_y )
            {
               //    Mat row_map_gray(image_size,CV_16UC1,Scalar::all(0));
               
               double ret = 0;
               // decode image pair into bit
               if( enable_monochrome_processing )
               {
                   vector<Mat> y_chs,yi_chs;
                   split(y_pattern,   y_chs);
                   split(y_pattern_i,yi_chs);
                   ret = decodeImagePair(y_chs[monochrome_channel_idx], yi_chs[monochrome_channel_idx], rgb_map_tmp, bit_map_tmp, diff_threshold);
               }
               else
               {
                   ret = decodeImagePair(y_pattern, y_pattern_i, rgb_map_tmp, bit_map_tmp, diff_threshold);
               }

               // decode image pair into bit
               //double ret = decodeImagePair(y_pattern, y_pattern_i, rgb_map_tmp, bit_map_tmp, 5);
               
              // printf("|  y bit [%d] : diff = %0.3f\n",bitfield, ret);
               //fflush(stdout);
               
               //imshow("bit",bit_map_tmp);
//               waitKey(0);
               
               
               // merge new bit into y graycode map
					int bit_idx = bits-1-bitfield;					
               mergeNewBit(bit_map_tmp, row_map_gray, bit_idx, minbit);
					printf("[%2d] MERGING NEW Y BITFIELD : [bitfield %2d] [minbit %d] [diff = %0.3f]",bitfield, bit_idx, minbit, ret);


            }
				if( show_x || show_y )
				{
					printf("\n");
				}
         }
      
         
         
         // add bitfield into decoded bits
         
         
         //if(bitfield>0&&bitfield<bits)
         //   printf(", ");
         //printf("%d",bitfield);
         //fflush(stdout);
         
      }
      // image sequence -> binary gray code COMPLETE
      // -------------------------------------------
      
      rgb_map_tmp.copyTo(rgb_map_x);
      rgb_map_tmp.copyTo(rgb_map_y);
      
      // sequence decoding done
      printf("\n");
      
      // convert gray code to binary (invalid=UINT16_MAX)
      // row_map_gray -> row_map_bin
      // col_map_gray -> col_map_bin
      for(int r = 0; r < image_size.height; r++){
         ushort* row_gray = row_map_gray.ptr<ushort>(r);
         ushort* col_gray = col_map_gray.ptr<ushort>(r);
         ushort* row_bin  = row_map_bin.ptr<ushort>(r);
         ushort* col_bin  = col_map_bin.ptr<ushort>(r);
         
         for(int c = 0; c < image_size.width; c++){
            // count valid pixels
            if( row_gray[c] != UINT16_MAX )
               tot_y_pix++;
            
            if( col_gray[c] != UINT16_MAX )
               tot_x_pix++;
            
            // invalidate other coordinate
            if( row_gray[c] == UINT16_MAX || col_gray[c] == UINT16_MAX){
               row_gray[c] = UINT16_MAX;
               col_gray[c] = UINT16_MAX;
               row_bin[c]  = UINT16_MAX;
               col_bin[c]  = UINT16_MAX;
               continue;
            }
            
            row_bin[c] = gray2bin(row_gray[c]);
            col_bin[c] = gray2bin(col_gray[c]);

            if(truncate_value>0)
            {
               ushort b = 1<<max(truncate_value-1,0);
               row_bin[c] &= (~b);
               col_bin[c] &= (~b);
               //row_bin[c] |= b;
               //col_bin[c] |= b;
               //row_bin[c] = row_bin[c]>>truncate_value;
               //col_bin[c] = col_bin[c]>>truncate_value;
            }
            
         }
      }
      
      // collect matches into output vectors of coordinates
      // row_map_bin, col_map_bin -> vector< Point2f > camera_points, projector_points
      for(int r=0; r<image_size.height; r++){
         const ushort* row_px_bin = row_map_bin.ptr<ushort>(r);
         const ushort* col_px_bin = col_map_bin.ptr<ushort>(r);
         for(int c=0; c<image_size.width; c++){
            
            if( col_px_bin[c] == UINT16_MAX || row_px_bin[c] == UINT16_MAX )
               continue;
            
            Point2f cam_pt(c,r);
            Point2f pro_pt(col_px_bin[c],row_px_bin[c]);
            
            camera_points.push_back( cam_pt );
            projector_points.push_back( pro_pt );
         }
      }
      
      CV_Assert( camera_points.size() == projector_points.size() );
      
      // collect rgb data into output vector
      // vector<Point2f> camera_points, rgb_map_tmp -> vector<Point3f> camera_points_rgb
      for(int i=0; i<camera_points.size(); i++){
         Point2i pt((int)(camera_points[i].x),(int)(camera_points[i].y));
         Vec4f rgb_in = rgb_map_tmp.at<Vec4f>(pt.y,pt.x);
         
         Point3f new_col;
         new_col.x = fmin(fmax(rgb_in[0]/fmax(rgb_in[3],1),0),255);
         new_col.y = fmin(fmax(rgb_in[1]/fmax(rgb_in[3],1),0),255);
         new_col.z = fmin(fmax(rgb_in[2]/fmax(rgb_in[3],1),0),255);

         camera_points_rgb.push_back(new_col);

//         Vec4f x_rgb = rgb_map_x.at<Vec4f>(pt.y,pt.x);
//         Vec4f y_rgb = rgb_map_y.at<Vec4f>(pt.y,pt.x);
//         Vec4f src_col;
//         if(show_x && show_y){
//            src_col = x_rgb + y_rgb;
//         } else {
//            if(show_x)
//               src_col = x_rgb;
//            if(show_y)
//               src_col = y_rgb;
//         }
//         Point3f new_col;
//         if( src_col[3] > 1 ){
//            new_col.x = src_col[0]/src_col[3];
//            new_col.y = src_col[1]/src_col[3];
//            new_col.z = src_col[2]/src_col[3];
//         } else {
//            new_col.x = src_col[0];
//            new_col.y = src_col[1];
//            new_col.z = src_col[2];
//         }
         
//         new_col.x = new_col.x;// fmin(fmax(new_col.x,0),255);
//         new_col.y = new_col.y;// fmin(fmax(new_col.y,0),255);
//         new_col.z = new_col.z;// fmin(fmax(new_col.z,0),255);
         
      }
      
      printf("camera_points.size() = %d\n",(int)camera_points.size());
      printf("camera_points_rgb() = %d\n",(int)camera_points_rgb.size());
      
      // sanity check
      CV_Assert( projector_points.size() == camera_points.size() );
      CV_Assert( projector_points.size() == camera_points_rgb.size() );
      
      // find min/max object space values
      Mat minmax_mask = (row_map_bin!=UINT16_MAX);
      minMaxLoc(row_map_bin,&y_min,&y_max,0,0,minmax_mask);
      minMaxLoc(col_map_bin,&x_min,&x_max,0,0,minmax_mask);
      
      if(x_min<0 || x_max<0 || y_min<0 || y_max<0){
         printf("ERROR: MINMAX [ %0.1f, %0.1f, %0.1f, %0.1f ]\n",
                x_min,x_max,y_min,y_max);
      }
      
      /////////////////
      // COALESCE
      
      // for each unique projector-camera codeword match, compute average of decoded camera space pixels
      // vector<Point2f> camera_points, projector_points -> vector<Point2f> projector_points_avg, camera_points_avg
      if( enable_coalesce ){
         printf("coalesce pass..\n");
         
         int obj_w = static_cast<int>( x_max )+1;
         int obj_h = static_cast<int>( y_max )+1;
         
         // projector space data
         rgb_map_avg.create(obj_h,obj_w,CV_32FC4); // rgb data
         row_map_avg.create(obj_h,obj_w,CV_32FC2); // y coordinate average
         col_map_avg.create(obj_h,obj_w,CV_32FC2); // x coordinate average
         
         rgb_map_avg.setTo(Scalar::all(0));
         row_map_avg.setTo(Scalar::all(0));
         col_map_avg.setTo(Scalar::all(0));
         
         // sum camera coordinates per decoded projector pixel
         // sum RGB data per decoded projector pixel
         for(int i=0; i<camera_points.size(); i++){
            Point2i cam_pt(static_cast<int>(camera_points[i].x),   static_cast<int>(camera_points[i].y));
            Point2i pro_pt(static_cast<int>(projector_points[i].x),static_cast<int>(projector_points[i].y));
            Point3f rgb_pt = camera_points_rgb[i];
            
            rgb_map_avg.at<Vec4f>(pro_pt.y,pro_pt.x) += Vec4f( rgb_pt.x, rgb_pt.y, rgb_pt.z, 1.0f );
//            if( show_x )
//               rgb_map_avg.at<Vec4f>(pro_pt.y,pro_pt.x) += rgb_map_x.at<Vec4f>(cam_pt.y,cam_pt.x);
//            if( show_y )
//               rgb_map_avg.at<Vec4f>(pro_pt.y,pro_pt.x) += rgb_map_y.at<Vec4f>(cam_pt.y,cam_pt.x);
            float gray_value = (rgb_pt.x*2 + rgb_pt.y*4 + rgb_pt.z*1 )/(7.0f*255);
            gray_value = min(max(gray_value,0.0f),1.0f);
            //gray_value = pow(gray_value,2.0f);
            //gray_value = 1;
            
            row_map_avg.at<Vec2f>(pro_pt.y,pro_pt.x) += Vec2f(cam_pt.y*gray_value,gray_value);
            col_map_avg.at<Vec2f>(pro_pt.y,pro_pt.x) += Vec2f(cam_pt.x*gray_value,gray_value);
         }

         // print for debug
         for(int i=0; i<camera_points.size(); i++){
            Point2i cam_pt(static_cast<int>(camera_points[i].x),   static_cast<int>(camera_points[i].y));
            Point2i pro_pt(static_cast<int>(projector_points[i].x),static_cast<int>(projector_points[i].y));
            Point3f rgb_pt = camera_points_rgb[i];
            
            Vec4f vec = rgb_map_avg.at<Vec4f>(pro_pt.y,pro_pt.x);
            
            if(DBUG && i<200)
            {
               
               printf("rgb_map_avg[%d] = %0.3f, %0.3f, %0.3f, %0.3f;\n",i,vec[0], vec[1], vec[2], vec[3]);
               printf("camera_points_rgb[%d] = %0.3f, %0.3f, %0.3f, %0.3f;\n",i,vec[0], vec[1], vec[2], vec[3]);
            }
         
         }
         
         // projector space
         // divide to get average and place in vectors
         for(int r=0; r<obj_h; r++){
            Vec2f* row_ptr = row_map_avg.ptr<Vec2f>(r);
            Vec2f* col_ptr = col_map_avg.ptr<Vec2f>(r);
            Vec4f* rgb_ptr = rgb_map_avg.ptr<Vec4f>(r);
            for(int c=0; c<obj_w; c++){
               if( row_ptr[c][1] > 0.5 && col_ptr[c][1] > 0.5){
                  // divide by totals
                  //row_ptr[c][0] /= row_ptr[c][1];
                  //col_ptr[c][0] /= col_ptr[c][1];
                  Point2f pro_pt(c,r);
                  Point2f cam_pt;//(col_ptr[c][0],row_ptr[c][0]);
                  cam_pt.x = col_ptr[c][0] / col_ptr[c][1];
                  cam_pt.y = row_ptr[c][0] / row_ptr[c][1];
                  
                  // get sum and divide by # of samples
                  Vec4f v = rgb_ptr[c];
                  float _r = v[0] / v[3];
                  float _g = v[1] / v[3];
                  float _b = v[2] / v[3];
                  Point3f rgb_pt(_r,_g,_b);
                  
//                  cout << rgb_pt << endl;
                  
                  if( enable_subsamp ){
                     if( subsamp_type == SUBSAMPLE_GRID ){
                        if( (c % subsamp_factor != 0) && (r % subsamp_factor != 0) )
                           continue;
                     } else if ( subsamp_type == SUBSAMPLE_POINT ){
                        if( (c % subsamp_factor != 0) || (r % subsamp_factor != 0) )
                           continue;
                     }
                  }
                  projector_points_avg.push_back(pro_pt);
                  camera_points_avg.push_back(cam_pt);
                  camera_points_rgb_avg.push_back(rgb_pt);
                  
               }
            }
         }
         
//         printf("===================================\n");
//         printf("rgb_map_tmp.size = %d\n",(int)rgb_map_tmp.rows*rgb_map_tmp.cols);
//         for(int i=0; i<100; i++)
//         {
//            Point2i cam_pt(static_cast<int>(camera_points[i].x),   static_cast<int>(camera_points[i].y));
//            Vec4f v = rgb_map_tmp.at<Vec4f>(cam_pt.x,cam_pt.y);
//            printf("\t[%d] = (%f,%f,%f,%f)\n", i, v[0],v[1],v[2],v[3]);
//         }
//         printf("===================================\n");
//
//         
//         printf("***********************************\n");
//         printf("camera_points_rgb_avg.size = %d\n",(int)camera_points_rgb_avg.size());
//         for(int i=0; i<100; i++)
//         {
//            Point3f pt = camera_points_rgb_avg[i];
//            printf("\t[%d] = (%f,%f,%f)\n", i, pt.x,pt.y,pt.z);
//         }
//         printf("***********************************\n");
//         printf("***********************************\n");
//         printf("camera_points_rgb.size = %d\n",(int)camera_points_rgb.size());
//         for(int i=0; i<100; i++)
//         {
//            Point3f pt = camera_points_rgb[i];
//            printf("\t[%d] = (%f,%f,%f)\n", i, pt.x,pt.y,pt.z);
//         }
//         printf("***********************************\n");

         
         printf("object size: [%d, %d]\n",obj_w,obj_h);
         printf("camera_points has        %d elements\n",(int)camera_points.size());
         printf("projector_points has     %d elements\n",(int)projector_points.size());
         printf("camera_points_avg has    %d elements\n",(int)camera_points_avg.size());
         printf("projector_points_avg has %d elements\n",(int)projector_points_avg.size());
         printf("projector space has     ~%d elements\n",obj_w*obj_h);
         printf("coverage is %0.2f%%\n",100*projector_points_avg.size()/(float)(obj_w*obj_h));
         
      }
      
   }
   // patterns fully decoded
   
   // transform chesboard corners into projector space
   // (x,y) -> (x,y,0)
   if( chess_image_input ) {
      
      //        Mat small_chess,small_gray;
      //        Size target_size(1920,1080);
      //        float ratio = min((target_size.width )/(float)color_chess.cols,
      //                          (target_size.height)/(float)color_chess.rows);
      //        resize( color_chess, small_chess, Size(), ratio, ratio, INTER_AREA );
      //        cvtColor(small_chess,small_gray,COLOR_BGR2GRAY);
      
      // generate world space object points from chess size+scale
      for(int r=0; r<chess_size.height; r++){
         for(int c=0; c<chess_size.width; c++){
            object_corners.push_back(Point3f(c*chess_scale,r*chess_scale,0.f));
         }
      }
      
      if(chess_found){
         
         printf("mapping camera points to projector points..\n");
         
         // generate projector space coordinates from camera space chess corners
         // using multiple homography approach from http://mesh.brown.edu/calibration/
         bool chess_transform_success = true;
         for(int i=0; i<chess_corners.size() && chess_transform_success; i++)
         {
            const Point2f &p = chess_corners[i];
            Point2f q;
            
            unsigned HALF_WIDTH = 30;
            uint32_t x_min = HALF_WIDTH;
            uint32_t x_max = color_chess.cols - HALF_WIDTH;
            uint32_t y_min = HALF_WIDTH;
            uint32_t y_max = color_chess.rows - HALF_WIDTH;
            
            vector<Point2f> img_points, proj_points;
            if( x_min < p.x && p.x < x_max && y_min < p.y && p.y < y_max )
//            if (p.x>HALF_WIDTH && p.y>HALF_WIDTH && p.x+HALF_WIDTH<color_chess.cols && p.y+HALF_WIDTH<color_chess.rows)
            {
               for(unsigned r=p.y-HALF_WIDTH; r<p.y+HALF_WIDTH; r++)
               {
                  for(unsigned c=p.x-HALF_WIDTH; c<p.x+HALF_WIDTH; c++)
                  {
                     const uint16_t pat_x = col_map_bin.at<uint16_t>(r,c);
                     const uint16_t pat_y = row_map_bin.at<uint16_t>(r,c);
                     if( pat_x == UINT16_MAX || pat_y == UINT16_MAX )
                     {
                        continue;
                     }
                     
                     img_points.push_back(Point2f(c,r));
                     proj_points.push_back(Point2f(pat_x,pat_y));
                  }
               }
               
               //                    printf("corner %d : %d img points, %d proj points\n",
               //                           i,
               //                           (int)img_points.size(),
               //                           (int)proj_points.size());
               printf("img_points.size = %d, proj_points.size = %d\n",(int)img_points.size(),(int)proj_points.size());
               if( img_points.size() < 4 )
               {
                  printf("**** degenerate chessboard scan! CHESS DATA OUTPUT DISABLED ****\n");
                  chess_transform_success = false;
                  chess_image_input = false;
                  chess_found = false;
                  continue;
               }
               Mat H = findHomography(img_points, proj_points);
               Mat P = Mat(Point3d(p.x,p.y,1));
               //                    printf("P[%d x %d] (depth=%d) (ch=%d)\n",P.rows,P.cols,P.depth(),P.channels());
               //                    printf("H[%d x %d] (depth=%d) (ch=%d)\n",H.rows,H.cols,H.depth(),H.channels());
               //                    cout << "H:\n" << H << endl;
               //                    printf("pre-gemm....");
               fflush(stdout);
               Point3d Q = Point3d(Mat(  H * P  ));
               //                    printf("post-gemm....\n");
               
               q = Point2f(Q.x/Q.z, Q.y/Q.z);
               //                    cout << "q: " << q << endl;
            } else {
               printf("BREAKING FROM CHESSBOARD PROJECTOR MAPPING\n");
               break;
            }
            
            projector_corners.push_back(q);
            //                i++;
         }
         
         if( chess_transform_success )
         {
            printf("done\n");
            CV_Assert( object_corners.size() == projector_corners.size() );
            
            int obj_w = static_cast<int>( x_max )+1;
            int obj_h = static_cast<int>( y_max )+1;
            
            if( enable_prosize_input )
            {
                //pro_size_input
                    //base_pro_size
               //obj_w = fmax( pro_size_input.width,  obj_w );
               //obj_h = fmax( pro_size_input.height, obj_h );

               obj_w = pro_size_input.width;
               obj_h = pro_size_input.height;
            }
            
            // visualize chess corners
            float scl = 4;
            Size bigsize(image_size.width*scl,image_size.height*scl);
            Mat camchessvis_big(bigsize,CV_8UC3,Scalar::all(0));
            Mat camchessvis(image_size,CV_8UC3,Scalar::all(0));
            Mat prochessvis(obj_h,obj_w,CV_8UC3,Scalar::all(0));
            
            // chess from camera's POV
            resize( color_chess, camchessvis_big, bigsize, 0, 0, INTER_AREA );
                        if( false ){
               //            for(int i=0; i<chess_corners.size(); i++){
               //               Scalar col(0,0,255);
               //               Scalar col2 = float2Color(i/(float)(chess_corners.size()-1));
               //               int s = 8*scl;
               //               Point2f dx(1,0);
               //               Point2f dy(0,1);
               //               Point2f p = chess_corners[i]*scl;
               //               circle(camchessvis_big,p,s,col2,1*scl);
               //               line(camchessvis_big,p-dx*s,p-dx*s*0.5,col2,1*scl);
               //               line(camchessvis_big,p+dx*s,p+dx*s*0.5,col2,1*scl);
               //               line(camchessvis_big,p-dy*s,p-dy*s*0.5,col2,1*scl);
               //               line(camchessvis_big,p+dy*s,p+dy*s*0.5,col2,1*scl);
               //               // inter-circle line
               //               if(i<chess_corners.size()-1)
               //               {
               //                  Point2f q = chess_corners[i+1]*scl;
               //                  line(camchessvis_big,p,q,col2,1*scl);
               //               }
               //            }
            }
            // chess from projector's POV
            resize( camchessvis_big, camchessvis, image_size, 0, 0, INTER_AREA );
            
            
            drawChessboardCorners( prochessvis, chess_size, Mat(projector_corners), chess_found );
            if(false){
            // draw chess points and connecting lines
            for(int i=0; i<projector_corners.size(); i++){
               Scalar col(0,0,255);
               
               Vec3b col3 = float2Color(i/(float)(chess_corners.size()-1));
               Scalar col2(col3[0],col3[1],col3[2]);
               int s = 8;
               Point2f dx(1,0);
               Point2f dy(0,1);
               Point2f p = projector_corners[i];
               circle(prochessvis,p,s,col2,1);
               line(prochessvis,p-dx*s,p-dx*s*0.5,col2,1);
               line(prochessvis,p+dx*s,p+dx*s*0.5,col2,1);
               line(prochessvis,p-dy*s,p-dy*s*0.5,col2,1);
               line(prochessvis,p+dy*s,p+dy*s*0.5,col2,1);
               
               //                line(prochessvis,p-dx*s,p+dx*s,col2,1);
               //                line(prochessvis,p-dy*s,p+dy*s,col2,1);
            }
         }
            
            //            cvtColor(camchessvis_alt,camchessvis_gray,COLOR_BGR2GRAY);
            
            //            color_chess.copyTo(camchessvis);
            //            camchessvis_alt.copyTo(camchessvis,camchessvis_gray>0);
            
            //         imwrite("chessvis_cam.png",camchessvis);
            //         imwrite("chessvis_pro.png",prochessvis);
            
            imwrite(outChessCamFN,camchessvis);
            imwrite(outChessProFN,prochessvis);
            
            
            printf("    projector_corners : %d\n",(int)projector_corners.size());
            printf("        chess_corners : %d\n",(int)chess_corners.size());
            
            //            imshow("img",color_chess);
            //            waitKey(0);
         }
      }
      
   }
   
   // downmix into output structures
   if( save_file ){
      
      // print coverage info
      if( !enable_mapload )
      {
         printf(    "total pixels: ( %8d px )\n", tot_pix);
         if( show_x )
            printf("   x decoded: ( %8d px = %6.2f%% ) range: [%d,%d] w = %4d\n", tot_x_pix, 100*tot_x_pix/(float)tot_pix, (int)x_min, (int)x_max, (int)(1+x_max-x_min));
         if( show_y )
            printf("   y decoded: ( %8d px = %6.2f%% ) range: [%d,%d] h = %4d\n", tot_y_pix, 100*tot_y_pix/(float)tot_pix, (int)y_min, (int)y_max,(int)(1+y_max-y_min));
         
         printf("coverage details ----\n");
         printf("     ");
         if(show_x)
            printf("[               x                ]");
         if(show_x&&show_y)
            printf(" ");
         if(show_y)
            printf("[               y                ]");
         printf("\n");
         
         printf(" bit ");
         if(show_x)
            printf("[    decoded     fill%%  avg diff ]");
         if(show_x&&show_y)
            printf(" ");
         if(show_y)
            printf("[    decoded     fill%%  avg diff ]");
         if(show_x&&show_y)
            printf(" bit");
         printf("\n");
         
         for (int i=0; i<bits; i++) {
            printf("[%2d] ",i);
            if( show_x ){
               const double* xd = xdiff_map.ptr<double>(0);
               double avg_diff = xd[2*i] / xd[2*i+1];
               double percent = 100 * xd[2*i+1] / tot_pix;
               printf("[ %8d px  %6.2f%%  %7.3f  ]", (int)xd[2*i+1], percent, avg_diff );
            }
            
            if( show_x && show_y )
               printf(" ");
            
            if( show_y ){
               const double* yd = ydiff_map.ptr<double>(0);
               double avg_diff = yd[2*i] / yd[2*i+1];
               double percent = 100 * yd[2*i+1] / tot_pix;
               printf("[ %8d px  %6.2f%%  %7.3f  ]", (int)yd[2*i+1], percent, avg_diff );
            }
            
            if( show_x && show_y )
               printf(" [%2d]",i);
            printf("\n");
         }
         printf("---------------------\n");
      }
      
      Mat cam_pts;
      Mat pro_pts;
      Mat cam_pts_rgb;
      
      // do final sanity checks
      // format data into cv::Mat
      if( enable_coalesce ){
         CV_Assert( camera_points_avg.size() == projector_points_avg.size() );
         CV_Assert( projector_points_avg.size() == camera_points_rgb_avg.size() );
         
         cam_pts  = Mat(camera_points_avg);
         pro_pts = Mat(projector_points_avg);
         cam_pts_rgb = Mat(camera_points_rgb_avg);
      } else {
         CV_Assert( camera_points.size() == projector_points.size() );
         CV_Assert( projector_points.size() == camera_points_rgb.size() );
         
         cam_pts  = Mat(camera_points);
         pro_pts = Mat(projector_points);
         cam_pts_rgb = Mat(camera_points_rgb);
      }
      
      // print final processed point count
      printf(" %d total points = %0.2f%%\n", (int)camera_points.size(), 100*camera_points.size()/(float)tot_pix );
      if(enable_coalesce)
      {
         printf(" %d total points = %0.2f%% (%0.2f%%) (coalesced)\n",
                (int)camera_points_avg.size(), 100*camera_points_avg.size()/(float)tot_pix,
                100*camera_points_avg.size()/(float)camera_points.size());
      }
      
      /*
       //subsampling
       if( enable_subsamp && !enable_coalesce ){
       if( subsamp_type == SUBSAMPLE_GRID ){
       if( (c % subsamp_factor != 0) && (r % subsamp_factor != 0) )
       continue;
       } else if ( subsamp_type == SUBSAMPLE_POINT ){
       if( (c % subsamp_factor != 0) || (r % subsamp_factor != 0) )
       continue;
       }
       }
       */
      
      // write point pair YAML file
      if( save_yaml && !enable_mapload ){
         cout << "writing file: " << outMapFN << endl;
         
         // open file
         FileStorage fs;
         fs.open(outMapFN, FileStorage::WRITE);
         
         string cam_make = checkExifMap( exifList[0], "Make" );
         string cam_model = checkExifMap( exifList[0], "Model" );
         
         string cam_serial  = checkExifMap( exifList[0], "SerialNumber");
         string cam_intserial  = checkExifMap( exifList[0], "InternalSerialNumber");
         
         string lens_str    = checkExifMap( exifList[0], "Lens" );
         string lens_id     = checkExifMap( exifList[0], "LensID" );
         string lens_model  = checkExifMap( exifList[0], "LensModel" );
         string lens_type   = checkExifMap( exifList[0], "LensType" );
         string lens_serial = checkExifMap( exifList[0], "LensSerialNumber" );
         
         float chess_scl_tmp = chess_scale;
         // debug
//         cout << "cam_size "        << Mat(Point2i(image_size)) << endl;
//         cout << "cam_serial "      << cam_serial << endl;
//         cout << "cam_intserial "   << cam_intserial << endl;
//         cout << "cam_make "        << cam_make << endl;
//         cout << "cam_model "       << cam_model << endl;
//         cout << "cam_lens_str "    << lens_str << endl;
//         cout << "cam_lens_id "     << lens_id << endl;
//         cout << "cam_lens_model "  << lens_model << endl;
//         cout << "cam_lens_type "   << lens_type << endl;
//         cout << "cam_lens_serial " << lens_serial << endl;
//         cout << "chess_scale "     << chess_scale << endl;
//         cout << "note "            << scanMessage << endl;

			printf("saving decoded ranges..\n");
			fs << "pro_range_x" << cv::Size(x_min,x_max);
			fs << "pro_range_y" << cv::Size(y_min,y_max);
			//fs << "cam_range_x" << ;
			//fs << "cam_range_y" << ;

         printf("saving exif data..\n");

         // EXIF data
         fs << "cam_size"        << Mat(Point2i(image_size));
         fs << "cam_make"        << cam_make;
         fs << "cam_model"       << cam_model;
         fs << "cam_serial"      << cam_serial;
         fs << "cam_intserial"   << cam_intserial;
         fs << "cam_lens_str"    << lens_str;
         fs << "cam_lens_id"     << lens_id;
         fs << "cam_lens_model"  << lens_model;
         fs << "cam_lens_type"   << lens_type;
         fs << "cam_lens_serial" << lens_serial;
         
         fs << "chess_scale" << chess_scale;
         fs << "note" << scanMessage;
         
         if( chess_image_input )
         {
//            cout << "chess_size "        << Mat(Point2i(chess_size)) << endl;;
//            cout << "obj_chess_corners " << Mat(object_corners) << endl;;
//            cout << "cam_chess_corners " << Mat(chess_corners) << endl;;
//            cout << "pro_chess_corners " << Mat(projector_corners) << endl;;
//
            printf("saving chess images..\n");
            
            fs << "chess_size"        << Mat(Point2i(chess_size));
            fs << "obj_chess_corners" << Mat(object_corners);
            fs << "cam_chess_corners" << Mat(chess_corners);
            fs << "pro_chess_corners" << Mat(projector_corners);
         }
         
         if( save_matches )
         {
            printf("saving matches..\n");
            //                fs << "cam_points_rgb" << Mat( camera_points_rgb );;
            fs << "cam_points" << cam_pts;
            fs << "pro_points" << pro_pts;
         }
         
         // close file
         fs.release();
         
         cout << "write finished." << endl;
         
      }
      
      // save visualization of processed data
      if( save_preview ){
         
         // save RGB images
         {
            Mat vis_img;
            
            vis_img.create(image_size,CV_8UC3);
            vis_img.setTo(Scalar::all(0));
            
            for(int i=0; i<camera_points.size(); i++){
               int x = camera_points[i].x;
               int y = camera_points[i].y;
               
               uchar* pix_ptr = vis_img.ptr<uchar>(y);
               uchar* pix = &pix_ptr[x*3];
               
               pix[0] = camera_points_rgb[i].x;
               pix[1] = camera_points_rgb[i].y;
               pix[2] = camera_points_rgb[i].z;
               
            }
            
            cout << "writing rgb vis image: " << outViscFN << endl;
            imwrite(outViscFN,vis_img);
            
            //                cout << "writing direct vis image: " << outViscFN << endl;
            //                imwrite(outVisDirFN,max_map);
            //
            //                cout << "writing indirect vis image: " << outViscFN << endl;
            //                imwrite(outVisAmbFN,min_map);
            
         }
         
         // camera points, projector points
         vector< Point2f >* c_pts;
         vector< Point2f >* p_pts;
         
         if( enable_coalesce )
         {
            c_pts = &camera_points_avg;
            p_pts = &projector_points_avg;
         } else {
            c_pts = &camera_points;
            p_pts = &projector_points;
            
         }
         
         // save x vis image
         if( show_x ){
            // create preview image
            Mat vis_img;
            Size2i super_size;
            super_size.width  = (int)(image_size.width  * vis_supersample);
            super_size.height = (int)(image_size.height * vis_supersample);
            
            vis_img.create(super_size,CV_8UC3);
            vis_img.setTo(Scalar::all(32));
            
            // only process decoded points
            for(int i=0; i<c_pts->size(); i++){
               Point2f obj_idx = (*p_pts)[i];
               Point2i img_idx((int)((*c_pts)[i].x * vis_supersample),(int)((*c_pts)[i].y * vis_supersample));
               uchar* pix = vis_img.ptr<uchar>(img_idx.y);
               int x = img_idx.x;
               
               // output
               float t = (obj_idx.x-x_min) / (float)(x_max-x_min);
               t *= vis_scl;
               if(t>1.0)
                  t-=floor(t);
               Vec3b col = float2Color( t );
               //                    Vec3b col(255,0,0);
               
               pix[x*3+0] = col[0];
               pix[x*3+1] = col[1];
               pix[x*3+2] = col[2];
               
               //                pix[1] = (uint8_t)fmin(fmax((val2*255),0.0),255.0);
               //                pix[2] = (uint8_t)fmin(fmax((val3*255),0.0),255.0);
               //                circle(vis_img,img_idx,8,Scalar(255,255,255),-1,8);
            }
            cout << "writing x-axis vis image: " << outVisxFN << endl;
            imwrite(outVisxFN, vis_img);
            
         }
         
         // save y vis image
         if( show_y ){
            // create preview image
            Mat vis_img;
            Size2i super_size;
            super_size.width  = (int)(image_size.width  * vis_supersample);
            super_size.height = (int)(image_size.height * vis_supersample);
            
            vis_img.create(super_size,CV_8UC3);
            vis_img.setTo(Scalar::all(32));
            
            // only process decoded points
            for(int i=0; i<c_pts->size(); i++){
               Point2f obj_idx = (*p_pts)[i];
               Point2i img_idx((int)((*c_pts)[i].x * vis_supersample),(int)((*c_pts)[i].y * vis_supersample));
               uchar* pix = vis_img.ptr<uchar>(img_idx.y);
               int x = img_idx.x;
               
               // output
               //float t = obj_idx.y / (float)y_max;
               float t = (obj_idx.y-y_min) / (float)(y_max-y_min);
               
               t *= vis_scl;
               if(t>1.0)
                  t-=floor(t);
               Vec3b col = float2Color( t );
               
               pix[x*3+0] = col[0];
               pix[x*3+1] = col[1];
               pix[x*3+2] = col[2];
               
               //                pix[1] = (uint8_t)fmin(fmax((val2*255),0.0),255.0);
               //                pix[2] = (uint8_t)fmin(fmax((val3*255),0.0),255.0);
               //                circle(vis_img,img_idx,8,Scalar(255,255,255),-1,8);
            }
            cout << "writing y-axis vis image: " << outVisyFN << endl;
            imwrite(outVisyFN, vis_img);
         }
         
      }
      
      // 3D MODEL OUTPUT
      // save PLY using procam calibration
      if( save_ply && enable_procam ){
         printf("saving ply file..\n");
         
         //float blah[] = {0.947791, 1.28523e-07, -0.318892, 0.030939, 0.995282, 0.0919554, 0.317388, -0.0970207, 0.94332};
         //Mat blah_R(3,3,CV_32F, blah);
         //Mat vec_blah_R;
         Mat vec_cam_R,vec_pro_R,vec_campro_R;
         Rodrigues(cam_R,vec_cam_R);
         Rodrigues(pro_R,vec_pro_R);
         Rodrigues(campro_R,vec_campro_R);
         //Rodrigues(blah_R,vec_blah_R);
         cout << "cam_R = " << vec_cam_R << endl;
         cout << "pro_R = " << vec_pro_R << endl;
         cout << "campro_R = " << vec_campro_R << endl;
         //cout << "blah_R = " << blah_R << endl;
         //cout << "blah_R = " << vec_blah_R << endl;
         
         
         // undistort camera/projector coordinates
         vector< Point2f > camera_points_u;
         vector< Point2f > projector_points_u;
         if( enable_coalesce )
         {
            undistortPoints(camera_points_avg,    camera_points_u,    cam_K, cam_kc, cam_R, cam_P);
            undistortPoints(projector_points_avg, projector_points_u, pro_K, pro_kc, pro_R, pro_P);
         } else {
            undistortPoints(camera_points,        camera_points_u,    cam_K, cam_kc, cam_R, cam_P);
            undistortPoints(projector_points,     projector_points_u, pro_K, pro_kc, pro_R, pro_P);
         }
         
         // triangulate points using camera-projector correspondence and stereo projection matrices
         vector< Vec3d > triPoints3d;
         
         // rotate to be centered in camera coordinate system
         Point3d cpt(campro_T);
         Matx44d rot;
         for(int r=0;r<3;r++)
         {
            for(int c=0;c<3;c++)
            {
               rot(r,c) = cam_R.at<double>(c,r);
            }
         }
         rot(0,3) = 0;// cpt.x;
         rot(1,3) = 0;// cpt.y;
         rot(2,3) = 0;// cpt.z;
         rot(3,3) = 1.0;
         rot(3,0) = 0.0;
         rot(3,1) = 0.0;
         rot(3,2) = 0.0;
         
         cout << "ROT : [4x4]" <<  rot << endl;
         
         cout << "cam_P : " << cam_P.size() << endl <<  cam_P << endl << endl;
         cout << "pro_P : " << pro_P.size() << endl <<  pro_P << endl << endl;
         
         for(int i=0; i<camera_points_u.size(); i++)
         {
            // homogeneous undistorted xy coordinates for camera and projector
            Point3d u0(   camera_points_u[i].x,    camera_points_u[i].y, 1);
            Point3d u1(projector_points_u[i].x, projector_points_u[i].y, 1);
            // 3x4 projection matrices for camera and projector
            Matx34d P0(cam_P);
            Matx34d P1(pro_P);
            Mat X = LinearLSTriangulation(u0,P0,u1,P1);
            
            // transform point
            Point3d pt(X);
            
            Vec4d v0(pt.x,pt.y,pt.z,1);
            Vec4d v1 = rot*v0;
            if(i<10)
               cout << v0 << " : " << v1 << endl;
            pt.x=v1[0]/v1[3];
            pt.y=v1[1]/v1[3];
            pt.z=v1[2]/v1[3];
            
            triPoints3d.push_back(pt);
         }
         
         
         // convert to floating point for PLY saving
         vector< Vec3f > triPoints3f;
         vector< Vec3b > rgbPoints;
         if(true){
            // pack each point into vector triPoints3f
            for(int i=0; i<triPoints3d.size(); i++)
            {
               float x,y,z,w;
               
               // negate yz axis to make it nice for 3d model viewer apps
               x =  static_cast<float>(triPoints3d[i][0]);
               y =  -static_cast<float>(triPoints3d[i][1]); // -
               z =  -static_cast<float>(triPoints3d[i][2]); // -
               triPoints3f.push_back(Vec3f(x,y,z));
               
               /*
                //                w = pts3D.at<float>(3,i);
                //                if(w<=0.001) w = 1;
                //                x = pts3D.at<float>(0,i) / w;
                //                y = pts3D.at<float>(1,i) / w;
                //                z = pts3D.at<float>(2,i) / w;
                
                //                float v = i/(float)pts3D.cols;
                //                x = v*100;
                //                y = 2*sin(3.1415*2*v*32);
                //                z = 0;
                */
            }
         } else {
            Mat pts3D(1,camera_points.size(),CV_32FC4);
            triangulatePoints(cam_P,pro_P,camera_points_u,projector_points_u,pts3D);
            for(int i=0; i<pts3D.cols; i++)
            {
               float x,y,z,w;
               
               w = pts3D.at<float>(3,i);
               if(w<=0.001) w = 1;
               x = pts3D.at<float>(0,i) / w;
               y = pts3D.at<float>(1,i) / w;
               z = pts3D.at<float>(2,i) / w;
               triPoints3f.push_back(Vec3f(x,y,z));
               
            }
            //                convertPointsFromHomogeneous(pts3D,triPoints3f);
         }
         
         if( enable_coalesce )
         {
            float r_avg=0;
            float g_avg=0;
            float b_avg=0;
            int N = fmax(r_avg/triPoints3d.size(),1);
            for(int i=0; i<triPoints3d.size(); i++)
            {
               uint8_t r = static_cast<uint8_t>(fmin(fmax(camera_points_rgb_avg[i].z,0.0),255.0));
               uint8_t g = static_cast<uint8_t>(fmin(fmax(camera_points_rgb_avg[i].y,0.0),255.0));
               uint8_t b = static_cast<uint8_t>(fmin(fmax(camera_points_rgb_avg[i].x,0.0),255.0));
               rgbPoints.push_back(Vec3b(r,g,b));
               r_avg += r;
               g_avg += g;
               b_avg += b;
            }
            printf("r,g,b avg = %0.4f, %0.4f, %0.4f\n", r_avg/N, g_avg/N, b_avg/N);

         } else {
            float r_avg=0;
            float g_avg=0;
            float b_avg=0;
            int N = r_avg/triPoints3d.size();
            for(int i=0; i<N; i++)
            {
               uint8_t r = static_cast<uint8_t>(camera_points_rgb[i].z);
               uint8_t g = static_cast<uint8_t>(camera_points_rgb[i].y);
               uint8_t b = static_cast<uint8_t>(camera_points_rgb[i].x);
               rgbPoints.push_back(Vec3b(r,g,b));
               
               r_avg += r;
               g_avg += g;
               b_avg += b;
               
            }
            printf("r,g,b avg = %0.4f, %0.4f, %0.4f\n", r_avg/N, g_avg/N, b_avg/N);

         }
         
         printf("***********************************\n");
         printf("camera_points_rgb_avg.size = %d\n",(int)camera_points_rgb_avg.size());
         if(DBUG)
         for(int i=0; i<25; i++)
         {
            Point3f pt = camera_points_rgb_avg[i];
            printf("\t[%d] = (%f,%f,%f)\n", i, pt.x,pt.y,pt.z);
         }
         printf("rgbPoints.size = %d\n", (int)rgbPoints.size());
         if(DBUG)
         for(int i=0; i<25; i++)
         {
            Vec3b pt = rgbPoints[i];
            printf("\t[%d] = (%d,%d,%d)\n", i, pt[0],pt[1],pt[2]);
         }
         printf("***********************************\n");
         
         // write PLY file
         ofstream myfile;
         myfile.open (outPlyFN, ios::out | ios::binary);
         
         if(myfile.is_open()){
            printf("writing PLY file: [%s] [%d points]\n", outPlyFN, (int)triPoints3f.size() );
            char buf[50];
            sprintf(buf, "%d", (int)triPoints3f.size());
            
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
            for(int i=0;i<triPoints3f.size();i++){
               myfile.write( (char*)(&triPoints3f[i]), sizeof(float)  *3 );
               myfile.write( (char*)(  &rgbPoints[i]), sizeof(uint8_t)*3 );
            }
         }
      }
   }
}
