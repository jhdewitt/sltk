#include <iostream>
#include <vector>
#include <string>
#include "opencv2/opencv.hpp"
//#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/imgcodecs/imgcodecs.hpp"

#include "util.h"

using namespace std;
using namespace cv;

static void help()
{
   printf("This program plots lens matrix with distortion.\n"
          "Usage: lensplot\n"
          "    -f <x y>       # specify x and y focal length values\n"
          "    -c <x y>       # specify x and y principal point values\n"
          "    -d 1 0 0 0       # specify radial distortion coefficients k0 k1 k2 k3\n"
          "    [-show]                    # show a preview of the undistorted image\n"
          "    [-save]                    # save the undistorted image\n"
          "    <input_file>               # the input filename for the image to be undistorted\n"
          "    -cam                       # use camera calibration if multiple exist\n"
          "    -pro                       # use projector calibration if multiple exist\n"
          "\n");
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

int main( int argc, char** argv ){
   
   float squareSize = 1.f;
   int n=0;
   
   const char* outVisFN = findNextName( "lensplot_%04d.png",&n);
   const char* inCalibFN  = 0;
   
   bool use_args = false;
   Vec2i pixel_dim(0,0);
   Vec2f focal_len(0,0);
   Vec2f principal_pt(0,0);
   Vec4f distortion_coeff(1,0,0,0);
   
   bool use_cam = true;
   bool use_pro = false;
   
   bool plot_display = false;
   bool plot_save = true;
   
   vector< string > scanList;
   
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
         return 1;
      }
      else if( strcmp(s,"-show") == 0 )
      {
         plot_display = true;
      }
      else if( strcmp(s,"-noshow") == 0 )
      {
         plot_display = false;
      }
      else if( strcmp(s,"-save") == 0 )
      {
         plot_save = true;
      }
      else if( strcmp(s,"-cam") == 0 )
      {
         use_cam = true;
         use_pro = false;
      }
      else if( strcmp(s,"-pro") == 0 )
      {
         use_cam = false;
         use_pro = true;
      }
      
      
      //   "    -f <x y>       # specify x and y focal length values\n"
      //   "    -c <x y>       # specify x and y principal point values\n"
      //   "    -d 1 0 0 0       # specify radial distortion coefficients k0 k1 k2 k3\n"

      else if( strcmp(s,"-size") == 0 )
      {
         use_args = true;
         int x,y;
         if( i+2 >= argc )
            return fprintf( stderr, "Image resolution values not provided\n"), -1;
         if( sscanf( argv[++i], "%d", &x) != 1 || x<2 )
            return fprintf( stderr, "Invalid x resolution\n"), -1;
         if( sscanf( argv[++i], "%d", &y) != 1 || y<2 )
            return fprintf( stderr, "Invalid y resolution\n"), -1;
         pixel_dim[0] = x;
         pixel_dim[1] = y;
      }
      
      else if( strcmp(s,"-f") == 0 )
      {
         use_args = true;
         int x,y;
         if( i+2 >= argc )
            return fprintf( stderr, "Focal length values not provided\n"), -1;
         if( sscanf( argv[++i], "%d", &x) != 1 || x<2 )
            return fprintf( stderr, "Invalid focal length x\n"), -1;
         if( sscanf( argv[++i], "%d", &y) != 1 || y<2 )
            return fprintf( stderr, "Invalid focal length y\n"), -1;
         bool use_args = false;

         focal_len[0] = x;
         focal_len[1] = y;
      }
      else if( strcmp(s,"-d") == 0 )
      {
         use_args = true;
         int k0,k1,k2,k3;
         if( i+3 >= argc )
            return fprintf( stderr, "Radial distortion values not provided\n"), -1;
         if( sscanf( argv[++i], "%d", &k0) != 1 )
            return fprintf( stderr, "Invalid distortion value x\n"), -1;
         if( sscanf( argv[++i], "%d", &k1) != 1 )
            return fprintf( stderr, "Invalid distortion value x\n"), -1;
         if( sscanf( argv[++i], "%d", &k2) != 1 )
            return fprintf( stderr, "Invalid distortion value x\n"), -1;
         if( sscanf( argv[++i], "%d", &k3) != 1 )
            return fprintf( stderr, "Invalid distortion value x\n"), -1;
         bool use_args = false;
         distortion_coeff[0] = k0;
         distortion_coeff[1] = k1;
         distortion_coeff[2] = k2;
         distortion_coeff[3] = k3;
      }

      else if( strcmp(s,"-t") == 0 )
      {
         int v = 5;
         if( i+1 >= argc )
            return fprintf( stderr, "Threshold value not provided\n"), -1;
         if( sscanf( argv[++i], "%d", &v ) != 1 || v < 1 )
            return fprintf( stderr, "Invalid threshold value\n"), -1;
         //diff_threshold = v;
      }
      else if( s[0] != '-' )
      {
         inCalibFN = s;
      }
      else
         return fprintf( stderr, "Unknown option %s", s ), -1;
   }
   
   if( inCalibFN == 0 ){
      printf("no file provided\n");
      return 0;
   }
   
   FileStorage fs;
   fs.open(inCalibFN, FileStorage::READ);
   
   if (!fs.isOpened())
      return fprintf(stderr,"failed to open %s",inCalibFN), -1;
   
   printf("reading data from %s..\n",inCalibFN);
   
   //    Mat camera_matrix;
   //    Mat distortion_coefficients;
   
   // camera input
   Mat cam_dims;
   Mat cam_K;
   Mat cam_kc;
   
   // projector input
   Mat pro_dims;
   Mat pro_K;
   Mat pro_kc;
   
   if( !use_args ){
      fs["pro_size"] >> pro_dims;
      fs["pro_K"]    >> pro_K;
      fs["pro_kc"]   >> pro_kc;
      
      fs["cam_size"] >> cam_dims;
      fs["cam_K"]    >> cam_K;
      fs["cam_kc"]   >> cam_kc;
   } else {
      Vec2i pixel_dim(0,0);
      Vec2f focal_len(0,0);
      Vec2f principal_pt(0,0);
      Vec4f distortion_coeff(1,0,0,0);
      
      //cam_dims = pixel_dim;
      
      //float 
      
      float fx = focal_len[0];
      float fy = focal_len[1];
      
      float px = principal_pt[0];
      float py = principal_pt[1];
      
      //if(px<0.1||py<0.1)
        // px =
      
      float k0 = distortion_coeff[0];
      float k1 = distortion_coeff[1];
      float k2 = distortion_coeff[2];
      float k3 = distortion_coeff[3];
      
      //cam_K = Matx33f(fx,0,cx, 0,fy,cy, 0,0,1);
      
      
   }
   
   // output
   Mat dat_dims;
   Mat dat_K;
   Mat dat_kc;
   int dat_w = -1;
   int dat_h = -1;
   
   // assign to output data
   if( use_cam && !use_pro )
   {
      dat_dims = cam_dims;
      dat_K    = cam_K;
      dat_kc   = cam_kc;
   }
   else if ( use_pro && !use_cam )
   {
      dat_dims = pro_dims;
      dat_K    = pro_K;
      dat_kc   = pro_kc;
   }
   else
   {
      printf("error, please only enable one source at a time\n");
      return 1;
   }
   
   // fail if no calibration data
   if( dat_dims.data == NULL || dat_K.data == NULL || dat_kc.data == NULL )
   {
      printf("error loading calibration values\n");
      return 1;
   }
   // read width and height data
   else
   {
      dat_w = dat_dims.ptr<int>(0)[0];
      dat_h = dat_dims.ptr<int>(0)[1];
   }
   
   
   // print calibration info
   
   cout << "image width  = " << dat_w << endl;
   cout << "image height = " << dat_h << endl;
   if( use_cam )
      cout << "cam matrix : " << dat_K << endl;
   if( use_pro )
      cout << "pro matrix : " << dat_K << endl;
   cout << "dist coeffs : " << dat_kc << endl;
   
   Mat img(dat_h,dat_w,CV_8UC3,Scalar::all(32));
   
   int div = 1;
   int x_bord = ( img.cols - div*(int)(img.cols/div))/2;
   int y_bord = ( img.rows - div*(int)(img.rows/div))/2;
   printf("border = [%d, %d]\n",x_bord,y_bord);
   vector< Point2f > pts;
   for(int r=0; r<img.rows; r+=div){
      for(int c=0; c<img.cols; c+=div){
         pts.push_back(Point2f(x_bord+c,y_bord+r));
      }
   }
   
   // calculate original and undistorted points
   Mat orig_pts(pts);
   cout << orig_pts.elemSize() << ", " << orig_pts.channels() << endl;
   
   Mat undist_pts;
   undistortPoints( orig_pts, undist_pts, dat_K, dat_kc, noArray(), dat_K );
   
   // find differences by splitting matrix and finding x/y offsets
   Mat diff_pts = orig_pts - undist_pts;
   vector< Mat > xy_chan;
   split( diff_pts, xy_chan );
   
   Mat norm_diff = xy_chan[0].clone();
   norm_diff.setTo(Scalar::all(0));
   for(int i=0; i<diff_pts.rows; i++)
   {
      float dx = xy_chan[0].at<float>(i);
      float dy = xy_chan[1].at<float>(i);
      norm_diff.at<float>(i) = (float)pow(dx*dx+dy*dy,0.5);
   }
   
   double xdiff_min, xdiff_max;
   double ydiff_min, ydiff_max;
   double mdiff_min, mdiff_max; // magnitude
   minMaxLoc(xy_chan[0],&xdiff_min,&xdiff_max);
   minMaxLoc(xy_chan[1],&ydiff_min,&ydiff_max);
   minMaxLoc(norm_diff ,&mdiff_min,&mdiff_max);
   
   Scalar diff_mean, diff_stddev;
   meanStdDev(diff_pts, diff_mean, diff_stddev);
   
   cout << "diff_mean   = " << diff_mean << endl;
   cout << "diff_stddev = " << diff_stddev << endl;

   meanStdDev(norm_diff, diff_mean, diff_stddev);

   float mag_mean = diff_mean[0];
   cout << "diff_mag_mean   = " << diff_mean << endl;
   cout << "diff_mag_stddev = " << diff_stddev << endl;
   
   printf("diff_x_range = [%f < %f]\n",xdiff_min,xdiff_max);
   printf("diff_y_range = [%f < %f]\n",ydiff_min,ydiff_max);
   printf("diff_m_range = [%f < %f]\n",mdiff_min,mdiff_max);
   //    cout << "diff_xmax   = " << xdiff_max << endl;
   //    cout << "diff_xmin   = " << xdiff_min << endl;
   //    cout << "diff_ymax   = " << ydiff_max << endl;
   //    cout << "diff_ymin   = " << ydiff_min << endl;
   
   double abs_x_diff = fmax(abs(xdiff_min),abs(xdiff_max));
   double abs_y_diff = fmax(abs(ydiff_min),abs(ydiff_max));
   double abs_sqrt_diff = pow(pow(abs_x_diff,2.0) + pow(abs_y_diff,2.0), 0.5);
   
   printf("x, y, sqrt diff : %0.2f, %0.2f, %0.2f\n",abs_x_diff,abs_y_diff,abs_sqrt_diff);
   
   Mat diffmat(dat_h,dat_w,CV_32FC1);
   for(int i=0; i<orig_pts.rows; i++){
      Point2f p = orig_pts.at<Point2f>(i);
      Point2f u = undist_pts.at<Point2f>(i);
      Point2f v = u-p;
      
      double hyp = pow(v.x*v.x+v.y*v.y,0.5);
      double d = fmax(fmin(hyp/abs_sqrt_diff,1.0),0);
      
      Vec3b col = float2Color(d);
      
      diffmat.at<float>((int)p.y,(int)p.x) = (float)hyp;
//      cout << i << " : " << diffmat.at<float>((int)p.y,(int)p.x) << endl;
      img.at<cv::Vec3b>((int)p.y,(int)p.x) = col;
   }
   
   // calculate isolines using threshold and findContours
   Mat isoline_img(dat_h,dat_w,CV_8UC3,Scalar::all(0));
   Mat isoline_tmp = isoline_img.clone();
   Mat thresh_diffmat;
   Mat thresh_img(dat_h,dat_w,CV_8UC1);
   
   int isoline_N = 100;
   for(int i=1; i<isoline_N; i++)
   {
      float max_level = mdiff_max;
//      max_level = mag_mean;
      float v = i/(float)(isoline_N-1);
//      v = pow(v,2.0);
      
      float level = max_level*v;
//      cout << level << endl;
      threshold(diffmat, thresh_diffmat, level, 255, THRESH_BINARY);
      for(int r=0; r<diffmat.rows; r++)
      {
         float* pixf = thresh_diffmat.ptr<float>(r);
         uchar* pixb = thresh_img.ptr<unsigned char>(r);
         for(int c=0; c<diffmat.cols; c++)
         {
            pixb[c] = (unsigned char)pixf[c];
         }
      }
      //   thresh_diffmat.convertTo(thresh_img,CV_8UC1);
      
      vector<vector<Point> > contours0;
      vector<Vec4i> hierarchy;
      findContours( thresh_img, contours0, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE);
      
//      printf("contours length: %d\n",(int)contours0.size());
      drawContours( isoline_tmp, contours0, -1, Scalar(255,255,255),
                   1, 8, hierarchy);
      
      addWeighted(isoline_tmp, 1, isoline_img, 1, 0, isoline_img);
      
   }
   
//   thresh_img.setTo(Scalar::all(255));
//   imshow("img",isoline_img);
//   waitKey(0);
   
   
   /*
   //    cout << undist_pts << endl;
   int c = 1;
   for(int i=0; i<undist_pts.rows; i++){
      Point2f p = orig_pts.at<Point2f>(i);
      Point2f u = undist_pts.at<Point2f>(i);
      Point2f v = u-p;
      
      double hyp = pow(v.x*v.x+v.y*v.y,0.5);
      double d = fmax(fmin(hyp/abs_sqrt_diff,1.0),0);
      
      //        line(img,p,p+v,Scalar::all(0),1);
      //        line(img,p,p+Point2f(50,0),Scalar::all(0),1);
      //        line(img,p,p+Point2f(0,50),Scalar::all(0),1);
      //        line(img,p,u,Scalar::all(0),1);
      Scalar col = float2Color(d);
      Scalar col2(0,255*(1-d),255*d);
      Scalar col3(0,0,0);
      
      int s = (int)(4+d*18);
      circle(img,p,s+2,col3,-1);
      circle(img,p,s,col,-1);
      //        circle(img,p,s,Scalar(0,0,0),-1);
      //        circle(img,u,s,Scalar(0,0,255),-1);
   }
   
   
   Mat uimg;
   undistort(img,uimg,dat_K,dat_kc,dat_K);
   
   if( false )
   for(int i=0; i<undist_pts.rows; i++){
      Point2f p = orig_pts.at<Point2f>(i);
      Point2f u = undist_pts.at<Point2f>(i);
      Point2f v = u-p;
      
      double hyp = pow(v.x*v.x+v.y*v.y,0.5);
      double d = fmax(fmin(hyp/abs_sqrt_diff,1.0),0);
      
      int s = (int)(4+d*18);
      Scalar lcol(0,255*(1-d),255*d);
      Scalar col = float2Color(d);
      Scalar col2(0,0,0);
      line(uimg,p,u,col,2);
      //        circle(uimg,p,s,Scalar(0,255*(1-d),255*d),-1);
      //        circle(uimg,p,s+2,col2,-1);
      //        circle(uimg,p,s,col,-1);
      
   }
   */
   
   Point center_pt(dat_K.at<double>(0,2), dat_K.at<double>(1,2));
   circle(img, center_pt, 8, Scalar::all(255), 1, CV_AA);
   addWeighted(isoline_img, 1, img, 1, 0, img);

   cout << "dat_K : \n" << dat_K << endl;
   cout << center_pt << endl;
   
   
   Size sz(1280,800);
   Mat disp(sz,CV_8UC3,Scalar::all(0));
   // resize image
   float ratio = min((sz.width )/(float)img.cols,
                     (sz.height)/(float)img.rows);
   resize( img, disp, Size(), ratio, ratio, INTER_AREA );
   
   
   if( plot_display )
   {
      imshow("disp",disp);
      waitKey(0);
   }
   if( plot_save )
   {
      imwrite(string(outVisFN),img);
   }
   
   return 0;
}


















