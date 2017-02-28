#include <string>
#include <map>
#include <unistd.h>
#include <sys/time.h>
#include <vector>
#include <iostream>

#include <sys/types.h>
#include <sys/stat.h>

//#include "opencv2/core/core.hpp"
//#include "opencv2/imgproc/imgproc.hpp"

#ifndef UTIL_H
#define UTIL_H

struct exifstub{
   std::string make;
   std::string model;
   std::string lens_info;
   std::string lens_make;
   std::string lens_model;
   
   std::string serial_num;
   std::string serial_num_lens;
   std::string serial_num_internal;
   
   float exposure_t;
   float fnumber;
   int iso;
   int image_w;
   int image_h;
   float focal_length;
   float scale_factor_35efl;
   float hyperfocaldistance;
   float fov;
   float light_value;
};

typedef std::map<std::string,std::string> stringmap;
typedef std::map<std::string, stringmap > exifmap;

double curtime();
bool is_dir(const char *filename);
bool file_exists(const char * filename);
std::string get_pattern_filename( std::string pattern );
//cv::Rect maskToROI(cv::Mat img);
//bool readStringList( const std::string& filename, std::vector<std::string>& l );
bool getExif(std::string fname, stringmap &_map);
bool getExifMap(std::string fname, exifmap &m);
bool getExifList( std::vector< std::string > &fnames, std::vector< stringmap > &maps );
exifstub parseExifMap(exifmap m);
std::string getVal(stringmap m, std::string s);
void printStub(exifstub s);

char* findNextName( const char* pattern, int* n );
std::string findNextName( std::string pattern, int* n);
std::string checkExifMap( stringmap m, std::string q );

#endif
