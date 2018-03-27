#include <string>
#include <stdint.h>
#include <cstdio>
#include <unistd.h>
#include <sys/time.h>
#include <assert.h>

//#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgcodecs.hpp>

#include <opencv2/imgproc/imgproc.hpp>


#define AXIS_X 0
#define AXIS_Y 1
#define THRESHOLD_VALUE 5

#ifndef SL_UTIL_H
#define SL_UTIL_H

uint16_t gray2bin(uint16_t gray);
uint16_t bin2gray( uint16_t num );
double decodeImagePair( cv::Mat &src,
                     cv::Mat &inv,
                     cv::Mat &out_rgb,
                     cv::Mat &out_bit,
                     int threshold);
void mergeNewBit( cv::Mat &bit, cv::Mat &map, int bitfield, int minbit);
void parseImagePair( cv::Mat &pattern,
                     cv::Mat &pattern_inv,
                     cv::Mat &out_map,
                     cv::Mat &out_rgb,
                     cv::Mat &out_diff,
                     int bitfield,
                     int threshold );
void generatePattern( cv::Mat img, cv::Size sz, int axis, int used_bit, bool inv );
void printbin( int v );

#endif
