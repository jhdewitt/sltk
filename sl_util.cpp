// sl_util.cpp
// helper functions for structured light
// by John De Witt

#include "sl_util.h"

#define DBUG 0

using namespace std;
using namespace cv;

// Convert a 10-bit Gray code value into a binary integer representation
// Thanks to http://www.dspguru.com/dsp/tricks/gray-code-conversion
uint16_t gray2bin(uint16_t gray){
	uint16_t temp = gray ^ (gray>>8);
	
	// Our error condition should propagate
	if(gray == UINT16_MAX) return -1;
	
	temp ^= (temp>>4);
	temp ^= (temp>>2);
	temp ^= (temp>>1);
	return temp;
}

uint16_t bin2gray( uint16_t num ) {
	return (num>>1) ^ num;
}

void printbin( int v ){
    printf("[");
    for (int i=0; i<32; i++) {
        printf("%c", (v>>(32-i-1))&1?'1':'0');
        if((i+1)%4==0) printf(" ");
    }
    printf("]");
}

//// convert unsigned binary to gray code value
//unsigned int bin2gray( unsigned int num ) {
//	return (num>>1) ^ num;
//}
//
//// Convert a 10-bit Gray code value into a binary integer representation
//// Thanks to http://www.dspguru.com/dsp/tricks/gray-code-conversion
//int gray2bin(int gray){
//	unsigned int temp = gray ^ (gray>>8);
//	
//	// Our error condition should propagate
//	if(gray == -1) return -1;
//	
//	temp ^= (temp>>4);
//	temp ^= (temp>>2);
//	temp ^= (temp>>1);
//	return temp;
//}


void generatePattern( Mat img, Size sz, int axis, int used_bit, bool inv ){
    img.create(sz,CV_8UC3);
    
    uint8_t* arr = NULL;
    for(int r=0; r<img.rows; r++){
        arr = img.ptr<uint8_t>(r);
        
        for(int c=0; c<img.cols; c++){
            uint16_t v = 0;
            switch(axis){
                case AXIS_X:
                    v = c;
                    break;
                case AXIS_Y:
                    v = r;
                    break;
            }
            uint8_t p = ( (bin2gray(v) >> used_bit) & 1 ) == 1 ? 255 : 0;
            arr[c*3+0] = p;
            arr[c*3+1] = p;
            arr[c*3+2] = p;
       }
    }
    
}

cv::Mat get_gray_image(const std::string & filename){
    cv::Mat rgb_image = cv::imread(filename);
    if(rgb_image.rows>0 && rgb_image.cols>0)
    {
        cv::Mat gray_image;
        cvtColor(rgb_image, gray_image, COLOR_BGR2GRAY);
        return gray_image;
    }
    return cv::Mat();
}

// create a two channel grayscale image with global/direct components
cv::Mat get_direct_global(const std::vector<cv::Mat>& images, float b)
{
    static const unsigned MAXIMG = 10;
    
    unsigned count = images.size();
    if (count<1)
    {   // no images
        return cv::Mat();
    }
    
//    std::cout << "--- get_direct_global START ---" << endl;
    
    if( MAXIMG < count )
    {
//        std::cout << "WARNING: Using only " << MAXIMG << " of "<< count << std::endl;
        count = MAXIMG;
    }
    
    for(int i=0; i<count; i++){
        if( images.at(i).type() != CV_8UC1 ){
//            std::cout << "only grayscale images are supported" << std::endl;
            return cv::Mat();
        }
    }
    
    cv::Size size = images.at(0).size();
    
    cv::Mat light(size,CV_8UC2);
    
    double b1 = 1.0/(1.0 - b);
    double b2 = 2.0/(1.0 - b*b);
    
    for(int r=0; r<size.height; r++)
    {
        // get pointer to all input images
        const uint8_t* row[MAXIMG];
        for(int i=0; i<count; i++){
            row[i] = images.at(i).ptr<uint8_t>(r);
        }
        cv::Vec2b* row_light = light.ptr<cv::Vec2b>(r);
        
        for(int c=0; c<size.width; c++)
        {
            unsigned Lmax = row[0][c];
            unsigned Lmin = row[0][c];
            for(int i=0; i<count; i++)
            {
                if(row[i][c] > Lmax) Lmax = row[i][c];
                if(row[i][c] < Lmin) Lmin = row[i][c];
            }
            
            int Ld = static_cast<int>(b1*(Lmax - Lmin) + 0.5);
            int Lg = static_cast<int>(b2*(Lmin - b*Lmax) + 0.5);
            row_light[c][0] = (Lg>0 ? static_cast<unsigned>(Ld) : Lmax);
            row_light[c][1] = (Lg>0 ? static_cast<unsigned>(Lg) : 0);
            
        }
    }
    
//    std::cout << "--- get_direct_global END   ---" << endl;
    
    return light;
}


// input: 2x 8UC3, one image of binary pattern, one image of binary pattern inverse/complement
// output 1x 8UC1, of per-pixel decoded bitfield
// output 1x 32FC4, of per-pixel observed color (accumulates)
double decodeImagePair( Mat &src,
                     Mat &inv,
                     Mat &out_rgb,
                     Mat &out_bit,
                     int threshold)
{
   double diff_tot = 0;
   int diff_n = 0;

   for(int r=0; r<src.rows; r++)
   {
      const Vec3b* src_px = src.ptr<Vec3b>(r);
      const Vec3b* inv_px = inv.ptr<Vec3b>(r);
      Vec4f* rgb_px = out_rgb.ptr<Vec4f>(r);
      uchar* bit_px = out_bit.ptr<uchar>(r);
      
      for(int c=0; c<src.cols; c++)
      {
         int gray0 = (int)(0.299*src_px[c][2] + 0.587*src_px[c][1] + 0.114*src_px[c][0]);
         int gray1 = (int)(0.299*inv_px[c][2] + 0.587*inv_px[c][1] + 0.114*inv_px[c][0]);
         int diff = gray0 - gray1;
         
         // found signal
         if( abs(diff) > threshold ){
            
            diff_tot += abs(diff);
            diff_n++;
            
            // (first pattern brighter) : white / 1
            if( diff > 0 ){
               bit_px[c] = 255;
//               rgb_px[c] = src_px[c];
               rgb_px[c][0] += src_px[c][0];
               rgb_px[c][1] += src_px[c][1];
               rgb_px[c][2] += src_px[c][2];
               rgb_px[c][3] += 1;

            }
            
            // (second pattern brighter) : black / 0
            if( diff < 0 ){
               bit_px[c] = 0;
//               rgb_px[c] = inv_px[c];
               rgb_px[c][0] += inv_px[c][0];
               rgb_px[c][1] += inv_px[c][1];
               rgb_px[c][2] += inv_px[c][2];
               rgb_px[c][3] += 1;

            }
            
         } else {
            // set to "invalid" sentinel value 128
            bit_px[c] = 128;
            
         }
         
      }
   }
   
   if(diff_n>0)
   {
      return diff_tot/diff_n;
   }
   else
   {
      return -1;
   }
   
}

// input: 1x  8UC1, one bitfield from the output codeword
// output 1x 16UC1, edited code map updated with new bitfield
void mergeNewBit( Mat &bit, Mat &map, int bitfield, int minbit )
{
   //printf("MERGING NEW BITFIELD : [bitfield %d] [minbit %d]\n",bitfield,minbit);
	//
   ushort b = 1<<bitfield;
   
   // copy in new bits as needed
   for(int r=0; r<bit.rows; r++)
   {
      uchar*  bit_px = bit.ptr<uchar>(r);
      ushort* map_px = map.ptr<ushort>(r);
      
      if(bitfield < minbit)
      {
         
      }
      else
      {
         for(int c=0; c<bit.cols; c++)
         {
            if(map_px[c] != UINT16_MAX)
            {
               switch( bit_px[c] )
               {
                  case 0:
                     map_px[c] &= (~b);
                     break;
                  case 255:
                     map_px[c] |= (b);
                     break;
                  case 128:
                     //  if(bitfield > minbit)
                     map_px[c] = UINT16_MAX; // mark as invalid
                     break;
               }
            }
         }
      }
      
   }
   
}

// pattern and inverse
// correspondence map, rgb average map, and difference stats
void parseImagePair( Mat &pattern,
                    Mat &pattern_inv,
                     Mat &out_map,
                     Mat &out_rgb,
                     Mat &out_diff,
                     int bitfield,
                     int threshold )
{
    
    double* diff_p = out_diff.ptr<double>(0);
    
    // per pixel row
    for(int r = 0; r < pattern.rows; r++){
        const Vec3b* img_px = pattern.ptr<Vec3b>(r);
        const Vec3b* inv_px = pattern_inv.ptr<Vec3b>(r);
        
        ushort* map_px = out_map.ptr<ushort>(r);
//       float*  rgb_px = out_rgb.ptr<float>(r);
       Vec4f*  rgb_px = out_rgb.ptr<Vec4f>(r);
       
        // per pixel column
        for(int c = 0; c < pattern.cols; c++){
           
            if( map_px[c] == UINT16_MAX){
                continue;
            }
            
            int gray0 = (0.299*img_px[c][2] + 0.587*img_px[c][1] + 0.114*img_px[c][0]);
            int gray1 = (0.299*inv_px[c][2] + 0.587*inv_px[c][1] + 0.114*inv_px[c][0]);
            int diff = gray0 - gray1;
            
            map_px[c] = map_px[c] << 1;
            
            assert( (map_px[c] & 0x01) == 0 );
            
            // found signal
            if( abs(diff) > threshold ){
                
                diff_p[bitfield*2+0] += abs(diff);
                diff_p[bitfield*2+1] += 1;
                
                // white
                if( diff > 0 ){
                    map_px[c] |= 1;
                    
                    rgb_px[c][0] += img_px[c][0];
                    rgb_px[c][1] += img_px[c][1];
                    rgb_px[c][2] += img_px[c][2];
                    rgb_px[c][3] += 1;
                }
                
                // black
                if( diff < 0 ){
                    map_px[c] |= 0;
                    
                    rgb_px[c][0] += inv_px[c][0];
                    rgb_px[c][1] += inv_px[c][1];
                    rgb_px[c][2] += inv_px[c][2];
                    rgb_px[c][3] += 1;
                }
                
            } else {
                // set to "invalid" sentinel value
                map_px[c] = UINT16_MAX;
                
            }
        }
    }
}
