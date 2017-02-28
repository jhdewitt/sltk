#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
//#include <opencv2/imgcodecs.hpp>


#include <iostream>
#include <vector>
#include <cstdio>

using namespace cv;
using namespace std;

// specify board width and height
// specify output image size
// specify (optional) pixel size of squares

static void help(){
    printf("This is a chessboard pattern detector.\n"
           "Usage: chessfind <img>\n"
           "    -x <board_width>        # the number of inner corners on the horizontal dimension\n"
           "    -y <board_width>        # the number of inner corners on the vertical dimension\n"
           "    -s <square_size>        # the square edge length in preferred units\n"
           "    -save                   # enable saving of the pattern\n"
           "    -show                   # enable display of the pattern\n"
           "\n" );
}

int main(int argc, char** argv){
    
    Size boardSize,imageSize;
    int sqrSize = -1;
    bool save_image = false;
    bool show_image = false;

   
   imageSize = Size(1024,768);
    // parse arguments
    if( argc < 2 )
    {
        help();
        return 0;
    }
    
    for(int i=1;i<argc;i++)
    {
        const char* s = argv[i];
        
        if( strcmp( s, "-x" ) == 0 )
        {
            if( sscanf( argv[++i], "%u", &boardSize.width ) != 1 || boardSize.width <= 0 )
                return fprintf( stderr, "Invalid board height\n" ), -1;
        }
        else if( strcmp( s, "-y" ) == 0 )
        {
            if( sscanf( argv[++i], "%u", &boardSize.height ) != 1 || boardSize.height <= 0 )
                return fprintf( stderr, "Invalid board height\n" ), -1;
        }
        else if( strcmp( s, "-s" ) == 0 )
        {
            if( sscanf( argv[++i], "%u", &sqrSize ) != 1 || sqrSize <= 0 )
                return fprintf( stderr, "Invalid square size\n" ), -1;
        }
        else if(strcmp("-save",s)==0)
        {
            save_image = true;
        }
        else if(strcmp("-show",s)==0)
        {
            show_image = true;
        }
        else
            return fprintf( stderr, "unknown option %s", s), -1;
    }

    boardSize.width++;
    boardSize.height++;
    
    float scl = 0.75;
    
    if( sqrSize <= 0 ){
        
        // auto calculate square size
        float x_sz = imageSize.width / (float)boardSize.width;
        float y_sz = imageSize.height / (float)boardSize.height;
        x_sz *= scl;
        y_sz *= scl;
        
        sqrSize = min(x_sz, y_sz);
        cout << "square size = " << sqrSize << endl;
    }
    
    Size2i chess_img_size = boardSize * sqrSize;
    
    Point2i roi_vertex( (imageSize.width - chess_img_size.width)/2,
                       (imageSize.height - chess_img_size.height)/2 );
    
    Mat chessboard(imageSize,CV_8UC3,Scalar::all(255));
    Mat chessboard_roi( chessboard,Rect(roi_vertex.x,roi_vertex.y,chess_img_size.width,chess_img_size.height ) );
    
    cout << "Initializing chessboard..";
    for (int r=0; r<boardSize.height; r++)
    {
        for (int c=0; c<boardSize.width; c++)
        {
            Mat sub_roi(chessboard_roi,Rect(sqrSize*c,sqrSize*r,sqrSize,sqrSize));
            
            if( (r+c)%2 == 1 )
                sub_roi.setTo(Scalar::all(255));
            else
                sub_roi.setTo(Scalar::all(0));
        }
    }
    cout << "done" << endl;
    
    // generate filename
    char buf[50];
    sprintf(buf,"chessboard_%dx%d.png",boardSize.width-1,boardSize.height-1);
    string fname(buf);
    
    if( save_image ){
        imwrite(fname, chessboard);
    }
    
    if( show_image ){
        imshow(fname, chessboard);
        waitKey(0);
    }
    
    return 0;
}
