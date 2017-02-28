#include "util.h"

using namespace std;
//using namespace cv;
bool DBUG = false;

//static struct timeval _tv;
double curtime(){
    struct timeval _tv;
    gettimeofday( &_tv, NULL );
    double ret = _tv.tv_sec;
    ret += _tv.tv_usec*0.000001;
    return ret;
}

#ifdef __APPLE__

bool is_dir(const char *filename)
{
   struct stat st;
   int ret = stat(filename,&st);
   if( ret != 0 )
   {
      return false;
   }
   if( S_ISDIR(st.st_mode) )
   {
      return true;
   }
   return false;
}
#endif

bool file_exists(const char * filename) {
    FILE *file = fopen(filename, "r");
	if( file != NULL)
	{
        fclose(file);
//      cout << "FILE_EXISTS: " << filename << endl;
        return true;
    }
    return false;
}

string get_pattern_filename( string pattern ){
    int num = 0;
    char buf[50];
    sprintf(buf,pattern.c_str(),num);
    while (file_exists(buf)) {
        num++;
        sprintf(buf,pattern.c_str(),num);
    }
    return (string)buf;
}

char* findNextName( const char* pattern, int* n )
{
    char* buf = (char*)calloc(100,sizeof(char));
    sprintf(buf,pattern,*n);
    while (file_exists(buf)) {
        *n = (*n)+1;
        sprintf(buf,pattern,*n);
    }
    return buf;
}

string findNextName( string pattern, int* n)
{
    return string(findNextName( pattern.c_str(), n ));
}

//Rect maskToROI(Mat img){
//    vector< Vec4i > hierarchy;
//    vector< vector<Point> > contours;
//    
//    Mat tmp = img.clone();
//    findContours(tmp,contours,hierarchy,CV_RETR_LIST,CV_CHAIN_APPROX_SIMPLE);
//    
//    int xmin,xmax,ymin,ymax;
//    for(int i=0; i<contours.size(); i++){
//        for(int j=0; j<contours[i].size(); j++){
//            int x = contours[i][j].x;
//            int y = contours[i][j].y;
//            if(i==0 && j==0){
//                xmin = x;
//                xmax = x;
//                ymin = y;
//                ymax = y;
//            } else {
//                xmin = min(x,xmin);
//                xmax = max(x,xmax);
//                ymin = min(y,ymin);
//                ymax = max(y,ymax);
//            }
//        }
//    }
//    
//    int x,y,w,h;
//    x = xmin;
//    y = ymin;
//    w = xmax - xmin;
//    h = ymax - ymin;
//    
//    x = min(max(x,0),img.cols);
//    y = min(max(y,0),img.rows);
//    w = min(max(w,0),img.cols-x);
//    h = min(max(h,0),img.rows-y);
//    
//    return Rect(x,y,w,h);
//}

//bool readStringList( const string& filename, vector<string>& l )
//{
//    l.resize(0);
//    FileStorage fs(filename, FileStorage::READ);
//    if( !fs.isOpened() )
//        return false;
//    cv::FileNode n = fs.getFirstTopLevelNode();
//    if( n.type() != FileNode::SEQ )
//        return false;
//    FileNodeIterator it = n.begin(), it_end = n.end();
//    for( ; it != it_end; ++it )
//        l.push_back((string)*it);
//    return true;
//}

std::string checkExifMap( stringmap m, std::string q )
{
    stringmap::iterator it;
    it = m.find(q);
    if( it == m.end() )
    {
        return "";
    } else
    {
        return it->second;
    }
}


bool getExifMap(string fname, exifmap &m){
   
   FILE *fp;
   char command[2048];
   
   // execute exiftool -s
   sprintf( command, "/usr/local/bin/exiftool -s -G1 %s 2>&1", fname.c_str() );
   fp = popen(command, "r");
   
   if (fp == NULL) {
      fprintf( stderr, "Failed to run command" );
      return false;
   }
   
   const int buf_n = 16384;
   char sbuf[buf_n]; // read line
   char cbuf[buf_n]; // category
   char kbuf[buf_n]; // key
   char vbuf[buf_n]; // value
   
   while( fgets(sbuf, sizeof(sbuf), fp) != NULL )
   {
      // abort if file not found
      char fnf[] = "File not found:";
      if(strncmp( sbuf,fnf, sizeof(fnf)-1 )==0){
         
         fprintf( stderr, "bad filename: %s\n", fname.c_str() );
         pclose(fp);
         return false;
      }
      
      // typedef std::map<std::string,std::string> stringmap;
      // typedef std::map<std::string, stringmap > exifmap;
      
      // scan [category] key : value
      sscanf(sbuf,"[%[^]]] %s : %[^\n]\n",cbuf,kbuf,vbuf);
      
      string cat(cbuf);
      string key(kbuf);
      string val(vbuf);
      
      exifmap::iterator it;
      it = m.find(cat);
      if( it == m.end() ){
         m[cat];
         //            printf("cat \"%s\" not found\n",cat.c_str());
      }
      
      m[cat].insert(pair<string,string>(key,val));
      
      //        printf("%-12s  %-26s  %s\n",cbuf,kbuf,vbuf);
      //        string _k(kbuf);
      //        string _v(vbuf);
      //        map.insert(pair<string,string>(_k,_v));
   }
   
   pclose(fp);
   
   return true;
}

// read fnames and fill exif maps
// after function -> fnames.size() == maps.size()
bool getExifList( std::vector< std::string > &fnames, std::vector< stringmap > &maps ){
    
    // concatenate all file names
    string concat_list = "";
    for(int i=0;i<fnames.size();i++){
        if(i==0)
            concat_list = fnames[i];
        else
            concat_list = concat_list + " " + fnames[i];
    }

   if(DBUG)
        printf("(%d chars) list: [%s]\n",(int)concat_list.length(),concat_list.c_str());
    
    // call exiftool for output
    FILE *fp;
    const int buf_n = 65536;
    char command[buf_n];
    sprintf( command, "/usr/local/bin/exiftool -s %s 2>&1", concat_list.c_str() );
    if(DBUG)
        printf( "$%s\n", command );
    fp = popen(command, "r");
    
    // string bufffers
    char sbuf[buf_n];
    char fbuf[buf_n];
    char kbuf[buf_n];
    char vbuf[buf_n];
    
    if( fnames.size() == 1 ){
        stringmap _newmap;
        maps.push_back(_newmap);
    }
    
    // parse exiftool output line by line
    while( fgets(sbuf, sizeof(sbuf), fp) != NULL )
    {
        // abort if file not found
        char fnf[] = "File not found:";
        if(strncmp( sbuf, fnf, sizeof(fnf)-1 )==0){
            fprintf( stderr, "file for exif data not found\n" );
            pclose(fp);
            return false;
        }
        char fhead[] = "========";
        char ftail[] = "image files read";
        int total = 0;
        
        // only execute for multiple files
        if( fnames.size() > 1 ){
            // find header
            if(strncmp( sbuf,fhead, sizeof(fhead)-1 )==0){
                sscanf(sbuf,"======== %[^\n]\n",fbuf);
                if(DBUG)
                    std::cout << "fname : [" << fbuf << "]"<< std::endl;
                
                stringmap _newmap;
                maps.push_back(_newmap);
                
                // find output tail
            } else if(sscanf(sbuf,"%d image files read",&total)==1){
                if(DBUG)
                    printf("end found\n");
            } else {
                // get key and value pairs
                // scan first and second parts of exiftool s1 : s2 format
                sscanf(sbuf,"%s : %[^\n]\n",kbuf,vbuf);
                string _k(kbuf);
                string _v(vbuf);
                maps[maps.size()-1].insert(pair<string,string>(_k,_v));
            }
        } else if( fnames.size() == 1 ){
            // get key and value pairs
            // scan first and second parts of exiftool s1 : s2 format
            sscanf(sbuf,"%s : %[^\n]\n",kbuf,vbuf);
            string _k(kbuf);
            string _v(vbuf);
            maps[maps.size()-1].insert(pair<string,string>(_k,_v));
        }
    }
    
    pclose(fp);
    
    if(DBUG)
        printf("[%d maps parsed]\n", (int)maps.size());
    
    return true;
}

string getVal(stringmap m, string s){
   stringmap::iterator entry = m.find(s);
   if(entry!=m.end()){
      return entry->second;
   } else {
      return "";
   }
}

static float EPSILON = 1e-5;
exifstub parseExifMap(exifmap m){
   exifstub ret;
   
   if( m.find("IFD0") != m.end() ){
      stringmap ifd0 = m["IFD0"];
      
      string Make        = getVal(ifd0,"Make");
      string Model       = getVal(ifd0,"Model");
      string ImageWidth  = getVal(ifd0,"ImageWidth");
      string ImageHeight = getVal(ifd0,"ImageHeight");
      
      // parsing
      
      int img_w,img_h;
      if( sscanf(ImageWidth.c_str(),"%d",&img_w) != 1 ){
         img_w = -1;
      }
      if( sscanf(ImageHeight.c_str(),"%d",&img_h) != 1 ){
         img_h = -1;
      }
      
      ret.image_w = img_w;
      ret.image_h = img_h;
      ret.make = Make;
      ret.model = Model;
      
   }
   
   if( m.find("ExifIFD") != m.end() ){
      stringmap ifd0 = m["ExifIFD"];
      
      string ExposureTime    = getVal(ifd0,"ExposureTime");
      string FNumber         = getVal(ifd0,"FNumber");
      string ISO             = getVal(ifd0,"ISO");
      string FocalLength     = getVal(ifd0,"FocalLength");
      
      string LensInfo = getVal(ifd0,"LensInfo");
      string LensModel = getVal(ifd0,"LensModel");
      
      string ExifImageWidth  = getVal(ifd0,"ExifImageWidth");
      string ExifImageHeight = getVal(ifd0,"ExifImageHeight");
      //        string ColorSpace      = getVal(ifd0,"ColorSpace");
      //        string FocalPlaneXResolution    = getVal(ifd0,"FocalPlaneXResolution");
      //        string FocalPlaneYResolution    = getVal(ifd0,"FocalPlaneYResolution");
      //        string FocalPlaneResolutionUnit = getVal(ifd0,"FocalPlaneResolutionUnit");
      //
      // apple
      string LensMake = getVal(ifd0,"LensMake");
      //
      //        // canon
      string LensSerialNumber = getVal(ifd0,"LensSerialNumber");
      string SerialNumber = getVal(ifd0,"SerialNumber");
      
      // parsing
      
      
      int img_w,img_h;
      if( sscanf(ExifImageWidth.c_str(),"%d",&img_w) != 1 ){
         img_w = -1;
      }
      if( sscanf(ExifImageHeight.c_str(),"%d",&img_h) != 1 ){
         img_h = -1;
      }
      
      if( ret.image_w < 0 || ret.image_h < 0){
         ret.image_w = img_w;
         ret.image_h = img_h;
      }
      
      // exposure time
      {
         float exp_t = 0;
         float a=0,b=1;
         int res = sscanf(ExposureTime.c_str(),"%f/%f",&a,&b);
         if(res==2&&b<EPSILON)
            b = 1;
         if(res==1){
            exp_t = a;
         } else {
            exp_t = a/b;
         }
         ret.exposure_t = exp_t;
      }
      
      // f-stop number
      float fnum;
      if( sscanf(FNumber.c_str(),"%f",&fnum) != 1 ){
         fnum = -1;
      }
      ret.fnumber = fnum;
      
      // ISO
      int iso;
      if( sscanf(ISO.c_str(),"%d",&iso) != 1 ){
         iso = -1;
      }
      ret.iso = iso;
      
      // FocalLength
      float fl;
      if( sscanf(FocalLength.c_str(),"%f",&fl) != 1 ){
         fl = -1;
      }
      ret.focal_length = fl;
      
      ret.lens_info = LensInfo;
      ret.lens_model = LensModel;
      ret.serial_num = SerialNumber;
      ret.serial_num_lens = LensSerialNumber;
      ret.lens_make = LensMake;
      
      
   }
   
   // apple only keys
   if( m.find("GPS") != m.end() ){
      stringmap ifd0 = m["GPS"];
      
      // iphone
      string GPSTimeStamp    = getVal(ifd0,"GPSTimeStamp");
      string GPSAltitudeRef  = getVal(ifd0,"GPSAltitudeRef");
      string GPSLatitudeRef  = getVal(ifd0,"GPSLatitudeRef");
      string GPSLongitudeRef = getVal(ifd0,"GPSLongitudeRef");
      string GPSImgDirectionRef = getVal(ifd0,"GPSImgDirectionRef");
      string GPSImgDirection = getVal(ifd0,"GPSImgDirection");
      
   }
   
   if( m.find("Composite") != m.end() ){
      stringmap ifd0 = m["Composite"];
      
      string ScaleFactor35efl = getVal(ifd0,"ScaleFactor35efl");
      string FocalLength35efl = getVal(ifd0,"FocalLength35efl");
      string HyperfocalDistance = getVal(ifd0,"HyperfocalDistance");
      
      string SubSecCreateDate = getVal(ifd0,"SubSecCreateDate");
      string SubSecDateTimeOriginal = getVal(ifd0,"SubSecDateTimeOriginal");
      
      string FOV = getVal(ifd0,"FOV");
      string LightValue = getVal(ifd0,"LightValue");
      
      // canon
      string Lens = getVal(ifd0,"Lens");
      string LensID = getVal(ifd0,"LensID");
      string WB_RGGBLevels = getVal(ifd0,"WB_RGGBLevels");
      
      // iphone
      string GPSAltitude = getVal(ifd0,"GPSAltitude");
      string GPSLatitude = getVal(ifd0,"GPSLatitude");
      string GPSLongitude = getVal(ifd0,"GPSLongitude");
      string GPSPosition = getVal(ifd0,"GPSPosition");
      
      // parsing
      
      
      // crop factor
      float crop_factor;
      if( sscanf(ScaleFactor35efl.c_str(),"%f",&crop_factor) != 1 ){
         crop_factor = -1;
      }
      ret.scale_factor_35efl = crop_factor;
      
      // hyperfocal distance
      float hfd;
      if( sscanf(HyperfocalDistance.c_str(),"%f",&hfd) != 1 ){
         hfd = -1;
      }
      ret.hyperfocaldistance = hfd;
      
      // fov
      float fov;
      if( sscanf(FOV.c_str(),"%f deg",&fov) != 1 ){
         fov = -1;
      }
      ret.fov = fov;
      
      // fov
      float lv;
      if( sscanf(LightValue.c_str(),"%f",&lv) != 1 ){
         lv = -1;
      }
      ret.light_value = lv;
      
   }
   
   if( m.find("Canon") != m.end() ){
      stringmap ifd0 = m["Canon"];
      
      string InternalSerialNumber = getVal(ifd0,"InternalSerialNumber");
      
      string LensType = getVal(ifd0,"LensType");
      
      string MaxFocalLength = getVal(ifd0,"MaxFocalLength");
      string MinFocalLength = getVal(ifd0,"MinFocalLength");
      string FocalUnits = getVal(ifd0,"FocalUnits");
      string MaxAperture = getVal(ifd0,"MaxAperture");
      string MinAperture = getVal(ifd0,"MinAperture");
      string MeasuredEV = getVal(ifd0,"MeasuredEV");
      string CameraTemperature = getVal(ifd0,"CameraTemperature");
      string CameraOrientation = getVal(ifd0,"CameraOrientation");
      string FocusDistanceUpper = getVal(ifd0,"FocusDistanceUpper");
      string FocusDistanceLower = getVal(ifd0,"FocusDistanceLower");
      string CanonModelID = getVal(ifd0,"CanonModelID");
      
      string SensorWidth = getVal(ifd0,"SensorWidth");
      string SensorHeight = getVal(ifd0,"SensorHeight");
      string SensorLeftBorder = getVal(ifd0,"SensorLeftBorder");
      string SensorTopBorder = getVal(ifd0,"SensorTopBorder");
      string SensorRightBorder = getVal(ifd0,"SensorRightBorder");
      string SensorBottomBorder = getVal(ifd0,"SensorBottomBorder");
      string OriginalImageWidth = getVal(ifd0,"OriginalImageWidth");
      string OriginalImageHeight = getVal(ifd0,"OriginalImageHeight");
      
      ret.serial_num_internal = InternalSerialNumber;
      
   }
   
   
   return ret;
}

void printStub(exifstub s){
   printf("%s : %s\n",s.make.c_str(),s.model.c_str());
   printf("%s : %s\n",s.lens_info.c_str(),s.lens_model.c_str());
   printf("   exposure time = %f (%0.2fms)\n",s.exposure_t,s.exposure_t*1000);
   printf("   f/%0.1f : ISO %d\n",s.fnumber,s.iso);
   printf("   [ %d x %d ] @ %0.1f mm\n",s.image_w,s.image_h,s.focal_length);
   printf("   crop factor : %0.2f\n",s.scale_factor_35efl);
   printf("   fov = %0.1f deg ; EV = %0.2f\n",s.fov,s.light_value);
   printf("   hyperfocal distance = %0.2f m\n",s.hyperfocaldistance);
}


// read exif data for one file
// output into string -> string map
bool getExif(string fname, stringmap &_map){
   
    FILE *fp;
    char command[2048];
   
    // execute exiftool -s
    sprintf( command, "/usr/local/bin/exiftool -s %s 2>&1", fname.c_str() );
    fp = popen(command, "r");
   
    if (fp == NULL) {
        fprintf( stderr, "Failed to run command" );
        return false;
    }
    
    const int buf_n = 16384;
    char sbuf[buf_n];
    char kbuf[buf_n];
    char vbuf[buf_n];
    
    while( fgets(sbuf, sizeof(sbuf), fp) != NULL )
    {
        // abort if file not found
        char fnf[] = "File not found:";
        if(strncmp( sbuf,fnf, sizeof(fnf)-1 )==0){
            
            fprintf( stderr, "bad filename: %s\n", fname.c_str() );
            pclose(fp);
            return false;
        }
        
        // scan first and second parts of exiftool s1 : s2 format
        sscanf(sbuf,"%s : %[^\n]\n",kbuf,vbuf);
        string _k(kbuf);
        string _v(vbuf);
        _map.insert(pair<string,string>(_k,_v));
    }
    
    pclose(fp);
    
    return true;
}

