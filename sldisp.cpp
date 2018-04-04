#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <math.h>

#include <vector>
#include <string>

#include "util.h"
#include "ports.h"

#include "lo/lo.h"

#include <SDL.h>

#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#else
#ifdef _WIN32
  #include <windows.h>
#endif
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#endif

#define DBUG 1

#define UNIX_EPOCH 2208988800
#define TWO_POW_32 4294967296

using namespace std;

int done = 0;

void error(int num, const char *m, const char *path);


// init rendering and clear screen
bool initGL();
void handleKeys(unsigned char key, int x, int y);
//Shader loading utility programs
void printProgramLog( GLuint program );
void printShaderLog( GLuint shader );

int generic_handler(const char *path, const char *types, lo_arg ** argv,
                    int argc, void *data, void *user_data);
int display_info_request_handler(const char *path, const char *types, lo_arg ** argv,
                                 int argc, void *data, void *user_data);
int pattern_status_request_handler(const char *path, const char *types, lo_arg ** argv,
                                   int argc, void *data, void *user_data);
int pattern_change_request_handler(const char *path, const char *types, lo_arg ** argv,
                                   int argc, void *data, void *user_data);
int pattern_state_request_handler(const char *path, const char* types, lo_arg **argv, int argc, void *data, void *user_data);

int_least64_t maketimetag(double unixtime);

int send_display_info(lo_address the_t, int pix_w, int pix_h);
int send_pattern_status(lo_address the_t, int inv, int axis, int bitidx, float r, float g, float b);
int send_pattern_state(lo_address the_t, int type, float param1=-1., float param2=-1., float param3=-1., float param4=-1., float r=-1., float g=-1., float b=-1.);
int send_heartbeat(lo_address the_t);
void init_LUT(std::vector<unsigned int>& table);
unsigned int bin2gray( unsigned int num );
int gray2bin(int gray);

//--------------------------------------------------------------------------------

#define BIT_MAX 11
void init_LUT(std::vector<unsigned int>& table)
{
    table.clear();
    for(int i=0;i<pow(2,BIT_MAX);i++)
    {
        table.push_back( bin2gray((unsigned int)i) );
    }
    if(false)
    {
        for(int i=0;i<table.size();i++)
        {
            printf("%d : 0x%08x = 0x%08x\n",i,i,table[i]);
        }
    }
}

typedef struct {
    // pattern info
    int cur_rendermode;
    int cur_axis;
    int cur_bitindex;
    bool cur_invert;
    float cur_freq;
    float cur_phase;
    float cur_r;
    float cur_g;
    float cur_b;

    int shown_rendermode;
    int shown_axis;
    int shown_bitindex;
    bool shown_invert;
    float shown_freq;
    float shown_phase;
    float shown_r;
    float shown_g;
    float shown_b;
    
    bool sdl_rdy;
    bool gl_rdy;
    bool done;
    bool dirty;
    int dirty_n;
    double dirty_t;
    double send_t;
    bool fullscreen;
    bool cursorVisible;
    int tick;
    
    int window_w;
    int window_h;
    int window_w_def;
    int window_h_def;
    
    // graphics stuff
    SDL_Window   *window;
    SDL_Renderer *renderer;

    vector<SDL_Texture*> gcb_tex_array;
    
    SDL_Texture* sdltexture;
    SDL_Surface* sdlsurface;
    SDL_Surface* gScreenSurface;
    
    SDL_Surface* sdlsurface_graycodeX;
    SDL_Surface* sdlsurface_graycodeY;
    SDL_Surface* sdlsurface_sinusoidX;
    SDL_Surface* sdlsurface_sinusoidY;
    SDL_Surface* sdlsurface_monochrome;

    SDL_Texture* sdl_tex_gcb_X;
    SDL_Texture* sdl_tex_gcb_Y;
    SDL_Texture* sdl_tex_sin_X;
    SDL_Texture* sdl_tex_sin_Y;
    SDL_Texture* sdl_tex_rgb;
    SDL_Texture* sdl_RT;

    GLuint texture;
    GLuint textureXpattern;
    GLuint textureYpattern;

    GLfloat GL_vertices[12];
    GLfloat GL_texcoord[8];
    GLubyte GL_indices[6];
    
    std::vector<unsigned int> gray_LUT;
    
} SERVER_STATE;

void init_SERVER_STATE( SERVER_STATE* s )
{
    s->cur_freq = 0;
    s->cur_phase = 0;
    s->cur_rendermode = 2;
    
    s->cur_axis  = 0;
    s->cur_bitindex = 0;
    s->cur_invert   = false;
    s->cur_r = 0.4;
    s->cur_g = 0.4;
    s->cur_b = 0.2;
    
    s->shown_rendermode = 0;
    s->shown_freq = 0;
    s->shown_phase = 0;
    s->shown_r = 1;
    s->shown_g = 1;
    s->shown_b = 1;
    
    s->sdl_rdy = false;
    s->gl_rdy = false;
    s->done = false;
    s->dirty = true;
    s->dirty_n = 1;
    s->dirty_t = 0;
    s->send_t = 0;
    s->fullscreen = false;
    s->cursorVisible = true;
    s->tick = 0;

    s->window_w_def = 1920;
    s->window_h_def = 1080;
    s->window_w  = s->window_w_def;
    s->window_h = s->window_h_def;
    
    s->gScreenSurface = NULL;
    s->sdltexture = NULL;
    s->sdlsurface = NULL;
    s->sdlsurface_graycodeX = NULL;
    s->sdlsurface_graycodeY = NULL;
    s->sdlsurface_sinusoidX = NULL;
    s->sdlsurface_sinusoidY = NULL;
    s->sdlsurface_monochrome = NULL;
    init_LUT(s->gray_LUT);

    // fullscreen texture stuff
    GLfloat GL_vertices[12];
    GLfloat GL_texcoord[8];
    GLubyte GL_indices[6];

    s->GL_vertices[0]  = 0;
    s->GL_vertices[1]  = 0;
    s->GL_vertices[2]  = 0;
    s->GL_vertices[3]  = s->window_w;
    s->GL_vertices[4]  = 0;
    s->GL_vertices[5]  = 0;
    s->GL_vertices[6]  = s->window_w;
    s->GL_vertices[7]  = s->window_h;
    s->GL_vertices[8]  = 0;
    s->GL_vertices[9]  = 0;
    s->GL_vertices[10] = s->window_h;
    s->GL_vertices[11] = 0;

    s->GL_texcoord[0] = 0;
    s->GL_texcoord[1] = 0;
    s->GL_texcoord[2] = 1;
    s->GL_texcoord[3] = 0;
    s->GL_texcoord[4] = 1;
    s->GL_texcoord[5] = 1;
    s->GL_texcoord[6] = 0;
    s->GL_texcoord[7] = 1;

    s->GL_indices[0] = 0;
    s->GL_indices[1] = 1;
    s->GL_indices[2] = 2;
    s->GL_indices[3] = 0;
    s->GL_indices[4] = 2;
    s->GL_indices[5] = 3;
    
}

SERVER_STATE state;

/**
 * Log an SDL error with some error message to the output stream of our choice
 * @param os The output stream to write the message too
 * @param msg The error message to write, format will be msg error: SDL_GetError()
 */
void logSDLError(std::ostream &os, const std::string &msg)
{
    os << msg << " error: " << SDL_GetError() << std::endl;
}

// convert unsigned binary to gray code value
unsigned int bin2gray( unsigned int num )
{
    return (num>>1) ^ num;
}

// Convert a 10-bit Gray code value into a binary integer representation
// Thanks to http://www.dspguru.com/dsp/tricks/gray-code-conversion
int gray2bin(int gray)
{
    unsigned int temp = gray ^ (gray>>8);
    
    // Our error condition should propagate
    if(gray == -1) return -1;
    
    temp ^= (temp>>4);
    temp ^= (temp>>2);
    temp ^= (temp>>1);
    return temp;
}



// pattern generation calls


// bit [0..gray-1]
// axis (0=x) (1=y)
void gen_graycode(SDL_Surface* in, std::vector<unsigned int>& lut, bool invert, int axis, int bit, float red, float grn, float blu)
{
    if( SDL_MUSTLOCK( in ) ) {
        SDL_LockSurface( in );
    }
    
    // CV_8UC3
    // automatically fill the matrix (expects preallocated)
    int w = in->w;
    int h = in->h;
    unsigned char *img_data = (unsigned char*)(in->pixels);
    for(int r=0; r<h; r++) {
        for(int c=0; c<w; c++) {
            int idx = (r*w+c)*4;
            int address = (axis==0)?c:r;
            float v = ((lut[address]>>bit)&1) == 1 ? 1.0 : 0.0;
            v = invert?1.0-v:v;
            
            img_data[idx]   = static_cast<unsigned char>(255*blu*v);
            img_data[idx+1] = static_cast<unsigned char>(255*grn*v);
            img_data[idx+2] = static_cast<unsigned char>(255*red*v);
            img_data[idx+3] = 255;
            
            //v = ( ( bin2gray( (unsigned int) c ) >> bit ) & 1 ) == 1 ? 1.0 : 0.0;
            //v = ( ( bin2gray( (unsigned int) r ) >> bit ) & 1 ) == 1 ? 1.0 : 0.0;
        }
    }
    if( SDL_MUSTLOCK( in ) ) {
        //Lock the surface
        SDL_UnlockSurface( in );
    }
}

void gen_sine(SDL_Surface* in, bool invert, int axis, float freq, float phase, float red, float grn, float blu)
{
    if( SDL_MUSTLOCK( in ) ) {
        //Lock the surface
        SDL_LockSurface( in );
    }
    
    // 8UC3
    // automatically fill the matrix (expects preallocated)
    int w = in->w;
    int h = in->h;
    unsigned char *img_data = (unsigned char*)(in->pixels);
    for(int r=0; r<h; r++) {
        for(int c=0; c<w; c++) {
            int idx = (r*w+c)*4;
            float nx = (c/(float)(w-1));
            float ny = (r/(float)(h-1));
            float v = 0;
            //M_PI
            
            v = 0.5f + 0.5f * sin(2 * 3.14159265359f * freq * ((axis==0)?nx:ny) + phase);
            v = invert ? 1.0-v : v;
            v = fmax(fmin(v,1.0f),0.0f);
            
            img_data[idx]   = static_cast<unsigned char>(255*blu*v);
            img_data[idx+1] = static_cast<unsigned char>(255*grn*v);
            img_data[idx+2] = static_cast<unsigned char>(255*red*v);
            img_data[idx+3] = 255;
        }
    }
    
    if( SDL_MUSTLOCK( in ) ) {
        //Lock the surface
        SDL_UnlockSurface( in );
    }
}

// same r g b values are copied to every pixel onscreen
//void gen_rgb(SDL_Surface* in, float red, float grn, float blu)
void gen_rgb(SDL_Surface* in, float red, float grn, float blu)
{
    if( SDL_MUSTLOCK( in ) ) {
        SDL_LockSurface( in );
    }
    
    // CV_8UC3
    // automatically fill the matrix (expects preallocated)
    int w = in->w;
    int h = in->h;
    unsigned char *img_data = (unsigned char*)(in->pixels);
    for(int r=0; r<h; r++) {
        for(int c=0; c<w; c++) {
            /*
            Uint32 pixelPosition = r * (pitch / sizeof(unsigned int)) + c;
         if(false){
            Uint32 color = SDL_MapRGB(&pixelFormat,\
                    static_cast<Uint8>(fmin(fmax(255*blu,0),255)),\
                    static_cast<Uint8>(fmin(fmax(255*blu,0),255)),\
                    static_cast<Uint8>(fmin(fmax(255*blu,0),255)));
            
            pixels[pixelPosition] = color;
        }
        */
            int idx = (r*w+c)*4;
            img_data[idx]   = static_cast<unsigned char>(fmin(fmax(255*blu,0),255));
            img_data[idx+1] = static_cast<unsigned char>(fmin(fmax(255*grn,0),255));
            img_data[idx+2] = static_cast<unsigned char>(fmin(fmax(255*red,0),255));
            img_data[idx+3] = 255;
        }
    }
    if( SDL_MUSTLOCK( in ) ) {
        //Lock the surface
        SDL_UnlockSurface( in );
    }
    //SDL_UnlockTexture( in );
}



// initialization calls


// init SDL defaults
int configure_SDL(SERVER_STATE* the_state)
{
    // SDL INIT
    if (SDL_Init(SDL_INIT_VIDEO) != 0){
        logSDLError(std::cout, "SDL_Init");
        return 1;
    }
    
    // get current resolution
    SDL_DisplayMode videomode;
    if (SDL_GetCurrentDisplayMode (0, &videomode) != 0)
    {
        std::cerr << "Error getting current display mode: " << SDL_GetError() << std::endl;
        exit(EXIT_FAILURE);
    }
    std::cout << "Current screen mode: " << videomode.w << "x" << videomode.h << std::endl;
    the_state->window_w = videomode.w;
    the_state->window_h = videomode.h;
    
    // init window @ native res
    the_state->window = SDL_CreateWindow("structured light output",
                                         SDL_WINDOWPOS_CENTERED,
                                         SDL_WINDOWPOS_CENTERED,
                                         videomode.w, videomode.h,
                                         SDL_WINDOW_FULLSCREEN | SDL_WINDOW_OPENGL);
    if (the_state->window == NULL){
        logSDLError(std::cout, "CreateWindow");
        return 2;
    }
    
    // init renderer
    the_state->renderer = SDL_CreateRenderer(the_state->window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (the_state->renderer == NULL){
        logSDLError(std::cout, "CreateRenderer");
        return 3;
    }
    //SDL_GL_SetAttribute( SDL_GL_CONTEXT_MAJOR_VERSION, 2 );
    //SDL_GL_SetAttribute( SDL_GL_CONTEXT_MINOR_VERSION, 0 );
    //SDL_GL_SetAttribute( SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE );

    // print openGL version info
    int err, gl_maj_v, gl_min_v;
    err = SDL_GL_GetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION,&gl_maj_v);
    err = SDL_GL_GetAttribute(SDL_GL_CONTEXT_MINOR_VERSION,&gl_min_v);
    printf("SDL_GL_VERSION = [%d] [%d]\n",gl_maj_v,gl_min_v);
    
    // allocate textures and surfaces
    int w = the_state->window_w;
    int h = the_state->window_h;
    
    the_state->gScreenSurface = SDL_GetWindowSurface( the_state->window );
    
    the_state->sdlsurface            = SDL_CreateRGBSurface(0, w, h, 32,0,0,0,0);
    
    the_state->sdlsurface_graycodeX  = SDL_CreateRGBSurface(0, w, 1, 32,0,0,0,0);
    the_state->sdlsurface_graycodeY  = SDL_CreateRGBSurface(0, 1, h, 32,0,0,0,0);

    the_state->sdlsurface_sinusoidX  = SDL_CreateRGBSurface(0, w, 1, 32,0,0,0,0);
    the_state->sdlsurface_sinusoidY  = SDL_CreateRGBSurface(0, 1, h, 32,0,0,0,0);
    
    the_state->sdlsurface_monochrome = SDL_CreateRGBSurface(0, 1, 1, 32,0,0,0,0);

    /*
    the_state->sdltexture    = SDL_CreateTexture(the_state->renderer, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_STATIC, 1, 1);
    the_state->sdl_RT        = SDL_CreateTexture(the_state->renderer, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_TARGET, w, h);
    the_state->sdl_tex_gcb_X = SDL_CreateTexture(the_state->renderer, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_STATIC, w, 1);
    the_state->sdl_tex_gcb_Y = SDL_CreateTexture(the_state->renderer, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_STATIC, 1, h);
    the_state->sdl_tex_sin_X = SDL_CreateTexture(the_state->renderer, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_STATIC, w, 1);
    the_state->sdl_tex_sin_Y = SDL_CreateTexture(the_state->renderer, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_STATIC, 1, h);
    the_state->sdl_tex_rgb   = SDL_CreateTexture(the_state->renderer, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_STATIC, 1, 1);
    */
    
    return 0;
}

int configure_GL(SERVER_STATE* the_state)
{
  /*  
    bool success = true;
    the_state->gprogramID = glCreateProgram();
    
    GLuint vertexShader = glCreateShader( GL_VERTEX_SHADER ); 

    const GLchar* vertexShaderSource[] = { "#version 140\nin vec2 LVertexPos2D; void main() { gl_Position = vec4( LVertexPos2D.x, LVertexPos2D.y, 0, 1 ); }" };
    glShaderSource( vertexShader, 1, vertexShaderSource, NULL ); 
    glCompileShader( vertexShader ); 
    GLint vShaderCompiled = GL_FALSE;
    glGetShaderiv( vertexShader, GL_COMPILE_STATUS, &vShaderCompiled );
    
    if( vShaderCompiled != GL_TRUE ) {
        printf( "Unable to compile vertex shader %d!\n", vertexShader );
        printShaderLog( vertexShader ); success = false;
    }
*/


    // OpenGL setup
    
    // upload texture data
    glEnable(GL_TEXTURE_2D);
    glGenTextures( 1, &(the_state->texture) );
    glBindTexture( GL_TEXTURE_2D, the_state->texture );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );

    glGenTextures( 1, &(the_state->textureXpattern) );
    glBindTexture( GL_TEXTURE_2D, the_state->textureXpattern );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
    
    glGenTextures( 1, &(the_state->textureYpattern) );
    glBindTexture( GL_TEXTURE_2D, the_state->textureYpattern );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
    
    SDL_GL_SetSwapInterval(1);
    //glClearColor(0, 0, 0, 0);
    glMatrixMode(GL_PROJECTION|GL_MODELVIEW);
    glLoadIdentity();
    
    return 0;
}

/*
void draw_pattern_GL()
{
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
 
    glViewport(0,0,screen_size[0],screen_size[1]);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, (double)(screen_size[0])/(double)(screen_size[1]), 0.1,100.0);
    glMatrixMode(GL_MODELVIEW);
 
    glLoadIdentity();
    gluLookAt(2.0,3.0,4.0, 0.0,0.0,0.0, 0.0,1.0,0.0);
 
    //Set our loaded texture as the current 2D texture (this isn't actually technically necessary since our
    //texture was never unselected from above, but this is most clear)
    glBindTexture(GL_TEXTURE_2D,texture);
    //Tell OpenGL that all subsequent drawing operations should try to use the current 2D texture
    glEnable(GL_TEXTURE_2D);
 
    glBegin(GL_TRIANGLES);
    glTexCoord2f(0.0f,0.0f); //All subsequent vertices will have an associated texture coordinate of (0,0)
    glVertex3f( 0.0f, 0.1f, 0.0f);
    glTexCoord2f(1.0f,0.0f); //All subsequent vertices will have an associated texture coordinate of (1,0)
    glVertex3f(-0.1f,-0.1f, 0.7f);
    glTexCoord2f(0.0f,1.0f); //All subsequent vertices will have an associated texture coordinate of (0,1)
    glVertex3f( 1.0f,-0.2f, 0.0f);
    glEnd();
 
    //Tell OpenGL that all subsequent drawing operations should NOT try to use the current 2D texture
    glDisable(GL_TEXTURE_2D);
 
    glBegin(GL_LINES);
    glColor3f(1.0f,0.0f,0.0f); glVertex3f(0.0f,0.0f,0.0f); glVertex3f(1.0f,0.0f,0.0f);
    glColor3f(0.0f,1.0f,0.0f); glVertex3f(0.0f,0.0f,0.0f); glVertex3f(0.0f,1.0f,0.0f);
    glColor3f(0.0f,0.0f,1.0f); glVertex3f(0.0f,0.0f,0.0f); glVertex3f(0.0f,0.0f,1.0f);
    glColor3f(1.0f,1.0f,1.0f);
    glEnd();
 
    SDL_GL_SwapWindow(window);
}
*/




// pattern rendering calls

void draw_pattern_fast_GL(SDL_Texture* texture_in, SDL_Window *window_in, GLfloat* verts_in, GLfloat* texcoord_in, GLubyte* indices_in)
{
    int w,h;
    SDL_GetWindowSize(window_in,&w,&h);
    
    glClear(GL_COLOR_BUFFER_BIT);
    
    glViewport(0, 0, w, h);
    
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    
    glOrtho(0, w, h, 0, 1, -1);
    
    SDL_GL_BindTexture( texture_in, NULL, NULL);
    
    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(3, GL_FLOAT, 0, verts_in);

    glEnableClientState(GL_TEXTURE_COORD_ARRAY);
    glTexCoordPointer(2, GL_FLOAT, 0, texcoord_in);

    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_BYTE, indices_in);

    glDisableClientState(GL_TEXTURE_COORD_ARRAY);
    glDisableClientState(GL_VERTEX_ARRAY);

    SDL_GL_SwapWindow(window_in);
    SDL_UpdateWindowSurface(window_in);
}

void draw_pattern(SDL_Window *window)
{
    int w,h;
    SDL_GetWindowSize(window,&w,&h);
    
    glClear(GL_COLOR_BUFFER_BIT);
    
    glViewport(0, 0, w, h);
    
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    
    glOrtho(0, w, h, 0, 1, -1);
    
    glBegin(GL_QUADS);
    glTexCoord2f( 0, 0 ); glVertex3f(0, 0, 0);
    glTexCoord2f( 1, 0 ); glVertex3f(w, 0, 0);
    glTexCoord2f( 1, 1 ); glVertex3f(w, h, 0);
    glTexCoord2f( 0, 1 ); glVertex3f(0, h, 0);
    glEnd();
    /*
    glBegin(GL_TRIANGLES);
    glTexCoord2i( 0, 0 ); glVertex3f(0, 0, 1);
    glTexCoord2i( 1, 0 ); glVertex3f(w, 0, 1);
    glTexCoord2i( 0, 0 ); glVertex3f(0, 0, 1);
    glTexCoord2i( 0, 0 ); glVertex3f(0, 0, 1);
    glTexCoord2i( 1, 1 ); glVertex3f(w, h, 1);
    glTexCoord2i( 0, 1 ); glVertex3f(0, h, 1);
    glEnd();
    */  
    
    SDL_UpdateWindowSurface(window);
    //SDL_GL_SwapWindow(window);
}






int main(int argc, char *argv[])
{
    init_SERVER_STATE(&state);
    SERVER_STATE *s = &state;
    
    int sdl_err = configure_SDL(s);
    if(!sdl_err) { s->sdl_rdy = true; }
    
    int gl_err = configure_GL(s);
    if(!sdl_err) { s->gl_rdy = true; }
    

    // osc setup
    lo_server_thread st  = lo_server_thread_new(SERVER_DISPLAY_PORT, error);
    lo_server_thread_add_method(st,  NULL, NULL, generic_handler, NULL);
    // axis(x=0,y=1), bitindex(i>>N), inverted(0=F,1=T), R, G, B(0,1.0)
    //lo_server_thread_add_method(st, "/pattern_change", "iiifff", pattern_change_request_handler, NULL);
    lo_server_thread_add_method(st, "/display_info",   "i",      display_info_request_handler,   NULL);
    lo_server_thread_add_method(st, "/pattern_status", "i",      pattern_status_request_handler, NULL);
    lo_server_thread_add_method(st, "/pattern_state", "iiiifff",  pattern_state_request_handler, NULL); // gcb
    lo_server_thread_add_method(st, "/pattern_state", "iiifffff", pattern_state_request_handler, NULL); // sin
    lo_server_thread_add_method(st, "/pattern_state", "iifff",    pattern_state_request_handler, NULL); // rgb
    
    lo_server_thread_start(st);
    
    // slcapture.py
    lo_address t = lo_address_new(SERVER_CONTROL_ADDRESS, SERVER_CONTROL_PORT);
    
    // test sending display info packet
    if (send_display_info(t, 1920,1080) == -1) {
        printf("OSC error %d: %s\n", lo_address_errno(t), lo_address_errstr(t));
    }
    
    // test sending pattern status packet
    if(false)
    {
        int axis = 0;
        int bitidx = 0;
        int inv = 0;
        float r,g,b;
        r=1.0; g=1.0; b=1.0;
        //send_pattern_status(t, axis, bitidx, inv,r,g,b);

    }
    
    
    
    while (!(s->done)) {
	s->tick++;
         
        if( state.fullscreen ){
            SDL_ShowCursor(0);
        }

	SDL_Event event;
	while( SDL_PollEvent(&event) ){
	    switch (event.type) {
		case SDL_KEYDOWN:
		{
		    switch(event.key.keysym.sym){

			case SDLK_ESCAPE:
		        {
			    s->done = true;
			    break;
			}
                        
                        case 't':
                        {

                        }

			case 'f': // F key
			{
			    state.dirty = true;
			    state.fullscreen = !state.fullscreen;
			 

			    // transition to fullscreen
			    if( state.fullscreen ){
			       
				int disp = SDL_GetWindowDisplayIndex(s->window);
				printf("disp = %d\n",disp);
				SDL_Rect rect;
				SDL_GetDisplayBounds(disp, &rect);
				
				s->window_w = rect.w;
				s->window_h = rect.h;
				printf("new size: %d x %d\n",rect.w,rect.h);
				
				SDL_SetWindowFullscreen(s->window,SDL_WINDOW_FULLSCREEN_DESKTOP);
				
				SDL_ShowCursor(0);
				
				// cause heartbeat
				s->dirty_n = 60;
				
			     } else
				// transition to windowed
			     {
				SDL_SetWindowFullscreen(s->window,0);
				SDL_ShowCursor(1);
				SDL_SetWindowBordered(s->window,SDL_FALSE);
				
				s->dirty_n = 60;
				
				int w,h;
				SDL_GetWindowSize(s->window,&w,&h);
				SDL_SetWindowSize(s->window,
				    s->window_w_def,
				    s->window_h_def);
				    
				printf("new size: %d x %d\n",w,h);
			    }
			    
			    break;
			}

			break;
		    }
		    break;
		}
		case SDL_MOUSEBUTTONDOWN:
		{
		    break;
		}
            
		case SDL_MOUSEMOTION:
		{
		    break;
		}
	    }
	}
	// events processed
	
	// refresh windows dimensions
	if(s->fullscreen){
	    int disp = SDL_GetWindowDisplayIndex(s->window);
	    SDL_Rect rect;
	    SDL_GetDisplayBounds(disp, &rect);
	    s->window_w  = rect.w;
	    s->window_h = rect.h;
	} else {
	    int w,h;
	    SDL_GetWindowSize(s->window,&w,&h);
	    s->window_w  = w;
	    s->window_h = h;
	}
        
        // generate pattern
	// refresh display
        if( s->dirty_n > 0 ) { //|| ( s->dirty_n > 0 && curtime() - s->dirty_t > s->dirty_interval ) ){
            s->dirty = true;

            //SDL_SetRenderTarget(s->renderer, s->sdl_RT);
            //SDL_SetRenderDrawColor(s->renderer, 0x00, 0x00, 0x00, 0x00);
            //SDL_RenderClear( s->renderer );
            //SDL_SetRenderDrawColor( s->renderer, 255,255,255,255);

	    s->cur_r=fmin(fmax(s->cur_r,0),1);
	    s->cur_g=fmin(fmax(s->cur_g,0),1);
	    s->cur_b=fmin(fmax(s->cur_b,0),1);
            
            s->shown_r = s->cur_r;
            s->shown_g = s->cur_g;
            s->shown_b = s->cur_b;

            s->shown_rendermode = s->cur_rendermode;
            
            s->shown_invert     = s->cur_invert;
            s->shown_axis       = s->cur_axis;
            s->shown_bitindex   = s->cur_bitindex;
            s->shown_freq       = s->cur_freq;
            s->shown_phase      = s->cur_phase;
            
            SDL_Rect screen_rect;
            screen_rect.x = 0;
            screen_rect.y = 0;
            screen_rect.w = s->window_w;
            screen_rect.h = s->window_h;
            
            SDL_SetRenderDrawColor(s->renderer, 255, 0, 255, 255);
            SDL_RenderClear(s->renderer);            

            // GCB
            if(s->shown_rendermode==0)
            {
                // x axis surface
                if( s->shown_axis == 0 )
                {
                    double t0 = curtime();
                    
                    gen_graycode( s->sdlsurface_graycodeX, s->gray_LUT, s->shown_invert, 0, s->shown_bitindex, s->cur_r, s->cur_g, s->cur_b);

                    double t1 = curtime();
                    
                    if(s->sdl_tex_gcb_X!=NULL)
                    {
                        SDL_DestroyTexture(s->sdl_tex_gcb_X);
                    }
                    s->sdl_tex_gcb_X = SDL_CreateTextureFromSurface(s->renderer, s->sdlsurface_graycodeX);

                    double t2 = curtime();

                    SDL_RenderCopy( s->renderer, s->sdl_tex_gcb_X, NULL, NULL);

                    double t3 = curtime();
                    
                    SDL_RenderPresent(s->renderer);

                    double t4 = curtime();

                    printf("[%0.6f] gen_graycode                   took [%0.6f s]\n",t0,t1-t0);
                    printf("[%0.6f] SDL_CreateTextureFromSurface   took [%0.6f s]\n",t1,t2-t1);
                    printf("[%0.6f] SDL_RenderCopy                 took [%0.6f s]\n",t2,t3-t2);
                    printf("[%0.6f] SDL_RenderPresent              took [%0.6f s]\n",t3,t4-t3);
                    printf("[%0.6f] TOTAL                          took [%0.6f s] (%0.2f fps)\n",t4,t4-t0,1.0f/(t4-t0));
                    
                }

                // y axis surface
                if( s->shown_axis == 1 )
                {
                    double t0 = curtime();
                    
                    gen_graycode( s->sdlsurface_graycodeY, s->gray_LUT, s->shown_invert, 1, s->shown_bitindex, s->cur_r, s->cur_g, s->cur_b);

                    double t1 = curtime();
                    
                    if(s->sdl_tex_gcb_Y!=NULL)
                    {
                        SDL_DestroyTexture(s->sdl_tex_gcb_Y);
                    }
                    s->sdl_tex_gcb_Y = SDL_CreateTextureFromSurface(s->renderer, s->sdlsurface_graycodeY);

                    double t2 = curtime();

                    SDL_RenderCopy( s->renderer, s->sdl_tex_gcb_Y, NULL, NULL);

                    double t3 = curtime();
                    
                    SDL_RenderPresent(s->renderer);

                    double t4 = curtime();

                    printf("[%0.6f] gen_graycode                   took [%0.6f s]\n",t0,t1-t0);
                    printf("[%0.6f] SDL_CreateTextureFromSurface   took [%0.6f s]\n",t1,t2-t1);
                    printf("[%0.6f] SDL_RenderCopy                 took [%0.6f s]\n",t2,t3-t2);
                    printf("[%0.6f] SDL_RenderPresent              took [%0.6f s]\n",t3,t4-t3);
                    printf("[%0.6f] TOTAL                          took [%0.6f s] (%0.2f fps)\n",t4,t4-t0,1.0f/(t4-t0));

                    
                }
            }
            // SIN 
            if(s->shown_rendermode==1)
            {
                // x axis surface
                if( s->shown_axis == 0 )
                {
                    double t0 = curtime();
                    
                    gen_sine(s->sdlsurface_sinusoidX, s->shown_invert, s->shown_axis, s->shown_freq, s->shown_phase, s->shown_r, s->shown_g, s->shown_b);

                    double t1 = curtime();
                    
                    if(s->sdl_tex_sin_X!=NULL)
                    {
                        SDL_DestroyTexture(s->sdl_tex_sin_X);
                    }
                    s->sdl_tex_sin_X = SDL_CreateTextureFromSurface(s->renderer, s->sdlsurface_sinusoidX);

                    double t2 = curtime();

                    SDL_RenderCopy( s->renderer, s->sdl_tex_sin_X, NULL, NULL);

                    double t3 = curtime();
                    
                    SDL_RenderPresent(s->renderer);

                    double t4 = curtime();

                    printf("[%0.6f] gen_sine                       took [%0.6f s]\n",t0,t1-t0);
                    printf("[%0.6f] SDL_CreateTextureFromSurface   took [%0.6f s]\n",t1,t2-t1);
                    printf("[%0.6f] SDL_RenderCopy                 took [%0.6f s]\n",t2,t3-t2);
                    printf("[%0.6f] SDL_RenderPresent              took [%0.6f s]\n",t3,t4-t3);
                    printf("[%0.6f] TOTAL                          took [%0.6f s] (%0.2f fps)\n",t4,t4-t0,1.0f/(t4-t0));

                }
                
                // y axis surface
                if( s->shown_axis == 1 )
                {
                    double t0 = curtime();
                    
                    gen_sine(s->sdlsurface_sinusoidY, s->shown_invert, s->shown_axis, s->shown_freq, s->shown_phase, s->shown_r, s->shown_g, s->shown_b);

                    double t1 = curtime();
                    
                    if(s->sdl_tex_sin_Y!=NULL)
                    {
                        SDL_DestroyTexture(s->sdl_tex_sin_Y);
                    }
                    s->sdl_tex_sin_Y = SDL_CreateTextureFromSurface(s->renderer, s->sdlsurface_sinusoidY);

                    double t2 = curtime();

                    SDL_RenderCopy( s->renderer, s->sdl_tex_sin_Y, NULL, NULL);

                    double t3 = curtime();
                    
                    SDL_RenderPresent(s->renderer);

                    double t4 = curtime();

                    printf("[%0.6f] gen_sine                       took [%0.6f s]\n",t0,t1-t0);
                    printf("[%0.6f] SDL_CreateTextureFromSurface   took [%0.6f s]\n",t1,t2-t1);
                    printf("[%0.6f] SDL_RenderCopy                 took [%0.6f s]\n",t2,t3-t2);
                    printf("[%0.6f] SDL_RenderPresent              took [%0.6f s]\n",t3,t4-t3);
                    printf("[%0.6f] TOTAL                          took [%0.6f s] (%0.2f fps)\n",t4,t4-t0,1.0f/(t4-t0));

                }
            }
            
            // RGB
            if(s->shown_rendermode==2)
            {
                double t0 = curtime();
                
                gen_rgb(s->sdlsurface_monochrome, s->shown_r, s->shown_g, s->shown_b);

                double t1 = curtime();
                 
                if(s->sdltexture!=NULL)
                {
                    SDL_DestroyTexture(s->sdltexture);
                }
                s->sdltexture = SDL_CreateTextureFromSurface(s->renderer, s->sdlsurface_monochrome);

                double t2 = curtime();

                SDL_RenderCopy( s->renderer, s->sdltexture, NULL, &screen_rect);

                double t3 = curtime();
                
                SDL_RenderPresent(s->renderer);

                double t4 = curtime();

                printf("[%0.6f] gen_rgb                        took [%0.6f s]\n",t0,t1-t0);
                printf("[%0.6f] SDL_CreateTextureFromSurface   took [%0.6f s]\n",t1,t2-t1);
                printf("[%0.6f] SDL_RenderCopy                 took [%0.6f s]\n",t2,t3-t2);
                printf("[%0.6f] SDL_RenderPresent              took [%0.6f s]\n",t3,t4-t3);
                printf("[%0.6f] TOTAL                          took [%0.6f s] (%0.2f fps)\n",t4,t4-t0,1.0f/(t4-t0));

                
            }

	    s->dirty_n--;
            s->dirty_t = curtime();

	    if(s->dirty_n<1)
	    {
		s->dirty = false;
	    }
            
        }

        if( s->dirty == false && curtime() > s->dirty_t + 3/60.0f && curtime() > s->send_t + 15/60.0f ) {
            if(s->cur_rendermode == 0)
            {
                send_pattern_state(t, s->cur_rendermode, (s->shown_invert)?1.0:0.0, (float)(s->shown_axis), (float)(s->shown_bitindex), s->cur_r,s->cur_g,s->cur_b);
            }
            if(s->cur_rendermode == 1)
            {
                send_pattern_state(t, s->cur_rendermode, (s->shown_invert)?1.0:0.0, (float)(s->shown_axis), (float)(s->shown_bitindex), 0.0, s->cur_r,s->cur_g,s->cur_b);
            }
            if(s->cur_rendermode == 2)
            {
                send_pattern_state(t, s->cur_rendermode, (s->shown_invert)?1.0:0.0, s->cur_r,s->cur_g,s->cur_b);
            }
            s->send_t = curtime();
        }
        
	#ifdef WIN32
	    Sleep(1);
	#else
            usleep(8333);
	    //usleep(1000);
	#endif
    }
        
    SDL_DestroyRenderer(s->renderer);
    SDL_DestroyWindow(s->window);
   
    SDL_Quit();
   
    lo_server_thread_free(st);
    
    return 0;
}

void error(int num, const char *msg, const char *path)
{
    printf("liblo server error %d in path %s: %s\n", num, path, msg);
    fflush(stdout);
}

int_least64_t maketimetag(double unixtime)
{
    double ntpsec = unixtime + (double)UNIX_EPOCH;

    return 0;
}







/////////////////////////////////////////////////
// INCOMING PACKET HANDLERS
//

// catch any incoming messages and display them. returning 1 means that the
//message has not been fully handled and the server should try other methods
int generic_handler(const char *path, const char *types, lo_arg ** argv,
                    int argc, void *data, void *user_data)
{
    if( DBUG )
    {
        int i;
        
        printf("path: <%s>\n", path);
        for (i = 0; i < argc; i++) {
            printf("arg %d '%c' ", i, types[i]);
            lo_arg_pp((lo_type)types[i], argv[i]);
            printf("\n");
        }
        printf("\n");
        fflush(stdout);
    }
    
    return 1;
}

// process request to inform server of display info (resolution)
int display_info_request_handler(const char *path, const char *types, lo_arg ** argv,
                                 int argc, void *data, void *user_data)
{
    printf("*RECEIVED display_info_request* [@%0.3f]\n",curtime());
    
    return 1;
}

// process a request to inform server of currently displayed pattern
int pattern_status_request_handler(const char *path, const char *types, lo_arg ** argv,
                                   int argc, void *data, void *user_data)
{
    printf("*RECEIVED pattern_status_request* [@%0.3f]\n",curtime());
    int val    = argv[0]->i;
    
    return 1;
}

// generic handler for gray code, sinusoid, and monochrome
int pattern_state_request_handler(const char *path, const char* types, lo_arg **argv, int argc, void *data, void *user_data)
{
    //lo_server_thread_add_method(st, "/pattern_state", "iiiifff", pattern_state_request_handler, NULL);
    //lo_server_thread_add_method(st, "/pattern_state", "iifffff", pattern_state_request_handler, NULL);
    //lo_server_thread_add_method(st, "/pattern_state", "ifff",    pattern_state_request_handler, NULL);
    
    int type = fmin(fmax((int)(argv[0]->i),0),2);

    //graycode
    if( type == 0 && strcmp("iiiifff",types) == 0 )
    {
        bool inv = ((argv[1]->i)==0)?false:true;
        int axis =  argv[2]->i;
        int bit  =  argv[3]->i;
        float r  =  argv[4]->f;
        float g  =  argv[5]->f;
        float b  =  argv[6]->f;
        state.cur_invert = inv;
        // if bit == -1 do not change
        if(bit!=-1)
            state.cur_bitindex = bit;
        // if axis == -1 do not change
        if(axis>=0)
            state.cur_axis = axis;
        state.cur_r = r;
        state.cur_g = g;
        state.cur_b = b;
        
        state.dirty_n = 1;
        state.cur_rendermode = type;
        return 0;
    } 

    //sinusoid
    if( type == 1 && strcmp("iiifffff",types) == 0 )
    {
        bool  inv   =(argv[1]->i)==0?false:true;
        int   axis  = argv[2]->i;
        float freq  = argv[3]->f;
        float phase = argv[4]->f;
        float r    =  argv[5]->f;
        float g    =  argv[6]->f;
        float b    =  argv[7]->f;
        state.cur_freq = freq;
        state.cur_phase = phase;
        state.cur_invert = inv;
        state.cur_r = r;
        state.cur_g = g;
        state.cur_b = b;
        state.dirty_n = 1;
        state.cur_rendermode = type;
        return 0;
    }
    
    //monochrome flat
    if( type == 2 && strcmp("iifff",types) == 0 )
    {
        bool  inv = (argv[1]->i)==0?false:true;
        float r   =  argv[2]->f;
        float g   =  argv[3]->f;
        float b   =  argv[4]->f;
        state.cur_invert = inv;
        state.cur_r = r;
        state.cur_g = g;
        state.cur_b = b;
        state.cur_rendermode = type;
        state.dirty_n = 1;
        return 0;
    }
    
   return 1; 
}

// process a request to change the currently displayed pattern
int pattern_change_request_handler(const char *path, const char *types, lo_arg ** argv,
                                   int argc, void *data, void *user_data)
{
    //printf("*RECEIVED pattern_change_request* [@%0.3f]\n",curtime());
    //int axis     = argv[0]->i;
    //int index    = argv[1]->i;
    //int inverted = argv[2]->i;
    
    /* example showing pulling the argument values out of the argv array */
    int axis     = argv[0]->i;
    int bit      = argv[1]->i;
    bool inv     = (argv[2]->i)==0?false:true;
    float r      = argv[3]->f;
    float g      = argv[4]->f;
    float b      = argv[5]->f;
    state.cur_r = r;
    state.cur_g = g;
    state.cur_b = b;

    printf("[@%0.3f] *RECV pattern_change* ",curtime());
    printf("[axis:%d][idx:%d][inv:%c] [r,g,b=%0.2f,%0.2f,%0.2f]\n",
                axis, bit, inv?'T':'F', r,g,b);
    printf("%s <- i:%d i:%d i:%d\n\n", path, argv[0]->i, argv[1]->i, argv[2]->i);
    fflush(stdout);
    
    state.dirty_n = 1;
    
    // if bit == -1 do not change
    if(bit!=-1)
        state.cur_bitindex = bit;
    
    // if axis == -1 do not change
    if(axis!=-1)
        state.cur_axis = axis;
    
    state.cur_invert = inv;
    
    return 0;
}





////////////////////////////
// OUTGOING PACKET SENDERS
int send_display_info(lo_address the_t, int pix_w, int pix_h)
{
    int ret =  lo_send(the_t, "/display_info", "ii", pix_w, pix_h);
    
    return ret;
}

int send_pattern_status(lo_address the_t, int inv, int axis, int bitidx, float r, float g, float b)
{
    int ret =  lo_send(the_t, "/pattern_status", "iiifff", inv, axis, bitidx, r,g,b);
    
    return ret;
}

int send_pattern_state(lo_address the_t, int type, float param1, float param2, float param3, float param4, float r, float g, float b)
{
    int ret =  lo_send(the_t, "/pattern_state", "ifffffff", type, param1, param2, param3, param4, r,g,b);
    
    return ret;
}

int send_heartbeat(lo_address the_t)
{
    int ret =  lo_send(the_t, "/heartbeat", "i", 1);
    
    return ret;
}



/* vi:set ts=8 sts=4 sw=4: */
