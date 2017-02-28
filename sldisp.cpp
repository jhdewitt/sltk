#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <math.h>
#include <vector>

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

int done = 0;

void error(int num, const char *m, const char *path);


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
int send_pattern_state(lo_address the_t, int type, int param1, int param2, int param3, float r, float g, float b);
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
    for(int i=0;i<table.size();i++)
    {
        //printf("%d : 0x%08x = 0x%08x\n",i,i,table[i]);
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
    bool done;
    bool dirty;
    int dirty_n;
    double dirty_t;
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
    SDL_GLContext context;

    SDL_Texture* sdltexture;
    SDL_Surface* sdlsurface;

    SDL_Surface* gScreenSurface;
    
    GLuint texture;
    std::vector<unsigned int> gray_LUT;
    
} CLIENT_STATE;

void init_client_state( CLIENT_STATE* s )
{
    s->cur_freq = 0;
    s->cur_phase = 0;
    s->cur_rendermode = 2;
    
    s->cur_axis  = 0;
    s->cur_bitindex = 0;
    s->cur_invert   = false;
    s->cur_r = 0.1;
    s->cur_g = 0.1;
    s->cur_b = 0.1;
    
    s->shown_rendermode = 0;
    s->shown_freq = 0;
    s->shown_phase = 0;
    s->shown_r = 1;
    s->shown_g = 1;
    s->shown_b = 1;
    
    s->sdl_rdy = false;
    s->done = false;
    s->dirty = true;
    s->dirty_n = 1;
    s->dirty_t = 0;
    s->fullscreen = false;
    s->cursorVisible = true;
    s->tick = 0;

    s->window_w_def = 1920;
    s->window_h_def = 1080;
    s->window_w  = s->window_w_def;
    s->window_h = s->window_h_def;
    s->gScreenSurface = NULL;
    s->sdlsurface = NULL;
    init_LUT(s->gray_LUT);
}

CLIENT_STATE state;

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



// bit [0..gray-1]
// axis (0=x) (1=y)
void gen_graycode(SDL_Surface* in,std::vector<unsigned int>& lut, bool invert, int axis, int bit, float red, float grn, float blu)
{
    if( SDL_MUSTLOCK( in ) ) {
        //Lock the surface
        SDL_LockSurface( in );
    }
    // CV_16UC2
    // automatically fill the matrix (expects preallocated)
    int w = in->w;
    unsigned char *img_data = (unsigned char*)(in->pixels);
    for(int r=0; r<in->h; r++){
        for(int c=0; c<in->w; c++){
            int idx = r*w*3+c*3;
            float v = 0;
            if( axis == 0 ) // horizontal or x-axis
            {
                //v = ( ( bin2gray( (unsigned int) c ) >> bit ) & 1 ) == 1 ? 1.0 : 0.0;
                v = ((lut[c]>>bit)&1) == 1 ? 1.0 : 0.0;
            }
            if( axis == 1 ) // vertical or y-axis
            {
                //v = ( ( bin2gray( (unsigned int) r ) >> bit ) & 1 ) == 1 ? 1.0 : 0.0;
                v = ((lut[r]>>bit)&1) == 1 ? 1.0 : 0.0;
            }
            
            if( invert ) // invert
                v = 1.0 - v;
            
            img_data[idx]   = static_cast<unsigned char>(fmin(fmax(255*blu*v,0),255));
            img_data[idx+1] = static_cast<unsigned char>(fmin(fmax(255*grn*v,0),255));
            img_data[idx+2] = static_cast<unsigned char>(fmin(fmax(255*red*v,0),255));
            //img_data[idx+1] = img_data[idx];
            //img_data[idx+2] = img_data[idx];
        }
    }
    if( SDL_MUSTLOCK( in ) ) {
        //Lock the surface
        SDL_UnlockSurface( in );
    }
}

void gen_sine(SDL_Surface* in, int axis, float f, float p, float red, float grn, float blu)
{
    if( SDL_MUSTLOCK( in ) ) {
        //Lock the surface
        SDL_LockSurface( in );
    }
    // 8UC3
    // automatically fill the matrix (expects preallocated)
    int w = in->w;
    unsigned char *img_data = (unsigned char*)(in->pixels);
    for(int r=0; r<in->h; r++){
        for(int c=0; c<in->w; c++){
            float nx = (c/(float)((in->w)-1));
            float ny = (r/(float)((in->h)-1));
            float v = 0;
            if( axis == 0 ) // horizontal or x-axis
            {
                v = sin(2*M_PI*f*nx + p);
            }
            if( axis == 1 ) // vertical or y-axis
            {
                v = sin(2*M_PI*f*ny + p);
            }
            
            //if( invert ) // invert
            //    v = 1.0 - v;
            
            int idx = r*w*3+c*3;
            img_data[idx]   = static_cast<unsigned char>(fmin(fmax(255*blu*v,0),255));
            img_data[idx+1] = static_cast<unsigned char>(fmin(fmax(255*grn*v,0),255));
            img_data[idx+2] = static_cast<unsigned char>(fmin(fmax(255*red*v,0),255));
            //img_data[idx+1] = img_data[idx];
            //img_data[idx+2] = img_data[idx];
        }
    }
    if( SDL_MUSTLOCK( in ) ) {
        //Lock the surface
        SDL_UnlockSurface( in );
    }
}

// same r g b values are copied to every pixel onscreen
void gen_rgb(SDL_Surface* in, float red, float grn, float blu)
{
    if( SDL_MUSTLOCK( in ) ) {
        //Lock the surface
        SDL_LockSurface( in );
    }
    // CV_8UC3
    // automatically fill the matrix (expects preallocated)
    int w = in->w;
    unsigned char *img_data = (unsigned char*)(in->pixels);
    for(int r=0; r<in->h; r++){
        for(int c=0; c<in->w; c++){
            int idx = r*w*3+c*3;
            img_data[idx]   = static_cast<unsigned char>(fmin(fmax(255*blu,0),255));
            img_data[idx+1] = static_cast<unsigned char>(fmin(fmax(255*grn,0),255));
            img_data[idx+2] = static_cast<unsigned char>(fmin(fmax(255*red,0),255));
        }
    }
    if( SDL_MUSTLOCK( in ) ) {
        //Lock the surface
        SDL_UnlockSurface( in );
    }
}

// init SDL default with window renderer context
int configure_SDL(CLIENT_STATE* the_state)
{
    //Start up SDL and make sure it went ok
    if (SDL_Init(SDL_INIT_VIDEO) != 0){
        logSDLError(std::cout, "SDL_Init");
        return 1;
    }
    
    SDL_DisplayMode videomode;
    if (SDL_GetCurrentDisplayMode (0, &videomode) != 0)
    {
        std::cerr << "Error getting current display mode: " << SDL_GetError() << std::endl;
        exit(EXIT_FAILURE);
    }

    std::cout << "Current screen mode: " << videomode.w << "x" << videomode.h << std::endl;
    //Setup our window and renderer
    the_state->window = SDL_CreateWindow("structured light output",
                                         SDL_WINDOWPOS_CENTERED,
                                         SDL_WINDOWPOS_CENTERED,
                                         videomode.w, videomode.h,
                                         //SDL_WINDOW_OPENGL);
                                         SDL_WINDOW_FULLSCREEN | SDL_WINDOW_OPENGL);
    the_state->window_w = videomode.w;
    the_state->window_h = videomode.h;
    the_state->sdlsurface = SDL_CreateRGBSurface(0,the_state->window_w,the_state->window_h,24,0,0,0,0);

    the_state->gScreenSurface = SDL_GetWindowSurface( the_state->window );


    if (the_state->window == NULL){
        logSDLError(std::cout, "CreateWindow");
        return 2;
    }
    
    the_state->renderer = SDL_CreateRenderer(the_state->window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    
    if (the_state->renderer == NULL){
        logSDLError(std::cout, "CreateRenderer");
        return 3;
    }
    
    the_state->context = SDL_GL_CreateContext( the_state->window );
    return 0;
}

void configure_GL(CLIENT_STATE* the_state)
{
    // OpenGL setup
    SDL_GL_SetSwapInterval(1);
    glClearColor(0, 0, 0, 0);
    
    glMatrixMode(GL_PROJECTION|GL_MODELVIEW);
    glLoadIdentity();
    
    // upload texture data
    glEnable(GL_TEXTURE_2D);
    glGenTextures( 1, &(the_state->texture) );
    
    glBindTexture( GL_TEXTURE_2D, the_state->texture );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
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
    glTexCoord2i( 0, 0 ); glVertex3f(0, 0, 1);
    glTexCoord2i( 1, 0 ); glVertex3f(w, 0, 1);
    glTexCoord2i( 1, 1 ); glVertex3f(w, h, 1);
    glTexCoord2i( 0, 1 ); glVertex3f(0, h, 1);
    glEnd();
    
    //SDL_GL_SwapWindow(window);
    SDL_UpdateWindowSurface(window);
}

int main(int argc, char *argv[])
{
    init_client_state(&state);
    CLIENT_STATE *s = &state;
    
    lo_server_thread st  = lo_server_thread_new(CLIENT_DISPLAY_PORT, error);
    lo_server_thread_add_method(st,  NULL, NULL, generic_handler, NULL);
    // axis(x=0,y=1), bitindex(i>>N), inverted(0=F,1=T), R, G, B(0,1.0)
    //lo_server_thread_add_method(st, "/pattern_change", "iiifff", pattern_change_request_handler, NULL);
    lo_server_thread_add_method(st, "/display_info",   "i",      display_info_request_handler,   NULL);
    lo_server_thread_add_method(st, "/pattern_status", "i",      pattern_status_request_handler, NULL);
    
    lo_server_thread_add_method(st, "/pattern_state", "iiiifff",  pattern_state_request_handler, NULL); // gcb
    lo_server_thread_add_method(st, "/pattern_state", "iiifffff", pattern_state_request_handler, NULL); // sin
    lo_server_thread_add_method(st, "/pattern_state", "iifff",    pattern_state_request_handler, NULL); // rgb
    
    lo_server_thread_start(st);
    
    lo_address t = lo_address_new(SERVER_DISPLAY_ADDRESS, SERVER_DISPLAY_PORT);
    
    int sdl_err = configure_SDL(s);
    if(!sdl_err) { s->sdl_rdy = true; }
    
    configure_GL(s);

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
        send_pattern_status(t, axis, bitidx, inv,r,g,b);
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
	
        if( s->dirty == false && curtime() > s->dirty_t + 6/60.0f ) {
            send_pattern_status(t, s->shown_invert, s->shown_axis, s->shown_bitindex, s->cur_r,s->cur_g,s->cur_b);
            /*
            if(s->cur_rendermode == 0)
                send_pattern_state(t, 0, s->shown_invert, s->shown_axis, s->shown_bitindex,          0, s->cur_r,s->cur_g,s->cur_b);
            if(s->cur_rendermode == 1)
                send_pattern_state(t, 0, s->shown_invert, s->shown_axis, s->shown_freq, s->shown_phase, s->cur_r,s->cur_g,s->cur_b);
            if(s->cur_rendermode == 2)
                send_pattern_state(t, 0, s->shown_invert,             0,             0,              0, s->cur_r,s->cur_g,s->cur_b);
            */
            
        }

	// refresh display
        if( s->dirty_n > 0 ) { //|| ( s->dirty_n > 0 && curtime() - s->dirty_t > s->dirty_interval ) ){
            s->dirty = true;

            s->shown_rendermode = s->cur_rendermode;
            s->shown_axis = s->cur_axis;
            s->shown_bitindex = s->cur_bitindex;
            s->shown_invert = s->cur_invert;
            s->shown_freq = s->cur_freq;
            s->shown_phase = s->cur_phase;
            s->shown_r = s->cur_r;
            s->shown_g = s->cur_g;
            s->shown_b = s->cur_b;

            // generate pattern
            //cv::Mat pat = s->pattern;
            //pat.create( s->window_h, s->window_w, CV_8UC3 );
            
            if(s->shown_rendermode==0)
            {
                gen_graycode( s->sdlsurface, s->gray_LUT, s->shown_invert, s->shown_axis, s->shown_bitindex, s->cur_r, s->cur_g, s->cur_b);
            }
            
            if(s->shown_rendermode==1)
            {
                gen_sine(s->sdlsurface, s->shown_axis, s->shown_freq, s->shown_phase, s->shown_r, s->shown_g, s->shown_b);
            }

            if(s->shown_rendermode==2)
            {
                gen_rgb(s->sdlsurface,s->shown_r, s->shown_g, s->shown_b);
            }
            
            //s->sdltexture = SDL_CreateTextureFromSurface(s->renderer,s->sdlsurface);
	    s->cur_r=fmin(fmax(s->cur_r,0),1);
	    s->cur_g=fmin(fmax(s->cur_g,0),1);
	    s->cur_b=fmin(fmax(s->cur_b,0),1);
	    //pat = pat.mul(cv::Scalar(s->cur_b,s->cur_g,s->cur_r));
            
            // send image to OpenGL
            glBindTexture( GL_TEXTURE_2D, s->texture );
            glTexImage2D( GL_TEXTURE_2D, 0, 3, s->sdlsurface->w, s->sdlsurface->h, 0,
                         GL_BGR, GL_UNSIGNED_BYTE, (unsigned char*)(s->sdlsurface->pixels) );
            //SDL_RenderCopy(Main_Renderer, Background_Tx, NULL, NULL);
	    s->dirty_n--;
            s->dirty_t = curtime();

	    if(s->dirty_n<1)
	    {
		s->dirty = false;
	    }
        }
        
        draw_pattern(s->window);
        SDL_BlitSurface( s->sdlsurface, NULL, s->gScreenSurface, NULL );
        SDL_RenderPresent(s->renderer);
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
        bool inv = (argv[1]->i)==0?false:true;
        int axis =  argv[2]->i;
        int bit  =  argv[3]->i;
        float r  =  argv[4]->f;
        float g  =  argv[5]->f;
        float b  =  argv[6]->f;
        state.cur_invert = inv;
        state.cur_r = r;
        state.cur_g = g;
        state.cur_b = b;
        // if bit == -1 do not change
        if(bit!=-1)
            state.cur_bitindex = bit;
        // if axis == -1 do not change
        if(axis!=-1)
            state.cur_axis = axis;
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


int_least64_t maketimetag(double unixtime)
{
    double ntpsec = unixtime + UNIX_EPOCH;


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
