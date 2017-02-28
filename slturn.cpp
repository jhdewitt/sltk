#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <math.h>
#include <vector>
#include <termios.h>
#include <signal.h>
#include <sys/time.h>
//#include "hid.h"

#include "ports.h"

#include "lo/lo.h"

#define DBUG 0

// HID STUFF

extern "C" {
    int rawhid_open(int max, int vid, int pid, int usage_page, int usage);
    int rawhid_recv(int num, void *buf, int len, int timeout);
    int rawhid_send(int num, void *buf, int len, int timeout);
    void rawhid_close(int num);
}

typedef struct {
    bool done;
    bool dirty;
    int buf_n;
    char* obuf;
    char* ibuf;
    int step_cur;
    int step_increment;

    bool connected;
    bool queued_turn;
    int queued_dir;
    int queued_num;
} CLIENT_STATE;

//--------------------------------------------------------------------------------

void error(int num, const char *m, const char *path);
void init_client_state( CLIENT_STATE* s );
int turn_request_handler(const char *path, const char *types, lo_arg ** argv,
                         int argc, void *data, void *user_data);
void set_serial_config( struct termios* _cfg );
void pushTurnCommand( CLIENT_STATE *st, int dir, int steps );
int send_heartbeat(lo_address the_t);

//--------------------------------------------------------------------------------


CLIENT_STATE state;

void init_client_state( CLIENT_STATE* s )
{   
    s->done = false;
    s->dirty = false;
    s->step_cur = 0;
    s->step_increment = 16*40;
    s->buf_n = 64;
    s->obuf = (char*)calloc(s->buf_n,sizeof(char));
    s->ibuf = (char*)calloc(s->buf_n,sizeof(char));

    s->connected = false;
    s->queued_turn = false;
    s->queued_dir = 0;
    s->queued_num = 0;
}

double curtime(){
    struct timeval _tv;
    gettimeofday( &_tv, NULL );
    double ret = _tv.tv_sec;
    ret += _tv.tv_usec*0.000001;
    return ret;
}

void ctrlc(int sig)
{
    state.done = true;
}

bool checkIfReady(char* buf)
{
    if( buf[3]==1 )
    {
        return true;
    }
    else
    {
        return false;
    }
}

int open_teensy()
{
    int r;
    r = rawhid_open(1, 0x16C0, 0x0486, 0xFFAB, 0x0200);
    return r;
}


int main(int argc, char *argv[])
{
    init_client_state(&state);
    CLIENT_STATE *s = &state;
    
    //lo_server_thread st  = lo_server_thread_new(CLIENT_SERIAL_PORT, error);
    lo_server_thread st = lo_server_thread_new_with_proto(CLIENT_SERIAL_PORT, LO_TCP, error);
    lo_server serv = lo_server_thread_get_server(st);
    
    lo_server_thread_add_method(st, "/turn", "ii", turn_request_handler, NULL);

    lo_server_thread_start(st);
    lo_address t = lo_address_new("10.0.1.105", SERVER_SERIAL_PORT);
    
    signal(SIGINT,ctrlc);
    printf("Listening on TCP port %s\n", CLIENT_SERIAL_PORT);

    int r = open_teensy();
    if(r<=0) {
        printf("[turntable] no rawhid device found (e:%d)\n",r);
        //s->done = true;
        lo_server_thread_free(st);       
        return -1;
    }
    else
    {
       printf("CONNECTED TO USBHID TURNTABLE\n");
       s->connected = true;
    }

    
    while(!(s->done))
    {
        
        int num;
        num = rawhid_recv(0,s->ibuf,64,100);
        // error, fail and exit after reconnect attempt
        // message received
        if(num>0)
        {
            if(checkIfReady(s->ibuf) && DBUG)
            {
                printf("finished turning..\n");
            }
            
            if(DBUG)
            {
                printf("\nrecv %d bytes:\n", num);
                for (int i=0; i<num; i++)
                {
                    printf("%02X ", s->ibuf[i] & 255);
                    if (i % 16 == 15 && i < num-1) printf("\n");
                }
                printf("\n");
            }
        }
        
        if(num<0)
        {
            printf("\ndevice offline\n");
            
            s->connected = false;
            rawhid_close(0);
            usleep(1000*1000*5);
            
            /*
            int r = 0;
            while(r<1) {
                r = open_teensy();
                if(r<1)
                {
                    printf("device still offline, retrying..\n");
                    s->connected = false;
                    usleep(1000*1000);
                }
            }
            printf("reconnected!\n");
            s->connected = true;
            */
            s->done = true;
        }
        
        if ( s->connected && s->queued_turn )
        {
            pushTurnCommand( s, s->queued_dir, s->queued_num );
            s->queued_turn = false;
        }
        
        usleep(1000*16);
    }
    
    lo_server_thread_free(st);
    
    return 0;
}

void error(int num, const char *msg, const char *path)
{
    printf("liblo server error %d in path %s: %s\n", num, path, msg);
    fflush(stdout);
}

void set_serial_config( struct termios* _cfg ){
    if( _cfg == NULL ){
        return;
    }
    _cfg->c_oflag = 0;
    _cfg->c_lflag &= (~(ECHO | ECHONL | ICANON | IEXTEN | ISIG));
    
    _cfg->c_cflag &= (~(CSIZE | PARENB));
    _cfg->c_cflag |= CS8;
    
    _cfg->c_cc[VMIN]  = 0;
    _cfg->c_cc[VTIME] = 0;
}

void pushTurnCommand( CLIENT_STATE *st, int dir, int steps )
{
    if(steps==0)
    {
        steps = st->step_increment;
        steps = 16 * (steps/16);
        steps = 16*10;
    }
    
    if (steps > 2<<16-1)
        steps = 2<<16-1;
    st->obuf[0] = dir;
    st->obuf[1] = (steps>>8) & 0xFF; // msb
    st->obuf[2] = steps & 0xFF; // lsb
    for (int i=3; i<64; i++) {
        st->obuf[i] = 0;
    }

    int r = rawhid_send(0, st->obuf, 64, 0);
    int attempts = 0;
    if(false)
    {
        double retryStartTime = curtime();
        while( r == -110 && retryStartTime > curtime()-2.0 )
        {
            int num;
            num = rawhid_recv(0, st->ibuf,64, 1);
            if( num == 64 && checkIfReady(st->ibuf) )
            {
            }
            r = rawhid_send(0, st->obuf, 64, 1);
            usleep(1e3*100);
        }
    }
    if(DBUG)
    printf("rawhid_send: %d  (%d retries)\n",r, attempts);
}
/////////////////////////////////////////////////
// INCOMING PACKET HANDLERS
//

int turn_request_handler(const char *path, const char *types, lo_arg ** argv,
                           int argc, void *data, void *user_data)
{
    /* example showing pulling the argument values out of the argv array */
    int dir       = argv[0]->i;
    int steps     = argv[1]->i;

    if(DBUG)
    {
        printf("%s <- i:%d i:%d @ %0.6f\n\n", path, argv[0]->i, argv[1]->i,curtime());
        printf("\tdir = %d, steps = %d\n",dir,steps);
        fflush(stdout);
    }
    state.queued_turn = true;
    state.queued_dir = dir;
    state.queued_num = steps;
    state.dirty = true;
    
    return 0;
}

////////////////////////////
// OUTGOING PACKET SENDERS
int send_turn_complete(lo_address the_t)
{
    int ret =  lo_send(the_t, "/turncomplete", "i", 1);
    return ret;
}

