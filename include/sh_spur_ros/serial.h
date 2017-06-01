#ifndef __SERIAL__
#define __SERIAL__

//#define DEBUG
#ifdef DEBUG
#define DP(x) printf(x)
#define DP2(x,y) printf(x,y)
#define DEP(x) fprintf(stderr,x)
#define DF(x)   x
#else
#define DP(x)
#define DP2(x,y)
#define DEP(x) 
#define DF(x)   
#endif

#define STX 0x09
#define ETX 0x0a

#define ENABLE 1
#define DISABLE 0

#define BAUDRATE B38400

/*for measurement time estimation*/
#define SER_BOUDRATE 38400.0
#define SER_INTERVAL 0.0050
#define SER_BYTES    13.0
#define SER_TIME_BYTE (11.0/SER_BOUDRATE)


int serial_connect(char*);
/*----------------PBS_close------------------*/
int serial_close(void);
/*-------------------------------------------
  Utility program
 ------------------------------------------*/
void inverce(unsigned char *data,int Len);
int encode(unsigned char *src,int len,unsigned char *dst,int buf_max);
int decord(unsigned char *src,int len,unsigned char *dst,int buf_max);
int encode_write(int port,char* data,int len);
void print_hex(unsigned char *dat,int num);
double get_time(void);
int init_serial(char*);
void finalize_serial(void);
double time_estimate(int readnum);
double time_synchronize(double receive_time, int readnum, int wp);
int serial_receive(void);
#endif
