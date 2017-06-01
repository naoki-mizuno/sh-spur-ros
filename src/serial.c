/*high level I/O*/
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <strings.h>
#include <pthread.h>

/*low level I/O*/
#include <fcntl.h>
#include <sys/time.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>

/*serial*/
#include <sys/termios.h>

/*�ܥǥ��ѥ�᡼��*/
#include <sh_spur_ros/sh_vel_parameters.h>

/*SSM ��*/
#ifdef SSM_ENABLE
#include <ssm_common.h>
#include <Spur_odometry.h>
#include <PWS_Motor.h>
#endif

/*sh_spur��*/
#include <sh_spur_ros/serial.h>
#include <sh_spur_ros/param.h>
#include <sh_spur_ros/control.h>
#include <sh_spur_ros/command.h>

/*�饤�֥����*/
#include <sh_spur_ros/SHSpur.h>

/*pthread mutex��*/
extern pthread_mutex_t g_mutex;

/*SSM��*/
#ifdef SSM_ENABLE
extern SSM_sid g_odm_bs_sid, g_odm_sid, g_motor_sid;
extern int g_ssm_enable;
#endif 

/*���ɥ�ȥ�ǡ���*/
extern Odometry g_odometry;
extern CSptr g_GL, g_LC, g_BS, g_FS, g_BL;
extern int g_option;
extern char g_state[];
extern double g_interval;
extern double g_offset;
extern int g_offset_point;

/*���ꥢ���̿���*/
extern char g_device_name[];
extern int g_ser_port;
extern struct termios g_oldtio; //���Υ����ߥʥ������
extern int   g_err_count;

/*----------------serial_connect------------------*/
/*�ݡ��Ȥ򥪡��ץ󤷤� �̿��ν����򤹤�*/
int serial_connect(char* device_name)
{
  struct termios newtio;
  g_ser_port = open(device_name,O_RDWR);
  /*
  printf("%s\n",g_device_name);
  if(port == 0){
    g_ser_port = open( g_device_name,O_RDWR);
  }else{
    g_ser_port = open( "/dev/ttyS0",O_RDWR|O_NONBLOCK);
  }*/
   
  if(g_ser_port < 0){
    fprintf(stderr,"Can't open serial port.\n");
    return 0;
  }
  
  tcgetattr(g_ser_port, &g_oldtio); /* ���ߤΥ��ꥢ��ݡ��Ȥ���������򤵤���*/
  bzero(&newtio, sizeof(newtio)); /* �������ݡ��Ȥ�����ι�¤�Τ򥯥ꥢ���� */

  newtio.c_cflag = BAUDRATE |  CS8 | CLOCAL | CREAD | CRTSCTS  ;
  
  newtio.c_iflag = IGNPAR;   //�ѥ�ƥ����顼�Υǡ�����̵�뤹��
  newtio.c_oflag = 1;        //Raw �⡼�ɤǤν���
  
  newtio.c_cc[VTIME]    = 0;     /* ����饯���֥����ޤ�Ȥ�ʤ� */
  newtio.c_cc[VMIN]     = 1;     /* 1ʸ�����ޤǡ��ɤ߹��ߤ�֥�å����� */

  //��ǥ�饤��򥯥ꥢ�����ݡ��Ȥ������ͭ���ˤ���
  tcflush(g_ser_port, TCIFLUSH);
  if(tcsetattr(g_ser_port,TCSANOW,&newtio)){
    fprintf(stdout,"ser set err");
    return 0;
  }

  /*�����å�
    tcgetattr(ser_port,&newtio); 
    printf("%s\n",newtio.c_cflag&BAUDRATE?"ok":"failure");*/
  return 1;
}


/*----------------PBS_close------------------*/
int serial_close(void)
{
  //����򸵤��᤹
  //  tcsetattr(ser_port,TCSANOW,&g_oldtio);
  close(g_ser_port);
  return 1;
}



/*-------------------------------------------
  Utility program
 ------------------------------------------*/
void inverce(unsigned char *data,int Len){
  int i,j;
  unsigned char a;
  unsigned short b;
  
  for(i = 0; i < Len; i++){
    b =   data[i];
    a = 0;
    for(j = 0; j < 8; j++){
      a = a>>1;
      b = b<<1;
      if(b & 0x100 ) a |= 0x80;
    }
    data[i] = a;
  }
}

int encode(unsigned char *src,int len,unsigned char *dst,int buf_max)
{
     int pos,s_pos,w_pos;
     unsigned short b;
     pos = 0;   //read position
     w_pos = 1; //write_position
     s_pos = 0;
     b = 0;
   
     while(pos < len  || s_pos >= 6){
       if(s_pos >= 6){
	 dst[w_pos] = ((b>>10)&0x3f)+0x40;
	 w_pos++;if(w_pos >= buf_max)return(-1);
	 b = b<< 6;
	 s_pos-=6;
       }else{
	 b |= src[pos]<<(8-s_pos);
	 s_pos+=8;
	 pos++;
	 if(pos >= len)s_pos+=4;//�Ǹ�
       }
     }
     
     if(++w_pos >= buf_max)return(-1);
     dst[0] =0x09;// ECC_STX;    //0x09
     dst[w_pos-1] = 0x0a;// ECC_ETX;//0x0a  
     return w_pos;
}

int decord(unsigned char *src,int len,unsigned char *dst,int buf_max)
{
   unsigned short dat,b;
   int pos,s_pos,w_pos;
   int rerr;
   pos = 1;   //read position
   w_pos = 0; //write_position
   s_pos = 0; //shift position
   rerr = 0;
   dat = 0;
   b = 0;     
   while(pos < len-1){
      if( src[pos]>= 0x40)
	b = src[pos]-0x40;
      else rerr++;
      
      dat |= (b <<(10-s_pos));
      s_pos += 6;
      if(s_pos >= 8){/*�ǡ������ä�*/
	 dst[w_pos]=(dat>>8);
	 w_pos++;
	 if(w_pos >= buf_max)return 0;
	 s_pos -= 8;
	 dat = dat << 8;
      }
      pos++;
   }
   
 
   if(rerr)return -rerr;
   return w_pos;
}

int encode_write(int port, char* data,int len){
   unsigned char buf[100];
   int encode_len;

   encode_len = encode((unsigned char*)data, len, buf, 100);
   write(port,buf,encode_len);

   return 0;
}


void print_hex(unsigned char *dat,int num)
{
  int i;
  
  for(i = 0; i< num;i++){
    fprintf(stdout,"%x ",dat[i]);
  }fprintf(stdout,"\n");
}

/*parameter set command*/
int parameter_set(char param, char id, int value)
{
   char buf[7];
   buf[0] = param;
   buf[1] = id;
   buf[2] = ((Int_4Char)value).byte[3];
   buf[3] = ((Int_4Char)value).byte[2];
   buf[4] = ((Int_4Char)value).byte[1];
   buf[5] = ((Int_4Char)value).byte[0];
   encode_write(g_ser_port, buf, 6);
  
   return(0);
}


int init_serial(char* device_name){
  g_err_count = 0;
  return serial_connect(device_name);
}

void finalize_serial(void){
  serial_close();
}

double time_estimate(int readnum){
  return g_offset + g_interval*(double)(readnum-g_offset_point);
}

/*��¬����ο���*/
double time_synchronize(double receive_time, int readnum, int wp){
  static double time_log[100],start_time;
  static int time_wp, offset_wp;
  static double offset_min, diff_min, offset_log[10];
  static int  min_num, min_log[10];
  double estimated_time, measured_time;

  if(g_offset_point == 0){
    g_interval = SER_INTERVAL;
    g_offset = receive_time;
    g_offset_point = readnum;
    diff_min = 100000;
    time_wp = 0;
    offset_wp = 0;
    start_time = receive_time;
  }
  //  if(wp > 13)return receive_time;
  
  /*�����¬����η׻�*/
  if(wp > 0){/*���Τ������*/
    /*�Ǹ�Υǡ����λ���򼡤򲿥Х����ܤ�����������ǿ���*/
    measured_time = receive_time- wp*SER_TIME_BYTE - SER_INTERVAL;
    /*               ������������Х��ȿ�/���ò��Х��Ȥ���5ms*/
    time_log[time_wp%100] = measured_time;
    
    /*���˵�Ͽ*/
    estimated_time = time_estimate(readnum);
    if(time_wp >= 50){/*���ե��å���Ͽ�������׻�*/
      offset_log[offset_wp%10] = offset_min;
      min_log[offset_wp%10] = min_num;
      g_offset = offset_min;
      g_offset_point = min_num;
      if(offset_wp >= 10){
	g_interval = (offset_log[offset_wp%10]-offset_log[(offset_wp-9)%10])
	  / (min_log[offset_wp%10]-min_log[(offset_wp-9)%10]);
      }
      offset_wp++;
      time_wp = 0;
      diff_min = 100000;
      printf("%f %f \n",g_offset,g_interval*1000.0);
    }

    /*�Ǿ���*/
    if(diff_min > measured_time - estimated_time){ 
      diff_min = measured_time - estimated_time;
      offset_min = measured_time;
      min_num = readnum;
    }
    time_wp++;
    /* printf("%d %f %f \n",num,
	   estimated_time-(double)num*SER_INTERVAL- start_time,
	   measured_time-(double)num*SER_INTERVAL- start_time);
    */
  }else{
    estimated_time = time_estimate(readnum);
  }

  return estimated_time;
}

/*���ꥢ���������*/
int serial_receive(void){
  static char buf[4000], data[10];
  Short_2Char cnt1,cnt2,pwm1,pwm2;  
  int len,i;
  int odometry_updated;
  static int com_wp, receive_count;
  static char com_buf[100];
  int readdata_num;
  double receive_time;
  Odometry odm_log[100];
#ifdef SSM_ENABLE
  Spur_Odometry   odm;
  PWSMotor        motor_log[100];
#endif

  readdata_num = 0;
  odometry_updated  = 0;
  /*�Хåե������ɤ߹���*/
  len = read(g_ser_port, buf, 4000);
  receive_time = get_time();
  
  if(len > 0){
    for(i = 0;i < len;i++){
      if(buf[i] == 0x09 || com_wp != 0){/*�ɤ߹���*/
	
	for(;i < len;i++){
	  com_buf[com_wp] = buf[i];
	  com_wp++;
	  if(buf[i] == 0x0a){
	    /*�ǥ����ɽ���*/
	    decord((unsigned char*)com_buf, 
		   com_wp, (unsigned char*)data, 10);
	    cnt1.byte[1] = data[0];
	    cnt1.byte[0] = data[1];
	    cnt2.byte[1] = data[2];
	    cnt2.byte[0] = data[3];
	    pwm1.byte[1] = data[4];
	    pwm1.byte[0] = data[5];
	    pwm2.byte[1] = data[6];
	    pwm2.byte[0] = data[7];

#ifdef SSM_ENABLE
	    /*SSM�ؤν���*/
	    if(g_ssm_enable){
	      motor_log[readdata_num].counter1 = cnt1.integer;
	      motor_log[readdata_num].counter2 = cnt2.integer;
	      motor_log[readdata_num].pwm1 = pwm1.integer;
	      motor_log[readdata_num].pwm2 = pwm2.integer;
	    }
#endif 
	    if(g_state[SHSPUR_PARAM_MOTOR] ==ENABLE &&
	       g_state[SHSPUR_PARAM_VELOCITY] ==ENABLE &&
	       g_state[SHSPUR_PARAM_BODY] ==ENABLE){
	      odometry(&g_odometry, cnt1.integer, cnt2.integer, 0.005);
	      odm_log[odometry_updated] = g_odometry;
	      odometry_updated++;
	    }

	    if(g_option & OPTION_SHOW_ODOMETRY)
	      printf("%f %f %f\n",g_odometry.x,g_odometry.y,g_odometry.theta);

	    readdata_num++;
	    com_wp = 0;
	    break;
	  }
	}
      }else{
	/*���ߥǡ�������*/
	g_err_count ++;
	fprintf(stderr,">%d\n",g_err_count);
      }
      //	printf("%s", buf);
    }

    receive_count += readdata_num;
    time_synchronize(receive_time, receive_count, com_wp);

#ifdef SSM_ENABLE
    if(g_ssm_enable){
      /*SSM�ؤν���*/
      for(i = 0;i < odometry_updated;i++){
	
	/*BS�ν���*/
	odm.x = odm_log[i].x;
	odm.y = odm_log[i].y;
	odm.theta = odm_log[i].theta;
	odm.v = odm_log[i].v;
	odm.w = odm_log[i].w;
	//      writeSSM(g_odm_bs_sid, (char*)&odm, g_odometry.time+0.003);
	writeSSM(g_odm_bs_sid, (char*)&odm, 
		 time_estimate(receive_count-odometry_updated + i +1));
	/*GL�ν���*/
	CS_recursive_trans(g_GL, g_BS, &odm.x, &odm.y, &odm.theta);
	writeSSM(g_odm_sid, (char*)&odm,
		 time_estimate(receive_count-odometry_updated + i +1)); 
      }
      
      /*PWM,�������ͤν���*/
      for(i = 0;i < readdata_num;i++){
	writeSSM(g_motor_sid,(char*)&motor_log[i],
		 time_estimate(receive_count-readdata_num+i+1));
      }
    }
#endif
  }else{
    //    usleep(10000);
  }
  return 1;
}
