
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
#include <sh_spur_ros/sh_spur.h>
#include <sh_spur_ros/serial.h>
#include <sh_spur_ros/param.h>
#include <sh_spur_ros/control.h>
#include <sh_spur_ros/command.h>


/*�饤�֥����*/
//#include <sh_spur_ros/SHSpur.h>
//#include <sh_spur_ros/CoordinateSystem2D.h>


/*pthread mutex��*/
pthread_mutex_t mutex;

int ser_port;
struct termios oldtio; //���Υ����ߥʥ������

#ifdef SSM_ENABLE
SSM_sid g_odm_bs_sid,g_odm_sid,g_motor_sid,g_odm_adj_sid;
int g_ssm_enable;
int g_ssm_adj_enable;
#endif

Odometry g_odometry;
CSptr g_GL, g_LC, g_BS, g_FS, g_BL;

double before_v, before_w;/*ľ����®��*/

int g_option;
char g_state[8];

char g_parameter_filename[132];
char g_device_name[132];
double g_P[PARAM_NUM];

int g_run_mode;
double g_spur_x;
double g_spur_y;
double g_spur_theta;
double g_spur_v;
double g_spur_w;
double g_spur_d;
double g_spur_tilt; /*���߰��֤η���*/
double g_spur_dir;  /*�����ι⤤������tilt�����ʤ��*/

/*���ꥢ���̿���*/
int g_ser_port;
struct termios g_oldtio;
int   g_err_count;

double g_cnt2rad;
double g_2wradius;

/*����Ʊ����*/
double g_interval;
double g_offset;
int g_offset_point;
/*get now time stamp*/
double get_time(void)
{
  struct timeval current;

  gettimeofday(&current, NULL); 
  
  return  current.tv_sec + current.tv_usec/1000000.0;   
}

/*CS�ν����*/
void init_coordinate_systems(void){
  /**/
  g_BS = CS_add(0,   0,0,0);/*���ɥ�ȥ��ɸ��*/
  g_GL = CS_add(g_BS,0,0,0);/*�����Х��ɸ�ϡ����������ѡ�*/
  g_LC = CS_add(g_GL,0,0,0);/*�������ɸ��*/
  g_FS = CS_add(g_BS,0,0,0);/*���ʰ���*/
  g_BL = CS_add(g_BS,0,0,0);/*���ɥ�ȥ�������ɸ��*/
}


/*����äɤν����*/
void init_thread(pthread_t* thread){

  pthread_mutex_init(&mutex, NULL);

  if(pthread_create(thread, NULL, /*(void*)*/&command, NULL)!= 0){
    fprintf(stderr, "Can't create command thread\n");
  }
}

/*SSM�ν����*/
void init_SSM(void){
#ifdef SSM_ENABLE
  printf("    with SSM\n");
  if(!initSSM()){
    /*SSM����ư���Ƥ��ʤ�*/
    printf("\n SSM is not available.\n");
    g_ssm_enable = 0;
  }else{
    g_ssm_enable = 1;
    g_odm_bs_sid=createSSM_time("spur_odometry",0,sizeof(Spur_Odometry),5,0.005);
    g_odm_sid=createSSM_time("spur_global",0,sizeof(Spur_Odometry),5,0.005);
    g_motor_sid=createSSM_time("pws_motor",0,sizeof(PWSMotor),1,0.005);
  }
  g_ssm_adj_enable = 0;
#endif
}


/*main*/
int main(int argc, char *argv[]){
  pthread_t command_thread;

  arg_analyse(argc,argv);
  if(!init_serial()){
    printf("stop.\n");
    return 0;
  }

  printf("SH Spur \n");
  
  /*SSM�����*/
  init_SSM();

  /*����åɽ����*/
  init_thread(&command_thread);

  /**/
  init_coordinate_systems();

  /*�ѥ�᡼�����ɤ߹��ߡ����åȤ���*/
  if(!(g_option&OPTION_PARAM_CONTROL))
    set_param(g_parameter_filename);

  
  /*�����ܤ򤫤���*/
  if(g_state[SHSPUR_PARAM_MOTOR] == ENABLE && 
     g_state[SHSPUR_PARAM_VELOCITY] == ENABLE )
    motor_servo();

  fprintf(stderr,"start\n");
  /*�ᥤ��롼��*/
  while(1){
    /*���ɥ�ȥ����*/
    serial_receive();
    /*����*/
    run_control();
  }

  /*��λ����*/
  serial_close();
  pthread_join(command_thread, NULL);
  return 0;
}
