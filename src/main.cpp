#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>

/*high level I/O*/
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <strings.h>
#include <pthread.h>

extern "C"{

/*low level I/O*/
#include <fcntl.h>
#include <sys/time.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>

/*serial*/
#include <sys/termios.h>

/*ボディパラメータ*/
#include <sh_spur_ros/sh_vel_parameters.h>

/*SSM 用*/
#ifdef SSM_ENABLE
#include <ssm_common.h>
#include <Spur_odometry.h>
#include <PWS_Motor.h>
#endif

/*sh_spur用*/
#include <sh_spur_ros/sh_spur.h>
#include <sh_spur_ros/serial.h>
#include <sh_spur_ros/param.h>
#include <sh_spur_ros/control.h>
#include <sh_spur_ros/command.h>


/*ライブラリ用*/
//#include <sh_spur_ros/SHSpur.h>
//#include <sh_spur_ros/CoordinateSystem2D.h>

}

/*pthread mutex用*/
pthread_mutex_t mutex;

int ser_port;
struct termios oldtio; //元のターミナルの設定

#ifdef SSM_ENABLE
SSM_sid g_odm_bs_sid,g_odm_sid,g_motor_sid,g_odm_adj_sid;
int g_ssm_enable;
int g_ssm_adj_enable;
#endif

Odometry g_odometry;
CSptr g_GL, g_LC, g_BS, g_FS, g_BL;

double before_v, before_w;/*直前の速度*/

int g_option;
char g_state[8];

char g_parameter_filename[132];
char g_device_name[132];
double g_P[PARAM_NUM];

int g_run_mode=RUN_STOP;
double g_spur_x;
double g_spur_y;
double g_spur_theta;
double g_spur_v;
double g_spur_w;
volatile double g_spur_v_ros=0.0;
volatile double g_spur_w_ros=0.0;
double g_spur_d;
double g_spur_tilt; /*現在位置の傾き*/
double g_spur_dir;  /*傾きの高い方向（tiltが正なら）*/

/*シリアル通信用*/
int g_ser_port;
struct termios g_oldtio;
int   g_err_count;

double g_cnt2rad;
double g_2wradius;

/*時刻同期用*/
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

/*CSの初期化*/
void init_coordinate_systems(void){
  /**/
  g_BS = CS_add(0,   0,0,0);/*オドメトリ座標系*/
  g_GL = CS_add(g_BS,0,0,0);/*グローバル座標系（走行制御用）*/
  g_LC = CS_add(g_GL,0,0,0);/*ローカル座標系*/
  g_FS = CS_add(g_BS,0,0,0);/*自己位置*/
  g_BL = CS_add(g_BS,0,0,0);/*オドメトリローカル座標系*/
}


/*すれっどの初期化*/
void init_thread(pthread_t* thread){

  pthread_mutex_init(&mutex, NULL);

  if(pthread_create(thread, NULL, /*(void*)*/&command, NULL)!= 0){
    fprintf(stderr, "Can't create command thread\n");
  }
}

/*SSMの初期化*/
void init_SSM(void){
#ifdef SSM_ENABLE
  printf("    with SSM\n");
  if(!initSSM()){
    /*SSMが起動していない*/
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

/*for ROS subscriber */
ros::Time last_mesurement;
void velCallback(const geometry_msgs::Twist& msg){
  last_mesurement=ros::Time::now();
  g_run_mode=RUN_WHEEL_VEL;
  g_spur_v_ros=msg.linear.x;
  g_spur_w_ros=msg.angular.z;
  //ROS_WARN("v=%f,w=%f",g_spur_v_ros,g_spur_w_ros);
}

/*main*/
int main(int argc, char *argv[]){
  pthread_t command_thread;
  ros::init(argc,argv,"sh_spur_ros");
  ros::NodeHandle nh("~");
  ros::NodeHandle nh_topics;
  nav_msgs::Odometry odom;
  tf::TransformBroadcaster odom_broadcaster;
  ros::Rate loop_rate(200);
  ros::Publisher odom_pub=nh_topics.advertise<nav_msgs::Odometry>("odom",100);
  ros::Subscriber vel_sub=nh_topics.subscribe("cmd_vel",100,velCallback);
  ROS_INFO("started_sh_spur_ros.");
  //arg_analyse(argc,argv);
  std::string param_file_name;
  std::string device_name;
  nh.param<std::string>("param_file_name",param_file_name,"./robot.param");
  nh.param<std::string>("device_name",device_name,"/dev/ttyUSB0");
  ROS_INFO("device_name:%s",device_name.c_str());
  if(!init_serial((char*)device_name.c_str())){
    printf("stop.\n");
    return 0;
  }

  printf("SH Spur \n");

  /*SSM初期化*/
  init_SSM();

  /*スレッド初期化*/
  init_thread(&command_thread);

  /**/
  init_coordinate_systems();

  /*パラメータを読み込み、セットする*/
  ROS_INFO("param_file:%s",param_file_name.c_str());
  set_param((char*)param_file_name.c_str());
  //if(!(g_option&OPTION_PARAM_CONTROL))
    //if(argc<2){
    //set_param(g_parameter_filename);
  /*}else{
   int i = set_param(argv[1]);
   if(i==0){
     ROS_INFO("Loaded param file from:%s",argv[1]);
   }else{
     ROS_ERROR("Faild to load param file!(%s)",argv[1]);
     return -1;
   }
   }*/

  
  /*サーボをかける*/
  if(g_state[SHSPUR_PARAM_MOTOR] == ENABLE && 
     g_state[SHSPUR_PARAM_VELOCITY] == ENABLE )
    motor_servo();

  fprintf(stderr,"start\n");
  /*メインループ*/
  ros::Time last_time =ros::Time::now();
  while(ros::ok()){
    ros::spinOnce();
    /*オドメトリ受信->publish*/
    ros::Time current_time =ros::Time::now();
    double dt=(current_time-last_time).toSec();
    serial_receive();
    geometry_msgs::Quaternion odom_quat 
      = tf::createQuaternionMsgFromYaw(g_odometry.theta);
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = g_odometry.x;
    odom_trans.transform.translation.y = g_odometry.y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    odom_broadcaster.sendTransform(odom_trans);

    odom.header.stamp = current_time;
    odom.header.frame_id="odom";
    odom.child_frame_id="base_link";
    odom.pose.pose.position.x=g_odometry.x;
    odom.pose.pose.position.y=g_odometry.y;
    odom.pose.pose.position.z=0;
    odom.pose.pose.orientation=odom_quat;
    odom_pub.publish(odom);
    /*制御*/
    if((current_time-last_mesurement).toSec()>0.5){
      g_spur_v_ros=0.0;
      g_spur_w_ros=0.0;
    }
    run_control(g_spur_v_ros,g_spur_w_ros);
    //robot_speed_smooth(g_spur_v_ros,g_spur_w_ros);
    //motor_speed(g_spur_v_ros, g_spur_w_ros);
    last_time = current_time;
    ROS_INFO("x:%f, y:%f,theta:%f",g_odometry.x,g_odometry.y,g_odometry.theta);
    ROS_INFO("commanded v:%f, w:%f",g_spur_v_ros,g_spur_w_ros);
    loop_rate.sleep();
  }

  /*終了処理*/
  serial_close();
  //pthread_join(command_thread, NULL);
  return 0;
}
