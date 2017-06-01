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
#include <sh_spur_ros/CoordinateSystem2D.h>

#ifdef SSM_ENABLE
extern SSM_sid g_odm_bs_sid,g_odm_sid,g_motor_sid, g_odm_adj_sid;
extern int g_ssm_enable;
extern int g_ssm_adj_enable;
#endif

/*pthread mutex��*/
extern pthread_mutex_t mutex;
/**/
extern Odometry g_odometry;
extern CSptr g_GL,g_LC,g_BS,g_FS;
extern int g_option;
extern char g_state[];
extern int g_run_mode;
extern double g_P[];
/**/
extern double g_spur_x;
extern double g_spur_y;
extern double g_spur_theta;
extern double g_spur_v;
extern double g_spur_w;
extern double g_spur_d;
/*����*/
extern double g_spur_tilt;
extern double g_spur_dir;


/*���ɥ�ȥ�׻�*/
void odometry(OdometryPtr xp, short cnt1, short cnt2,double dt){
  double v,w, wr, wl;
  
  /*��®�ٷ׻�*/
  wr = 2.0*3.141592654*
    ((double)cnt2)/(g_P[COUNT_REV]*g_P[GEAR]*dt);
  wl = 2.0*3.141592654*
    ((double)cnt1)/(g_P[COUNT_REV]*g_P[GEAR]*dt);

  /*���ͥޥƥ������׻�*/
  v = g_P[RADIUS_R]*wr/2.0  + g_P[RADIUS_L]*wl/2.0;
  w = g_P[RADIUS_R]*wr/g_P[TREAD]- g_P[RADIUS_L]*wl/g_P[TREAD];
  
  /*���ɥ�ȥ�׻�*/
  xp->x =  xp->x + v*cos(xp->theta)*dt;
  xp->y =  xp->y + v*sin(xp->theta)*dt;
  xp->theta = xp->theta+w* dt;    
  xp->time = get_time();
  xp->v = v;
  xp->w = w;
  /*-PI< <PI��Ĵ��*/
  //if(xp->theta <-M_PI)xp->theta += 2*M_PI;
  //if(xp->theta > M_PI)xp->theta -= 2*M_PI;

  /*FS��ɸ�ϥ��å�*/
  CS_set(g_FS, xp->x, xp->y,xp->theta );
}

/*-PI < theta < PI��Ĵ������*/
double trans_q(double theta){
  while(theta > M_PI)theta -= 2.0*M_PI;
  while(theta < -M_PI)theta += 2.0*M_PI;
  return theta;
}

/*�߸��ɽ�*/
double circle_follow(OdometryPtr odm, double x, double y, double radius,
		   double v_max){
  double d,q,r,ang;
  
  r = sqrt((odm->x -x)*(odm->x -x) +(odm->y -y)*(odm->y -y));
  
  ang = atan2((odm->y - y), (odm->x - x));
  ang = trans_q(ang);

  // �쥮��졼��������Ѵ�
  d = fabs(radius) - r;
  q = trans_q(odm->theta - (ang + SIGN(radius) * (M_PI / 2.0)));
  
  return regurator(d, q, radius, v_max);
}

/*ľ���ɽ�*/
double line_follow(OdometryPtr odm, double x, double y, double theta,
		   double v_max){
  double d,q;
  
  d = -(odm->x-x)*sin(theta) + (odm->y-y)*cos(theta);
  q = odm->theta - theta;
  q = trans_q(q);

  return regurator(d, q, 100, v_max);
}

/*�����ɽ��쥮��졼��*/
double regurator(double d, double q, double r,  double v_max){
  double nv,nw;
  double v,w;
  double cd;

  now_speed(&nv,&nw);  
  
  v = v_max - SIGN(v_max) * g_P[L_C1] * fabs(nw);
  
  cd = d;
  if(cd > g_P[L_DIST])cd = g_P[L_DIST];
  if(cd < -g_P[L_DIST])cd = -g_P[L_DIST];
  w = nw - g_P[CONTROL_CYCLE]*
    (SIGN(r)*SIGN(nv) * g_P[L_K1]*cd + g_P[L_K2]*q + g_P[L_K3]*nw);
  
  /*FF*/
  if(fabs(r)>0.1 )
    w += 2*nv / r ; 

  v = v_max;

  robot_speed_smooth(v, w);  
  return d;
}

/*��ž*/
double spin(OdometryPtr odm, double theta, double w_max){
  double q, w_limit;
  double w;
  
  q = theta - odm->theta;
  q = trans_q(q);
  
  /*��ߤ���Τ˸³���®�٤�׻�*/
  w_limit = sqrt(0.8*2*g_P[MAX_ACC_W]*fabs(q)*g_P[CONTROL_CYCLE]/g_P[CYCLE]);

  if(w_max < w_limit){
    w = SIGN(q)*w_max;
  }else{
    w = SIGN(q)*w_limit;
    if(fabs(w) < M_PI/90.0)w = M_PI/90.0*SIGN(q);
  }
  
  robot_speed_smooth(0, w);  
  return fabs(q);
}

/*���ޤǤε�Υ*/
double dist_pos(OdometryPtr odm, double x, double y){
  double r;
  r = sqrt((odm->x -x)*(odm->x -x)+(odm->y -y)*(odm->y -y));
  
  return r;
}

/*ľ����ü���ޤǰ�ư���ߤޤ�*/
int to_point(OdometryPtr odm, double x, double y, double theta,double max_vel){
  double dist,a ;
  double vel;
  int over;

  dist = dist_pos(odm, x, y);

  a = (odm->x -x)*cos(theta) + (odm->y-y)*sin(theta);
  over = 0;
  if(a > 0){
    vel = 0;
    over = 3;
  }else if(dist > max_vel){
    vel = max_vel;
  }else if(dist > 0.05){
    over = 1;
    vel = dist;  
  }else{
    over = 2;
    vel = dist;
  }

  line_follow(odm, x,y,theta,vel);
  return over;
}

/*Odometry���ǡ����κ�ɸ�Ϥ��Ѵ�*/
void cs_odometry(CSptr target_cs, OdometryPtr odm, OdometryPtr dst_odm){
  double x,y,theta;
  x = odm->x;
  y = odm->y;
  theta = odm->theta;

  CS_recursive_trans(target_cs, g_BS, &x, &y, &theta );

  dst_odm->x = x;
  dst_odm->y = y;
  dst_odm->theta = theta;
  dst_odm->time = odm->time;
}

/*���ɥ�ȥ꽤������Ȥ�ͻ��*/
void coordinate_synchronize(void){
#ifdef SSM_ENABLE
  static double before_time;
  double now_time,time;
  Odometry bs_odometry, adj_odometry, target_pos;
  CoordinateSystem bs_cs, adj_cs;
  int tid;

  if(g_ssm_enable){/*20ms�衩*/
    if(!g_ssm_adj_enable){/*SSMcheck*/
      now_time = get_time();
      if(now_time > before_time+1){
	g_odm_adj_sid = openSSM("spur_global",1,0);
	if(g_odm_adj_sid > 0){//open�Ǥ���
	  g_ssm_adj_enable = 1;
	  printf("find adjust information.\n");
	}else{
	  g_ssm_adj_enable = 0;
	}
	before_time = now_time;
      }
    }else{
      /*�ѥ�᡼�����ѹ���������ʤ��褦�˥֥�å�*/
      pthread_mutex_lock(&mutex);

      //�ǿ��ν�������
      if((tid = readSSM(g_odm_adj_sid, (char*)&adj_odometry,&now_time,-1))>=0){
	//Ʊ�����GL��ɸ
	if(now_time > get_time()-1){
	  if((tid=
	      readSSM_time(g_odm_bs_sid,(char*)&bs_odometry,now_time,&time))>=0){
	    //���֤�1�ð���ʻߤޤäƤ��ʤ��ˤǡ��ǡ���������ʤ�¹�
	    /*��ɸ�Ϻ���*/
	    bs_cs.x = bs_odometry.x;
	    bs_cs.y = bs_odometry.y;
	    bs_cs.theta = bs_odometry.theta;
	    
	    adj_cs.x = adj_odometry.x;
	    adj_cs.y = adj_odometry.y;
	    adj_cs.theta = adj_odometry.theta;
	   
	    /*������ΰ��֤�׻�*/
	    target_pos = g_odometry;/*�ǿ��ΰ���*/
	    /*���߰��֤Ȥ���̯�ʺ���׻�*/
	    trans_cs(&bs_cs,&target_pos.x,&target_pos.y, &target_pos.theta);
	    /*��̯�ʺ����դ��ä���*/
	    inv_trans_cs(&adj_cs,&target_pos.x,&target_pos.y,&target_pos.theta);
	    /*��ܥå�(FS)��GL���x,y,theta�˸�����Ȥ���Ȥ�*/
	    /*FS����GL���ɤ��˸����뤫(GL->FS => FS->GL)*/
	    CS_turn_base(&target_pos.x,&target_pos.y,&target_pos.theta);
	    /*�����BS��Τɤ���*/
	    CS_recursive_trans(g_BS, g_FS,
			       &target_pos.x,&target_pos.y,&target_pos.theta);
	    /*GL�򥻥å�*/
	    CS_set(g_GL, target_pos.x, target_pos.y, target_pos.theta);
	  }
	}
      }
      /*�ݸ���*/
      pthread_mutex_unlock(&mutex);

    }

  }
#endif
}

/*�������*/
double gravity_compensation(Odometry* odm){
  double tilt,f;
 
  /*������׻�*/
  tilt = atan(cos(odm->theta-g_spur_dir)*tan(g_spur_tilt));
  /*�Ϥ�׻�*/
  f = g_P[MASS]*GRAVITY*sin(tilt);
  /*[N]=[kg]*[m/ss]*tilt*/
  /*�ȥ륯��׻�*/
  g_P[TORQUE_OFFSET] = f * g_P[RADIUS_R]/g_P[GEAR];
  /*[Nm]              [N]* [m]          /[in/out] */
  printf("%f %f\n",f,g_P[TORQUE_OFFSET]*g_P[TORQUE_UNIT]/2.0);
  parameter_set(PARAM_p_toq_offset,0,g_P[TORQUE_OFFSET]*g_P[TORQUE_UNIT]/2.0); 
  parameter_set(PARAM_p_toq_offset,1,g_P[TORQUE_OFFSET]*g_P[TORQUE_UNIT]/2.0); 
  return tilt;
} 

/*�ɽ����פ˱���������*/
void run_control(double v, double w){
  static double before_time;
  double now_time;
  Odometry gl_odometry;

  now_time = get_time();

  if(now_time > before_time+0.02 &&
     g_state[SHSPUR_PARAM_MOTOR]    == ENABLE  &&
     g_state[SHSPUR_PARAM_VELOCITY] == ENABLE){/*20ms�衩*/

    coordinate_synchronize();
    /*�ѥ�᡼�����ѹ���������ʤ��褦�˥֥�å�*/
    pthread_mutex_lock(&mutex);
    /*just give set robot speed (added by Kojima 2016/05/30)*/
    robot_speed_smooth(v,w);
    cs_odometry(g_GL, &g_odometry, &gl_odometry);
    
    before_time = now_time;//+=0.02;
    
    //�������
    if(g_state[SHSPUR_PARAM_GRAVITY] == ENABLE){
      gravity_compensation(&gl_odometry);      
    }
    /*
    //���Ծ��֤˱���������
    switch(g_run_mode){
    case RUN_FREE:
      robot_speed_smooth(0,0);
      break;
    case RUN_STOP://���ȥåפ���ʥ��ԡ��ɤ�0�ˤ����
      robot_speed_smooth(0,0);
      break;
    case RUN_WHEEL_VEL://®��ľ�ܻ���
      motor_speed(g_spur_v, g_spur_w);
      break;
    case RUN_VEL://®�ٳ�®�ٻ���
      if(g_state[SHSPUR_PARAM_BODY] == ENABLE)
	robot_speed_smooth(g_spur_v, g_spur_w);
      break;
    case RUN_LINEFOLLOW: //ľ���ɽ�
      if(g_state[SHSPUR_PARAM_BODY] == ENABLE && 
	 g_state[SHSPUR_PARAM_TRACKING] == ENABLE)
	line_follow(&gl_odometry, g_spur_x, g_spur_y, g_spur_theta,g_spur_v);
      break;
    case RUN_TO_POINT: //û�դؤΰ�ư
      if(g_state[SHSPUR_PARAM_BODY] == ENABLE && 
	 g_state[SHSPUR_PARAM_TRACKING] == ENABLE)
	to_point(&gl_odometry, g_spur_x, g_spur_y, g_spur_theta, g_spur_v);
      break;
    case RUN_CIRCLEFOLLOW: //�߸��ɽ�
      if(g_state[SHSPUR_PARAM_BODY] == ENABLE && 
	 g_state[SHSPUR_PARAM_TRACKING] == ENABLE)
	circle_follow(&gl_odometry, g_spur_x, g_spur_y, g_spur_d, g_spur_v);
      break;
    case RUN_SPIN://��ž
      if(g_state[SHSPUR_PARAM_BODY] == ENABLE && 
	 g_state[SHSPUR_PARAM_TRACKING] == ENABLE)
	spin(&gl_odometry, g_spur_theta, g_spur_w);
      break;
    }//*/
    /*�ݸ���*/
    pthread_mutex_unlock(&mutex);

  }

}
