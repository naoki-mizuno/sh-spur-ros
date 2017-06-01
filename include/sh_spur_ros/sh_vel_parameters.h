#ifndef __PARAMETER__
#define __PARAMETER__

typedef union{
  int integer;
  char byte[4];
}Int_4Char;


typedef union _short_2char{
  short integer;
  char  byte[2];
}Short_2Char;

enum{
  SERVO_LEVEL_STOP=0,
    SERVO_LEVEL_COUNTER,
    SERVO_LEVEL_TORQUE,
    SERVO_LEVEL_VELOCITY,
    SERVO_LEVEL_POSITION// n/a
};

enum{
  PARAM_w_ref = 0,
    PARAM_w_ref_diff,
    PARAM_p_ki,
    PARAM_p_kv,
    PARAM_p_fr_plus,
    PARAM_p_fr_wplus,
    PARAM_p_fr_minus,
    PARAM_p_fr_wminus,
    PARAM_p_A,
    PARAM_p_B,
    PARAM_p_C,
    PARAM_p_D,
    PARAM_p_E,
    PARAM_p_F,
    PARAM_p_pi_kp,
    PARAM_p_pi_ki,
    PARAM_pwm_max,
    PARAM_pwm_min,
    PARAM_toq_max,
    PARAM_toq_min,
    PARAM_int_max,
    PARAM_int_min,
    PARAM_p_toq_offset,
    PARAM_servo = 64,
    PARAM_watch_dog_limit
    };

#endif 
