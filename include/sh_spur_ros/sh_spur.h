#ifndef __SH_SPUR__
#define __SH_SPUR__

double get_time(void);
/*CS$B$N=i4|2=(B*/
void init_coordinate_systems(void);
/*$B$9$l$C$I$N=i4|2=(B*/
void init_thread(pthread_t* thread);
/*SSM$B$N=i4|2=(B*/
void init_SSM(void);

#endif
