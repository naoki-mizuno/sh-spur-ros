#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <sh_spur_ros/CoordinateSystem2D.h>

CSptr CSroot_ptr;

/*$B:BI87O$N@8@.(B*/
CSptr CS_add(CSptr parent_cs, double x, double y, double theta){
   CSptr a_cs,p_cs;   
     
   /*$B?7$7$$:BI87O$r:n@.(B*/
   a_cs = (CSptr)malloc(sizeof(CoordinateSystem));
   if(!a_cs)return (CSptr)0;/*$B:n@.<:GT(B*/

   if(parent_cs == 0){/*$B:BI87O$N?@(B*/
     CSroot_ptr    = a_cs;
     a_cs->parent  = 0;
     a_cs->brother = 0;
     a_cs->child   = 0;
     a_cs->x     = 0;
     a_cs->y     = 0;
     a_cs->theta = 0;
     a_cs->level = 0;
   }else{/*$BC/$+$N;R(B*/
     p_cs = parent_cs;
     
     if(!p_cs->child){/*$BD9CK!#(B*/
       p_cs->child = a_cs;
     }else{/*$B4{$K7;$,$$$k(B*/
       p_cs = p_cs->child;
       while(p_cs->brother)p_cs = p_cs->brother;
       p_cs->brother = a_cs;
     }
     a_cs->parent  = parent_cs;
     a_cs->brother = 0;
     a_cs->child   = 0;
     a_cs->x     = x;
     a_cs->y     = y;
     a_cs->theta = theta;
     a_cs->level = parent_cs->level+1;
   }

   return a_cs;
}

/*$B:BI87O$N:o=|!J;R!&7;Do$b$m$H$b!K(B*/
/* $B<BAuCfESH>C<(B */
int CS_delete(CSptr target_cs)
{
  if(target_cs->child){
    CS_delete(target_cs->child);
  }

  if(target_cs->brother){
    CS_delete(target_cs->brother);
  }

  if(target_cs){
    free((void*)target_cs);
  }
  return 1;
}

/*$B:BI87O$N@_Dj(B*/
int CS_set(CSptr target_cs, double x, double y,double theta){
  if(!target_cs)return 0;

  target_cs->x = x;
  target_cs->y = y;
  target_cs->theta = theta;

  return 1;
}

/*$B$"$k:BI87O>e$N:BI8$rF~NO(B*/
int CS_set_on_CS(CSptr target_cs, CSptr on_cs, double x, double y,double theta){
  if(!target_cs || !on_cs)return 0;
  if(target_cs->parent)
    CS_recursive_trans(target_cs->parent, on_cs,
		       &x, &y, &theta);
  return 1;
}

/*$B;R$+$i8+$??F$N0LCV$r!"?F$+$i8+$?<+J,$N0LCV$KJQ49$9$k(B*/
void CS_turn_base(double *x,double *y,double *theta){
  double xx,yy;

  xx = -(*x)*cos(-(*theta)) + (*y)*sin(-(*theta));
  yy = -(*x)*sin(-(*theta)) - (*y)*cos(-(*theta));
  *theta = -(*theta);
  *x = xx;
  *y = yy;
}

/*-----------------$B:BI8JQ49$^$o$j(B-----------------*/
/*$BL\E*$N:BI87O$X$R$H$C$H$S!)(B*/
/*$B!!!!!!!!!!!!!!!!!!!!!!!!!!(B*/
void CS_recursive_trans(CSptr target_cs, CSptr now_cs,
			 double *x, double *y, double *theta){
  /*$B:BI87O$,M-8z$+(B*/
  if(!target_cs || !now_cs)return;
  //printf("now %d\n",now_cs->level); 
 /*$BF1$8:BI87O$+(B(case A)*/
  if(target_cs == now_cs)return;/*$B=*N;(B*/
 
  /*$B2<$N:BI87O$+(B*/
  if(target_cs->level == now_cs->level){/*$BF1$8%l%Y%k$K$$$k$1$I0c$&:BI87O(B*/
    //printf(".down from %d\n",now_cs->level);
    inv_trans_cs(now_cs, x, y, theta);/*$B:BI87O$r$R$H$D2<$k(B*/
    CS_recursive_trans(target_cs->parent,now_cs->parent, x, y, theta);
    //printf(".up to %d\n"  ,target_cs->level);
    trans_cs(target_cs, x, y, theta);/*$B:BI87O$r$R$H$DEP$k(B*/
  }else  if(target_cs->level > now_cs->level){/*$B8=:_0LCV$NJ}$,2<(B case C*/
    CS_recursive_trans(target_cs->parent,now_cs, x, y, theta);/*$B$R$H$D2<$k(B*/
    //printf("up to %d\n",target_cs->level);
    trans_cs(target_cs, x, y, theta);/*$B:BI87O$r$R$H$DEP$k(B*/
  }else{/*$B8=:_0LCV$NJ}$,>e(B case D*/
    //printf("down from %d\n"  ,now_cs->level);
    inv_trans_cs(now_cs, x, y, theta);/*$B:BI87O$r$R$H$D2<$k(B*/
    CS_recursive_trans(target_cs,now_cs->parent, x, y, theta);/*$B$R$H$D2<$k(B*/
  }

  return;
}

/*$B:BI87O$r0lCJ2<$k!J(B1$BCJA0$N:BI87O$G$N:BI8$KJQ49$9$k!K(B*/
void inv_trans_cs(CSptr target_cs, double *x, double *y, double *theta){
  double xx,yy;
  if(target_cs){
    xx = *x * cos(target_cs->theta) - *y *sin(target_cs->theta)+ target_cs->x;
    yy = *x * sin(target_cs->theta) + *y *cos(target_cs->theta)+ target_cs->y;
    *x = xx;
    *y = yy;
    *theta += target_cs->theta;
  }
}

/*$B:BI87O$r0lCJ$"$,$k!J0lCJ8e$G$N:BI87O$G$N:BI8$KJQ49$9$k!K(B*/
void trans_cs(CSptr target_cs,double *x,double *y,double *theta ){
  double xx,yy;
  if(target_cs){
     xx = (*x-target_cs->x)*cos(-target_cs->theta) 
       -  (*y-target_cs->y)*sin(-target_cs->theta);
     yy = (*x-target_cs->x)*sin(-target_cs->theta)
       +  (*y-target_cs->y)*cos(-target_cs->theta);
     *x = xx;
     *y = yy;
     *theta -= target_cs->theta;
   }
}

/*$B>e$K$"$,$k$@$1(B*/
void trace_trans_cs(CSptr target_cs,double *x,double *y,double *theta ){
  if(target_cs == CSroot_ptr)return;
  trace_trans_cs(target_cs->parent,x,y,theta);
  trans_cs(target_cs, x,y,theta);

  return ;
}
