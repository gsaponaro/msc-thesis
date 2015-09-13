// note by rasm (27/Feb/2006):
// Visual C++ 98 doesn't include M_PI and such, thus
// a definition is forced here
#define M_PI       3.14159265358979323846
#define M_PI_2     1.57079632679489661923

#include <math.h>
#include <stdio.h>

#include "iCub/kinematics/robmatrix.h"
#include "kinpronator.h"

/***************************************************************
*
* inputs:
* pos - position p[]={x,y,z}
* t3e - theta 3
* th - variable for returning the calculated th
*
* returns:
*   0 - position achieved
*   -i - i joint outside the physical limits
*   1 - too far away   
***************************************************************/
int ikpospronator(double *pos, double t3e, double* th, RobMatrix *T)
{
int ret=0;
double lim1[2]={-2.35, 0.5};
double lim2[2]={-2.35, 0.26};
double lim3[2]={-3.0, 3.0};
double lim4[2]={-2.7, 0.02};

/**/
double l1=29.13;
double l2=26.95;
double d1=2.82;
double d2=2.18;

double /*v,*/ d;
//double xb, yb, zb;
double aux1, aux2, aux3;
double a,b/*,c*/;
double t1,t2,t3,t4;
double s[2],s2[2];
float x,y,z;

    x = (float) pos[0];
	y = (float) pos[1];
	z = (float) pos[2];
    
    d=(x*x+y*y+z*z-l1*l1-l2*l2)/(2*l1*l2);

    if(fabs(d)>1) {
        fprintf(stderr,"\nout of reach\n");
        return 1;
    }

    aux1 = 2*(-d2*d1+l2*l1);
    aux2 = -2*(d1*l2+d2*l1);
    aux3 = x*x+y*y+z*z-d2*d2-d1*d1-l2*l2-l1*l1;

	if(myasbccsol(aux1,aux2,aux3,s)) {
		//printf("DEBUG: I am in myasbccsol\n");
        return 0;
	}
    else {

		if(limitjoint( s, lim4, &t4 )) {
			//printf("DEBUG: I am in limitjoint\n");
            return 1;
		}
    }
        
    th[3]=t4;
t3=t3e;

a=109.0/50.0*cos(t4)+539.0/20.0*sin(t4)-141.0/50.0;
b=(-109.0/50.0*sin(t4)+539.0/20.0*cos(t4)+2913.0/100.0);
//b=0.01*(-218*sin(t4)+2695*cos(t4)+2913)

aux1=a*sin(t3);
aux2=b;
aux3=z;

if( (aux1*aux1+aux2*aux2-aux3*aux3) < 0 ) {
double t3m;

    t3m=-fabs(asin( sqrt(z*z-b*b) / a ));
    if(t3>t3m) {
		//printf(" o valor minimo para theta_3 e %0.4f",t3m*180/M_PI);
        t3=t3m;
        aux1=a*sin(t3);
    }
}

    th[2]=t3;
//printf("aux for t2 > %f %f %f\n",aux1,aux2,aux3);

	if(myasbccsol(aux1,aux2,aux3,s)) {
			//printf("DEBUG: I am in myasbccsol\n");
            return 0;
		}
    else {
		if(limitjoint( s, lim2, &t2 )) {
			//printf("DEBUG: I am in limitjoint\n");
            return 1;
		}
    }
    printf("t2 %f %f\n",s[0],s[1]);
    th[1]=t2;
    
    aux1=-cos(t2)*x;
    aux2=-cos(t2)*y;
    aux3=sin(t4)*d2-cos(t4)*l2-l1+sin(t2)*z;
    
    myasbccsol(aux1,aux2,aux3,s);
    
    aux1=-y;
    aux2=x;
    aux3=-cos(t3)*(cos(t4)*d2+sin(t4)*l2-d1);

    myasbccsol(aux1,aux2,aux3,s2);

	if(fabs(s[0]-s2[0])<0.01) {
		//printf("DEBUG, point A: setting th[0]\n");
        t1=s[0];
	}
	else if(fabs(s[1]-s2[1])<0.01) {
		//printf("DEBUG, point B: setting th[0]\n");
        t1=s[1];
	}
    else
        return -1;
        
    printf("t1 %f %f %f %f\n",s[0],s[1],s2[0],s2[1]);
    th[0]=t1;
    
    return 0;
    
}

/*********************************************************************
*
*
*
*
*
***********************************************************************/
int ikpronator(double *pos, double t3e, double *pl, double* th, RobMatrix *T) {
double lim5[2]={-M_PI_2,M_PI_2};
double lim6[2]={-1.221,1.221};
int ret=0;
double s[2];
/**/
double l1=29.13;
double l2=26.95;

//double v, d;
//double xb, yb, zb;
double aux1, aux2, aux3;
double t5,t6;

float x,y,z;
double v1,v2,v3;

    ret=ikpospronator( pos, t3e, th, T);
    printf("position ret = %d\n",ret);

    RobMatrix T04,T05,T06;

    x = (float) pos[0];
	y = (float) pos[1];
	z = (float) pos[2];
    v1=pl[0];   v2=pl[1];   v3=pl[2];
    
    // Orientation

    T04=T[0].V(th[0])*T[1].V(th[1])*T[2].V(th[2])*T[3].V(th[3]);
//    T04.print();
    
    t6 = -asin( v1 * T04.M[0][1] + v2 * T04.M[1][1] + v3* T04.M[2][1]);
    s[0]=t6;s[1]=t6;
    if(limitjoint( s, lim6, &t6 ))
                return -6;
    
    
    th[5]=t6;
                    
    aux1 = -v1 * T04.M[0][2] - v2 * T04.M[1][2] - v3 * T04.M[2][2];
    aux2 = -v1 * T04.M[0][0] - v2 * T04.M[1][0] - v3 * T04.M[2][0];
    aux3 = cos(t6);
        
    myasbccsol(aux1,aux2,aux3,s);
    if(myasbccsol(aux1,aux2,aux3,s))
            return -5;
    else {

        if(limitjoint( s, lim5, &t5 ))
                return -5;
    }
        
    th[4]=t5;
    
//    T[4].V(t5).print();
//    T[5].V(t6).print();    
    
    T[7]=T04*T[4].V(t5)*T[5].V(t6);
//    T[7].print();
    
    return 0;
}

/********************************************************************
*
* it is limited to horizontal hand
*
*
********************************************************************/
int ikorientationpronator( double *pl, double* th, RobMatrix T04) {
double aux;

th[4] = atan(  T04.M[0][0] /  T04.M[0][2] );
th[5] = atan(  T04.M[0][1] / (- T04.M[0][0]*sin(th[4])- T04.M[0][2]*cos(th[4])));

aux = (- T04.M[0][0]*sin(th[4])- T04.M[0][2]*cos(th[4]))+T04.M[0][1]*sin(th[5]);

if(aux<-0.99)
	printf("ok\n");
else if(aux>0.99)
{
	th[4] = atan(  T04.M[0][0] /  T04.M[0][2] )-M_PI;
	th[5] = atan(  T04.M[0][1] / (- T04.M[0][0]*sin(th[4])- T04.M[0][2]*cos(th[4])));
	printf("ok with -180\n");	
}
else
	printf("no solution\n");

/*
double v1,v2,v3;
double t5,t6;
double s[2];
double lim5[2]={-M_PI_2,M_PI_2};
double lim6[2]={-1.221,1.221};
double aux1, aux2, aux3;


    v1=pl[0];   v2=pl[1];   v3=pl[2];
    
    // Orientation
    t6 = -asin( v1 * T04.M[0][1] + v2 * T04.M[1][1] + v3* T04.M[2][1]);
    s[0]=t6;s[1]=t6;
    if(limitjoint( s, lim6, &t6 ))
                return -6;
    
    
    th[5]=t6;
                    
    aux1 = -v1 * T04.M[0][2] - v2 * T04.M[1][2] - v3 * T04.M[2][2];
    aux2 = -v1 * T04.M[0][0] - v2 * T04.M[1][0] - v3 * T04.M[2][0];
    aux3 = cos(t6);
        
    myasbccsol(aux1,aux2,aux3,s);
    if(myasbccsol(aux1,aux2,aux3,s))
            return -5;
    else {

        if(limitjoint( s, lim5, &t5 ))
                return -5;
    }
        
    th[4]=t5;
  */    
    return 0;    
}
