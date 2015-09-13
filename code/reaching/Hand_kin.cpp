/*****************************************************************************************
Description: Cumputes the arm's inverse kinematics using the iterative algorithm suggested
			 in the paper paper "An anthropomorphic robot torso for imitation: design and
			 experiments"

Contents:	 int limites(double *sol, double *lim, double junta[2])
			 int transsol(double a, double b, double c, double *sol)
			 int cininvit(double *pos, double teta3, double junta_s[6][MAX_SOL])
			 int calccin(double *pos, double junta[6][MAX_SOL], float n1, float n2, float n3)

Recives:	 The position cordinates and the normal vector components to the disered
			 orientation.

Returns:	 Vector with the joint angles in radians.

Author:		 Paulo Carreiras

Date:		 24.01.2007
*****************************************************************************************/

#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <fstream>
#include <conio.h>
#include "BALTAZAR/myMatrix.h"
#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif
#ifndef MAX_SOL
#define MAX_SOL 50
#endif
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/*********************************************************************
Function:	limites
Recives: vector with 2 possible solutions, vector with joint's limits 
		 and the joint vector
Return: the number of solutions found within the physical limits
*********************************************************************/
int limites(double *sol, double *lim, double junta[2]) {
	int aux=-1;
    
	if((sol[0]>lim[0])&&(sol[0]<lim[1])){
		aux++;
		junta[aux]=sol[0];
	}
	if((sol[1]>lim[0])&&(sol[1]<lim[1])){
		aux++;
		junta[aux]=sol[1];
	}
	//if(aux==-1)
	//	return -2;
	//else
		return aux;
}
   


/*********************************************************************
Function:	transsol
Recives: a,b,c with respect to the paper equation and the solution vector
Returns: -1 if no solution is found to the transcewndental equation
		 0 if a solution is found 
***********************************************************************/
int transsol(double a, double b, double c, double *sol){

    if((a+c)==0) {
        sol[0]=M_PI_2;
        sol[1]=-M_PI_2;
    }
    else
        if(a*a+b*b-c*c==0){
                sol[0]=2*atan(b/(a+c));
                sol[1]=sol[0];
        }
        else
			if(a*a+b*b-c*c<0)
                return -1;
			else{
                sol[0]=2*atan((b+sqrt(a*a+b*b-c*c))/(a+c));
                sol[1]=2*atan((b-sqrt(a*a+b*b-c*c))/(a+c));
        }

    return 0;        
}

/*********************************************************************
Function:	cininv
Recives: position, joints vector and normal components
Returns: -1 if the position is unreachable
		 n  if n solutions are found for the disred position
***********************************************************************/
int cininv(double *pos,double junta_s[6][MAX_SOL], float n1, float n2, float n3){
	//double lim1[2]={-2.35, 0.5},lim2[2]={-2.35, 0.26},lim3[2]={-3.0, 3.0},lim4[2]={-2.7, 0.02};
	
	// as written by Paulo: [-135 45] degrees, contrary to what's written on paper
	double lim1[2]={-2.35, 0.78};

	//double lim1[2]={-2.35, 0.5};

	// Giovanni: [-45 135] as on paper
	//double lim1[2] = {-0.78, 2.35};

	double lim2[2]={-1.91, 0.18}, lim3[2]={-1.57, 0}, lim4[2]={-1.57, 0}, lim5[2]={-1.57,1.57}, lim6[2]={-0.78,0.78};;
	double l1=29.13, l2=26.95, d1=2.82, d2=2.18;

	double a,a1,b,c;
	double sol[2],sol2[2];
	double teta3,teta3m;

	int i=-1,j,s=0;
	double junta[6][2];


	if(fabs((pos[0]*pos[0]+pos[1]*pos[1]+pos[2]*pos[2]-l1*l1-l2*l2)/(2*l1*l2))>1){ //the position is unreachable
		junta[0][0]=-5;
		return -1;
	}
	junta[2][0]=-M_PI_2;
	double last=-M_PI_2;
	
	while(junta[2][0]<0){
		teta3=junta[2][0];
		//Determinig teta4
		a=2*(-d2*d1+l2*l1);
		b=-2*(d1*l2+d2*l1);
		c=pos[0]*pos[0]+pos[1]*pos[1]+pos[2]*pos[2]-d2*d2-d1*d1-l2*l2-l1*l1;
		if(transsol(a,b,c,sol)==-1) //the equation has no solutions 
			return -1;
		for(i=limites(sol,lim4,junta[3]);i>=0;i--){ //i<->junta[3]
			//determining teta3
			a1=d2*cos(junta[3][i])+l2*sin(junta[3][i])-d1;
			a=a1*sin(teta3);
			b=-d1*sin(junta[3][i])+l2*cos(junta[3][i])+l1;
			c=pos[2];

			if((a*a+b*b-c*c)<0){ //restriction in teta3
				teta3m=-fabs(asin(sqrt(pos[2]*pos[2]-b*b)/a1));
				if(teta3>teta3m){
					teta3=teta3m;
					a=a1*sin(teta3);
				}
			}
			sol[0]=teta3;
			sol[1]=teta3;

			//determinig teta2
			if(transsol(a,b,c,sol)==-1)
				break;
			for(j=limites(sol,lim2,junta[1]);j>=0;j--){//j<->junta[1]
				//determiningteta1
				a=-cos(junta[1][j])*pos[0];
				b=-cos(junta[1][j])*pos[1];
				c=sin(junta[3][i])*d2-cos(junta[3][i])*l2-l1+sin(junta[1][j])*pos[2];
				//a=-pos[0]*(sin(junta[1])+cos(junta[1]));
				//b=-pos[1]*(sin(junta[1])+cos(junta[1]));
				//c=-pos[2]*cos(junta[1])-d1*sin(junta[2])+d2*cos(junta[3])*sin(junta[2])+l2*sin(junta[2])*sin(junta[3])+pos[2]*sin(junta[1])-l1-l2*cos(junta[3])+d2*sin(junta[3]);
				transsol(a,b,c,sol);
			    
				a=-pos[1];
				b=pos[0];
				c=-cos(junta[2][0])*(cos(junta[3][i])*d2+sin(junta[3][i])*l2-d1);
				transsol(a,b,c,sol2);

				if(fabs(sol[0]-sol2[0])<0.01&&(teta3-last>0.1||i>0||j>0||i==-1)){
					last=junta[2][0];
					junta[0][0]=sol[0];
				}
				else
					if(fabs(sol[1]-sol2[1])<0.01&&(teta3-last>0.1||i>0||j>0||i==-1)){
						last=junta[2][0];
						junta[0][0]=sol[1];
					}
					else
						break;
				sol[0]=junta[0][0];
				sol[1]=junta[0][0];
				if(limites(sol,lim1,junta[0])==-1)
					break;

				junta[5][0]=asin(-n2*cos(junta[2][0])*(-cos(junta[0][0])*cos(junta[1][j])+sin(junta[0][0])
					*sin(junta[1][j]))+n1*(-cos(junta[3][i])*(-cos(junta[0][0])*cos(junta[1][j])+
					sin(junta[0][0])*sin(junta[1][j]))*sin(junta[2][0])-(cos(junta[1][j])*sin(junta[0][0])
					+cos(junta[0][0])*sin(junta[1][j]))*sin(junta[3][i]))+n3*(cos(junta[3][i])*(cos(junta[1][j])
					*sin(junta[0][0])+cos(junta[0][0])*sin(junta[1][j]))-(-cos(junta[0][0])*cos(junta[1][j])+
					sin(junta[0][0])*sin(junta[1][j]))*sin(junta[2][0])*sin(junta[3][i])));

				junta[4][0]=atan2(-n2*cos(junta[2][0])*(-cos(junta[1][j])*sin(junta[0][0])-cos(junta[0][0])
					*sin(junta[1][j]))+n1*(-cos(junta[3][i])*(-cos(junta[1][j])*sin(junta[0][0])-cos(junta[0][0])
					*sin(junta[1][j]))*sin(junta[2][0])-(-cos(junta[0][0])*cos(junta[1][j])+sin(junta[0][0])
					*sin(junta[1][j]))*sin(junta[3][i]))+n3*(cos(junta[3][i])*(-cos(junta[0][0])*cos(junta[1][j])
					+sin(junta[0][0])*sin(junta[1][j]))-(-cos(junta[1][j])*sin(junta[0][0])-cos(junta[0][0])
					*sin(junta[1][j]))*sin(junta[2][0])*sin(junta[3][i])),n1*cos(junta[2][0])*cos(junta[3][i])
					-n2*sin(junta[2][0])+n3*cos(junta[2][0])*sin(junta[3][i]));

				sol[1]=4;
				sol[0]=junta[4][0];
				if(limites(sol,lim5,junta[4])==-1) //if the orientation is unreachable apply 0º
					junta[4][0]=0;
				sol[0]=junta[5][0];
				if(limites(sol,lim6,junta[5])==-1) //if the orientation is unreachable apply 0º
					junta[5][0]=0;


				junta_s[0][s]=junta[0][0];
				junta_s[1][s]=junta[1][j];
				junta_s[2][s]=junta[2][0];
				junta_s[3][s]=junta[3][i];
				junta_s[4][s]=junta[4][0];
				junta_s[5][s]=junta[5][0];
				s++;
				if(s>MAX_SOL)
					return s;
				//junta[3]=-(junta[3]+M_PI_2); //adaptação do valor da junta para o webots
			}
		}
		junta[2][0]+=0.1*M_PI/180;
	}
	//if(i==-2||j==-2) //test if the solutions are within the joint's limits
	//	return -1;
	return s;
}

/*********************************************************************
Function:	dkinematics
Recives: joints' positions, nb of desired joint (1-5) and vector to fill
Returns: fills the vector res with the njoint joint 3D position
***********************************************************************/
void dkinematics (double *juntas, int njoint, double res[3]){
	double l1=29.13,l2=26.95,a1=2.82,a2=2.18;

	//Modifeied Denavit-Hartenberg parameters (Craig)
	double alfa[6]={0,M_PI_2,M_PI_2,M_PI_2,-M_PI_2,M_PI_2};
	double a[6]={0,0,0,a1,-a2,0};
	double d[6]={0,0,l1,0,l2,0};
	myMatrix<double> tehta(6,1,juntas);
	double temp[6]={0,M_PI_2,M_PI_2,0,M_PI_2,0};
	myMatrix<double> aux(6,1,temp);
	tehta=tehta+aux;
	//cout<<"tehta"<<tehta<<endl;
	myMatrix<double>ta(4,4);
	myMatrix<double> T(4,4,1);


	//Transformation for each joint
	for (int j=0;j<njoint;j++){
		ta.setValue(0,cos(tehta.getValue(j)));
		ta.setValue(1,-sin(tehta.getValue(j)));
		ta.setValue(2,0);
		ta.setValue(3,a[j]);
		ta.setValue(4,sin(tehta.getValue(j))*cos(alfa[j]));
		ta.setValue(5,cos(tehta.getValue(j))*cos(alfa[j]));
		ta.setValue(6,-sin(alfa[j]));
		ta.setValue(7,-sin(alfa[j])*d[j]);
		ta.setValue(8,sin(tehta.getValue(j))*sin(alfa[j]));
		ta.setValue(9,cos(tehta.getValue(j))*sin(alfa[j]));
		ta.setValue(10,cos(alfa[j]));
		ta.setValue(11,cos(alfa[j])*d[j]);
		ta.setValue(12,0);
		ta.setValue(13,0);
		ta.setValue(14,0);
		ta.setValue(15,1);
		//cout<<"t"<<ta<<endl;
		T=T*ta;
	}
	//cout<<"T(junta"<<njoint+1<<")"<<T<<endl;

	res[0]=T.getValue(3);
	res[1]=T.getValue(7);
	res[2]=T.getValue(11);
}



/*
not using this (Giovanni 081027)

Function:	changeRef
Recive:		Vector with the object's coordenates in the world's referential
			Intiger that especifies the coordenate's referencial output
				->S=0; ->coordenates REF of cameras
				->P=1; ->coordenates REF of shoulder
Return:		Vector with the object's coordenates in the camera's referential

void changeRef(double *saidaR, double *mundR, int SP){
	
	//transf de coord do ref do mundo centrado em (0,0,0) para o ref das camaras
	switch(SP){
		case 0:
	
			saidaR[0]=-(mundR[1]-1.435)*100;
			saidaR[1]=-(mundR[2]+0.0856-0.1)*100;
			saidaR[2]=(mundR[0]+0.2957-0.1)*100;
			break;
		case 1:
			saidaR[0]=-(mundR[1]-0.8659)*100;
			saidaR[1]=-(mundR[2]+0.0856)*100;
			saidaR[2]=(mundR[0]+0.2957)*100;
			break;
	}
}
*/


/*
// reimplemented in reaching.cpp (Giovanni 081027)

Function:	armCalcs
Recives: vector with actual position, joint's vector and elbow
		 restrictions
Return: update the content of *juntas with the minimal energy solution 
void armCalcs(double *rOP, double *juntas, double posm[3], int plat){
	double objPos_RW[3], objPos[3];
	double junta_s[6][MAX_SOL];
	int s=0;

	for(int i=0; i<3; i++)
		objPos_RW[i]=rOP[i];


	//Transformation of the object's coordenates to the shoulder's referential
	if(plat==0)
		changeRef(objPos, objPos_RW, 1);
	else
		changeRef(objPos, objPos_RW, 0);
	//printf("robot_REF_shoulder(%f, %f, %f);\n", objPos[0], objPos[1], objPos[2]);

	//Make the inverse kinematics
	s=cininv(objPos,junta_s,1,0,0);

	//use energy minimization criterio for choosing the nearest joints positions
	int aux=0,aux1=-1;
	double e,E,Ea=-1,res[3];


	double W[6]={1,1,1.47,1.89,1.22,1.43}; //weights of each joint accordingly to the servo capabilities
	if(s>0){
		int j;
		do{
			E=999999;
			for(j=0;j<s;j++){
				e=0;
				for(int k=0;k<6;k++)
					e+=W[k]*abs(juntas[k]-junta_s[k][j]); //'juntas' has the last joint values applied
				if(e<E||(e>Ea&&aux1!=-1)){
					aux=0;
					E=e;
					//update the joints positions
					juntas[0]=junta_s[0][j];
					juntas[1]=junta_s[1][j];
					juntas[2]=junta_s[2][j];
					juntas[3]=junta_s[3][j];
					juntas[4]=junta_s[4][j];
					juntas[5]=junta_s[5][j];
					dkinematics(juntas,3,res);
					if(res[0]>posm[0]&&res[1]>posm[1]&&res[2]>posm[2]){
						Ea=e;
						aux=1;
					}
				}
			}
			aux1=0;
		}while(aux==0 && j<s);
	}


	if(junta_s[0][0]==-5)
		aux=0;
	else{
	if(s<1||aux==0) //the position isn't reachable
		juntas[0]=-4;
	}
	
}
*/



