#ifndef _HANDCIN_H_
#define _HANDCIN_H_

#ifndef MAX_SOL
#define MAX_SOL 50
#endif

void armCalcs(double *rOP, double *juntas, double posm[3], int plat);
void changeRef(double *saidaR, double *mundR, int SP);
int limites(double *sol, double *lim, double junta[2]);
int transsol(double a, double b, double c, double *sol);
int cininv(double *pos,double junta_s[6][MAX_SOL], float n1, float n2, float n3);
void dkinematics (double *juntas,int njoint, double res[3]);

#endif
