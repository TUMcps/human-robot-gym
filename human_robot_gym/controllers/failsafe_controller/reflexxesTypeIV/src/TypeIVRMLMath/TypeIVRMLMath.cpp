































#include <TypeIVRMLMath.h>
#include <TypeIVRMLDefinitions.h>
#include <math.h>


void qPN_6::m4lDi(double*rw_g_,double*WmQCA){double BgHLH=*rw_g_,vDA6B=*WmQCA,
tqhG1=x96T7*(vDA6B-BgHLH)+oMyUa;*rw_g_+=tqhG1;*WmQCA-=tqhG1;if(*rw_g_<=*WmQCA){
return;}else{*rw_g_=BgHLH;*WmQCA=vDA6B;}return;}

void qPN_6::q2lFe(double*rw_g_,double*WmQCA){double BgHLH=*rw_g_,vDA6B=*WmQCA;*
rw_g_=(BgHLH-oMyUa-x96T7*(vDA6B+BgHLH))/(1.0-2.0*x96T7);*WmQCA=(vDA6B+oMyUa-
x96T7*(vDA6B+BgHLH))/(1.0-2.0*x96T7);if(*rw_g_>*WmQCA){*rw_g_=BgHLH;*WmQCA=vDA6B
;}if(m85bi(*rw_g_)!=m85bi(*WmQCA)){if(fabs(*WmQCA)>fabs(*rw_g_)){*rw_g_=0.0;}
else{*WmQCA=0.0;}}return;}

double qPN_6::d01Xk(const double&M0tW3){unsigned int i=(0x135b+4951-0x26b2);
double ReturnValue=1.0,Time=M0tW3;if(Time>50.0){Time*=0.1;for(i=
(0x585+3061-0x117a);i<(0x26b2+91-0x2701);i++){if(Time>10.0){Time*=0.1;
ReturnValue*=4.0;}else{break;}}return(ReturnValue);}else{return(0.0);}}

unsigned int qPN_6::UifCr(const RMLBoolVector&j9Ldc){unsigned int i=
(0x1078+3988-0x200c),iTzDP=(0xc18+6625-0x25f9);for(i=(0x14d7+3512-0x228f);i<
j9Ldc.VectorDimension;i++){if(j9Ldc.VecData[i]){iTzDP++;}}return(iTzDP);}
