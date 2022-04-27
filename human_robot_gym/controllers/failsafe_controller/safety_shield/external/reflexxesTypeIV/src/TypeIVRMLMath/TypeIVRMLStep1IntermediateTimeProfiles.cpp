

































#include <TypeIVRMLStep1IntermediateTimeProfiles.h>
#include <TypeIVRMLMath.h>
#include <TypeIVRMLDefinitions.h>
#include <math.h>



void qPN_6::aepf8(double*TfLg_,double*JRB5m,double*jEwtH,double*EgDTv,double*
EI730){

*TfLg_=-(*TfLg_);*JRB5m=-(*JRB5m);*jEwtH=-(*jEwtH);*EgDTv=-(*EgDTv);*EI730=-(*
EI730);return;}


void qPN_6::OuC3N(double*wBmQ7,double*TfLg_,double*JRB5m,double*jEwtH,const 
double&DAwhk,const double&b4jXr){
double mn8nn=0.0;

mn8nn=((*jEwtH)-DAwhk)/b4jXr;*wBmQ7+=(mn8nn);*TfLg_+=(*JRB5m)*mn8nn+0.5*(*jEwtH)
*VNvlY(mn8nn)-b4jXr*qn3x8(mn8nn)/6.0;*JRB5m+=(*jEwtH)*mn8nn-0.5*b4jXr*VNvlY(
mn8nn);*jEwtH=DAwhk;return;}


void qPN_6::n9iwH(double*wBmQ7,double*TfLg_,double*JRB5m,double*jEwtH,const 
double&b4jXr){
double mn8nn=0.0;

mn8nn=(*jEwtH)/b4jXr;*wBmQ7+=mn8nn;*TfLg_+=(*JRB5m)*mn8nn+0.5*(*jEwtH)*VNvlY(
mn8nn)-b4jXr*qn3x8(mn8nn)/6.0;*JRB5m+=(*jEwtH)*mn8nn-0.5*b4jXr*VNvlY(mn8nn);*
jEwtH=0.0;return;}


void qPN_6::JY88s(double*wBmQ7,double*TfLg_,double*JRB5m,double*jEwtH,const 
double&b4jXr){
double mn8nn=0.0;

mn8nn=-(*jEwtH)/b4jXr;*wBmQ7+=mn8nn;*TfLg_+=(*JRB5m)*mn8nn+0.5*(*jEwtH)*VNvlY(
mn8nn)+b4jXr*qn3x8(mn8nn)/6.0;*JRB5m+=(*jEwtH)*mn8nn+0.5*b4jXr*VNvlY(mn8nn);*
jEwtH=0.0;return;}


void qPN_6::Rr9wP(double*wBmQ7,double*TfLg_,double*JRB5m,double*jEwtH,const 
double&hxixi,const double&b4jXr){
double mn8nn=0.0
,L38Np=0.0;
L38Np=pbQOc(VNvlY(*jEwtH)-2.0*b4jXr*(hxixi+(*JRB5m)));
mn8nn=(L38Np-(*jEwtH))/b4jXr;*wBmQ7+=mn8nn;*TfLg_+=(*JRB5m)*mn8nn+0.5*(*jEwtH)*
VNvlY(mn8nn)+b4jXr*qn3x8(mn8nn)/6.0;*JRB5m=-hxixi;*jEwtH=L38Np;return;}


void qPN_6::Fj8b0(double*wBmQ7,double*TfLg_,double*JRB5m,double*jEwtH,const 
double&hxixi,const double&DAwhk,const double&b4jXr){
double mn8nn=0.0;

mn8nn=(DAwhk-(*jEwtH))/b4jXr;*wBmQ7+=mn8nn;*TfLg_+=(*JRB5m)*mn8nn+0.5*(*jEwtH)*
VNvlY(mn8nn)+b4jXr*qn3x8(mn8nn)/6.0;*JRB5m+=(*jEwtH)*mn8nn+0.5*b4jXr*VNvlY(mn8nn
);*jEwtH=DAwhk;
mn8nn=(-hxixi-(*JRB5m))/DAwhk;*wBmQ7+=mn8nn;*TfLg_+=(*JRB5m)*mn8nn+0.5*DAwhk*
VNvlY(mn8nn);*JRB5m=-hxixi;*jEwtH=DAwhk;return;}


void qPN_6::QBBJJ(double*wBmQ7,double*TfLg_,double*JRB5m,double*jEwtH,const 
double&hxixi,const double&b4jXr){
double mn8nn=0.0
,BawAR=0.0;
BawAR=pbQOc((VNvlY((*jEwtH))+2.0*b4jXr*(hxixi-(*JRB5m)))/2.0);
mn8nn=(BawAR-(*jEwtH))/b4jXr;*wBmQ7+=mn8nn;*TfLg_+=(*JRB5m)*mn8nn+0.5*(*jEwtH)*
VNvlY(mn8nn)+b4jXr*qn3x8(mn8nn)/6.0;*JRB5m+=(*jEwtH)*mn8nn+0.5*b4jXr*VNvlY(mn8nn
);*jEwtH=BawAR;
mn8nn=(*jEwtH)/b4jXr;*wBmQ7+=mn8nn;*TfLg_+=(*JRB5m)*mn8nn+0.5*(*jEwtH)*VNvlY(
mn8nn)-b4jXr*qn3x8(mn8nn)/6.0;*JRB5m=hxixi;*jEwtH=0.0;return;}


void qPN_6::Ut1Kv(double*wBmQ7,double*TfLg_,double*JRB5m,double*jEwtH,const 
double&hxixi,const double&b4jXr){
double mn8nn=0.0
,BawAR=0.0
,OuKGy=0.0;

BawAR=pbQOc((VNvlY((*jEwtH))+2.0*b4jXr*(hxixi-(*JRB5m)))/2.0);
mn8nn=(BawAR-(*jEwtH))/b4jXr;*wBmQ7+=mn8nn;*TfLg_+=(*JRB5m)*mn8nn+0.5*(*jEwtH)*
VNvlY(mn8nn)+b4jXr*qn3x8(mn8nn)/6.0;*JRB5m+=(*jEwtH)*mn8nn+0.5*b4jXr*VNvlY(mn8nn
);*jEwtH=BawAR;
OuKGy=pbQOc(VNvlY((*jEwtH))+2.0*b4jXr*(hxixi+(*JRB5m)));
mn8nn=((*jEwtH)-OuKGy)/b4jXr;*wBmQ7+=mn8nn;*TfLg_+=(*JRB5m)*mn8nn+0.5*(*jEwtH)*
VNvlY(mn8nn)-b4jXr*qn3x8(mn8nn)/6.0;*JRB5m=-hxixi;*jEwtH=OuKGy;return;}


void qPN_6::jqBVl(double*wBmQ7,double*TfLg_,double*JRB5m,double*jEwtH,const 
double&hxixi,const double&b4jXr){

double mn8nn=0.0
,BawAR=0.0;
*JRB5m=-(*JRB5m);
BawAR=pbQOc(b4jXr*(hxixi-(*JRB5m)));
BawAR=-BawAR;*JRB5m=-(*JRB5m);
mn8nn=(-BawAR)/b4jXr;*wBmQ7+=mn8nn;*TfLg_+=(*JRB5m)*mn8nn+0.5*(*jEwtH)*VNvlY(
mn8nn)-b4jXr*qn3x8(mn8nn)/6.0;*JRB5m+=(*jEwtH)*mn8nn-0.5*b4jXr*VNvlY(mn8nn);*
jEwtH=BawAR;


*wBmQ7+=mn8nn;*TfLg_+=(*JRB5m)*mn8nn+0.5*(*jEwtH)*VNvlY(mn8nn)+b4jXr*qn3x8(mn8nn
)/6.0;*JRB5m=-hxixi;*jEwtH=0.0;return;}


void qPN_6::nkINu(double*wBmQ7,double*TfLg_,double*JRB5m,double*jEwtH,const 
double&hxixi,const double&DAwhk,const double&b4jXr){

double cjcAG=0.0
,StX1T=0.0
,m0JG3=0.0
,sddwV=0.0
,rT061=0.0
,y71cs=0.0;

cjcAG=(DAwhk-(*jEwtH))/b4jXr;
m0JG3=DAwhk/b4jXr;sddwV=(*jEwtH)*cjcAG+0.5*b4jXr*VNvlY(cjcAG);y71cs=0.5*VNvlY(
DAwhk)/b4jXr;
rT061=hxixi-(*JRB5m)-sddwV-y71cs;StX1T=rT061/DAwhk;
*wBmQ7+=cjcAG;*TfLg_+=(*JRB5m)*cjcAG+0.5*(*jEwtH)*VNvlY(cjcAG)+b4jXr*qn3x8(cjcAG
)/6.0;*JRB5m+=sddwV;*jEwtH=DAwhk;
*wBmQ7+=StX1T;*TfLg_+=(*JRB5m)*StX1T+0.5*(*jEwtH)*VNvlY(StX1T);*JRB5m+=rT061;*
jEwtH=DAwhk;
*wBmQ7+=m0JG3;*TfLg_+=(*JRB5m)*m0JG3+0.5*(*jEwtH)*VNvlY(m0JG3)-b4jXr*qn3x8(m0JG3
)/6.0;*JRB5m=hxixi;*jEwtH=0.0;return;}


void qPN_6::qNBFJ(double*wBmQ7,double*TfLg_,double*JRB5m,double*jEwtH,const 
double&hxixi,const double&DAwhk,const double&b4jXr){
double cjcAG=0.0
,StX1T=0.0
,m0JG3=0.0
,sddwV=0.0
,rT061=0.0
,y71cs=0.0
,BawAR=0.0;
cjcAG=(DAwhk-(*jEwtH))/b4jXr;


sddwV=(*jEwtH)*cjcAG+0.5*b4jXr*VNvlY(cjcAG);y71cs=0.5*VNvlY(DAwhk)/b4jXr;
rT061=hxixi-(*JRB5m)-sddwV-y71cs;StX1T=rT061/DAwhk;
*wBmQ7+=cjcAG;*TfLg_+=(*JRB5m)*cjcAG+0.5*(*jEwtH)*VNvlY(cjcAG)+b4jXr*qn3x8(cjcAG
)/6.0;*JRB5m+=sddwV;*jEwtH=DAwhk;
*wBmQ7+=StX1T;*TfLg_+=(*JRB5m)*StX1T+0.5*(*jEwtH)*VNvlY(StX1T);*JRB5m+=rT061;*
jEwtH=DAwhk;
BawAR=pbQOc(VNvlY((*jEwtH))+2.0*b4jXr*(hxixi+(*JRB5m)));m0JG3=((*jEwtH)-BawAR)/
b4jXr;
*wBmQ7+=m0JG3;*TfLg_+=(*JRB5m)*m0JG3+0.5*(*jEwtH)*VNvlY(m0JG3)-b4jXr*qn3x8(m0JG3
)/6.0;*JRB5m=-hxixi;*jEwtH=BawAR;return;}


void qPN_6::SFHdU(double*wBmQ7,double*TfLg_,double*JRB5m,double*jEwtH,const 
double&hxixi,const double&DAwhk,const double&b4jXr){

double cjcAG=0.0
,StX1T=0.0
,m0JG3=0.0
,sddwV=0.0
,rT061=0.0
,y71cs=0.0;

cjcAG=m0JG3=DAwhk/b4jXr;sddwV=y71cs=-0.5*VNvlY(DAwhk)/b4jXr;
rT061=-hxixi-(*JRB5m)-sddwV-y71cs;StX1T=rT061/(-DAwhk);
*wBmQ7+=cjcAG;*TfLg_+=(*JRB5m)*cjcAG-b4jXr*qn3x8(cjcAG)/6.0;*JRB5m+=sddwV;*jEwtH
=-DAwhk;
*wBmQ7+=StX1T;*TfLg_+=(*JRB5m)*StX1T+0.5*(*jEwtH)*VNvlY(StX1T);*JRB5m+=rT061;*
jEwtH=-DAwhk;
*wBmQ7+=m0JG3;*TfLg_+=(*JRB5m)*m0JG3+0.5*(*jEwtH)*VNvlY(m0JG3)+b4jXr*qn3x8(m0JG3
)/6.0;*JRB5m=-hxixi;*jEwtH=0.0;return;}


void qPN_6::YJ3N_(double*wBmQ7,double*TfLg_,double*JRB5m,double*jEwtH,const 
double&hxixi,const double&DAwhk,const double&b4jXr){

double cjcAG=0.0
,StX1T=0.0
,m0JG3=0.0
,sddwV=0.0
,rT061=0.0
,y71cs=0.0;

cjcAG=(DAwhk-(*jEwtH))/b4jXr;
m0JG3=DAwhk/b4jXr;sddwV=(*jEwtH)*cjcAG+0.5*b4jXr*VNvlY(cjcAG);y71cs=0.5*VNvlY(
DAwhk)/b4jXr;
rT061=-hxixi-(*JRB5m)-sddwV-y71cs;StX1T=rT061/DAwhk;
*wBmQ7+=cjcAG;*TfLg_+=(*JRB5m)*cjcAG+0.5*(*jEwtH)*VNvlY(cjcAG)+b4jXr*qn3x8(cjcAG
)/6.0;*JRB5m+=sddwV;*jEwtH=DAwhk;
*wBmQ7+=StX1T;*TfLg_+=(*JRB5m)*StX1T+0.5*(*jEwtH)*VNvlY(StX1T);*JRB5m+=rT061;*
jEwtH=DAwhk;
*wBmQ7+=m0JG3;*TfLg_+=(*JRB5m)*m0JG3+0.5*(*jEwtH)*VNvlY(m0JG3)-b4jXr*qn3x8(m0JG3
)/6.0;*JRB5m=-hxixi;*jEwtH=0.0;return;}


void qPN_6::WlCk8(double*wBmQ7,double*TfLg_,double*JRB5m,double*jEwtH,const 
double&hxixi,const double&b4jXr){
double mn8nn=0.0
,BawAR=0.0;
BawAR=pbQOc((VNvlY((*jEwtH))+2.0*b4jXr*(-hxixi-(*JRB5m)))/2.0);
mn8nn=(BawAR-(*jEwtH))/b4jXr;*wBmQ7+=mn8nn;*TfLg_+=(*JRB5m)*mn8nn+0.5*(*jEwtH)*
VNvlY(mn8nn)+b4jXr*qn3x8(mn8nn)/6.0;*JRB5m+=(*jEwtH)*mn8nn+0.5*b4jXr*VNvlY(mn8nn
);*jEwtH=BawAR;
mn8nn=(*jEwtH)/b4jXr;*wBmQ7+=mn8nn;*TfLg_+=(*JRB5m)*mn8nn+0.5*(*jEwtH)*VNvlY(
mn8nn)-b4jXr*qn3x8(mn8nn)/6.0;*JRB5m=-hxixi;*jEwtH=0.0;return;}
