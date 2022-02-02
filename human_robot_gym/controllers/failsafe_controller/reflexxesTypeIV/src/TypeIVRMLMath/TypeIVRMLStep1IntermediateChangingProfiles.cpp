

































#include <TypeIVRMLStep1IntermediateChangingProfiles.h>
#include <TypeIVRMLMath.h>
#include <TypeIVRMLDefinitions.h>
#include <math.h>


void qPN_6::hUOTj(double*TfLg_,double*JRB5m,double*jEwtH,double*EgDTv,double*
EI730){

*TfLg_=-(*TfLg_);*JRB5m=-(*JRB5m);*jEwtH=-(*jEwtH);*EgDTv=-(*EgDTv);*EI730=-(*
EI730);return;}


void qPN_6::GWLrL(double*TfLg_,double*JRB5m,double*jEwtH,const double&DAwhk,
const double&b4jXr){
double mn8nn=0.0;

mn8nn=((*jEwtH)-DAwhk)/b4jXr;*TfLg_+=(*JRB5m)*mn8nn+0.5*(*jEwtH)*VNvlY(mn8nn)-
b4jXr*qn3x8(mn8nn)/6.0;*JRB5m+=(*jEwtH)*mn8nn-0.5*b4jXr*VNvlY(mn8nn);*jEwtH=
DAwhk;return;}


void qPN_6::_lF6E(double*TfLg_,double*JRB5m,double*jEwtH,const double&b4jXr){
double mn8nn=0.0;

mn8nn=(*jEwtH)/b4jXr;*TfLg_+=(*JRB5m)*mn8nn+0.5*(*jEwtH)*VNvlY(mn8nn)-b4jXr*
qn3x8(mn8nn)/6.0;*JRB5m+=(*jEwtH)*mn8nn-0.5*b4jXr*VNvlY(mn8nn);*jEwtH=0.0;return
;}


void qPN_6::l5BRQ(double*TfLg_,double*JRB5m,double*jEwtH,const double&b4jXr){
double mn8nn=0.0;

mn8nn=(-(*jEwtH))/b4jXr;*TfLg_+=(*JRB5m)*mn8nn+0.5*(*jEwtH)*VNvlY(mn8nn)+b4jXr*
qn3x8(mn8nn)/6.0;*JRB5m+=(*jEwtH)*mn8nn+0.5*b4jXr*VNvlY(mn8nn);*jEwtH=0.0;return
;}


void qPN_6::pLzTb(double*TfLg_,double*JRB5m,double*jEwtH,const double&hxixi,
const double&b4jXr){
double mn8nn=0.0
,L38Np=0.0;
L38Np=pbQOc(VNvlY(*jEwtH)-2.0*b4jXr*(hxixi+(*JRB5m)));
mn8nn=(L38Np-(*jEwtH))/b4jXr;*TfLg_+=(*JRB5m)*mn8nn+0.5*(*jEwtH)*VNvlY(mn8nn)+
b4jXr*qn3x8(mn8nn)/6.0;*JRB5m=-hxixi;*jEwtH=L38Np;return;}


void qPN_6::Cpnf2(double*TfLg_,double*JRB5m,double*jEwtH,const double&hxixi,
const double&DAwhk,const double&b4jXr){

double mn8nn=0.0;
mn8nn=(DAwhk-(*jEwtH))/b4jXr;*TfLg_+=(*JRB5m)*mn8nn+0.5*(*jEwtH)*VNvlY(mn8nn)+
b4jXr*qn3x8(mn8nn)/6.0;*JRB5m+=(*jEwtH)*mn8nn+0.5*b4jXr*VNvlY(mn8nn);*jEwtH=
DAwhk;
mn8nn=(-hxixi-(*JRB5m))/DAwhk;*TfLg_+=(*JRB5m)*mn8nn+0.5*DAwhk*VNvlY(mn8nn);*
JRB5m=-hxixi;*jEwtH=DAwhk;return;}


void qPN_6::hHRnV(double*TfLg_,double*JRB5m,double*jEwtH,const double&hxixi,
const double&b4jXr){
double mn8nn=0.0
,BawAR=0.0
,OuKGy=0.0;

BawAR=pbQOc((VNvlY((*jEwtH))+2.0*b4jXr*(hxixi-(*JRB5m)))/2.0);
mn8nn=(BawAR-(*jEwtH))/b4jXr;*TfLg_+=(*JRB5m)*mn8nn+0.5*(*jEwtH)*VNvlY(mn8nn)+
b4jXr*qn3x8(mn8nn)/6.0;*JRB5m+=(*jEwtH)*mn8nn+0.5*b4jXr*VNvlY(mn8nn);*jEwtH=
BawAR;
OuKGy=pbQOc(VNvlY((*jEwtH))+2.0*b4jXr*(hxixi+(*JRB5m)));
mn8nn=((*jEwtH)-OuKGy)/b4jXr;*TfLg_+=(*JRB5m)*mn8nn+0.5*(*jEwtH)*VNvlY(mn8nn)-
b4jXr*qn3x8(mn8nn)/6.0;*JRB5m=-hxixi;*jEwtH=OuKGy;return;}


void qPN_6::UuSaH(double*TfLg_,double*JRB5m,double*jEwtH,const double&hxixi,
const double&DAwhk,const double&b4jXr){
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
*TfLg_+=(*JRB5m)*cjcAG+0.5*(*jEwtH)*VNvlY(cjcAG)+b4jXr*qn3x8(cjcAG)/6.0;*JRB5m+=
sddwV;*jEwtH=DAwhk;
*TfLg_+=(*JRB5m)*StX1T+0.5*(*jEwtH)*VNvlY(StX1T);*JRB5m+=rT061;*jEwtH=DAwhk;
BawAR=pbQOc(VNvlY((*jEwtH))+2.0*b4jXr*(hxixi+(*JRB5m)));m0JG3=((*jEwtH)-BawAR)/
b4jXr;
*TfLg_+=(*JRB5m)*m0JG3+0.5*(*jEwtH)*VNvlY(m0JG3)-b4jXr*qn3x8(m0JG3)/6.0;*JRB5m=-
hxixi;*jEwtH=BawAR;return;}


void qPN_6::_04Qv(double*TfLg_,double*JRB5m,double*jEwtH,const double&hxixi,
const double&DAwhk,const double&b4jXr){

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
*TfLg_+=(*JRB5m)*cjcAG+0.5*(*jEwtH)*VNvlY(cjcAG)+b4jXr*qn3x8(cjcAG)/6.0;*JRB5m+=
sddwV;*jEwtH=DAwhk;
*TfLg_+=(*JRB5m)*StX1T+0.5*(*jEwtH)*VNvlY(StX1T);*JRB5m+=rT061;*jEwtH=DAwhk;
*TfLg_+=(*JRB5m)*m0JG3+0.5*(*jEwtH)*VNvlY(m0JG3)-b4jXr*qn3x8(m0JG3)/6.0;*JRB5m=-
hxixi;*jEwtH=0.0;return;}


void qPN_6::XYZkn(double*TfLg_,double*JRB5m,double*jEwtH,const double&hxixi,
const double&b4jXr){
double mn8nn=0.0
,BawAR=0.0;
BawAR=pbQOc((VNvlY((*jEwtH))+2.0*b4jXr*(-hxixi-(*JRB5m)))/2.0);
mn8nn=(BawAR-(*jEwtH))/b4jXr;*TfLg_+=(*JRB5m)*mn8nn+0.5*(*jEwtH)*VNvlY(mn8nn)+
b4jXr*qn3x8(mn8nn)/6.0;*JRB5m+=(*jEwtH)*mn8nn+0.5*b4jXr*VNvlY(mn8nn);*jEwtH=
BawAR;
mn8nn=(*jEwtH)/b4jXr;*TfLg_+=(*JRB5m)*mn8nn+0.5*(*jEwtH)*VNvlY(mn8nn)-b4jXr*
qn3x8(mn8nn)/6.0;*JRB5m=-hxixi;*jEwtH=0.0;return;}
