

































#include <TypeIVRMLStep2IntermediateProfiles.h>
#include <TypeIVRMLPolynomial.h>
#include <TypeIVRMLMath.h>
#include <TypeIVRMLDefinitions.h>
#include <math.h>



void qPN_6::FA5vV(double*TfLg_,double*JRB5m,double*jEwtH,double*EgDTv,double*
EI730,bool*U7RiH){

*TfLg_=-(*TfLg_);*JRB5m=-(*JRB5m);*jEwtH=-(*jEwtH);*EgDTv=-(*EgDTv);*EI730=-(*
EI730);*U7RiH=!(*U7RiH);return;}


void qPN_6::fNPdo(double*Z4lSz,double*TfLg_,double*JRB5m,double*jEwtH,const 
double&DAwhk,const double&b4jXr,XkwFr*_2E29,const bool&U7RiH){
double mn8nn=0.0;

mn8nn=((*jEwtH)-DAwhk)/b4jXr;
if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(((b4jXr)/6.0),(0.5*(-(*jEwtH))),(-(*
JRB5m)),(-(*TfLg_)),(*Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(b4jXr*0.5),(
-(*jEwtH)),(-(*JRB5m)),(*Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,b4jXr,
(-(*jEwtH)),(*Z4lSz));}else{_2E29->fCxBi[_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),(0.5
*(*jEwtH)),(*JRB5m),(*TfLg_),(*Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,((-
b4jXr)*0.5),(*jEwtH),(*JRB5m),(*Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0
,(-b4jXr),(*jEwtH),(*Z4lSz));}_2E29->FfhZi[_2E29->djbCD]=(*Z4lSz)+mn8nn;_2E29->
djbCD++;
*Z4lSz+=(mn8nn);*TfLg_+=(*JRB5m)*mn8nn+0.5*(*jEwtH)*VNvlY(mn8nn)-b4jXr*qn3x8(
mn8nn)/6.0;*JRB5m+=(*jEwtH)*mn8nn-0.5*b4jXr*VNvlY(mn8nn);*jEwtH=DAwhk;return;}


void qPN_6::tmpuh(double*Z4lSz,double*TfLg_,double*JRB5m,double*jEwtH,const 
double&b4jXr,XkwFr*_2E29,const bool&U7RiH){
double mn8nn=0.0;

mn8nn=(*jEwtH)/b4jXr;
if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(((b4jXr)/6.0),(0.5*(-(*jEwtH))),(-(*
JRB5m)),(-(*TfLg_)),(*Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(b4jXr*0.5),(
-(*jEwtH)),(-(*JRB5m)),(*Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,b4jXr,
(-(*jEwtH)),(*Z4lSz));}else{_2E29->fCxBi[_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),(0.5
*(*jEwtH)),(*JRB5m),(*TfLg_),(*Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,((-
b4jXr)*0.5),(*jEwtH),(*JRB5m),(*Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0
,(-b4jXr),(*jEwtH),(*Z4lSz));}_2E29->FfhZi[_2E29->djbCD]=(*Z4lSz)+mn8nn;_2E29->
djbCD++;
*Z4lSz+=mn8nn;*TfLg_+=(*JRB5m)*mn8nn+0.5*(*jEwtH)*VNvlY(mn8nn)-b4jXr*qn3x8(mn8nn
)/6.0;*JRB5m+=(*jEwtH)*mn8nn-0.5*b4jXr*VNvlY(mn8nn);*jEwtH=0.0;return;}


void qPN_6::LNPAh(double*Z4lSz,double*TfLg_,double*JRB5m,double*jEwtH,const 
double&hxixi,const double&b4jXr,XkwFr*_2E29,const bool&U7RiH){
double mn8nn=0.0
,L38Np=0.0;

L38Np=pbQOc(VNvlY(*jEwtH)-2.0*b4jXr*(hxixi+(*JRB5m)));
mn8nn=(L38Np-(*jEwtH))/b4jXr;
if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),(-0.5*(*jEwtH)),(-(*
JRB5m)),(-(*TfLg_)),(*Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(-b4jXr*0.5),
(-(*jEwtH)),(-(*JRB5m)),(*Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,-
b4jXr,(-(*jEwtH)),(*Z4lSz));}else{_2E29->fCxBi[_2E29->djbCD].jGQqQ((b4jXr/6.0),(
0.5*(*jEwtH)),(*JRB5m),(*TfLg_),(*Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(
b4jXr*0.5),(*jEwtH),(*JRB5m),(*Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,
b4jXr,(*jEwtH),(*Z4lSz));}_2E29->FfhZi[_2E29->djbCD]=(*Z4lSz)+mn8nn;_2E29->djbCD
++;
*Z4lSz+=mn8nn;*TfLg_+=(*JRB5m)*mn8nn+0.5*(*jEwtH)*VNvlY(mn8nn)+b4jXr*qn3x8(mn8nn
)/6.0;*JRB5m=-hxixi;*jEwtH=L38Np;return;}


void qPN_6::nosmu(double*Z4lSz,double*TfLg_,double*JRB5m,double*jEwtH,const 
double&hxixi,const double&DAwhk,const double&b4jXr,XkwFr*_2E29,const bool&U7RiH)
{
double mn8nn=0.0;

mn8nn=(DAwhk-(*jEwtH))/b4jXr;
if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),(-0.5*(*jEwtH)),(-(*
JRB5m)),(-(*TfLg_)),(*Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(-b4jXr*0.5),
(-(*jEwtH)),(-(*JRB5m)),(*Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,-
b4jXr,(-(*jEwtH)),(*Z4lSz));}else{_2E29->fCxBi[_2E29->djbCD].jGQqQ((b4jXr/6.0),(
0.5*(*jEwtH)),(*JRB5m),(*TfLg_),(*Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(
b4jXr*0.5),(*jEwtH),(*JRB5m),(*Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,
b4jXr,(*jEwtH),(*Z4lSz));}_2E29->FfhZi[_2E29->djbCD]=(*Z4lSz)+mn8nn;_2E29->djbCD
++;
*Z4lSz+=mn8nn;*TfLg_+=(*JRB5m)*mn8nn+0.5*(*jEwtH)*VNvlY(mn8nn)+b4jXr*qn3x8(mn8nn
)/6.0;*JRB5m+=(*jEwtH)*mn8nn+0.5*b4jXr*VNvlY(mn8nn);*jEwtH=DAwhk;
mn8nn=(-hxixi-(*JRB5m))/DAwhk;
if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0,(-0.5*DAwhk),(-(*JRB5m)),(-(*
TfLg_)),(*Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,(-DAwhk),(-(*JRB5m)),
(*Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,(-DAwhk),(*Z4lSz));}else{
_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0,(0.5*DAwhk),(*JRB5m),(*TfLg_),(*Z4lSz));
_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,DAwhk,(*JRB5m),(*Z4lSz));_2E29->NYWGt[
_2E29->djbCD].jGQqQ(0.0,0.0,0.0,DAwhk,(*Z4lSz));}_2E29->FfhZi[_2E29->djbCD]=(*
Z4lSz)+mn8nn;_2E29->djbCD++;
*Z4lSz+=mn8nn;*TfLg_+=(*JRB5m)*mn8nn+0.5*DAwhk*VNvlY(mn8nn);*JRB5m=-hxixi;*jEwtH
=DAwhk;return;}


void qPN_6::aRhKo(double*Z4lSz,double*TfLg_,double*JRB5m,double*jEwtH,const 
double&hxixi,const double&b4jXr,XkwFr*_2E29,const bool&U7RiH){
double mn8nn=0.0
,BawAR=0.0
,OuKGy=0.0;

BawAR=pbQOc((VNvlY((*jEwtH))+2.0*b4jXr*(hxixi-(*JRB5m)))/2.0);
mn8nn=(BawAR-(*jEwtH))/b4jXr;
if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),(-0.5*(*jEwtH)),(-(*
JRB5m)),(-(*TfLg_)),(*Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(-b4jXr*0.5),
(-(*jEwtH)),(-(*JRB5m)),(*Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,-
b4jXr,(-(*jEwtH)),(*Z4lSz));}else{_2E29->fCxBi[_2E29->djbCD].jGQqQ((b4jXr/6.0),(
0.5*(*jEwtH)),(*JRB5m),(*TfLg_),(*Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(
b4jXr*0.5),(*jEwtH),(*JRB5m),(*Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,
b4jXr,(*jEwtH),(*Z4lSz));}_2E29->FfhZi[_2E29->djbCD]=(*Z4lSz)+mn8nn;_2E29->djbCD
++;
*Z4lSz+=mn8nn;*TfLg_+=(*JRB5m)*mn8nn+0.5*(*jEwtH)*VNvlY(mn8nn)+b4jXr*qn3x8(mn8nn
)/6.0;*JRB5m+=(*jEwtH)*mn8nn+0.5*b4jXr*VNvlY(mn8nn);*jEwtH=BawAR;
OuKGy=pbQOc(VNvlY((*jEwtH))+2.0*b4jXr*(hxixi+(*JRB5m)));
mn8nn=((*jEwtH)-OuKGy)/b4jXr;
if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ((b4jXr/6.0),(-0.5*(*jEwtH)),(-(*JRB5m
)),(-(*TfLg_)),(*Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(b4jXr*0.5),(-(*
jEwtH)),(-(*JRB5m)),(*Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,b4jXr,(-(
*jEwtH)),(*Z4lSz));}else{_2E29->fCxBi[_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),(0.5*(*
jEwtH)),(*JRB5m),(*TfLg_),(*Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,((-
b4jXr)*0.5),(*jEwtH),(*JRB5m),(*Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0
,-b4jXr,(*jEwtH),(*Z4lSz));}_2E29->FfhZi[_2E29->djbCD]=(*Z4lSz)+mn8nn;_2E29->
djbCD++;
*Z4lSz+=mn8nn;*TfLg_+=(*JRB5m)*mn8nn+0.5*(*jEwtH)*VNvlY(mn8nn)-b4jXr*qn3x8(mn8nn
)/6.0;*JRB5m=-hxixi;*jEwtH=OuKGy;return;}


void qPN_6::mJy4H(double*Z4lSz,double*TfLg_,double*JRB5m,double*jEwtH,const 
double&hxixi,const double&DAwhk,const double&b4jXr,XkwFr*_2E29,const bool&U7RiH)
{
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
if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),(-0.5*(*jEwtH)),(-(*
JRB5m)),(-(*TfLg_)),(*Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(-0.5*b4jXr),
(-(*jEwtH)),(-(*JRB5m)),(*Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,-
b4jXr,(-(*jEwtH)),(*Z4lSz));}else{_2E29->fCxBi[_2E29->djbCD].jGQqQ((b4jXr/6.0),(
0.5*(*jEwtH)),(*JRB5m),(*TfLg_),(*Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(
0.5*b4jXr),(*jEwtH),(*JRB5m),(*Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,
b4jXr,(*jEwtH),(*Z4lSz));}_2E29->FfhZi[_2E29->djbCD]=(*Z4lSz)+cjcAG;_2E29->djbCD
++;
*Z4lSz+=cjcAG;*TfLg_+=(*JRB5m)*cjcAG+0.5*(*jEwtH)*VNvlY(cjcAG)+b4jXr*qn3x8(cjcAG
)/6.0;*JRB5m+=sddwV;*jEwtH=DAwhk;
if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0,(-0.5*DAwhk),(-(*JRB5m)),(-(*
TfLg_)),(*Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,-DAwhk,(-(*JRB5m)),(*
Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,-DAwhk,(*Z4lSz));}else{
_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0,(0.5*DAwhk),(*JRB5m),(*TfLg_),(*Z4lSz));
_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,DAwhk,(*JRB5m),(*Z4lSz));_2E29->NYWGt[
_2E29->djbCD].jGQqQ(0.0,0.0,0.0,DAwhk,(*Z4lSz));}_2E29->FfhZi[_2E29->djbCD]=(*
Z4lSz)+StX1T;_2E29->djbCD++;
*Z4lSz+=StX1T;*TfLg_+=(*JRB5m)*StX1T+0.5*(*jEwtH)*VNvlY(StX1T);*JRB5m+=rT061;*
jEwtH=DAwhk;
BawAR=pbQOc(VNvlY((*jEwtH))+2.0*b4jXr*(hxixi+(*JRB5m)));m0JG3=((*jEwtH)-BawAR)/
b4jXr;
if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ((b4jXr/6.0),(-0.5*(*jEwtH)),(-(*JRB5m
)),(-(*TfLg_)),(*Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(b4jXr*0.5),(-(*
jEwtH)),(-(*JRB5m)),(*Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,b4jXr,(-(
*jEwtH)),(*Z4lSz));}else{_2E29->fCxBi[_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),(0.5*(*
jEwtH)),(*JRB5m),(*TfLg_),(*Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,((-
b4jXr)*0.5),(*jEwtH),(*JRB5m),(*Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0
,-b4jXr,(*jEwtH),(*Z4lSz));}_2E29->FfhZi[_2E29->djbCD]=(*Z4lSz)+m0JG3;_2E29->
djbCD++;
*Z4lSz+=m0JG3;*TfLg_+=(*JRB5m)*m0JG3+0.5*(*jEwtH)*VNvlY(m0JG3)-b4jXr*qn3x8(m0JG3
)/6.0;*JRB5m=-hxixi;*jEwtH=BawAR;return;}


void qPN_6::kLMlb(double*Z4lSz,double*TfLg_,double*JRB5m,double*jEwtH,const 
double&hxixi,const double&DAwhk,const double&b4jXr,XkwFr*_2E29,const bool&U7RiH)
{
double cjcAG=0.0
,StX1T=0.0
,m0JG3=0.0
,sddwV=0.0
,rT061=0.0
,y71cs=0.0;

cjcAG=(DAwhk-(*jEwtH))/b4jXr;


sddwV=(*jEwtH)*cjcAG+0.5*b4jXr*VNvlY(cjcAG);y71cs=0.5*VNvlY(DAwhk)/b4jXr;
rT061=-hxixi-(*JRB5m)-sddwV-y71cs;StX1T=rT061/DAwhk;
if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),(-0.5*(*jEwtH)),(-(*
JRB5m)),(-(*TfLg_)),(*Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(-0.5*b4jXr),
(-(*jEwtH)),(-(*JRB5m)),(*Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,-
b4jXr,(-(*jEwtH)),(*Z4lSz));}else{_2E29->fCxBi[_2E29->djbCD].jGQqQ((b4jXr/6.0),(
0.5*(*jEwtH)),(*JRB5m),(*TfLg_),(*Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(
0.5*b4jXr),(*jEwtH),(*JRB5m),(*Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,
b4jXr,(*jEwtH),(*Z4lSz));}_2E29->FfhZi[_2E29->djbCD]=(*Z4lSz)+cjcAG;_2E29->djbCD
++;
*Z4lSz+=cjcAG;*TfLg_+=(*JRB5m)*cjcAG+0.5*(*jEwtH)*VNvlY(cjcAG)+b4jXr*qn3x8(cjcAG
)/6.0;*JRB5m+=sddwV;*jEwtH=DAwhk;
if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0,(-0.5*DAwhk),(-(*JRB5m)),(-(*
TfLg_)),(*Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,-DAwhk,(-(*JRB5m)),(*
Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,-DAwhk,(*Z4lSz));}else{
_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0,(0.5*DAwhk),(*JRB5m),(*TfLg_),(*Z4lSz));
_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,DAwhk,(*JRB5m),(*Z4lSz));_2E29->NYWGt[
_2E29->djbCD].jGQqQ(0.0,0.0,0.0,DAwhk,(*Z4lSz));}_2E29->FfhZi[_2E29->djbCD]=(*
Z4lSz)+StX1T;_2E29->djbCD++;
*Z4lSz+=StX1T;*TfLg_+=(*JRB5m)*StX1T+0.5*(*jEwtH)*VNvlY(StX1T);*JRB5m+=rT061;*
jEwtH=DAwhk;m0JG3=(*jEwtH)/b4jXr;
if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ((b4jXr/6.0),(-0.5*(*jEwtH)),(-(*JRB5m
)),(-(*TfLg_)),(*Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(b4jXr*0.5),(-(*
jEwtH)),(-(*JRB5m)),(*Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,b4jXr,(-(
*jEwtH)),(*Z4lSz));}else{_2E29->fCxBi[_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),(0.5*(*
jEwtH)),(*JRB5m),(*TfLg_),(*Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,((-
b4jXr)*0.5),(*jEwtH),(*JRB5m),(*Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0
,-b4jXr,(*jEwtH),(*Z4lSz));}_2E29->FfhZi[_2E29->djbCD]=(*Z4lSz)+m0JG3;_2E29->
djbCD++;
*Z4lSz+=m0JG3;*TfLg_+=(*JRB5m)*m0JG3+0.5*(*jEwtH)*VNvlY(m0JG3)-b4jXr*qn3x8(m0JG3
)/6.0;*JRB5m=-hxixi;*jEwtH=0.0;return;}


void qPN_6::kpKw7(double*Z4lSz,double*TfLg_,double*JRB5m,double*jEwtH,const 
double&hxixi,const double&b4jXr,XkwFr*_2E29,const bool&U7RiH){
double mn8nn=0.0
,oZg86=0.0;
oZg86=0.70710678118654752440084436210485*pbQOc(VNvlY(b4jXr)*(VNvlY(*jEwtH)+2.0*
b4jXr*(-hxixi-(*JRB5m))))/b4jXr;
mn8nn=(oZg86-(*jEwtH))/b4jXr;
if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),(-0.5*(*jEwtH)),(-(*
JRB5m)),(-(*TfLg_)),(*Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(-b4jXr*0.5),
(-(*jEwtH)),(-(*JRB5m)),(*Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,-
b4jXr,(-(*jEwtH)),(*Z4lSz));}else{_2E29->fCxBi[_2E29->djbCD].jGQqQ((b4jXr/6.0),(
0.5*(*jEwtH)),(*JRB5m),(*TfLg_),(*Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(
b4jXr*0.5),(*jEwtH),(*JRB5m),(*Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,
b4jXr,(*jEwtH),(*Z4lSz));}_2E29->FfhZi[_2E29->djbCD]=(*Z4lSz)+mn8nn;_2E29->djbCD
++;
*Z4lSz+=mn8nn;*TfLg_+=(*JRB5m)*mn8nn+0.5*(*jEwtH)*VNvlY(mn8nn)+b4jXr*qn3x8(mn8nn
)/6.0;*JRB5m+=(*jEwtH)*mn8nn+0.5*b4jXr*VNvlY(mn8nn);*jEwtH=oZg86;
mn8nn=(*jEwtH)/b4jXr;
if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ((b4jXr/6.0),(-0.5*(*jEwtH)),(-(*JRB5m
)),(-(*TfLg_)),(*Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(b4jXr*0.5),(-(*
jEwtH)),(-(*JRB5m)),(*Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,b4jXr,(-(
*jEwtH)),(*Z4lSz));}else{_2E29->fCxBi[_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),(0.5*(*
jEwtH)),(*JRB5m),(*TfLg_),(*Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,((-
b4jXr)*0.5),(*jEwtH),(*JRB5m),(*Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0
,-b4jXr,(*jEwtH),(*Z4lSz));}_2E29->FfhZi[_2E29->djbCD]=(*Z4lSz)+mn8nn;_2E29->
djbCD++;
*Z4lSz+=mn8nn;*TfLg_+=(*JRB5m)*mn8nn+0.5*(*jEwtH)*VNvlY(mn8nn)-b4jXr*qn3x8(mn8nn
)/6.0;*JRB5m=-hxixi;*jEwtH=0.0;return;}
