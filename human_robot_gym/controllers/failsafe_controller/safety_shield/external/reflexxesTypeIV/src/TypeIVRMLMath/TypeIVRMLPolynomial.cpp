
































#include <TypeIVRMLPolynomial.h>
#include <TypeIVRMLMath.h>
#include <TypeIVRMLDefinitions.h>


qPN_6::oa0l4::oa0l4(){ChgZj=0.0;Aw4RZ=0.0;ct8hz=0.0;CY0Ek=0.0;Rwvfx=0.0;k16wm=
(0xa40+6100-0x2214);}

qPN_6::oa0l4::~oa0l4(){}


void qPN_6::oa0l4::jGQqQ(const double&s4u5t,const double&spAQE,const double&
pPbQq,const double&cMMsQ,const double&RX81O){ChgZj=cMMsQ;Aw4RZ=pPbQq;ct8hz=spAQE
;CY0Ek=s4u5t;Rwvfx=RX81O;if(CY0Ek!=0.0){k16wm=(0x2164+1086-0x259f);return;}if(
ct8hz!=0.0){k16wm=(0x13b4+4636-0x25ce);return;}if(Aw4RZ!=0.0){k16wm=
(0x48b+3838-0x1388);return;}k16wm=(0x1534+2291-0x1e27);return;}

void qPN_6::oa0l4::hYaB2(double*s4u5t,double*spAQE,double*pPbQq,double*cMMsQ,
double*RX81O)const{*s4u5t=this->CY0Ek;*spAQE=this->ct8hz;*pPbQq=this->Aw4RZ;*
cMMsQ=this->ChgZj;*RX81O=this->Rwvfx;return;}


double qPN_6::oa0l4::zixaV(const double&h4z3k)const{return((k16wm==
(0x172d+959-0x1ae9))?(CY0Ek*(h4z3k-Rwvfx)*(h4z3k-Rwvfx)*(h4z3k-Rwvfx)+ct8hz*(
h4z3k-Rwvfx)*(h4z3k-Rwvfx)+Aw4RZ*(h4z3k-Rwvfx)+ChgZj):((k16wm==
(0x73+5682-0x16a3))?(ct8hz*(h4z3k-Rwvfx)*(h4z3k-Rwvfx)+Aw4RZ*(h4z3k-Rwvfx)+ChgZj
):((k16wm==(0xa3d+7201-0x265d))?(Aw4RZ*(h4z3k-Rwvfx)+ChgZj):(ChgZj))));}

void qPN_6::oa0l4::at3zX(unsigned int*pAVmT,double*Ct7O1,double*GxGT4,double*
FRbqD)const{

if((this->k16wm==(0x90d+3844-0x1811))||((this->k16wm==(0x4df+3508-0x1292))&&(
this->Aw4RZ==0.0))||((this->k16wm==(0x7d7+885-0xb4a))&&(this->ct8hz==0.0)&&(this
->Aw4RZ==0.0))||((this->k16wm==(0xcfc+841-0x1042))&&(this->CY0Ek==0.0)&&(this->
ct8hz==0.0)&&(this->Aw4RZ==0.0))){*Ct7O1=0.0;*GxGT4=0.0;*FRbqD=0.0;*pAVmT=
(0x85f+5496-0x1dd7);}

if((this->k16wm==(0xaa1+1625-0x10f9))||((this->k16wm==(0x1102+3183-0x1d6f))&&(
this->ct8hz==0.0))||((this->k16wm==(0x1f41+1996-0x270a))&&(this->CY0Ek==0.0)&&(
this->ct8hz==0.0))){*Ct7O1=-this->ChgZj/this->Aw4RZ+this->Rwvfx;*GxGT4=0.0;*
FRbqD=0.0;*pAVmT=(0x11a0+2003-0x1972);return;}

if((this->k16wm==(0x1903+1248-0x1de1))||((this->k16wm==(0x952+5527-0x1ee6))&&(
this->CY0Ek==0.0))){double K7qNw=this->ChgZj/this->ct8hz,rqjC7=this->Aw4RZ/this
->ct8hz,n29hV=0.25*VNvlY(rqjC7)-K7qNw;if(n29hV<iZLMI){
*Ct7O1=0.0;*GxGT4=0.0;*FRbqD=0.0;*pAVmT=(0x42+4356-0x1146);}else{
n29hV=qPN_6::pbQOc(n29hV);*Ct7O1=-0.5*rqjC7+n29hV+this->Rwvfx;*GxGT4=-0.5*rqjC7-
n29hV+this->Rwvfx;*FRbqD=0.0;*pAVmT=(0x7ff+1341-0xd3a);}return;}

if(this->k16wm==(0x19d+2468-0xb3e)){
*Ct7O1=0.0;*GxGT4=0.0;*FRbqD=0.0;*pAVmT=(0x1401+2965-0x1f96);return;}return;}
