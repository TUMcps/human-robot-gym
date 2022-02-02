































#include <TypeIVRMLStep1Profiles.h>
#include <TypeIVRMLStep2WithoutSynchronization.h>
#include <TypeIVRMLDecisionTree2.h>
#include <TypeIVRMLMath.h>
#include <TypeIVRMLDefinitions.h>
#include <TypeIVRMLStep2IntermediateProfiles.h>


void qPN_6::_oHSd(const double&iPzPj,const double&AQzqu,const double&k6sf4,const
 double&b4jXr,const double&DAwhk,const double&hxixi,const double&ldeCA,const 
double&HcINC,const double&w8MXY,const qPN_6::FOwyh&wrr5h,const int&bocnd,XkwFr*
_2E29){bool U7RiH=false;qPN_6::FOwyh OifK3=wrr5h;double Z4lSz=0.0,TfLg_=iPzPj,
JRB5m=AQzqu,jEwtH=k6sf4,EgDTv=ldeCA,EI730=HcINC,Kaclx=0.0,EvE6x=0.0,FvoS1=0.0,
LOUhW=0.0,Qogm3=0.0,CCBY_=0.0,X6X_j=0.0,xek11=0.0,ZaFFa=0.0,_HBf3=0.0,T5yjB=0.0,
OLj8b=0.0,CnAKY=0.0,oxEkR=0.0,Ixvmb=0.0,zlPa_=0.0,g7nwu=0.0,mpXRu=0.0,xHUUb=0.0,
_qvll=0.0,deZtJ=0.0,uREfL=0.0,wmBRB=0.0,RIZdZ=0.0;
eecN8(&Z4lSz,&TfLg_,&JRB5m,&jEwtH,b4jXr,DAwhk,hxixi,bocnd,&EgDTv,&EI730,_2E29,&
U7RiH);
if((OifK3==qPN_6::Ehd4a)||(OifK3==qPN_6::yu32u)||(OifK3==qPN_6::TFVwj)||(OifK3==
qPN_6::SG192)||(OifK3==qPN_6::IVmDp)||(OifK3==qPN_6::KELeG)||(OifK3==qPN_6::
VieEo)||(OifK3==qPN_6::LVHAx)){tmpuh(&Z4lSz,&TfLg_,&JRB5m,&jEwtH,b4jXr,&(*_2E29)
,U7RiH);

if(JRB5m>hxixi){JRB5m=hxixi;
}
FA5vV(&TfLg_,&JRB5m,&jEwtH,&EgDTv,&EI730,&U7RiH);
switch(OifK3){case qPN_6::Ehd4a:OifK3=qPN_6::YvMlr;break;case qPN_6::yu32u:OifK3
=qPN_6::JSPnP;break;case qPN_6::TFVwj:OifK3=qPN_6::ihEds;break;case qPN_6::SG192
:OifK3=qPN_6::aBNZo;break;case qPN_6::IVmDp:OifK3=qPN_6::Gy8r_;break;case qPN_6
::KELeG:OifK3=qPN_6::CWXWH;break;case qPN_6::VieEo:OifK3=qPN_6::Tuwfw;break;case
 qPN_6::LVHAx:OifK3=qPN_6::cvsfI;break;default:break;}}

switch(OifK3){case qPN_6::YvMlr:Kaclx=(DAwhk-jEwtH)/b4jXr;if(Kaclx<0.0){Kaclx=
0.0;}FvoS1=DAwhk/b4jXr;xek11=jEwtH*Kaclx+0.5*b4jXr*VNvlY(Kaclx);_HBf3=0.5*VNvlY(
DAwhk)/b4jXr;ZaFFa=hxixi-JRB5m-xek11-_HBf3;if(ZaFFa<0.0){ZaFFa=0.0;}EvE6x=ZaFFa/
DAwhk;Ixvmb=JRB5m*Kaclx+0.5*jEwtH*VNvlY(Kaclx)+b4jXr*qn3x8(Kaclx)/6.0;zlPa_=(
JRB5m+xek11)*EvE6x+0.5*DAwhk*VNvlY(EvE6x);g7nwu=(JRB5m+xek11+ZaFFa)*FvoS1+0.5*
DAwhk*VNvlY(FvoS1)-b4jXr*qn3x8(FvoS1)/6.0;T5yjB=0.0;OLj8b=-_HBf3;oxEkR=-_HBf3;
Qogm3=FvoS1;X6X_j=FvoS1;uREfL=JRB5m+xek11+ZaFFa+_HBf3;CnAKY=EI730-JRB5m-xek11-
ZaFFa-_HBf3-OLj8b-oxEkR;if(CnAKY>0.0){CnAKY=0.0;}CCBY_=-CnAKY/DAwhk;xHUUb=uREfL*
Qogm3-b4jXr*qn3x8(Qogm3)/6.0;_qvll=(uREfL+OLj8b)*CCBY_-0.5*DAwhk*VNvlY(CCBY_);
deZtJ=(uREfL+OLj8b+CnAKY)*X6X_j-0.5*DAwhk*VNvlY(X6X_j)+b4jXr*qn3x8(X6X_j)/6.0;
mpXRu=EgDTv-TfLg_-Ixvmb-zlPa_-g7nwu-xHUUb-_qvll-deZtJ;if(uREfL==0.0){uREfL=T68ug
;}LOUhW=mpXRu/uREfL;if(LOUhW<0.0){LOUhW=0.0;}



if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),(0.5*(-jEwtH)),(-JRB5m
),(-TfLg_),(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,((-b4jXr)*0.5),(-jEwtH)
,(-JRB5m),(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,-b4jXr,(-jEwtH),(
Z4lSz));}else{_2E29->fCxBi[_2E29->djbCD].jGQqQ((b4jXr/6.0),(0.5*jEwtH),JRB5m,
TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(b4jXr*0.5),jEwtH,JRB5m,(
Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,b4jXr,jEwtH,(Z4lSz));}_2E29->
FfhZi[_2E29->djbCD]=(Z4lSz)+Kaclx;_2E29->djbCD++;
Z4lSz+=(Kaclx);TfLg_+=Ixvmb;JRB5m+=xek11;jEwtH=DAwhk;

if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0,(0.5*(-DAwhk)),(-JRB5m),(-TfLg_),
(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,-DAwhk,(-JRB5m),(Z4lSz));_2E29
->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,-DAwhk,(Z4lSz));}else{_2E29->fCxBi[_2E29
->djbCD].jGQqQ(0.0,(0.5*DAwhk),JRB5m,TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].
jGQqQ(0.0,0.0,DAwhk,JRB5m,(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,
DAwhk,(Z4lSz));}_2E29->FfhZi[_2E29->djbCD]=(Z4lSz)+EvE6x;_2E29->djbCD++;
Z4lSz+=(EvE6x);TfLg_+=zlPa_;JRB5m+=ZaFFa;jEwtH=DAwhk;

if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ((b4jXr/6.0),(0.5*(-DAwhk)),(-JRB5m),(
-TfLg_),(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*b4jXr),-DAwhk,(-JRB5m
),(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,b4jXr,-DAwhk,(Z4lSz));}else{
_2E29->fCxBi[_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),(0.5*DAwhk),JRB5m,TfLg_,(Z4lSz))
;_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*(-b4jXr)),DAwhk,JRB5m,(Z4lSz));_2E29
->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,-b4jXr,DAwhk,(Z4lSz));}_2E29->FfhZi[_2E29->
djbCD]=(Z4lSz)+FvoS1;_2E29->djbCD++;
Z4lSz+=(FvoS1);TfLg_+=g7nwu;JRB5m+=_HBf3;jEwtH=0.0;

if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0,0.0,(-JRB5m),(-TfLg_),(Z4lSz));
_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,(-JRB5m),(Z4lSz));_2E29->NYWGt[
_2E29->djbCD].jGQqQ(0.0,0.0,0.0,0.0,(Z4lSz));}else{_2E29->fCxBi[_2E29->djbCD].
jGQqQ(0.0,0.0,JRB5m,TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,
JRB5m,(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,0.0,(Z4lSz));}_2E29
->FfhZi[_2E29->djbCD]=(Z4lSz)+LOUhW;_2E29->djbCD++;
Z4lSz+=(LOUhW);TfLg_+=mpXRu;JRB5m+=T5yjB;jEwtH=0.0;

if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(((b4jXr)/6.0),0.0,(-JRB5m),(-TfLg_),(
Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*b4jXr),0.0,(-JRB5m),(Z4lSz));
_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,b4jXr,0.0,(Z4lSz));}else{_2E29->fCxBi[
_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),0.0,JRB5m,TfLg_,(Z4lSz));_2E29->diBqY[_2E29->
djbCD].jGQqQ(0.0,(0.5*(-b4jXr)),0.0,JRB5m,(Z4lSz));_2E29->NYWGt[_2E29->djbCD].
jGQqQ(0.0,0.0,-b4jXr,0.0,(Z4lSz));}_2E29->FfhZi[_2E29->djbCD]=(Z4lSz)+Qogm3;
_2E29->djbCD++;
Z4lSz+=(Qogm3);TfLg_+=xHUUb;JRB5m+=OLj8b;jEwtH=-DAwhk;

if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0,(0.5*DAwhk),-JRB5m,-TfLg_,(Z4lSz)
);_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,DAwhk,-JRB5m,(Z4lSz));_2E29->NYWGt[
_2E29->djbCD].jGQqQ(0.0,0.0,0.0,DAwhk,(Z4lSz));}else{_2E29->fCxBi[_2E29->djbCD].
jGQqQ(0.0,(0.5*(-DAwhk)),JRB5m,TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(
0.0,0.0,-DAwhk,JRB5m,(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,-
DAwhk,(Z4lSz));}_2E29->FfhZi[_2E29->djbCD]=(Z4lSz)+CCBY_;_2E29->djbCD++;
Z4lSz+=(CCBY_);TfLg_+=_qvll;JRB5m+=CnAKY;jEwtH=-DAwhk;

if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),(0.5*DAwhk),-JRB5m,-
TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*(-b4jXr)),DAwhk,-JRB5m,
(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,(-b4jXr),DAwhk,(Z4lSz));}else{
_2E29->fCxBi[_2E29->djbCD].jGQqQ((b4jXr/6.0),(0.5*(-DAwhk)),JRB5m,TfLg_,(Z4lSz))
;_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*b4jXr),-DAwhk,JRB5m,(Z4lSz));_2E29->
NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,b4jXr,-DAwhk,(Z4lSz));}_2E29->FfhZi[_2E29->
djbCD]=(Z4lSz)+X6X_j;_2E29->djbCD++;
Z4lSz+=(X6X_j);TfLg_+=deZtJ;JRB5m+=oxEkR;jEwtH=0.0;
if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0,0.0,-JRB5m,-TfLg_,(Z4lSz));_2E29
->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,-JRB5m,(Z4lSz));_2E29->NYWGt[_2E29->
djbCD].jGQqQ(0.0,0.0,0.0,0.0,(Z4lSz));}else{_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0
,0.0,JRB5m,TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,JRB5m,(
Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,0.0,(Z4lSz));}_2E29->FfhZi[
_2E29->djbCD]=(Z4lSz)+tlPiC;_2E29->djbCD++;break;




case qPN_6::JSPnP:Kaclx=(DAwhk-jEwtH)/b4jXr;if(Kaclx<0.0){Kaclx=0.0;}FvoS1=DAwhk
/b4jXr;xek11=jEwtH*Kaclx+0.5*b4jXr*VNvlY(Kaclx);_HBf3=0.5*VNvlY(DAwhk)/b4jXr;
ZaFFa=hxixi-JRB5m-xek11-_HBf3;if(ZaFFa<0.0){ZaFFa=0.0;}EvE6x=ZaFFa/DAwhk;Ixvmb=
JRB5m*Kaclx+0.5*jEwtH*VNvlY(Kaclx)+b4jXr*qn3x8(Kaclx)/6.0;zlPa_=(JRB5m+xek11)*
EvE6x+0.5*DAwhk*VNvlY(EvE6x);g7nwu+=(JRB5m+xek11+ZaFFa)*FvoS1+0.5*DAwhk*VNvlY(
FvoS1)-b4jXr*qn3x8(FvoS1)/6.0;T5yjB=0.0;uREfL=JRB5m+xek11+ZaFFa+_HBf3;RIZdZ=-
pbQOc(-b4jXr*(EI730-uREfL));Qogm3=-RIZdZ/b4jXr;CCBY_=Qogm3;OLj8b=0.5*Qogm3*RIZdZ
;CnAKY=OLj8b;xHUUb=uREfL*Qogm3-b4jXr*qn3x8(Qogm3)/6.0;_qvll=(uREfL+OLj8b)*CCBY_+
0.5*RIZdZ*VNvlY(CCBY_)+b4jXr*qn3x8(CCBY_)/6.0;mpXRu=EgDTv-TfLg_-Ixvmb-zlPa_-
g7nwu-xHUUb-_qvll;if(uREfL==0.0){uREfL=T68ug;}LOUhW=mpXRu/uREfL;if(LOUhW<0.0){
LOUhW=0.0;}



if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),(0.5*(-jEwtH)),(-JRB5m
),(-TfLg_),(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,((-b4jXr)*0.5),(-jEwtH)
,(-JRB5m),(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,-b4jXr,(-jEwtH),(
Z4lSz));}else{_2E29->fCxBi[_2E29->djbCD].jGQqQ((b4jXr/6.0),(0.5*jEwtH),JRB5m,
TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(b4jXr*0.5),jEwtH,JRB5m,(
Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,b4jXr,jEwtH,(Z4lSz));}_2E29->
FfhZi[_2E29->djbCD]=(Z4lSz)+Kaclx;_2E29->djbCD++;
Z4lSz+=(Kaclx);TfLg_+=Ixvmb;JRB5m+=xek11;jEwtH=DAwhk;

if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0,(0.5*(-DAwhk)),(-JRB5m),(-TfLg_),
(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,-DAwhk,(-JRB5m),(Z4lSz));_2E29
->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,-DAwhk,(Z4lSz));}else{_2E29->fCxBi[_2E29
->djbCD].jGQqQ(0.0,(0.5*DAwhk),JRB5m,TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].
jGQqQ(0.0,0.0,DAwhk,JRB5m,(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,
DAwhk,(Z4lSz));}_2E29->FfhZi[_2E29->djbCD]=(Z4lSz)+EvE6x;_2E29->djbCD++;
Z4lSz+=(EvE6x);TfLg_+=zlPa_;JRB5m+=ZaFFa;jEwtH=DAwhk;

if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ((b4jXr/6.0),(0.5*(-DAwhk)),(-JRB5m),(
-TfLg_),(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*b4jXr),-DAwhk,(-JRB5m
),(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,b4jXr,-DAwhk,(Z4lSz));}else{
_2E29->fCxBi[_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),(0.5*DAwhk),JRB5m,TfLg_,(Z4lSz))
;_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*(-b4jXr)),DAwhk,JRB5m,(Z4lSz));_2E29
->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,-b4jXr,DAwhk,(Z4lSz));}_2E29->FfhZi[_2E29->
djbCD]=(Z4lSz)+FvoS1;_2E29->djbCD++;
Z4lSz+=(FvoS1);TfLg_+=g7nwu;JRB5m+=_HBf3;jEwtH=0.0;

if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0,0.0,(-JRB5m),(-TfLg_),(Z4lSz));
_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,(-JRB5m),(Z4lSz));_2E29->NYWGt[
_2E29->djbCD].jGQqQ(0.0,0.0,0.0,0.0,(Z4lSz));}else{_2E29->fCxBi[_2E29->djbCD].
jGQqQ(0.0,0.0,JRB5m,TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,
JRB5m,(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,0.0,(Z4lSz));}_2E29
->FfhZi[_2E29->djbCD]=(Z4lSz)+LOUhW;_2E29->djbCD++;
Z4lSz+=(LOUhW);TfLg_+=mpXRu;JRB5m+=T5yjB;jEwtH=0.0;

if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(((b4jXr)/6.0),0.0,(-JRB5m),(-TfLg_),(
Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*b4jXr),0.0,(-JRB5m),(Z4lSz));
_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,b4jXr,0.0,(Z4lSz));}else{_2E29->fCxBi[
_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),0.0,JRB5m,TfLg_,(Z4lSz));_2E29->diBqY[_2E29->
djbCD].jGQqQ(0.0,(0.5*(-b4jXr)),0.0,JRB5m,(Z4lSz));_2E29->NYWGt[_2E29->djbCD].
jGQqQ(0.0,0.0,-b4jXr,0.0,(Z4lSz));}_2E29->FfhZi[_2E29->djbCD]=(Z4lSz)+Qogm3;
_2E29->djbCD++;
Z4lSz+=(Qogm3);TfLg_+=xHUUb;JRB5m+=OLj8b;jEwtH=RIZdZ;

if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),(0.5*(-RIZdZ)),-JRB5m,
-TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*(-b4jXr)),-RIZdZ,-
JRB5m,(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,-b4jXr,-RIZdZ,(Z4lSz));}
else{_2E29->fCxBi[_2E29->djbCD].jGQqQ((b4jXr/6.0),(0.5*RIZdZ),JRB5m,TfLg_,(Z4lSz
));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*b4jXr),RIZdZ,JRB5m,(Z4lSz));_2E29->
NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,b4jXr,RIZdZ,(Z4lSz));}_2E29->FfhZi[_2E29->
djbCD]=(Z4lSz)+CCBY_;_2E29->djbCD++;
Z4lSz+=(CCBY_);TfLg_+=_qvll;JRB5m+=CnAKY;jEwtH=0.0;
if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0,0.0,-JRB5m,-TfLg_,(Z4lSz));_2E29
->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,-JRB5m,(Z4lSz));_2E29->NYWGt[_2E29->
djbCD].jGQqQ(0.0,0.0,0.0,0.0,(Z4lSz));}else{_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0
,0.0,JRB5m,TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,JRB5m,(
Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,0.0,(Z4lSz));}_2E29->FfhZi[
_2E29->djbCD]=(Z4lSz)+tlPiC;_2E29->djbCD++;break;




case qPN_6::ihEds:wmBRB=pbQOc((VNvlY(jEwtH)+2.0*b4jXr*(hxixi-JRB5m))/2.0);Kaclx=
(wmBRB-jEwtH)/b4jXr;if(Kaclx<0.0){Kaclx=0.0;}EvE6x=wmBRB/b4jXr;xek11=jEwtH*Kaclx
+0.5*Kaclx*(wmBRB-jEwtH);if(xek11<0.0){xek11=0.0;}ZaFFa=0.5*wmBRB*EvE6x;Ixvmb=
JRB5m*Kaclx+0.5*jEwtH*VNvlY(Kaclx)+b4jXr*qn3x8(Kaclx)/6.0;zlPa_=(JRB5m+xek11)*
EvE6x+0.5*wmBRB*VNvlY(EvE6x)-b4jXr*qn3x8(EvE6x)/6.0;uREfL=JRB5m+xek11+ZaFFa;
LOUhW=DAwhk/b4jXr;CCBY_=LOUhW;T5yjB=-0.5*VNvlY(DAwhk)/b4jXr;CnAKY=T5yjB;OLj8b=
EI730-JRB5m-xek11-ZaFFa-T5yjB-CnAKY;if(OLj8b>0.0){OLj8b=0.0;}Qogm3=-OLj8b/DAwhk;
mpXRu=uREfL*LOUhW-b4jXr*qn3x8(LOUhW)/6.0;xHUUb=(uREfL+T5yjB)*Qogm3-0.5*DAwhk*
VNvlY(Qogm3);_qvll=(uREfL+T5yjB+OLj8b)*CCBY_-0.5*DAwhk*VNvlY(CCBY_)+b4jXr*qn3x8(
CCBY_)/6.0;g7nwu=EgDTv-TfLg_-Ixvmb-zlPa_-mpXRu-xHUUb-_qvll;if(uREfL==0.0){uREfL=
T68ug;}FvoS1=g7nwu/uREfL;if(FvoS1<0.0){FvoS1=0.0;}



if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),(0.5*(-jEwtH)),(-JRB5m
),(-TfLg_),(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,((-b4jXr)*0.5),(-jEwtH)
,(-JRB5m),(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,-b4jXr,(-jEwtH),(
Z4lSz));}else{_2E29->fCxBi[_2E29->djbCD].jGQqQ((b4jXr/6.0),(0.5*jEwtH),JRB5m,
TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(b4jXr*0.5),jEwtH,JRB5m,(
Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,b4jXr,jEwtH,(Z4lSz));}_2E29->
FfhZi[_2E29->djbCD]=(Z4lSz)+Kaclx;_2E29->djbCD++;
Z4lSz+=(Kaclx);TfLg_+=Ixvmb;JRB5m+=xek11;jEwtH=wmBRB;


if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ((b4jXr/6.0),(0.5*(-wmBRB)),-JRB5m,-
TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*b4jXr),-wmBRB,-JRB5m,(
Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,b4jXr,-wmBRB,(Z4lSz));}else{
_2E29->fCxBi[_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),(0.5*wmBRB),JRB5m,TfLg_,(Z4lSz))
;_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*(-b4jXr)),wmBRB,JRB5m,(Z4lSz));_2E29
->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,-b4jXr,wmBRB,(Z4lSz));}_2E29->FfhZi[_2E29->
djbCD]=(Z4lSz)+EvE6x;_2E29->djbCD++;
Z4lSz+=(EvE6x);TfLg_+=zlPa_;JRB5m+=ZaFFa;jEwtH=0.0;


if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0,0.0,(-JRB5m),(-TfLg_),(Z4lSz));
_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,(-JRB5m),(Z4lSz));_2E29->NYWGt[
_2E29->djbCD].jGQqQ(0.0,0.0,0.0,0.0,(Z4lSz));}else{_2E29->fCxBi[_2E29->djbCD].
jGQqQ(0.0,0.0,JRB5m,TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,
JRB5m,(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,0.0,(Z4lSz));}_2E29
->FfhZi[_2E29->djbCD]=(Z4lSz)+FvoS1;_2E29->djbCD++;
Z4lSz+=(FvoS1);TfLg_+=g7nwu;JRB5m+=_HBf3;jEwtH=0.0;


if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ((b4jXr/6.0),0.0,-JRB5m,-TfLg_,(Z4lSz)
);_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*b4jXr),0.0,-JRB5m,(Z4lSz));_2E29->
NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,b4jXr,0.0,(Z4lSz));}else{_2E29->fCxBi[_2E29->
djbCD].jGQqQ(((-b4jXr)/6.0),0.0,JRB5m,TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].
jGQqQ(0.0,(0.5*(-b4jXr)),0.0,JRB5m,(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0
,0.0,-b4jXr,0.0,(Z4lSz));}_2E29->FfhZi[_2E29->djbCD]=(Z4lSz)+LOUhW;_2E29->djbCD
++;
Z4lSz+=(LOUhW);TfLg_+=mpXRu;JRB5m+=T5yjB;jEwtH=DAwhk;


if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0,(0.5*DAwhk),-JRB5m,-TfLg_,(Z4lSz)
);_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,DAwhk,-JRB5m,(Z4lSz));_2E29->NYWGt[
_2E29->djbCD].jGQqQ(0.0,0.0,0.0,DAwhk,(Z4lSz));}else{_2E29->fCxBi[_2E29->djbCD].
jGQqQ(0.0,(0.5*(-DAwhk)),JRB5m,TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(
0.0,0.0,-DAwhk,JRB5m,(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,-
DAwhk,(Z4lSz));}_2E29->FfhZi[_2E29->djbCD]=(Z4lSz)+Qogm3;_2E29->djbCD++;
Z4lSz+=(Qogm3);TfLg_+=xHUUb;JRB5m+=OLj8b;jEwtH=DAwhk;


if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),(0.5*DAwhk),-JRB5m,-
TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*(-b4jXr)),DAwhk,-JRB5m,
(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,(-b4jXr),DAwhk,(Z4lSz));}else{
_2E29->fCxBi[_2E29->djbCD].jGQqQ((b4jXr/6.0),(0.5*(-DAwhk)),JRB5m,TfLg_,(Z4lSz))
;_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*b4jXr),-DAwhk,JRB5m,(Z4lSz));_2E29->
NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,b4jXr,-DAwhk,(Z4lSz));}_2E29->FfhZi[_2E29->
djbCD]=(Z4lSz)+CCBY_;_2E29->djbCD++;
Z4lSz+=(CCBY_);TfLg_+=_qvll;JRB5m+=CnAKY;jEwtH=0.0;
if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0,0.0,-JRB5m,-TfLg_,(Z4lSz));_2E29
->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,-JRB5m,(Z4lSz));_2E29->NYWGt[_2E29->
djbCD].jGQqQ(0.0,0.0,0.0,0.0,(Z4lSz));}else{_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0
,0.0,JRB5m,TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,JRB5m,(
Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,0.0,(Z4lSz));}_2E29->FfhZi[
_2E29->djbCD]=(Z4lSz)+tlPiC;_2E29->djbCD++;break;




case qPN_6::aBNZo:wmBRB=pbQOc((VNvlY(jEwtH)+2.0*b4jXr*(hxixi-JRB5m))/2.0);Kaclx=
(wmBRB-jEwtH)/b4jXr;if(Kaclx<0.0){Kaclx=0.0;}EvE6x=wmBRB/b4jXr;xek11=jEwtH*Kaclx
+0.5*Kaclx*(wmBRB-jEwtH);if(xek11<0.0){xek11=0.0;}ZaFFa=0.5*wmBRB*EvE6x;Ixvmb=
JRB5m*Kaclx+0.5*jEwtH*VNvlY(Kaclx)+b4jXr*qn3x8(Kaclx)/6.0;zlPa_=(JRB5m+xek11)*
EvE6x+0.5*wmBRB*VNvlY(EvE6x)-b4jXr*qn3x8(EvE6x)/6.0;uREfL=JRB5m+xek11+ZaFFa;
RIZdZ=-pbQOc(-b4jXr*(EI730-uREfL));LOUhW=-RIZdZ/b4jXr;Qogm3=LOUhW;T5yjB=0.5*
LOUhW*RIZdZ;OLj8b=T5yjB;mpXRu=uREfL*LOUhW-b4jXr*qn3x8(LOUhW)/6.0;xHUUb=(uREfL+
T5yjB)*Qogm3+0.5*RIZdZ*VNvlY(Qogm3)+b4jXr*qn3x8(Qogm3)/6.0;g7nwu=EgDTv-TfLg_-
Ixvmb-zlPa_-mpXRu-xHUUb;if(uREfL==0.0){uREfL=T68ug;}FvoS1=g7nwu/uREfL;if(FvoS1<
0.0){FvoS1=0.0;}



if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),(0.5*(-jEwtH)),(-JRB5m
),(-TfLg_),(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,((-b4jXr)*0.5),(-jEwtH)
,(-JRB5m),(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,-b4jXr,(-jEwtH),(
Z4lSz));}else{_2E29->fCxBi[_2E29->djbCD].jGQqQ((b4jXr/6.0),(0.5*jEwtH),JRB5m,
TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(b4jXr*0.5),jEwtH,JRB5m,(
Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,b4jXr,jEwtH,(Z4lSz));}_2E29->
FfhZi[_2E29->djbCD]=(Z4lSz)+Kaclx;_2E29->djbCD++;
Z4lSz+=(Kaclx);TfLg_+=Ixvmb;JRB5m+=xek11;jEwtH=wmBRB;


if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ((b4jXr/6.0),(0.5*(-wmBRB)),-JRB5m,-
TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*b4jXr),-wmBRB,-JRB5m,(
Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,b4jXr,-wmBRB,(Z4lSz));}else{
_2E29->fCxBi[_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),(0.5*wmBRB),JRB5m,TfLg_,(Z4lSz))
;_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*(-b4jXr)),wmBRB,JRB5m,(Z4lSz));_2E29
->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,-b4jXr,wmBRB,(Z4lSz));}_2E29->FfhZi[_2E29->
djbCD]=(Z4lSz)+EvE6x;_2E29->djbCD++;
Z4lSz+=(EvE6x);TfLg_+=zlPa_;JRB5m+=ZaFFa;jEwtH=0.0;


if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0,0.0,(-JRB5m),(-TfLg_),(Z4lSz));
_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,(-JRB5m),(Z4lSz));_2E29->NYWGt[
_2E29->djbCD].jGQqQ(0.0,0.0,0.0,0.0,(Z4lSz));}else{_2E29->fCxBi[_2E29->djbCD].
jGQqQ(0.0,0.0,JRB5m,TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,
JRB5m,(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,0.0,(Z4lSz));}_2E29
->FfhZi[_2E29->djbCD]=(Z4lSz)+FvoS1;_2E29->djbCD++;
Z4lSz+=(FvoS1);TfLg_+=g7nwu;JRB5m+=_HBf3;jEwtH=0.0;


if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ((b4jXr/6.0),0.0,-JRB5m,-TfLg_,(Z4lSz)
);_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*b4jXr),0.0,-JRB5m,(Z4lSz));_2E29->
NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,b4jXr,0.0,(Z4lSz));}else{_2E29->fCxBi[_2E29->
djbCD].jGQqQ(((-b4jXr)/6.0),0.0,JRB5m,TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].
jGQqQ(0.0,(0.5*(-b4jXr)),0.0,JRB5m,(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0
,0.0,-b4jXr,0.0,(Z4lSz));}_2E29->FfhZi[_2E29->djbCD]=(Z4lSz)+LOUhW;_2E29->djbCD
++;
Z4lSz+=(LOUhW);TfLg_+=mpXRu;JRB5m+=T5yjB;jEwtH=RIZdZ;


if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),(0.5*(-RIZdZ)),-JRB5m,
-TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*(-b4jXr)),-RIZdZ,-
JRB5m,(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,(-b4jXr),-RIZdZ,(Z4lSz))
;}else{_2E29->fCxBi[_2E29->djbCD].jGQqQ((b4jXr/6.0),(0.5*RIZdZ),JRB5m,TfLg_,(
Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*b4jXr),RIZdZ,JRB5m,(Z4lSz));
_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,b4jXr,RIZdZ,(Z4lSz));}_2E29->FfhZi[
_2E29->djbCD]=(Z4lSz)+Qogm3;_2E29->djbCD++;
Z4lSz+=(Qogm3);TfLg_+=xHUUb;JRB5m+=OLj8b;jEwtH=0.0;
if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0,0.0,-JRB5m,-TfLg_,(Z4lSz));_2E29
->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,-JRB5m,(Z4lSz));_2E29->NYWGt[_2E29->
djbCD].jGQqQ(0.0,0.0,0.0,0.0,(Z4lSz));}else{_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0
,0.0,JRB5m,TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,JRB5m,(
Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,0.0,(Z4lSz));}_2E29->FfhZi[
_2E29->djbCD]=(Z4lSz)+tlPiC;_2E29->djbCD++;break;




case qPN_6::Gy8r_:Kaclx=(DAwhk-jEwtH)/b4jXr;if(Kaclx<0.0){Kaclx=0.0;}EvE6x=w8MXY
;if(EvE6x<0.0){EvE6x=0.0;}FvoS1=DAwhk/b4jXr;xek11=jEwtH*Kaclx+0.5*b4jXr*VNvlY(
Kaclx);ZaFFa=DAwhk*EvE6x;_HBf3=0.5*VNvlY(DAwhk)/b4jXr;Ixvmb=JRB5m*Kaclx+0.5*
jEwtH*VNvlY(Kaclx)+b4jXr*qn3x8(Kaclx)/6.0;zlPa_=(JRB5m+xek11)*EvE6x+0.5*DAwhk*
VNvlY(EvE6x);g7nwu=(JRB5m+xek11+ZaFFa)*FvoS1+0.5*DAwhk*VNvlY(FvoS1)-b4jXr*qn3x8(
FvoS1)/6.0;uREfL=JRB5m+xek11+ZaFFa+_HBf3;LOUhW=FvoS1;CCBY_=FvoS1;T5yjB=-_HBf3;
CnAKY=T5yjB;OLj8b=EI730-JRB5m-xek11-ZaFFa-_HBf3-T5yjB-CnAKY;if(OLj8b>0.0){OLj8b=
0.0;}Qogm3=-OLj8b/DAwhk;mpXRu=uREfL*LOUhW-b4jXr*qn3x8(LOUhW)/6.0;xHUUb=(uREfL+
T5yjB)*Qogm3-0.5*DAwhk*VNvlY(Qogm3);_qvll=(uREfL+T5yjB+OLj8b)*CCBY_-0.5*DAwhk*
VNvlY(CCBY_)+b4jXr*qn3x8(CCBY_)/6.0;



if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),(0.5*(-jEwtH)),(-JRB5m
),(-TfLg_),(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,((-b4jXr)*0.5),(-jEwtH)
,(-JRB5m),(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,-b4jXr,(-jEwtH),(
Z4lSz));}else{_2E29->fCxBi[_2E29->djbCD].jGQqQ((b4jXr/6.0),(0.5*jEwtH),JRB5m,
TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(b4jXr*0.5),jEwtH,JRB5m,(
Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,b4jXr,jEwtH,(Z4lSz));}_2E29->
FfhZi[_2E29->djbCD]=(Z4lSz)+Kaclx;_2E29->djbCD++;
Z4lSz+=(Kaclx);TfLg_+=Ixvmb;JRB5m+=xek11;jEwtH=DAwhk;

if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0,(0.5*(-DAwhk)),(-JRB5m),(-TfLg_),
(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,-DAwhk,(-JRB5m),(Z4lSz));_2E29
->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,-DAwhk,(Z4lSz));}else{_2E29->fCxBi[_2E29
->djbCD].jGQqQ(0.0,(0.5*DAwhk),JRB5m,TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].
jGQqQ(0.0,0.0,DAwhk,JRB5m,(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,
DAwhk,(Z4lSz));}_2E29->FfhZi[_2E29->djbCD]=(Z4lSz)+EvE6x;_2E29->djbCD++;
Z4lSz+=(EvE6x);TfLg_+=zlPa_;JRB5m+=ZaFFa;jEwtH=DAwhk;

if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ((b4jXr/6.0),(0.5*(-DAwhk)),(-JRB5m),(
-TfLg_),(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*b4jXr),-DAwhk,(-JRB5m
),(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,b4jXr,-DAwhk,(Z4lSz));}else{
_2E29->fCxBi[_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),(0.5*DAwhk),JRB5m,TfLg_,(Z4lSz))
;_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*(-b4jXr)),DAwhk,JRB5m,(Z4lSz));_2E29
->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,-b4jXr,DAwhk,(Z4lSz));}_2E29->FfhZi[_2E29->
djbCD]=(Z4lSz)+FvoS1;_2E29->djbCD++;
Z4lSz+=(FvoS1);TfLg_+=g7nwu;JRB5m+=_HBf3;jEwtH=0.0;

if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(((b4jXr)/6.0),0.0,(-JRB5m),(-TfLg_),(
Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*b4jXr),0.0,(-JRB5m),(Z4lSz));
_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,b4jXr,0.0,(Z4lSz));}else{_2E29->fCxBi[
_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),0.0,JRB5m,TfLg_,(Z4lSz));_2E29->diBqY[_2E29->
djbCD].jGQqQ(0.0,(0.5*(-b4jXr)),0.0,JRB5m,(Z4lSz));_2E29->NYWGt[_2E29->djbCD].
jGQqQ(0.0,0.0,-b4jXr,0.0,(Z4lSz));}_2E29->FfhZi[_2E29->djbCD]=(Z4lSz)+LOUhW;
_2E29->djbCD++;
Z4lSz+=(LOUhW);TfLg_+=mpXRu;JRB5m+=T5yjB;jEwtH=-DAwhk;

if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0,(0.5*DAwhk),-JRB5m,-TfLg_,(Z4lSz)
);_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,DAwhk,-JRB5m,(Z4lSz));_2E29->NYWGt[
_2E29->djbCD].jGQqQ(0.0,0.0,0.0,DAwhk,(Z4lSz));}else{_2E29->fCxBi[_2E29->djbCD].
jGQqQ(0.0,(0.5*(-DAwhk)),JRB5m,TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(
0.0,0.0,-DAwhk,JRB5m,(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,-
DAwhk,(Z4lSz));}_2E29->FfhZi[_2E29->djbCD]=(Z4lSz)+Qogm3;_2E29->djbCD++;
Z4lSz+=(Qogm3);TfLg_+=xHUUb;JRB5m+=OLj8b;jEwtH=-DAwhk;

if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),(0.5*DAwhk),-JRB5m,-
TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*(-b4jXr)),DAwhk,-JRB5m,
(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,(-b4jXr),DAwhk,(Z4lSz));}else{
_2E29->fCxBi[_2E29->djbCD].jGQqQ((b4jXr/6.0),(0.5*(-DAwhk)),JRB5m,TfLg_,(Z4lSz))
;_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*b4jXr),-DAwhk,JRB5m,(Z4lSz));_2E29->
NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,b4jXr,-DAwhk,(Z4lSz));}_2E29->FfhZi[_2E29->
djbCD]=(Z4lSz)+CCBY_;_2E29->djbCD++;
Z4lSz+=(CCBY_);TfLg_+=_qvll;JRB5m+=CnAKY;jEwtH=0.0;
if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0,0.0,-JRB5m,-TfLg_,(Z4lSz));_2E29
->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,-JRB5m,(Z4lSz));_2E29->NYWGt[_2E29->
djbCD].jGQqQ(0.0,0.0,0.0,0.0,(Z4lSz));}else{_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0
,0.0,JRB5m,TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,JRB5m,(
Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,0.0,(Z4lSz));}_2E29->FfhZi[
_2E29->djbCD]=(Z4lSz)+tlPiC;_2E29->djbCD++;break;




case qPN_6::CWXWH:Kaclx=(DAwhk-jEwtH)/b4jXr;if(Kaclx<0.0){Kaclx=0.0;}xek11=jEwtH
*Kaclx+0.5*b4jXr*VNvlY(Kaclx);Ixvmb=JRB5m*Kaclx+0.5*jEwtH*VNvlY(Kaclx)+b4jXr*
qn3x8(Kaclx)/6.0;_HBf3=0.5*VNvlY(DAwhk)/b4jXr;FvoS1=DAwhk/b4jXr;RIZdZ=w8MXY;if(
RIZdZ>0.0){RIZdZ=0.0;}LOUhW=-RIZdZ/b4jXr;Qogm3=LOUhW;T5yjB=0.5*RIZdZ*LOUhW;OLj8b
=T5yjB;ZaFFa=EI730-JRB5m-xek11-_HBf3-T5yjB-OLj8b;if(ZaFFa<0.0){ZaFFa=0.0;}EvE6x=
ZaFFa/DAwhk;zlPa_=(JRB5m+xek11)*EvE6x+0.5*DAwhk*VNvlY(EvE6x);g7nwu=(JRB5m+xek11+
ZaFFa)*FvoS1+0.5*DAwhk*VNvlY(FvoS1)-b4jXr*qn3x8(FvoS1)/6.0;mpXRu=(JRB5m+xek11+
ZaFFa+_HBf3)*LOUhW-b4jXr*qn3x8(LOUhW)/6.0;xHUUb=(JRB5m+xek11+ZaFFa+_HBf3+T5yjB)*
Qogm3+0.5*RIZdZ*VNvlY(Qogm3)+b4jXr*qn3x8(Qogm3)/6.0;



if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),(0.5*(-jEwtH)),(-JRB5m
),(-TfLg_),(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,((-b4jXr)*0.5),(-jEwtH)
,(-JRB5m),(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,-b4jXr,(-jEwtH),(
Z4lSz));}else{_2E29->fCxBi[_2E29->djbCD].jGQqQ((b4jXr/6.0),(0.5*jEwtH),JRB5m,
TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(b4jXr*0.5),jEwtH,JRB5m,(
Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,b4jXr,jEwtH,(Z4lSz));}_2E29->
FfhZi[_2E29->djbCD]=(Z4lSz)+Kaclx;_2E29->djbCD++;
Z4lSz+=(Kaclx);TfLg_+=Ixvmb;JRB5m+=xek11;jEwtH=DAwhk;

if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0,(0.5*(-DAwhk)),(-JRB5m),(-TfLg_),
(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,-DAwhk,(-JRB5m),(Z4lSz));_2E29
->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,-DAwhk,(Z4lSz));}else{_2E29->fCxBi[_2E29
->djbCD].jGQqQ(0.0,(0.5*DAwhk),JRB5m,TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].
jGQqQ(0.0,0.0,DAwhk,JRB5m,(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,
DAwhk,(Z4lSz));}_2E29->FfhZi[_2E29->djbCD]=(Z4lSz)+EvE6x;_2E29->djbCD++;
Z4lSz+=(EvE6x);TfLg_+=zlPa_;JRB5m+=ZaFFa;jEwtH=DAwhk;

if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ((b4jXr/6.0),(0.5*(-DAwhk)),(-JRB5m),(
-TfLg_),(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*b4jXr),-DAwhk,(-JRB5m
),(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,b4jXr,-DAwhk,(Z4lSz));}else{
_2E29->fCxBi[_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),(0.5*DAwhk),JRB5m,TfLg_,(Z4lSz))
;_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*(-b4jXr)),DAwhk,JRB5m,(Z4lSz));_2E29
->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,-b4jXr,DAwhk,(Z4lSz));}_2E29->FfhZi[_2E29->
djbCD]=(Z4lSz)+FvoS1;_2E29->djbCD++;
Z4lSz+=(FvoS1);TfLg_+=g7nwu;JRB5m+=_HBf3;jEwtH=0.0;

if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(((b4jXr)/6.0),0.0,(-JRB5m),(-TfLg_),(
Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*b4jXr),0.0,(-JRB5m),(Z4lSz));
_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,b4jXr,0.0,(Z4lSz));}else{_2E29->fCxBi[
_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),0.0,JRB5m,TfLg_,(Z4lSz));_2E29->diBqY[_2E29->
djbCD].jGQqQ(0.0,(0.5*(-b4jXr)),0.0,JRB5m,(Z4lSz));_2E29->NYWGt[_2E29->djbCD].
jGQqQ(0.0,0.0,-b4jXr,0.0,(Z4lSz));}_2E29->FfhZi[_2E29->djbCD]=(Z4lSz)+LOUhW;
_2E29->djbCD++;
Z4lSz+=(LOUhW);TfLg_+=mpXRu;JRB5m+=T5yjB;jEwtH=RIZdZ;

if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),(0.5*(-RIZdZ)),-JRB5m,
-TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*(-b4jXr)),-RIZdZ,-
JRB5m,(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,-b4jXr,-RIZdZ,(Z4lSz));}
else{_2E29->fCxBi[_2E29->djbCD].jGQqQ((b4jXr/6.0),(0.5*RIZdZ),JRB5m,TfLg_,(Z4lSz
));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*b4jXr),RIZdZ,JRB5m,(Z4lSz));_2E29->
NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,b4jXr,RIZdZ,(Z4lSz));}_2E29->FfhZi[_2E29->
djbCD]=(Z4lSz)+Qogm3;_2E29->djbCD++;
Z4lSz+=(Qogm3);TfLg_+=xHUUb;JRB5m+=OLj8b;jEwtH=0.0;
if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0,0.0,-JRB5m,-TfLg_,(Z4lSz));_2E29
->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,-JRB5m,(Z4lSz));_2E29->NYWGt[_2E29->
djbCD].jGQqQ(0.0,0.0,0.0,0.0,(Z4lSz));}else{_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0
,0.0,JRB5m,TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,JRB5m,(
Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,0.0,(Z4lSz));}_2E29->FfhZi[
_2E29->djbCD]=(Z4lSz)+tlPiC;_2E29->djbCD++;break;




case qPN_6::Tuwfw:wmBRB=w8MXY;if(wmBRB<jEwtH){wmBRB=jEwtH;}Kaclx=(wmBRB-jEwtH)/
b4jXr;EvE6x=wmBRB/b4jXr;xek11=jEwtH*Kaclx+0.5*(wmBRB-jEwtH)*Kaclx;if(xek11<0.0){
xek11=0.0;}ZaFFa=0.5*wmBRB*EvE6x;Ixvmb=JRB5m*Kaclx+0.5*jEwtH*VNvlY(Kaclx)+b4jXr*
qn3x8(Kaclx)/6.0;zlPa_=(JRB5m+xek11)*EvE6x+0.5*wmBRB*VNvlY(EvE6x)-b4jXr*qn3x8(
EvE6x)/6.0;FvoS1=DAwhk/b4jXr;Qogm3=FvoS1;_HBf3=-0.5*DAwhk*FvoS1;OLj8b=_HBf3;
T5yjB=EI730-JRB5m-xek11-ZaFFa-_HBf3-OLj8b;if(T5yjB>0.0){T5yjB=0.0;}LOUhW=-T5yjB/
DAwhk;g7nwu=(JRB5m+xek11+ZaFFa)*FvoS1-b4jXr*qn3x8(FvoS1)/6.0;mpXRu=(JRB5m+xek11+
ZaFFa+_HBf3)*LOUhW-0.5*DAwhk*VNvlY(LOUhW);xHUUb=(JRB5m+xek11+ZaFFa+_HBf3+T5yjB)*
Qogm3-0.5*DAwhk*VNvlY(Qogm3)+b4jXr*qn3x8(Qogm3)/6.0;



if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),(0.5*(-jEwtH)),(-JRB5m
),(-TfLg_),(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,((-b4jXr)*0.5),(-jEwtH)
,(-JRB5m),(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,-b4jXr,(-jEwtH),(
Z4lSz));}else{_2E29->fCxBi[_2E29->djbCD].jGQqQ((b4jXr/6.0),(0.5*jEwtH),JRB5m,
TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(b4jXr*0.5),jEwtH,JRB5m,(
Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,b4jXr,jEwtH,(Z4lSz));}_2E29->
FfhZi[_2E29->djbCD]=(Z4lSz)+Kaclx;_2E29->djbCD++;
Z4lSz+=(Kaclx);TfLg_+=Ixvmb;JRB5m+=xek11;jEwtH=wmBRB;


if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ((b4jXr/6.0),(0.5*(-wmBRB)),-JRB5m,-
TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*b4jXr),-wmBRB,-JRB5m,(
Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,b4jXr,-wmBRB,(Z4lSz));}else{
_2E29->fCxBi[_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),(0.5*wmBRB),JRB5m,TfLg_,(Z4lSz))
;_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*(-b4jXr)),wmBRB,JRB5m,(Z4lSz));_2E29
->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,-b4jXr,wmBRB,(Z4lSz));}_2E29->FfhZi[_2E29->
djbCD]=(Z4lSz)+EvE6x;_2E29->djbCD++;
Z4lSz+=(EvE6x);TfLg_+=zlPa_;JRB5m+=ZaFFa;jEwtH=0.0;


if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ((b4jXr/6.0),0.0,-JRB5m,-TfLg_,(Z4lSz)
);_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*b4jXr),0.0,-JRB5m,(Z4lSz));_2E29->
NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,b4jXr,0.0,(Z4lSz));}else{_2E29->fCxBi[_2E29->
djbCD].jGQqQ(((-b4jXr)/6.0),0.0,JRB5m,TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].
jGQqQ(0.0,(0.5*(-b4jXr)),0.0,JRB5m,(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0
,0.0,-b4jXr,0.0,(Z4lSz));}_2E29->FfhZi[_2E29->djbCD]=(Z4lSz)+FvoS1;_2E29->djbCD
++;
Z4lSz+=(FvoS1);TfLg_+=g7nwu;JRB5m+=_HBf3;jEwtH=DAwhk;


if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0,(0.5*DAwhk),-JRB5m,-TfLg_,(Z4lSz)
);_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,DAwhk,-JRB5m,(Z4lSz));_2E29->NYWGt[
_2E29->djbCD].jGQqQ(0.0,0.0,0.0,DAwhk,(Z4lSz));}else{_2E29->fCxBi[_2E29->djbCD].
jGQqQ(0.0,(0.5*(-DAwhk)),JRB5m,TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(
0.0,0.0,-DAwhk,JRB5m,(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,-
DAwhk,(Z4lSz));}_2E29->FfhZi[_2E29->djbCD]=(Z4lSz)+LOUhW;_2E29->djbCD++;
Z4lSz+=(LOUhW);TfLg_+=mpXRu;JRB5m+=T5yjB;jEwtH=DAwhk;


if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),(0.5*DAwhk),-JRB5m,-
TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*(-b4jXr)),DAwhk,-JRB5m,
(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,(-b4jXr),DAwhk,(Z4lSz));}else{
_2E29->fCxBi[_2E29->djbCD].jGQqQ((b4jXr/6.0),(0.5*(-DAwhk)),JRB5m,TfLg_,(Z4lSz))
;_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*b4jXr),-DAwhk,JRB5m,(Z4lSz));_2E29->
NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,b4jXr,-DAwhk,(Z4lSz));}_2E29->FfhZi[_2E29->
djbCD]=(Z4lSz)+Qogm3;_2E29->djbCD++;
Z4lSz+=(Qogm3);TfLg_+=xHUUb;JRB5m+=OLj8b;jEwtH=0.0;
if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0,0.0,-JRB5m,-TfLg_,(Z4lSz));_2E29
->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,-JRB5m,(Z4lSz));_2E29->NYWGt[_2E29->
djbCD].jGQqQ(0.0,0.0,0.0,0.0,(Z4lSz));}else{_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0
,0.0,JRB5m,TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,JRB5m,(
Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,0.0,(Z4lSz));}_2E29->FfhZi[
_2E29->djbCD]=(Z4lSz)+tlPiC;_2E29->djbCD++;break;




case qPN_6::cvsfI:wmBRB=w8MXY;if(wmBRB<jEwtH){wmBRB=jEwtH;}Kaclx=(wmBRB-jEwtH)/
b4jXr;EvE6x=wmBRB/b4jXr;xek11=jEwtH*Kaclx+0.5*(wmBRB-jEwtH)*Kaclx;if(xek11<0.0){
xek11=0.0;}ZaFFa=0.5*wmBRB*EvE6x;Ixvmb=JRB5m*Kaclx+0.5*jEwtH*VNvlY(Kaclx)+b4jXr*
qn3x8(Kaclx)/6.0;zlPa_=(JRB5m+xek11)*EvE6x+0.5*wmBRB*VNvlY(EvE6x)-b4jXr*qn3x8(
EvE6x)/6.0;RIZdZ=-pbQOc(-b4jXr*(EI730-(JRB5m+xek11+ZaFFa)));FvoS1=-RIZdZ/b4jXr;
LOUhW=FvoS1;_HBf3=0.5*RIZdZ*FvoS1;T5yjB=_HBf3;g7nwu=(JRB5m+xek11+ZaFFa)*FvoS1-
b4jXr*qn3x8(FvoS1)/6.0;mpXRu=(JRB5m+xek11+ZaFFa+_HBf3)*LOUhW+0.5*RIZdZ*VNvlY(
LOUhW)+b4jXr*qn3x8(LOUhW)/6.0;



if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),(0.5*(-jEwtH)),(-JRB5m
),(-TfLg_),(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,((-b4jXr)*0.5),(-jEwtH)
,(-JRB5m),(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,-b4jXr,(-jEwtH),(
Z4lSz));}else{_2E29->fCxBi[_2E29->djbCD].jGQqQ((b4jXr/6.0),(0.5*jEwtH),JRB5m,
TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(b4jXr*0.5),jEwtH,JRB5m,(
Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,b4jXr,jEwtH,(Z4lSz));}_2E29->
FfhZi[_2E29->djbCD]=(Z4lSz)+Kaclx;_2E29->djbCD++;
Z4lSz+=(Kaclx);TfLg_+=Ixvmb;JRB5m+=xek11;jEwtH=wmBRB;


if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ((b4jXr/6.0),(0.5*(-wmBRB)),-JRB5m,-
TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*b4jXr),-wmBRB,-JRB5m,(
Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,b4jXr,-wmBRB,(Z4lSz));}else{
_2E29->fCxBi[_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),(0.5*wmBRB),JRB5m,TfLg_,(Z4lSz))
;_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*(-b4jXr)),wmBRB,JRB5m,(Z4lSz));_2E29
->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,-b4jXr,wmBRB,(Z4lSz));}_2E29->FfhZi[_2E29->
djbCD]=(Z4lSz)+EvE6x;_2E29->djbCD++;
Z4lSz+=(EvE6x);TfLg_+=zlPa_;JRB5m+=ZaFFa;jEwtH=0.0;


if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ((b4jXr/6.0),0.0,-JRB5m,-TfLg_,(Z4lSz)
);_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*b4jXr),0.0,-JRB5m,(Z4lSz));_2E29->
NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,b4jXr,0.0,(Z4lSz));}else{_2E29->fCxBi[_2E29->
djbCD].jGQqQ(((-b4jXr)/6.0),0.0,JRB5m,TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].
jGQqQ(0.0,(0.5*(-b4jXr)),0.0,JRB5m,(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0
,0.0,-b4jXr,0.0,(Z4lSz));}_2E29->FfhZi[_2E29->djbCD]=(Z4lSz)+FvoS1;_2E29->djbCD
++;
Z4lSz+=(FvoS1);TfLg_+=g7nwu;JRB5m+=_HBf3;jEwtH=RIZdZ;


if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),(0.5*(-RIZdZ)),-JRB5m,
-TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*(-b4jXr)),-RIZdZ,-
JRB5m,(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,(-b4jXr),-RIZdZ,(Z4lSz))
;}else{_2E29->fCxBi[_2E29->djbCD].jGQqQ((b4jXr/6.0),(0.5*RIZdZ),JRB5m,TfLg_,(
Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*b4jXr),RIZdZ,JRB5m,(Z4lSz));
_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,b4jXr,RIZdZ,(Z4lSz));}_2E29->FfhZi[
_2E29->djbCD]=(Z4lSz)+LOUhW;_2E29->djbCD++;
Z4lSz+=(LOUhW);TfLg_+=mpXRu;JRB5m+=T5yjB;jEwtH=0.0;
if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0,0.0,-JRB5m,-TfLg_,(Z4lSz));_2E29
->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,-JRB5m,(Z4lSz));_2E29->NYWGt[_2E29->
djbCD].jGQqQ(0.0,0.0,0.0,0.0,(Z4lSz));}else{_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0
,0.0,JRB5m,TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,JRB5m,(
Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,0.0,(Z4lSz));}_2E29->FfhZi[
_2E29->djbCD]=(Z4lSz)+tlPiC;_2E29->djbCD++;break;




case qPN_6::YItCk:wmBRB=w8MXY;if(wmBRB>jEwtH){wmBRB=jEwtH;}Kaclx=(jEwtH-wmBRB)/
b4jXr;xek11=wmBRB*Kaclx+0.5*(jEwtH-wmBRB)*Kaclx;Ixvmb=JRB5m*Kaclx+0.5*jEwtH*
VNvlY(Kaclx)-b4jXr*qn3x8(Kaclx)/6.0;RIZdZ=pbQOc(0.5*VNvlY(wmBRB)+b4jXr*(EI730-(
JRB5m+xek11)));if(RIZdZ<wmBRB){RIZdZ=wmBRB;}EvE6x=(RIZdZ-wmBRB)/b4jXr;ZaFFa=
wmBRB*EvE6x+0.5*(RIZdZ-wmBRB)*EvE6x;zlPa_=(JRB5m+xek11)*EvE6x+0.5*wmBRB*VNvlY(
EvE6x)+b4jXr*qn3x8(EvE6x)/6.0;FvoS1=RIZdZ/b4jXr;_HBf3=0.5*RIZdZ*FvoS1;g7nwu=(
JRB5m+xek11+ZaFFa)*FvoS1+0.5*RIZdZ*VNvlY(FvoS1)-b4jXr*qn3x8(FvoS1)/6.0;



if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ((b4jXr/6.0),(0.5*(-jEwtH)),(-JRB5m),(
-TfLg_),(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(b4jXr*0.5),(-jEwtH),(-
JRB5m),(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,b4jXr,(-jEwtH),(Z4lSz))
;}else{_2E29->fCxBi[_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),(0.5*jEwtH),JRB5m,TfLg_,(
Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,((-b4jXr)*0.5),jEwtH,JRB5m,(Z4lSz))
;_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,-b4jXr,jEwtH,(Z4lSz));}_2E29->FfhZi[
_2E29->djbCD]=(Z4lSz)+Kaclx;_2E29->djbCD++;
Z4lSz+=(Kaclx);TfLg_+=Ixvmb;JRB5m+=xek11;jEwtH=wmBRB;

if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),(0.5*(-wmBRB)),(-JRB5m
),(-TfLg_),(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*(-b4jXr)),-wmBRB,(
-JRB5m),(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,-b4jXr,-wmBRB,(Z4lSz))
;}else{_2E29->fCxBi[_2E29->djbCD].jGQqQ((b4jXr/6.0),(0.5*wmBRB),JRB5m,TfLg_,(
Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*b4jXr),wmBRB,JRB5m,(Z4lSz));
_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,b4jXr,wmBRB,(Z4lSz));}_2E29->FfhZi[
_2E29->djbCD]=(Z4lSz)+EvE6x;_2E29->djbCD++;
Z4lSz+=(EvE6x);TfLg_+=zlPa_;JRB5m+=ZaFFa;jEwtH=RIZdZ;

if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ((b4jXr/6.0),(0.5*(-RIZdZ)),-JRB5m,-
TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*b4jXr),-RIZdZ,-JRB5m,(
Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,b4jXr,-RIZdZ,(Z4lSz));}else{
_2E29->fCxBi[_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),(0.5*RIZdZ),JRB5m,TfLg_,(Z4lSz))
;_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*(-b4jXr)),RIZdZ,JRB5m,(Z4lSz));_2E29
->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,-b4jXr,RIZdZ,(Z4lSz));}_2E29->FfhZi[_2E29->
djbCD]=(Z4lSz)+FvoS1;_2E29->djbCD++;
Z4lSz+=(FvoS1);TfLg_+=g7nwu;JRB5m+=_HBf3;jEwtH=0.0;
if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0,0.0,-JRB5m,-TfLg_,(Z4lSz));_2E29
->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,-JRB5m,(Z4lSz));_2E29->NYWGt[_2E29->
djbCD].jGQqQ(0.0,0.0,0.0,0.0,(Z4lSz));}else{_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0
,0.0,JRB5m,TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,JRB5m,(
Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,0.0,(Z4lSz));}_2E29->FfhZi[
_2E29->djbCD]=(Z4lSz)+tlPiC;_2E29->djbCD++;break;




case qPN_6::n1iUG:wmBRB=w8MXY;if(wmBRB>jEwtH){wmBRB=jEwtH;}Kaclx=(jEwtH-wmBRB)/
b4jXr;xek11=wmBRB*Kaclx+0.5*(jEwtH-wmBRB)*Kaclx;Ixvmb=JRB5m*Kaclx+0.5*jEwtH*
VNvlY(Kaclx)-b4jXr*qn3x8(Kaclx)/6.0;EvE6x=(DAwhk-wmBRB)/b4jXr;ZaFFa=wmBRB*EvE6x+
0.5*(DAwhk-wmBRB)*EvE6x;zlPa_=(JRB5m+xek11)*EvE6x+0.5*wmBRB*VNvlY(EvE6x)+b4jXr*
qn3x8(EvE6x)/6.0;LOUhW=DAwhk/b4jXr;T5yjB=0.5*DAwhk*LOUhW;_HBf3=EI730-JRB5m-xek11
-ZaFFa-T5yjB;if(_HBf3<0.0){_HBf3=0.0;}FvoS1=_HBf3/DAwhk;g7nwu=(JRB5m+xek11+ZaFFa
)*FvoS1+0.5*DAwhk*VNvlY(FvoS1);mpXRu=(JRB5m+xek11+ZaFFa+_HBf3)*LOUhW+0.5*DAwhk*
VNvlY(LOUhW)-b4jXr*qn3x8(LOUhW)/6.0;



if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ((b4jXr/6.0),(0.5*(-jEwtH)),(-JRB5m),(
-TfLg_),(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(b4jXr*0.5),(-jEwtH),(-
JRB5m),(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,b4jXr,(-jEwtH),(Z4lSz))
;}else{_2E29->fCxBi[_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),(0.5*jEwtH),JRB5m,TfLg_,(
Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,((-b4jXr)*0.5),jEwtH,JRB5m,(Z4lSz))
;_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,-b4jXr,jEwtH,(Z4lSz));}_2E29->FfhZi[
_2E29->djbCD]=(Z4lSz)+Kaclx;_2E29->djbCD++;
Z4lSz+=(Kaclx);TfLg_+=Ixvmb;JRB5m+=xek11;jEwtH=wmBRB;

if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),(0.5*(-wmBRB)),(-JRB5m
),(-TfLg_),(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*(-b4jXr)),-wmBRB,(
-JRB5m),(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,-b4jXr,-wmBRB,(Z4lSz))
;}else{_2E29->fCxBi[_2E29->djbCD].jGQqQ((b4jXr/6.0),(0.5*wmBRB),JRB5m,TfLg_,(
Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*b4jXr),wmBRB,JRB5m,(Z4lSz));
_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,b4jXr,wmBRB,(Z4lSz));}_2E29->FfhZi[
_2E29->djbCD]=(Z4lSz)+EvE6x;_2E29->djbCD++;
Z4lSz+=(EvE6x);TfLg_+=zlPa_;JRB5m+=ZaFFa;jEwtH=DAwhk;

if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0,(0.5*(-DAwhk)),-JRB5m,-TfLg_,(
Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,-DAwhk,-JRB5m,(Z4lSz));_2E29->
NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,-DAwhk,(Z4lSz));}else{_2E29->fCxBi[_2E29->
djbCD].jGQqQ(0.0,(0.5*DAwhk),JRB5m,TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].
jGQqQ(0.0,0.0,DAwhk,JRB5m,(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,
DAwhk,(Z4lSz));}_2E29->FfhZi[_2E29->djbCD]=(Z4lSz)+FvoS1;_2E29->djbCD++;
Z4lSz+=(FvoS1);TfLg_+=g7nwu;JRB5m+=_HBf3;jEwtH=DAwhk;

if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ((b4jXr/6.0),(0.5*(-DAwhk)),-JRB5m,-
TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*b4jXr),-DAwhk,-JRB5m,(
Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,b4jXr,-DAwhk,(Z4lSz));}else{
_2E29->fCxBi[_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),(0.5*DAwhk),JRB5m,TfLg_,(Z4lSz))
;_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*(-b4jXr)),DAwhk,JRB5m,(Z4lSz));_2E29
->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,-b4jXr,DAwhk,(Z4lSz));}_2E29->FfhZi[_2E29->
djbCD]=(Z4lSz)+LOUhW;_2E29->djbCD++;
Z4lSz+=(LOUhW);TfLg_+=mpXRu;JRB5m+=T5yjB;jEwtH=0.0;
if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0,0.0,-JRB5m,-TfLg_,(Z4lSz));_2E29
->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,-JRB5m,(Z4lSz));_2E29->NYWGt[_2E29->
djbCD].jGQqQ(0.0,0.0,0.0,0.0,(Z4lSz));}else{_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0
,0.0,JRB5m,TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,JRB5m,(
Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,0.0,(Z4lSz));}_2E29->FfhZi[
_2E29->djbCD]=(Z4lSz)+tlPiC;_2E29->djbCD++;break;default:break;}return;}
