
































#include <TypeIVRMLDecisionTree1B3.h>
#include <TypeIVRMLStep1Decisions.h>
#include <TypeIVRMLMath.h>
#include <TypeIVRMLDefinitions.h>
#include <TypeIVRMLStep1IntermediateTimeProfiles.h>
#include <TypeIVRMLStep1RootFunctions.h>
#include <TypeIVRMLStep1Profiles.h>
#include <RMLPositionFlags.h>


bool qPN_6::v746A(const double&iPzPj,const double&AQzqu,const double&k6sf4,const
 double&b4jXr,const double&DAwhk,const double&hxixi,const double&ldeCA,const 
double&HcINC,const int&srTPK,const int&bocnd,FOwyh*ataeF,double*izcH4,double*
Ib5FR){bool DBRdu=false;double Z4lSz=0.0,TfLg_=iPzPj,JRB5m=AQzqu,jEwtH=k6sf4,
EgDTv=ldeCA,EI730=HcINC,ebyTQ=0.0,P9_Ry=tlPiC,OFDtk=tlPiC;*Ib5FR=tlPiC;
if(ZsQgx(jEwtH)){goto lq3ue;}
else{
aepf8(&TfLg_,&JRB5m,&jEwtH,&EgDTv,&EI730);goto lq3ue;}lq3ue:
if(PsauU(jEwtH,DAwhk)){goto h0X65;}
else{
OuC3N(&Z4lSz,&TfLg_,&JRB5m,&jEwtH,DAwhk,b4jXr);goto h0X65;}h0X65:
if(_JUe9(jEwtH,JRB5m,hxixi,b4jXr)){goto ja69l;}
else{
n9iwH(&Z4lSz,&TfLg_,&JRB5m,&jEwtH,b4jXr);
aepf8(&TfLg_,&JRB5m,&jEwtH,&EgDTv,&EI730);goto LU5qP;}ja69l:
if(ZX6Y5(JRB5m,hxixi)){goto aJatG;}
else{goto LU5qP;}LU5qP:if(bocnd==RMLPositionFlags::GET_INTO_BOUNDARIES_FAST){
if(ix0lH(jEwtH,DAwhk,JRB5m,hxixi,b4jXr)){goto Vrcjl;}
else{goto RLE5j;}}else
{
if(vhEYc(jEwtH,DAwhk,JRB5m,-hxixi,b4jXr)){YJ3N_(&Z4lSz,&TfLg_,&JRB5m,&jEwtH,
hxixi,DAwhk,b4jXr);}
else{WlCk8(&Z4lSz,&TfLg_,&JRB5m,&jEwtH,hxixi,b4jXr);}goto aJatG;}Vrcjl:
if(kCbyv(jEwtH,JRB5m,hxixi,b4jXr)){
Rr9wP(&Z4lSz,&TfLg_,&JRB5m,&jEwtH,hxixi,b4jXr);goto aJatG;}
else{
Ut1Kv(&Z4lSz,&TfLg_,&JRB5m,&jEwtH,hxixi,b4jXr);goto aJatG;}RLE5j:
if(pjX4V(jEwtH,DAwhk,JRB5m,hxixi,b4jXr)){goto j8hHe;}
else{
Fj8b0(&Z4lSz,&TfLg_,&JRB5m,&jEwtH,hxixi,DAwhk,b4jXr);goto aJatG;}j8hHe:
if(huDFX(jEwtH,DAwhk,JRB5m,hxixi,b4jXr)){
Ut1Kv(&Z4lSz,&TfLg_,&JRB5m,&jEwtH,hxixi,b4jXr);goto aJatG;}
else{
qNBFJ(&Z4lSz,&TfLg_,&JRB5m,&jEwtH,hxixi,DAwhk,b4jXr);goto aJatG;}aJatG:ebyTQ=
fabs(jEwtH/b4jXr);if((OI259((JRB5m+0.5*jEwtH*ebyTQ),EI730,y1AP2))&&(OI259((TfLg_
+JRB5m*ebyTQ+0.5*jEwtH*VNvlY(ebyTQ)-B4Gkb(jEwtH)*qn3x8(ebyTQ)*b4jXr/6.0),EgDTv,
y1AP2))){Z4lSz+=ebyTQ;goto X4nOF;}
if(PWANe(jEwtH,JRB5m,EI730,b4jXr)){goto kk9wX;}else{Z4lSz=tlPiC;goto X4nOF;}
kk9wX:
if(eAfAf(srTPK)){goto MBiIH;}else{Z4lSz=tlPiC;goto X4nOF;}MBiIH:
if(YIBe5(jEwtH,DAwhk,JRB5m,EI730,b4jXr)){goto enpJy;}else{goto xHNfu;}enpJy:
if(WHaCg(jEwtH,DAwhk,JRB5m,EI730,b4jXr)){goto YCp5b;}else{goto ahWwG;}YCp5b:
if(wJ_yV(jEwtH,DAwhk,JRB5m,EI730,TfLg_,EgDTv,b4jXr)){goto BLPSs;}else{
z_T_D(&Z4lSz,TfLg_,EgDTv,JRB5m,hxixi,EI730,jEwtH,DAwhk,b4jXr,0.0,&DBRdu);*ataeF=
n1iUG;goto X4nOF;}BLPSs:


if(V66ya(jEwtH,DAwhk,JRB5m,hxixi,EI730,TfLg_,EgDTv,b4jXr,&P9_Ry,&OFDtk)){

if(OFDtk!=tlPiC){OFDtk+=Z4lSz;}Z4lSz+=P9_Ry;*ataeF=n1iUG;goto X4nOF;}else{Z4lSz=
tlPiC;goto X4nOF;}ahWwG:
if(fOpp3(jEwtH,JRB5m,EI730,TfLg_,EgDTv,b4jXr)){goto Yv0CU;}else{goto rhRqc;}
Yv0CU:


if(TIjb6(jEwtH,DAwhk,JRB5m,hxixi,EI730,TfLg_,EgDTv,b4jXr,&P9_Ry,&OFDtk)){

if(OFDtk!=tlPiC){OFDtk+=Z4lSz;}Z4lSz+=P9_Ry;*ataeF=n1iUG;goto X4nOF;}else{goto 
cZ_aO;}rhRqc:
if(CFnu3(jEwtH,DAwhk,JRB5m,hxixi,EI730,TfLg_,EgDTv,b4jXr)){
rXwpz(&Z4lSz,TfLg_,EgDTv,JRB5m,hxixi,EI730,jEwtH,DAwhk,b4jXr,0.0,&DBRdu);if(
DBRdu){
z_T_D(&Z4lSz,TfLg_,EgDTv,JRB5m,hxixi,EI730,jEwtH,DAwhk,b4jXr,0.0,&DBRdu);*ataeF=
n1iUG;}else{*ataeF=YItCk;}goto X4nOF;}else{
z_T_D(&Z4lSz,TfLg_,EgDTv,JRB5m,hxixi,EI730,jEwtH,DAwhk,b4jXr,0.0,&DBRdu);if(
DBRdu){
rXwpz(&Z4lSz,TfLg_,EgDTv,JRB5m,hxixi,EI730,jEwtH,DAwhk,b4jXr,0.0,&DBRdu);*ataeF=
YItCk;}else{*ataeF=n1iUG;}goto X4nOF;}xHNfu:
if(EVz5o(jEwtH,JRB5m,EI730,TfLg_,EgDTv,b4jXr)){goto cZ_aO;}else{
rXwpz(&Z4lSz,TfLg_,EgDTv,JRB5m,hxixi,EI730,jEwtH,DAwhk,b4jXr,0.0,&DBRdu);*ataeF=
YItCk;goto X4nOF;}cZ_aO:


if(EyW6C(jEwtH,DAwhk,JRB5m,hxixi,EI730,TfLg_,EgDTv,b4jXr,&P9_Ry,&OFDtk)){

if(OFDtk!=tlPiC){OFDtk+=Z4lSz;}Z4lSz+=P9_Ry;*ataeF=YItCk;goto X4nOF;}else{Z4lSz=
tlPiC;goto X4nOF;}X4nOF:*izcH4=Z4lSz;*Ib5FR=OFDtk;return(DBRdu);}
