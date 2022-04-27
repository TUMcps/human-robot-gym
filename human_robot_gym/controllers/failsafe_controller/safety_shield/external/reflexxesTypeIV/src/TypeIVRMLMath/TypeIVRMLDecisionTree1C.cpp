
































#include <TypeIVRMLDecisionTree1C.h>
#include <TypeIVRMLStep1Decisions.h>
#include <TypeIVRMLMath.h>
#include <TypeIVRMLStep1IntermediateTimeProfiles.h>
#include <TypeIVRMLStep1Profiles.h>
#include <RMLPositionFlags.h>


bool qPN_6::xlKsn(const double&iPzPj,const double&AQzqu,const double&k6sf4,const
 double&b4jXr,const double&DAwhk,const double&hxixi,const double&ldeCA,const 
double&HcINC,const FOwyh&srTPK,const FOwyh&_U3eu,const int&bocnd,double*yeziv){
bool DBRdu=false;double Z4lSz=0.0,TfLg_=iPzPj,JRB5m=AQzqu,jEwtH=k6sf4,EgDTv=
ldeCA,EI730=HcINC;
if(X0vCV(jEwtH)){goto pHHHr;}
else{
aepf8(&TfLg_,&JRB5m,&jEwtH,&EgDTv,&EI730);goto pHHHr;}pHHHr:
if(EcoOx(jEwtH,DAwhk)){goto IsNKf;}
else{
OuC3N(&Z4lSz,&TfLg_,&JRB5m,&jEwtH,DAwhk,b4jXr);goto IsNKf;}IsNKf:
if(Pfc0r(jEwtH,JRB5m,hxixi,b4jXr)){goto Zz746;}
else{
n9iwH(&Z4lSz,&TfLg_,&JRB5m,&jEwtH,b4jXr);
aepf8(&TfLg_,&JRB5m,&jEwtH,&EgDTv,&EI730);goto rDh5z;}Zz746:
if(VRv9s(JRB5m,hxixi)){goto hb_uG;}
else{goto rDh5z;}rDh5z:if(bocnd==RMLPositionFlags::GET_INTO_BOUNDARIES_FAST){
if(mzLXC(jEwtH,DAwhk,JRB5m,hxixi,b4jXr)){goto h9Fxx;}
else{goto wHa7K;}}else
{
if(U5VcP(jEwtH,DAwhk,JRB5m,-hxixi,b4jXr)){YJ3N_(&Z4lSz,&TfLg_,&JRB5m,&jEwtH,
hxixi,DAwhk,b4jXr);}
else{WlCk8(&Z4lSz,&TfLg_,&JRB5m,&jEwtH,hxixi,b4jXr);}goto hb_uG;}h9Fxx:
if(JqArP(jEwtH,JRB5m,hxixi,b4jXr)){
Rr9wP(&Z4lSz,&TfLg_,&JRB5m,&jEwtH,hxixi,b4jXr);goto hb_uG;}
else{
Ut1Kv(&Z4lSz,&TfLg_,&JRB5m,&jEwtH,hxixi,b4jXr);goto hb_uG;}wHa7K:
if(t8AAE(jEwtH,DAwhk,JRB5m,hxixi,b4jXr)){goto e7iOS;}
else{
Fj8b0(&Z4lSz,&TfLg_,&JRB5m,&jEwtH,hxixi,DAwhk,b4jXr);goto hb_uG;}e7iOS:
if(Uxx21(jEwtH,DAwhk,JRB5m,hxixi,b4jXr)){
Ut1Kv(&Z4lSz,&TfLg_,&JRB5m,&jEwtH,hxixi,b4jXr);goto hb_uG;}
else{
qNBFJ(&Z4lSz,&TfLg_,&JRB5m,&jEwtH,hxixi,DAwhk,b4jXr);goto hb_uG;}hb_uG:
if(pRuPw(EI730)){goto dZBlu;}else{

aepf8(&TfLg_,&JRB5m,&jEwtH,&EgDTv,&EI730);goto dZBlu;}dZBlu:
if(X6bhN(jEwtH)){goto cEVqD;}else{
JY88s(&Z4lSz,&TfLg_,&JRB5m,&jEwtH,b4jXr);if(JRB5m<-hxixi){JRB5m=-hxixi;
}goto psFvm;}cEVqD:
if(Jx7uV(srTPK,_U3eu)){n9iwH(&Z4lSz,&TfLg_,&JRB5m,&jEwtH,b4jXr);if(JRB5m>hxixi){
JRB5m=hxixi;
}goto psFvm;}else{goto psFvm;}psFvm:
if(J0Fq0(jEwtH,DAwhk,JRB5m,hxixi,b4jXr)){goto JiR1m;}else{goto WYvNr;}JiR1m:
if(zhkRv(jEwtH,DAwhk,JRB5m,hxixi,EI730,b4jXr)){goto P3PGI;}else{goto stXxn;}
P3PGI:
if(Cr43r(jEwtH,DAwhk,JRB5m,hxixi,EI730,TfLg_,EgDTv,b4jXr)){
nkINu(&Z4lSz,&TfLg_,&JRB5m,&jEwtH,hxixi,DAwhk,b4jXr);
dY7Ow(&Z4lSz,TfLg_,EgDTv,JRB5m,EI730,jEwtH,b4jXr,&DBRdu);goto X4nOF;}else{goto 
IfA_k;}IfA_k:
if(BNLYk(jEwtH,DAwhk,JRB5m,hxixi,EI730,TfLg_,EgDTv,b4jXr)){t61l2(&Z4lSz,TfLg_,
EgDTv,JRB5m,hxixi,EI730,jEwtH,DAwhk,b4jXr,&DBRdu);goto X4nOF;}else{VdEEd(&Z4lSz,
TfLg_,EgDTv,JRB5m,hxixi,EI730,jEwtH,DAwhk,b4jXr,&DBRdu);goto X4nOF;}stXxn:
if(wlfXo(jEwtH,DAwhk,JRB5m,hxixi,EI730,TfLg_,EgDTv,b4jXr)){
nkINu(&Z4lSz,&TfLg_,&JRB5m,&jEwtH,hxixi,DAwhk,b4jXr);
kvFss(&Z4lSz,TfLg_,EgDTv,JRB5m,EI730,jEwtH,DAwhk,b4jXr,&DBRdu);goto X4nOF;}else{
goto yP7PC;}yP7PC:
if(nAcyu(jEwtH,DAwhk,JRB5m,hxixi,EI730,TfLg_,EgDTv,b4jXr)){OAqlq(&Z4lSz,TfLg_,
EgDTv,JRB5m,hxixi,EI730,jEwtH,DAwhk,b4jXr,&DBRdu);goto X4nOF;}else{goto Ellvr;}
Ellvr:
if(ADtkh(jEwtH,DAwhk,JRB5m,EI730,b4jXr)){goto IfA_k;}else{goto esvcL;}esvcL:
if(mayMb(jEwtH,DAwhk,JRB5m,hxixi,EI730,TfLg_,EgDTv,b4jXr)){zcCeA(&Z4lSz,TfLg_,
EgDTv,JRB5m,hxixi,EI730,jEwtH,DAwhk,b4jXr,&DBRdu);if(DBRdu){VdEEd(&Z4lSz,TfLg_,
EgDTv,JRB5m,hxixi,EI730,jEwtH,DAwhk,b4jXr,&DBRdu);}goto X4nOF;}else{VdEEd(&Z4lSz
,TfLg_,EgDTv,JRB5m,hxixi,EI730,jEwtH,DAwhk,b4jXr,&DBRdu);if(DBRdu){zcCeA(&Z4lSz,
TfLg_,EgDTv,JRB5m,hxixi,EI730,jEwtH,DAwhk,b4jXr,&DBRdu);}goto X4nOF;}WYvNr:
if(QEfD1(jEwtH,DAwhk,JRB5m,hxixi,EI730,b4jXr)){goto sar_R;}else{goto Ch8lz;}
sar_R:
if(eltDG(jEwtH,JRB5m,hxixi,EI730,TfLg_,EgDTv,b4jXr)){
QBBJJ(&Z4lSz,&TfLg_,&JRB5m,&jEwtH,hxixi,b4jXr);
dY7Ow(&Z4lSz,TfLg_,EgDTv,JRB5m,EI730,jEwtH,b4jXr,&DBRdu);goto X4nOF;}else{VdEEd(
&Z4lSz,TfLg_,EgDTv,JRB5m,hxixi,EI730,jEwtH,DAwhk,b4jXr,&DBRdu);goto X4nOF;}Ch8lz
:
if(QyAWc(jEwtH,DAwhk,JRB5m,hxixi,EI730,TfLg_,EgDTv,b4jXr)){
QBBJJ(&Z4lSz,&TfLg_,&JRB5m,&jEwtH,hxixi,b4jXr);
kvFss(&Z4lSz,TfLg_,EgDTv,JRB5m,EI730,jEwtH,DAwhk,b4jXr,&DBRdu);goto X4nOF;}else{
goto esvcL;}X4nOF:*yeziv=Z4lSz;return(DBRdu);}
