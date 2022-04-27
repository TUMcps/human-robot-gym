
































#include <TypeIVRMLDecisionTree1B2.h>
#include <TypeIVRMLStep1Decisions.h>
#include <TypeIVRMLMath.h>
#include <TypeIVRMLDefinitions.h>
#include <TypeIVRMLStep1IntermediateTimeProfiles.h>
#include <TypeIVRMLStep1Profiles.h>
#include <RMLPositionFlags.h>


bool qPN_6::CcqKD(const double&iPzPj,const double&AQzqu,const double&k6sf4,const
 double&b4jXr,const double&DAwhk,const double&hxixi,const double&ldeCA,const 
double&HcINC,const double&Wln3F,const int&bocnd,FOwyh*ataeF,double*izcH4){bool 
DBRdu=false,jzHc3=false;double Z4lSz=0.0,TfLg_=iPzPj,JRB5m=AQzqu,jEwtH=k6sf4,
EgDTv=ldeCA,EI730=HcINC,ebyTQ=0.0;
if(n61tF(jEwtH)){goto aXI9K;}
else{
aepf8(&TfLg_,&JRB5m,&jEwtH,&EgDTv,&EI730);goto aXI9K;}aXI9K:
if(XX8Ub(jEwtH,DAwhk)){goto BT4JZ;}
else{
OuC3N(&Z4lSz,&TfLg_,&JRB5m,&jEwtH,DAwhk,b4jXr);goto BT4JZ;}BT4JZ:
if(ShcRT(jEwtH,JRB5m,hxixi,b4jXr)){goto IXb4z;}
else{
n9iwH(&Z4lSz,&TfLg_,&JRB5m,&jEwtH,b4jXr);
aepf8(&TfLg_,&JRB5m,&jEwtH,&EgDTv,&EI730);goto Plnbi;}IXb4z:
if(dGXca(JRB5m,hxixi)){goto eNc5E;}
else{goto Plnbi;}Plnbi:if(bocnd==RMLPositionFlags::GET_INTO_BOUNDARIES_FAST){
if(ofJY_(jEwtH,DAwhk,JRB5m,hxixi,b4jXr)){goto KIK47;}
else{goto x1u8n;}}else
{
if(P4hfD(jEwtH,DAwhk,JRB5m,-hxixi,b4jXr)){YJ3N_(&Z4lSz,&TfLg_,&JRB5m,&jEwtH,
hxixi,DAwhk,b4jXr);}
else{WlCk8(&Z4lSz,&TfLg_,&JRB5m,&jEwtH,hxixi,b4jXr);}goto eNc5E;}KIK47:
if(CEEIA(jEwtH,JRB5m,hxixi,b4jXr)){
Rr9wP(&Z4lSz,&TfLg_,&JRB5m,&jEwtH,hxixi,b4jXr);goto eNc5E;}
else{
Ut1Kv(&Z4lSz,&TfLg_,&JRB5m,&jEwtH,hxixi,b4jXr);goto eNc5E;}x1u8n:
if(A1qgc(jEwtH,DAwhk,JRB5m,hxixi,b4jXr)){goto Igroz;}
else{
Fj8b0(&Z4lSz,&TfLg_,&JRB5m,&jEwtH,hxixi,DAwhk,b4jXr);goto eNc5E;}Igroz:
if(XDwJW(jEwtH,DAwhk,JRB5m,hxixi,b4jXr)){
Ut1Kv(&Z4lSz,&TfLg_,&JRB5m,&jEwtH,hxixi,b4jXr);goto eNc5E;}
else{
qNBFJ(&Z4lSz,&TfLg_,&JRB5m,&jEwtH,hxixi,DAwhk,b4jXr);goto eNc5E;}eNc5E:ebyTQ=
fabs(jEwtH/b4jXr);if((OI259((JRB5m+0.5*jEwtH*ebyTQ),EI730,y1AP2))&&(OI259((TfLg_
+JRB5m*ebyTQ+0.5*jEwtH*VNvlY(ebyTQ)-B4Gkb(jEwtH)*qn3x8(ebyTQ)*b4jXr/6.0),EgDTv,
y1AP2))){Z4lSz+=ebyTQ;goto X4nOF;}
if(TAQ09(jEwtH,JRB5m,b4jXr)){

aepf8(&TfLg_,&JRB5m,&jEwtH,&EgDTv,&EI730);}
if(NMj7d(jEwtH)){jzHc3=false;goto Zg54w;}else{jzHc3=true;goto a6oD7;}Zg54w:
if(IpwId(jEwtH,JRB5m,EI730,hxixi,b4jXr)){goto I2Zn9;}else{
n9iwH(&Z4lSz,&TfLg_,&JRB5m,&jEwtH,b4jXr);if(JRB5m>hxixi){JRB5m=hxixi;
}goto LbUaV;}I2Zn9:
if(xrxMX(jEwtH,DAwhk,JRB5m,EI730,b4jXr)){goto AW7sB;}else{goto mgZby;}AW7sB:
if(E2PXV(jEwtH,DAwhk,JRB5m,EI730,TfLg_,EgDTv,b4jXr)){
n9iwH(&Z4lSz,&TfLg_,&JRB5m,&jEwtH,b4jXr);if(JRB5m>hxixi){JRB5m=hxixi;
}goto Bbeoh;}else{
z_T_D(&Z4lSz,TfLg_,EgDTv,JRB5m,hxixi,EI730,jEwtH,DAwhk,b4jXr,Wln3F,&DBRdu);*
ataeF=n1iUG;goto X4nOF;}Bbeoh:
if(DcUhz(jEwtH,DAwhk,JRB5m,b4jXr)){goto _tNlq;}else{goto _EWnI;}_tNlq:
if(Ne1C8(jEwtH,DAwhk,JRB5m,EI730,TfLg_,EgDTv,b4jXr)){
_3Rmz(&Z4lSz,TfLg_,EgDTv,JRB5m,hxixi,EI730,jEwtH,DAwhk,b4jXr,Wln3F,&DBRdu);*
ataeF=VieEo;goto X4nOF;}else{goto yqq8c;}yqq8c:
if(gqU3_(jEwtH,DAwhk,JRB5m,hxixi,EI730,TfLg_,EgDTv,b4jXr)){
_3Rmz(&Z4lSz,TfLg_,EgDTv,JRB5m,hxixi,EI730,jEwtH,DAwhk,b4jXr,Wln3F,&DBRdu);*
ataeF=VieEo;goto X4nOF;}else{goto Tol4F;}Tol4F:
if(RPg2y(jEwtH,DAwhk,JRB5m,EI730,TfLg_,EgDTv,b4jXr)){
Nr6Ki(&Z4lSz,TfLg_,EgDTv,JRB5m,hxixi,EI730,jEwtH,DAwhk,b4jXr,Wln3F,&DBRdu);if(
jzHc3){*ataeF=Gy8r_;}else{*ataeF=IVmDp;}goto X4nOF;}else{goto Qsgaz;}Qsgaz:
if(xOGeL(jEwtH,DAwhk,JRB5m,hxixi,EI730,TfLg_,EgDTv,b4jXr)){
Nr6Ki(&Z4lSz,TfLg_,EgDTv,JRB5m,hxixi,EI730,jEwtH,DAwhk,b4jXr,Wln3F,&DBRdu);if(
jzHc3){*ataeF=Gy8r_;}else{*ataeF=IVmDp;}goto X4nOF;}else{Z4lSz=tlPiC;goto X4nOF;
}_EWnI:
if(fwSUt(jEwtH,DAwhk,JRB5m,EI730,TfLg_,EgDTv,b4jXr)){
_3Rmz(&Z4lSz,TfLg_,EgDTv,JRB5m,hxixi,EI730,jEwtH,DAwhk,b4jXr,Wln3F,&DBRdu);*
ataeF=VieEo;goto X4nOF;}else{goto mv8_n;}mv8_n:
if(kLx4I(jEwtH,DAwhk,JRB5m,hxixi,EI730,TfLg_,EgDTv,b4jXr)){
_3Rmz(&Z4lSz,TfLg_,EgDTv,JRB5m,hxixi,EI730,jEwtH,DAwhk,b4jXr,Wln3F,&DBRdu);*
ataeF=VieEo;goto X4nOF;}else{Z4lSz=tlPiC;goto X4nOF;}mgZby:
if(KUS7o(jEwtH,JRB5m,EI730,TfLg_,EgDTv,b4jXr)){
n9iwH(&Z4lSz,&TfLg_,&JRB5m,&jEwtH,b4jXr);if(JRB5m>hxixi){JRB5m=hxixi;
}goto veDaH;}else{goto Iinur;}veDaH:
if(R1dyY(jEwtH,DAwhk,JRB5m,b4jXr)){goto RdBSH;}else{goto _nCaA;}RdBSH:
if(Sngn1(jEwtH,DAwhk,JRB5m,EI730,TfLg_,EgDTv,b4jXr)){goto evhJO;}else{goto gm9u0
;}evhJO:
if(K4n3U(jEwtH,DAwhk,JRB5m,EI730,TfLg_,EgDTv,b4jXr)){
gu1ug(&Z4lSz,TfLg_,EgDTv,JRB5m,hxixi,EI730,jEwtH,DAwhk,b4jXr,Wln3F,&DBRdu);*
ataeF=LVHAx;goto X4nOF;}else{
_3Rmz(&Z4lSz,TfLg_,EgDTv,JRB5m,hxixi,EI730,jEwtH,DAwhk,b4jXr,Wln3F,&DBRdu);*
ataeF=VieEo;goto X4nOF;}gm9u0:
if(kg15G(jEwtH,DAwhk,JRB5m,hxixi,EI730,TfLg_,EgDTv,b4jXr)){
gu1ug(&Z4lSz,TfLg_,EgDTv,JRB5m,hxixi,EI730,jEwtH,DAwhk,b4jXr,Wln3F,&DBRdu);*
ataeF=LVHAx;goto X4nOF;}else{goto yqq8c;}_nCaA:
if(hvltn(DAwhk,EI730,b4jXr)){goto FSUd3;}else{goto vvcDk;}FSUd3:
if(OyS6O(jEwtH,DAwhk,JRB5m,EI730,TfLg_,EgDTv,b4jXr)){goto evhJO;}else{goto Lerqv
;}Lerqv:
if(lebHt(jEwtH,DAwhk,JRB5m,hxixi,EI730,TfLg_,EgDTv,b4jXr)){
gu1ug(&Z4lSz,TfLg_,EgDTv,JRB5m,hxixi,EI730,jEwtH,DAwhk,b4jXr,Wln3F,&DBRdu);*
ataeF=LVHAx;goto X4nOF;}else{goto hgsS5;}hgsS5:
if(om8ol(jEwtH,DAwhk,JRB5m,hxixi,EI730,TfLg_,EgDTv,b4jXr)){goto evhJO;}else{
Z4lSz=tlPiC;goto X4nOF;}vvcDk:
if(vnQ65(jEwtH,JRB5m,EI730,TfLg_,EgDTv,b4jXr)){
gu1ug(&Z4lSz,TfLg_,EgDTv,JRB5m,hxixi,EI730,jEwtH,DAwhk,b4jXr,Wln3F,&DBRdu);if(
jzHc3){*ataeF=cvsfI;}else{*ataeF=LVHAx;}goto X4nOF;}else{goto mHCZP;}mHCZP:
if(c42JT(jEwtH,DAwhk,JRB5m,hxixi,EI730,TfLg_,EgDTv,b4jXr)){
gu1ug(&Z4lSz,TfLg_,EgDTv,JRB5m,hxixi,EI730,jEwtH,DAwhk,b4jXr,Wln3F,&DBRdu);if(
jzHc3){*ataeF=cvsfI;}else{*ataeF=LVHAx;}goto X4nOF;}else{Z4lSz=tlPiC;goto X4nOF;
}Iinur:
if(g45vp(jEwtH,DAwhk,JRB5m,hxixi,EI730,TfLg_,EgDTv,b4jXr)){
rXwpz(&Z4lSz,TfLg_,EgDTv,JRB5m,hxixi,EI730,jEwtH,DAwhk,b4jXr,Wln3F,&DBRdu);*
ataeF=YItCk;if(DBRdu){
z_T_D(&Z4lSz,TfLg_,EgDTv,JRB5m,hxixi,EI730,jEwtH,DAwhk,b4jXr,Wln3F,&DBRdu);*
ataeF=n1iUG;}goto X4nOF;}else{
z_T_D(&Z4lSz,TfLg_,EgDTv,JRB5m,hxixi,EI730,jEwtH,DAwhk,b4jXr,Wln3F,&DBRdu);*
ataeF=n1iUG;if(DBRdu){
rXwpz(&Z4lSz,TfLg_,EgDTv,JRB5m,hxixi,EI730,jEwtH,DAwhk,b4jXr,Wln3F,&DBRdu);*
ataeF=YItCk;}goto X4nOF;}LbUaV:
if(_DZiP(jEwtH,DAwhk,JRB5m,EI730,b4jXr)){goto M4tKz;}else{goto Trefr;}M4tKz:
if(Sqxp3(DAwhk,EI730,b4jXr)){goto krNgI;}else{goto RvERi;}krNgI:
if(Hiv6b(jEwtH,DAwhk,JRB5m,EI730,TfLg_,EgDTv,b4jXr)){
CQwwl(&Z4lSz,TfLg_,EgDTv,JRB5m,hxixi,EI730,jEwtH,DAwhk,b4jXr,Wln3F,&DBRdu);if(
jzHc3){*ataeF=CWXWH;}else{*ataeF=KELeG;}goto X4nOF;}else{goto CPbAb;}CPbAb:
if(r2Pca(jEwtH,DAwhk,JRB5m,hxixi,EI730,TfLg_,EgDTv,b4jXr)){
CQwwl(&Z4lSz,TfLg_,EgDTv,JRB5m,hxixi,EI730,jEwtH,DAwhk,b4jXr,Wln3F,&DBRdu);if(
jzHc3){*ataeF=CWXWH;}else{*ataeF=KELeG;}goto X4nOF;}else{goto Tol4F;}RvERi:
if(SEljH(jEwtH,DAwhk,JRB5m,EI730,TfLg_,EgDTv,b4jXr)){
CQwwl(&Z4lSz,TfLg_,EgDTv,JRB5m,hxixi,EI730,jEwtH,DAwhk,b4jXr,Wln3F,&DBRdu);if(
jzHc3){*ataeF=CWXWH;}else{*ataeF=KELeG;}goto X4nOF;}else{goto tkRfG;}tkRfG:
if(iwaWv(jEwtH,DAwhk,JRB5m,hxixi,EI730,TfLg_,EgDTv,b4jXr)){
CQwwl(&Z4lSz,TfLg_,EgDTv,JRB5m,hxixi,EI730,jEwtH,DAwhk,b4jXr,Wln3F,&DBRdu);if(
jzHc3){*ataeF=CWXWH;}else{*ataeF=KELeG;}goto X4nOF;}else{Z4lSz=tlPiC;goto X4nOF;
}Trefr:
if(PMwsB(jEwtH,DAwhk,JRB5m,b4jXr)){goto fTDv0;}else{goto vvcDk;}fTDv0:
if(qO4bH(jEwtH,DAwhk,JRB5m,EI730,TfLg_,EgDTv,b4jXr)){
gu1ug(&Z4lSz,TfLg_,EgDTv,JRB5m,hxixi,EI730,jEwtH,DAwhk,b4jXr,Wln3F,&DBRdu);if(
jzHc3){*ataeF=cvsfI;}else{*ataeF=LVHAx;}goto X4nOF;}else{goto KdScT;}KdScT:
if(Oqm0M(jEwtH,DAwhk,JRB5m,hxixi,EI730,TfLg_,EgDTv,b4jXr)){
gu1ug(&Z4lSz,TfLg_,EgDTv,JRB5m,hxixi,EI730,jEwtH,DAwhk,b4jXr,Wln3F,&DBRdu);if(
jzHc3){*ataeF=cvsfI;}else{*ataeF=LVHAx;}*ataeF=LVHAx;goto X4nOF;}else{goto M4tKz
;}a6oD7:
if(ay0sl(jEwtH,JRB5m,EI730,b4jXr)){goto m9Hl0;}else{goto xSib_;}m9Hl0:
if(uM3fA(jEwtH,DAwhk,JRB5m,EI730,b4jXr)){goto y7wJq;}else{goto aZUu8;}y7wJq:
if(ltkxg(jEwtH,DAwhk,JRB5m,EI730,TfLg_,EgDTv,b4jXr)){
giFax(&Z4lSz,TfLg_,EgDTv,JRB5m,hxixi,EI730,jEwtH,DAwhk,b4jXr,Wln3F,&DBRdu);*
ataeF=n1iUG;
goto X4nOF;}else{goto M4tKz;}aZUu8:
if(D7yxU(jEwtH,JRB5m,EI730,TfLg_,EgDTv,b4jXr)){goto fTngv;}else{goto LbUaV;}
fTngv:
if(snMmx(jEwtH,DAwhk,JRB5m,hxixi,EI730,TfLg_,EgDTv,b4jXr)){
OnI6t(&Z4lSz,TfLg_,EgDTv,JRB5m,hxixi,EI730,jEwtH,DAwhk,b4jXr,Wln3F,&DBRdu);*
ataeF=YItCk;if(DBRdu){
giFax(&Z4lSz,TfLg_,EgDTv,JRB5m,hxixi,EI730,jEwtH,DAwhk,b4jXr,Wln3F,&DBRdu);*
ataeF=n1iUG;}goto X4nOF;}else{
giFax(&Z4lSz,TfLg_,EgDTv,JRB5m,hxixi,EI730,jEwtH,DAwhk,b4jXr,Wln3F,&DBRdu);*
ataeF=n1iUG;if(DBRdu){
OnI6t(&Z4lSz,TfLg_,EgDTv,JRB5m,hxixi,EI730,jEwtH,DAwhk,b4jXr,Wln3F,&DBRdu);*
ataeF=YItCk;}goto X4nOF;}xSib_:
if(rLgKx(jEwtH,DAwhk,JRB5m,EI730,b4jXr)){goto ujKCW;}else{goto iiKT8;}ujKCW:
if(EqPQU(jEwtH,DAwhk,JRB5m,EI730,b4jXr)){goto Trefr;}else{goto CHO5U;}CHO5U:
if(TcZKW(DAwhk,EI730,b4jXr)){goto YTv_U;}else{goto vvcDk;}YTv_U:
if(XlBm5(jEwtH,DAwhk,JRB5m,EI730,TfLg_,EgDTv,b4jXr)){
gu1ug(&Z4lSz,TfLg_,EgDTv,JRB5m,hxixi,EI730,jEwtH,DAwhk,b4jXr,Wln3F,&DBRdu);if(
jzHc3){*ataeF=cvsfI;}else{*ataeF=LVHAx;}goto X4nOF;}else{goto P7cmu;}P7cmu:
if(l7hIc(jEwtH,DAwhk,JRB5m,hxixi,EI730,TfLg_,EgDTv,b4jXr)){
gu1ug(&Z4lSz,TfLg_,EgDTv,JRB5m,hxixi,EI730,jEwtH,DAwhk,b4jXr,Wln3F,&DBRdu);if(
jzHc3){*ataeF=cvsfI;}else{*ataeF=LVHAx;}goto X4nOF;}else{goto iiKT8;}iiKT8:
if(JvrEy(jEwtH,DAwhk,JRB5m,b4jXr)){goto Fk6eX;}else{goto _EWnI;}Fk6eX:
if(k64nO(jEwtH,DAwhk,JRB5m,EI730,TfLg_,EgDTv,b4jXr)){
_3Rmz(&Z4lSz,TfLg_,EgDTv,JRB5m,hxixi,EI730,jEwtH,DAwhk,b4jXr,Wln3F,&DBRdu);if(
jzHc3){*ataeF=Tuwfw;}else{*ataeF=VieEo;}goto X4nOF;}else{goto gWLXe;}gWLXe:
if(DHN7R(jEwtH,DAwhk,JRB5m,hxixi,EI730,TfLg_,EgDTv,b4jXr)){
_3Rmz(&Z4lSz,TfLg_,EgDTv,JRB5m,hxixi,EI730,jEwtH,DAwhk,b4jXr,Wln3F,&DBRdu);if(
jzHc3){*ataeF=Tuwfw;}else{*ataeF=VieEo;}goto X4nOF;}else{goto Tol4F;}X4nOF:*
izcH4=Z4lSz;return(DBRdu);}
