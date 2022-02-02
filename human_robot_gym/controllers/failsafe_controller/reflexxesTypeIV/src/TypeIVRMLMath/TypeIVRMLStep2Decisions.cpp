
































#include <TypeIVRMLStep1Decisions.h>
#include <TypeIVRMLStep2Decisions.h>
#include <TypeIVRMLMath.h>
#include <TypeIVRMLDefinitions.h>
#include <TypeIVRMLABK.h>
#include <TypeIVRMLStep2RootFunctions.h>
#include <math.h>








bool qPN_6::qetKs(const double&k6sf4){return(L5k8N(k6sf4));}


bool qPN_6::y6irZ(const double&k6sf4,const double&DAwhk){
return(t3fQZ(k6sf4,DAwhk));}


bool qPN_6::VgO1x(const double&k6sf4,const double&AQzqu,const double&hxixi,const
 double&b4jXr){
return(RNszr(k6sf4,AQzqu,hxixi,b4jXr));}


bool qPN_6::fERyr(const double&AQzqu,const double&hxixi){
return(KbW6_(AQzqu,hxixi));}


bool qPN_6::Afvpl(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&b4jXr){

return(wKs7P(k6sf4,DAwhk,AQzqu,hxixi,b4jXr));}


bool qPN_6::zo6a4(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&b4jXr){return(P90PB(k6sf4,DAwhk,AQzqu,HcINC,b4jXr));}


bool qPN_6::nj7BP(const double&k6sf4,const double&AQzqu,const double&hxixi,const
 double&b4jXr){

return(bSu3N(k6sf4,AQzqu,hxixi,b4jXr));}


bool qPN_6::cUEs5(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&b4jXr){

return(y15i4(k6sf4,DAwhk,AQzqu,hxixi,b4jXr));}


bool qPN_6::BthnT(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&b4jXr){

return(CpvSN(k6sf4,DAwhk,AQzqu,hxixi,b4jXr));}


bool qPN_6::WidGl(const double&k6sf4,const double&AQzqu,const double&HcINC,const
 double&hxixi,const double&b4jXr){
return(_3B72(k6sf4,AQzqu,HcINC,hxixi,b4jXr));}


bool qPN_6::ZF20r(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&b4jXr){
return(P90PB(k6sf4,DAwhk,AQzqu,HcINC,b4jXr));}


bool qPN_6::xqksy(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr,const 
double&hxixi,const double&rTEdP,const double&SynchronizationTime){
double HB9ew=0.0
,zRNIW=0.0
,nwQQL=0.0
,cbBNZ=0.0
,lURFQ=0.0
,Vsamh=0.0
,FYkEp=0.0
,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4;
HB9ew=(DAwhk-S8tqs)/b4jXr;lURFQ=S8tqs*HB9ew+0.5*b4jXr*VNvlY(HB9ew);
nwQQL=DAwhk/b4jXr;FYkEp=0.5*VNvlY(DAwhk)/b4jXr;
Vsamh=HcINC-a84tu-lURFQ-FYkEp;zRNIW=Vsamh/DAwhk;
cbBNZ=SynchronizationTime-rTEdP-HB9ew-zRNIW-nwQQL;
u4XFm+=a84tu*HB9ew+0.5*S8tqs*VNvlY(HB9ew)+b4jXr*qn3x8(HB9ew)/6.0;a84tu+=lURFQ;
S8tqs=DAwhk;
u4XFm+=a84tu*zRNIW+0.5*S8tqs*VNvlY(zRNIW);
a84tu+=Vsamh;S8tqs=DAwhk;
u4XFm+=a84tu*nwQQL+0.5*S8tqs*VNvlY(nwQQL)-b4jXr*qn3x8(nwQQL)/6.0;
a84tu=HcINC;S8tqs=0.0;
u4XFm+=a84tu*cbBNZ;
if(HcINC>0.0){return(u4XFm<=ldeCA+hxixi*EVXXF*(1.0-(hxixi-HcINC)/hxixi));}else{
return(u4XFm<=ldeCA);}}


bool qPN_6::n1jSJ(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&b4jXr,const double&rTEdP,const double&
SynchronizationTime){

double HB9ew=0.0
,zRNIW=0.0
,nwQQL=0.0
,WT1rZ=0.0
,PbPkG=0.0
,lURFQ=0.0
,rT061=0.0
,FYkEp=0.0
,XxdWk=0.0
,NIjjJ=0.0;

HB9ew=(DAwhk-k6sf4)/b4jXr;
nwQQL=WT1rZ=PbPkG=DAwhk/b4jXr;lURFQ=k6sf4*HB9ew+0.5*b4jXr*VNvlY(HB9ew);FYkEp=0.5
*VNvlY(DAwhk)/b4jXr;XxdWk=NIjjJ=-0.5*VNvlY(DAwhk)/b4jXr;
rT061=HcINC-AQzqu-lURFQ-FYkEp-XxdWk-NIjjJ;zRNIW=rT061/DAwhk;return((rTEdP+HB9ew+
zRNIW+nwQQL+WT1rZ+PbPkG)>=SynchronizationTime);}


bool qPN_6::EkfQK(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr,const 
double&rTEdP,const double&SynchronizationTime){

double HB9ew=0.0
,zRNIW=0.0
,nwQQL=0.0
,cbBNZ=0.0
,WT1rZ=0.0
,PbPkG=0.0
,lURFQ=0.0
,Vsamh=0.0
,FYkEp=0.0
,XxdWk=0.0
,NIjjJ=0.0
,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4;
HB9ew=(DAwhk-S8tqs)/b4jXr;lURFQ=S8tqs*HB9ew+0.5*b4jXr*VNvlY(HB9ew);
nwQQL=WT1rZ=PbPkG=DAwhk/b4jXr;FYkEp=0.5*VNvlY(DAwhk)/b4jXr;XxdWk=NIjjJ=-0.5*
VNvlY(DAwhk)/b4jXr;
Vsamh=HcINC-a84tu-lURFQ-FYkEp-XxdWk-NIjjJ;zRNIW=Vsamh/DAwhk;
cbBNZ=SynchronizationTime-rTEdP-HB9ew-zRNIW-nwQQL-WT1rZ-PbPkG;
u4XFm+=a84tu*HB9ew+0.5*S8tqs*VNvlY(HB9ew)+b4jXr*qn3x8(HB9ew)/6.0;a84tu+=lURFQ;
S8tqs=DAwhk;
u4XFm+=a84tu*zRNIW+0.5*S8tqs*VNvlY(zRNIW);
a84tu+=Vsamh;S8tqs=DAwhk;
u4XFm+=a84tu*nwQQL+0.5*S8tqs*VNvlY(nwQQL)-b4jXr*qn3x8(nwQQL)/6.0;
a84tu+=FYkEp;S8tqs=(0xa33+2991-0x15e2);
u4XFm+=a84tu*cbBNZ;
a84tu=a84tu;S8tqs=0.0;
u4XFm+=a84tu*WT1rZ+0.5*S8tqs*VNvlY(WT1rZ)-b4jXr*qn3x8(WT1rZ)/6.0;a84tu+=XxdWk;
S8tqs=-DAwhk;
u4XFm+=a84tu*PbPkG+0.5*S8tqs*VNvlY(PbPkG)+b4jXr*qn3x8(PbPkG)/6.0;
return(u4XFm<=ldeCA);}


bool qPN_6::rHU1O(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&b4jXr){
return(fwKhJ(k6sf4,DAwhk,AQzqu,HcINC,b4jXr));}


bool qPN_6::ZQzV_(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&b4jXr,const double&rTEdP,const double&
SynchronizationTime){
double EFQ3W=0.0
,HB9ew=0.0
,zRNIW=0.0
,nwQQL=0.0
,lURFQ=0.0
,Vsamh=0.0
,FYkEp=0.0
,a84tu=AQzqu;

EFQ3W=k6sf4/b4jXr;a84tu+=k6sf4*EFQ3W-0.5*b4jXr*VNvlY(EFQ3W);

HB9ew=nwQQL=DAwhk/b4jXr;lURFQ=FYkEp=0.5*VNvlY(DAwhk)/b4jXr;
Vsamh=HcINC-a84tu-lURFQ-FYkEp;zRNIW=Vsamh/DAwhk;return((rTEdP+EFQ3W+HB9ew+zRNIW+
nwQQL)>=SynchronizationTime);}


bool qPN_6::byDB4(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr,const 
double&rTEdP,const double&SynchronizationTime){double Qogm3=0.0,lmPpg=0.0,laktM=
0.0,TwXRm=0.0,EvE6x=0.0,TBVIO=0.0
,Tmhm2=0.0,iEiah=0.0,nDRrN=0.0,KSaj_=0.0,LQB6k=0.0,BIVJE=0.0,ntGsr=0.0;laktM=(
SynchronizationTime-rTEdP)*(SynchronizationTime-rTEdP);KSaj_=k6sf4*k6sf4;LQB6k=
b4jXr*b4jXr;BIVJE=0.1e1/LQB6k;EvE6x=DAwhk*DAwhk;Tmhm2=0.1e1/b4jXr;nDRrN=k6sf4*
DAwhk;lmPpg=LQB6k*b4jXr;Qogm3=pbQOc(KSaj_*LQB6k-0.2e1*nDRrN*LQB6k+0.4e1*EvE6x*
LQB6k-0.2e1*DAwhk*(SynchronizationTime-rTEdP)*lmPpg-0.2e1*lmPpg*AQzqu+0.2e1*
lmPpg*HcINC);TwXRm=pbQOc(0.2e1);iEiah=0.1e1/lmPpg;ntGsr=Qogm3*BIVJE;TBVIO=DAwhk*
laktM/0.2e1-KSaj_*k6sf4*BIVJE/0.6e1+KSaj_*DAwhk*BIVJE-0.3e1/0.2e1*k6sf4*EvE6x*
BIVJE-KSaj_*(SynchronizationTime-rTEdP)*Tmhm2/0.2e1+nDRrN*(SynchronizationTime-
rTEdP)*Tmhm2-0.3e1/0.2e1*EvE6x*(SynchronizationTime-rTEdP)*Tmhm2+iPzPj+(
SynchronizationTime-rTEdP)*AQzqu-DAwhk*AQzqu*Tmhm2+DAwhk*HcINC*Tmhm2-KSaj_*Qogm3
/TwXRm*iEiah/0.2e1+TwXRm*k6sf4*DAwhk*Qogm3*iEiah/0.2e1+TwXRm*DAwhk*(
SynchronizationTime-rTEdP)*Qogm3*BIVJE/0.2e1+TwXRm*AQzqu*ntGsr/0.2e1-TwXRm*HcINC
*ntGsr/0.2e1;return(TBVIO<=ldeCA);}


bool qPN_6::tCg8u(const double&k6sf4,const double&AQzqu,const double&HcINC,const
 double&b4jXr,const double&rTEdP,const double&SynchronizationTime){double a84tu=
AQzqu,S8tqs=k6sf4,StX1T=0.0
,m0JG3=0.0;

m0JG3=S8tqs/b4jXr;
StX1T=SynchronizationTime-rTEdP-m0JG3;
a84tu+=S8tqs*StX1T;S8tqs=S8tqs;
a84tu+=S8tqs*m0JG3-0.5*b4jXr*VNvlY(m0JG3);return(a84tu>HcINC);}


bool qPN_6::WxfSq(const double&k6sf4,const double&AQzqu,const double&HcINC,const
 double&iPzPj,const double&ldeCA,const double&b4jXr,const double&rTEdP,const 
double&SynchronizationTime){double fTK9I=0.0
,FrEi2=0.0
,cjYid=0.0
,EFQ3W=0.0
,FBPho=0.0
,LDLfE=0.0
,NJ168=0.0
,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4;
fTK9I=(b4jXr*(VNvlY(S8tqs)/b4jXr+2.0*a84tu-2.0*HcINC))/(2.0*S8tqs+2.0*b4jXr*
rTEdP-2.0*b4jXr*SynchronizationTime);
FrEi2=(S8tqs-fTK9I)/b4jXr;FBPho=S8tqs*FrEi2-0.5*b4jXr*VNvlY(FrEi2);
EFQ3W=fTK9I/b4jXr;NJ168=0.5*VNvlY(fTK9I)/b4jXr;
cjYid=SynchronizationTime-rTEdP-FrEi2-EFQ3W;LDLfE=fTK9I*cjYid;
u4XFm+=a84tu*FrEi2+0.5*S8tqs*VNvlY(FrEi2)-b4jXr*qn3x8(FrEi2)/6.0;a84tu+=FBPho;
S8tqs=fTK9I;
u4XFm+=a84tu*cjYid+0.5*S8tqs*VNvlY(cjYid);a84tu+=LDLfE;S8tqs=fTK9I;
u4XFm+=a84tu*EFQ3W+0.5*S8tqs*VNvlY(EFQ3W)-b4jXr*qn3x8(EFQ3W)/6.0;return(u4XFm<=
ldeCA);}


bool qPN_6::I7QAT(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr,const 
double&rTEdP,const double&SynchronizationTime){double fTK9I=0.0
,HB9ew=0.0
,FrEi2=0.0
,cjYid=0.0
,EFQ3W=0.0
,lURFQ=0.0
,FBPho=0.0
,LDLfE=0.0
,NJ168=0.0
,rDoKh=0.0,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4;rDoKh=(2.0*S8tqs-4.0*DAwhk-2.0*
b4jXr*rTEdP+2.0*b4jXr*SynchronizationTime);if(rDoKh==0.0){rDoKh=T68ug;}
fTK9I=(b4jXr*((1.0*VNvlY(S8tqs))/b4jXr-(2.0*VNvlY(DAwhk))/b4jXr-2.0*a84tu+2.0*
HcINC))/rDoKh;if(fTK9I>DAwhk){fTK9I=DAwhk;}if(fTK9I<0.0){fTK9I=0.0;}
HB9ew=(DAwhk-S8tqs)/b4jXr;lURFQ=S8tqs*HB9ew+0.5*b4jXr*VNvlY(HB9ew);
FrEi2=(DAwhk-fTK9I)/b4jXr;FBPho=DAwhk*FrEi2-0.5*b4jXr*VNvlY(FrEi2);
EFQ3W=fTK9I/b4jXr;NJ168=0.5*VNvlY(fTK9I)/b4jXr;
cjYid=SynchronizationTime-rTEdP-HB9ew-FrEi2-EFQ3W;LDLfE=cjYid*fTK9I;
u4XFm+=a84tu*HB9ew+0.5*S8tqs*VNvlY(HB9ew)+b4jXr*qn3x8(HB9ew)/6.0;a84tu+=lURFQ;
S8tqs=DAwhk;
u4XFm+=a84tu*FrEi2+0.5*S8tqs*VNvlY(FrEi2)-b4jXr*qn3x8(FrEi2)/6.0;a84tu+=FBPho;
S8tqs=fTK9I;
u4XFm+=a84tu*cjYid+0.5*S8tqs*VNvlY(cjYid);a84tu+=LDLfE;S8tqs=fTK9I;
u4XFm+=a84tu*EFQ3W+0.5*S8tqs*VNvlY(EFQ3W)-b4jXr*qn3x8(EFQ3W)/6.0;return(u4XFm<=
ldeCA);}


bool qPN_6::wTqNM(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&b4jXr,const double&rTEdP,const double&
SynchronizationTime){
double nhG1V=0.0
,HB9ew=0.0
,EFQ3W=0.0
,YDL_o=0.0
,lURFQ=0.0
,NJ168=0.0;

HB9ew=(DAwhk-k6sf4)/b4jXr;lURFQ=k6sf4*HB9ew+0.5*b4jXr*VNvlY(HB9ew);
EFQ3W=DAwhk/b4jXr;NJ168=0.5*VNvlY(DAwhk)/b4jXr;
nhG1V=SynchronizationTime-rTEdP-HB9ew-EFQ3W;YDL_o=k6sf4*nhG1V;return((AQzqu+
YDL_o+lURFQ+NJ168)>HcINC);}


bool qPN_6::ljhaG(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr,const 
double&rTEdP,const double&SynchronizationTime){
double lKqrZ=0.0;

lKqrZ=(0.08333333333333333333333333333333333*qn3x8(k6sf4)+VNvlY(k6sf4)*(-0.5*
DAwhk-0.25*b4jXr*rTEdP+0.25*b4jXr*SynchronizationTime)+k6sf4*(0.5*VNvlY(DAwhk)+
0.5*b4jXr*AQzqu-0.5*b4jXr*HcINC)+b4jXr*(VNvlY(DAwhk)*(0.5*rTEdP-0.5*
SynchronizationTime)+DAwhk*(-1.0*AQzqu+1.0*HcINC)+b4jXr*(-0.5*rTEdP*AQzqu+0.5*
SynchronizationTime*AQzqu-0.5*rTEdP*HcINC+0.5*SynchronizationTime*HcINC)))/VNvlY
(b4jXr);return((iPzPj+lKqrZ)<=ldeCA);}


bool qPN_6::VC639(const double&k6sf4,const double&AQzqu,const double&HcINC,const
 double&iPzPj,const double&ldeCA,const double&b4jXr,const double&rTEdP,const 
double&SynchronizationTime){double fTK9I=0.0
,iqKv9=0.0
,cjYid=0.0
,EFQ3W=0.0
,jmlSk=0.0
,LDLfE=0.0
,NJ168=0.0
,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4;
fTK9I=0.5*S8tqs-0.5*b4jXr*rTEdP+0.5*b4jXr*SynchronizationTime-0.25*b4jXr*pbQOc((
-4.0*VNvlY(S8tqs))/VNvlY(b4jXr)-(8.0*S8tqs*rTEdP)/b4jXr+4.0*VNvlY(rTEdP)+(8.0*
S8tqs*SynchronizationTime)/b4jXr-8.0*rTEdP*SynchronizationTime+4.0*VNvlY(
SynchronizationTime)+(16.0*a84tu)/b4jXr-(16.0*HcINC)/b4jXr);
iqKv9=(fTK9I-S8tqs)/b4jXr;jmlSk=S8tqs*iqKv9+0.5*b4jXr*VNvlY(iqKv9);
EFQ3W=fTK9I/b4jXr;NJ168=0.5*VNvlY(fTK9I)/b4jXr;
cjYid=SynchronizationTime-rTEdP-iqKv9-EFQ3W;LDLfE=fTK9I*cjYid;
u4XFm+=a84tu*iqKv9+0.5*S8tqs*VNvlY(iqKv9)+b4jXr*qn3x8(iqKv9)/6.0;a84tu+=jmlSk;
S8tqs=fTK9I;
u4XFm+=a84tu*cjYid+0.5*S8tqs*VNvlY(cjYid);a84tu+=LDLfE;S8tqs=fTK9I;
u4XFm+=a84tu*EFQ3W+0.5*S8tqs*VNvlY(EFQ3W)-b4jXr*qn3x8(EFQ3W)/6.0;return(u4XFm<=
ldeCA);}


bool qPN_6::coar7(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&b4jXr,const double&rTEdP,const double&
SynchronizationTime){return(wTqNM(k6sf4,DAwhk,AQzqu,HcINC,b4jXr,rTEdP,
SynchronizationTime));}


bool qPN_6::dqYWb(const double&k6sf4,const double&AQzqu,const double&HcINC,const
 double&iPzPj,const double&ldeCA,const double&b4jXr,const double&rTEdP,const 
double&SynchronizationTime){double cjYid=0.0
,UKSpI=0.0
,DVdae=0.0
,LDLfE=0.0
,WBuXo=0.0
,ri4IE=0.0
,L38Np=0.0
,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4;
L38Np=1.0*S8tqs+0.70710678118654752440084436210485*pbQOc(1.0*VNvlY(S8tqs)+2.0*
S8tqs*b4jXr*rTEdP-2.0*S8tqs*b4jXr*SynchronizationTime-2.0*b4jXr*a84tu+2.0*b4jXr*
HcINC);
UKSpI=(L38Np-S8tqs)/b4jXr;WBuXo=S8tqs*UKSpI+0.5*b4jXr*VNvlY(UKSpI);
DVdae=L38Np/b4jXr;ri4IE=0.5*VNvlY(L38Np)/b4jXr;
cjYid=SynchronizationTime-rTEdP-UKSpI-DVdae;LDLfE=cjYid*S8tqs;
u4XFm+=a84tu*cjYid+0.5*S8tqs*VNvlY(cjYid);a84tu+=LDLfE;S8tqs=S8tqs;
u4XFm+=a84tu*UKSpI+0.5*S8tqs*VNvlY(UKSpI)+b4jXr*qn3x8(UKSpI)/6.0;a84tu+=WBuXo;
S8tqs=L38Np;
u4XFm+=a84tu*DVdae+0.5*S8tqs*VNvlY(DVdae)-b4jXr*qn3x8(DVdae)/6.0;return(u4XFm<=
ldeCA);}


bool qPN_6::jzzAi(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr,const 
double&rTEdP,const double&SynchronizationTime){double cjYid=0.0
,LIlfP=0.0
,zAA9v=0.0
,rGqvR=0.0
,LDLfE=0.0
,vwyxx=0.0
,a4lTx=0.0
,Iqm8o=0.0
,rDoKh=0.0,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4;rDoKh=(2.0*S8tqs-2.0*DAwhk);if(
rDoKh==0.0){rDoKh=wNuTo;}
zAA9v=(VNvlY(S8tqs)/b4jXr-(4.0*S8tqs*DAwhk)/b4jXr+(2.0*VNvlY(DAwhk))/b4jXr-2.0*
S8tqs*rTEdP+2.0*S8tqs*SynchronizationTime+2.0*a84tu-2.0*HcINC)/rDoKh;a4lTx=zAA9v
*DAwhk;
LIlfP=(DAwhk-S8tqs)/b4jXr;vwyxx=S8tqs*LIlfP+0.5*b4jXr*VNvlY(LIlfP);
rGqvR=DAwhk/b4jXr;Iqm8o=0.5*VNvlY(DAwhk)/b4jXr;
cjYid=SynchronizationTime-rTEdP-LIlfP-zAA9v-rGqvR;LDLfE=cjYid*S8tqs;
u4XFm+=a84tu*cjYid+0.5*S8tqs*VNvlY(cjYid);a84tu+=LDLfE;S8tqs=S8tqs;
u4XFm+=a84tu*LIlfP+0.5*S8tqs*VNvlY(LIlfP)+b4jXr*qn3x8(LIlfP)/6.0;a84tu+=vwyxx;
S8tqs=DAwhk;
u4XFm+=a84tu*zAA9v+0.5*S8tqs*VNvlY(zAA9v);a84tu+=a4lTx;S8tqs=DAwhk;
u4XFm+=a84tu*rGqvR+0.5*S8tqs*VNvlY(rGqvR)-b4jXr*qn3x8(rGqvR)/6.0;return(u4XFm<=
ldeCA);}


bool qPN_6::ABNtV(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr,const 
double&rTEdP,const double&SynchronizationTime){double fTK9I=0.0
,iqKv9=0.0
,cjYid=0.0
,jcVDt=0.0
,wilOA=0.0
,jmlSk=0.0
,LDLfE=0.0
,KVYN5=0.0
,BhxbT=0.0
,rDoKh=0.0,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4;rDoKh=(2.0*S8tqs-4.0*DAwhk-2.0*
b4jXr*rTEdP+2.0*b4jXr*SynchronizationTime);if(rDoKh==0.0){rDoKh=T68ug;}
fTK9I=DAwhk+(1.0*VNvlY(S8tqs))/rDoKh-(2.0*S8tqs*DAwhk)/rDoKh+(2.0*pow(DAwhk,
(0xba6+1385-0x110d)))/rDoKh+(2.0*DAwhk*b4jXr*rTEdP)/rDoKh-(2.0*DAwhk*b4jXr*
SynchronizationTime)/rDoKh-(2.0*b4jXr*a84tu)/rDoKh+(2.0*b4jXr*HcINC)/rDoKh;
iqKv9=(fTK9I-S8tqs)/b4jXr;jmlSk=S8tqs*iqKv9+0.5*b4jXr*VNvlY(iqKv9);
jcVDt=(DAwhk-fTK9I)/b4jXr;KVYN5=fTK9I*jcVDt+0.5*b4jXr*VNvlY(jcVDt);
wilOA=DAwhk/b4jXr;BhxbT=0.5*VNvlY(DAwhk)/b4jXr;
cjYid=SynchronizationTime-rTEdP-iqKv9-jcVDt-wilOA;LDLfE=cjYid*fTK9I;
u4XFm+=a84tu*iqKv9+0.5*S8tqs*VNvlY(iqKv9)+b4jXr*qn3x8(iqKv9)/6.0;a84tu+=jmlSk;
S8tqs=fTK9I;
u4XFm+=a84tu*cjYid+0.5*S8tqs*VNvlY(cjYid);a84tu+=LDLfE;S8tqs=fTK9I;
u4XFm+=a84tu*jcVDt+0.5*S8tqs*VNvlY(jcVDt)+b4jXr*qn3x8(jcVDt)/6.0;a84tu+=KVYN5;
S8tqs=DAwhk;
u4XFm+=a84tu*wilOA+0.5*S8tqs*VNvlY(wilOA)-b4jXr*qn3x8(wilOA)/6.0;return(u4XFm<=
ldeCA);}


bool qPN_6::E0nn1(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr,const 
double&rTEdP,const double&SynchronizationTime){double EFQ3W=0.0
,cbBNZ=0.0
,LIlfP=0.0
,zAA9v=0.0
,rGqvR=0.0
,NJ168=0.0
,vwyxx=0.0
,a4lTx=0.0
,Iqm8o=0.0
,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4;
EFQ3W=S8tqs/b4jXr;NJ168=S8tqs*EFQ3W-0.5*VNvlY(EFQ3W)*b4jXr;
LIlfP=rGqvR=DAwhk/b4jXr;vwyxx=Iqm8o=0.5*VNvlY(DAwhk)/b4jXr;
a4lTx=HcINC-a84tu-NJ168-vwyxx-Iqm8o;zAA9v=a4lTx/DAwhk;
cbBNZ=SynchronizationTime-rTEdP-EFQ3W-LIlfP-zAA9v-rGqvR;
u4XFm+=a84tu*EFQ3W+0.5*S8tqs*VNvlY(EFQ3W)-b4jXr*qn3x8(EFQ3W)/6.0;a84tu+=NJ168;
S8tqs=0.0;
u4XFm+=a84tu*cbBNZ;
a84tu=a84tu;S8tqs=0.0;
u4XFm+=a84tu*LIlfP+0.5*S8tqs*VNvlY(LIlfP)+b4jXr*qn3x8(LIlfP)/6.0;a84tu+=vwyxx;
S8tqs=DAwhk;
u4XFm+=a84tu*zAA9v+0.5*S8tqs*VNvlY(zAA9v);
a84tu+=a4lTx;S8tqs=DAwhk;
u4XFm+=a84tu*rGqvR+0.5*S8tqs*VNvlY(rGqvR)-b4jXr*qn3x8(rGqvR)/6.0;return(u4XFm<=
ldeCA);}


bool qPN_6::CgajD(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&b4jXr){
double Time=0.0,a84tu=AQzqu,S8tqs=k6sf4;
Time=(DAwhk-S8tqs)/b4jXr;a84tu+=S8tqs*Time+0.5*b4jXr*VNvlY(Time);S8tqs=DAwhk;
Time=DAwhk/b4jXr;a84tu+=S8tqs*Time-0.5*b4jXr*VNvlY(Time);S8tqs=0.0;


a84tu+=S8tqs*Time+0.5*b4jXr*VNvlY(Time);S8tqs=DAwhk;


a84tu+=S8tqs*Time-0.5*b4jXr*VNvlY(Time);return(a84tu<=HcINC);}


bool qPN_6::ext3b(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&b4jXr,const double&rTEdP,const double&
SynchronizationTime){
double HB9ew=0.0
,nwQQL=0.0
,FP7H_=0.0
,LwKEC=0.0
,EcLHL=0.0
,lURFQ=0.0
,FYkEp=0.0
,eacZD=0.0
,nIcav=0.0
,HMqUy=0.0;

HB9ew=(DAwhk-k6sf4)/b4jXr;lURFQ=k6sf4*HB9ew+0.5*b4jXr*VNvlY(HB9ew);
nwQQL=FP7H_=EcLHL=DAwhk/b4jXr;FYkEp=eacZD=HMqUy=0.5*VNvlY(DAwhk)/b4jXr;
nIcav=HcINC-AQzqu-lURFQ-FYkEp-eacZD-HMqUy;LwKEC=nIcav/DAwhk;return((rTEdP+HB9ew+
nwQQL+FP7H_+LwKEC+EcLHL)>=SynchronizationTime);}


bool qPN_6::KkLb9(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr,const 
double&rTEdP,const double&SynchronizationTime){
return(byDB4(k6sf4,DAwhk,AQzqu,HcINC,iPzPj,ldeCA,b4jXr,rTEdP,SynchronizationTime
));}


bool qPN_6::HmgEe(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr,const 
double&rTEdP,const double&SynchronizationTime){
double sZyb6=0.0
,oTXmt=0.0
,FP7H_=0.0
,LwKEC=0.0
,EcLHL=0.0
,E2jtU=0.0
,Tdb4k=0.0
,eacZD=0.0
,nIcav=0.0
,HMqUy=0.0
,L38Np=0.0
,Yoeil=0.0,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4;Yoeil=SynchronizationTime-rTEdP;
L38Np=DAwhk-pbQOc(VNvlY(S8tqs)*VNvlY(b4jXr)-2.0*S8tqs*DAwhk*VNvlY(b4jXr)+4.0*
VNvlY(DAwhk)*VNvlY(b4jXr)-2.0*DAwhk*Yoeil*qn3x8(b4jXr)-2.0*qn3x8(b4jXr)*a84tu+
2.0*qn3x8(b4jXr)*HcINC)/(pbQOc(2.0)*b4jXr);
sZyb6=(L38Np-S8tqs)/b4jXr;E2jtU=S8tqs*sZyb6+0.5*b4jXr*VNvlY(sZyb6);
oTXmt=L38Np/b4jXr;Tdb4k=0.5*VNvlY(L38Np)/b4jXr;
FP7H_=EcLHL=DAwhk/b4jXr;eacZD=HMqUy=0.5*VNvlY(DAwhk)/b4jXr;
LwKEC=(S8tqs-2.0*DAwhk-2.0*L38Np+b4jXr*Yoeil)/b4jXr;nIcav=LwKEC*DAwhk;
u4XFm+=a84tu*sZyb6+0.5*S8tqs*VNvlY(sZyb6)+b4jXr*qn3x8(sZyb6)/6.0;a84tu+=E2jtU;
S8tqs=L38Np;
u4XFm+=a84tu*oTXmt+0.5*S8tqs*VNvlY(oTXmt)-b4jXr*qn3x8(oTXmt)/6.0;a84tu+=Tdb4k;
S8tqs=0.0;
u4XFm+=a84tu*FP7H_+0.5*S8tqs*VNvlY(FP7H_)+b4jXr*qn3x8(FP7H_)/6.0;a84tu+=eacZD;
S8tqs=DAwhk;
u4XFm+=a84tu*LwKEC+0.5*S8tqs*VNvlY(LwKEC);
a84tu+=nIcav;S8tqs=DAwhk;
u4XFm+=a84tu*EcLHL+0.5*S8tqs*VNvlY(EcLHL)-b4jXr*qn3x8(EcLHL)/6.0;
return(u4XFm<=ldeCA);}


bool qPN_6::aiMXR(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr,const 
double&rTEdP,const double&SynchronizationTime){
double HB9ew=0.0
,nwQQL=0.0
,cbBNZ=0.0
,FP7H_=0.0
,LwKEC=0.0
,EcLHL=0.0
,lURFQ=0.0
,FYkEp=0.0
,eacZD=0.0
,nIcav=0.0
,HMqUy=0.0
,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4;
HB9ew=(DAwhk-S8tqs)/b4jXr;lURFQ=S8tqs*HB9ew+0.5*b4jXr*VNvlY(HB9ew);
nwQQL=FP7H_=EcLHL=DAwhk/b4jXr;FYkEp=eacZD=HMqUy=0.5*VNvlY(DAwhk)/b4jXr;
nIcav=HcINC-a84tu-lURFQ-FYkEp-eacZD-HMqUy;LwKEC=nIcav/DAwhk;
cbBNZ=SynchronizationTime-rTEdP-HB9ew-nwQQL-FP7H_-LwKEC-EcLHL;
u4XFm+=a84tu*HB9ew+0.5*S8tqs*VNvlY(HB9ew)+b4jXr*qn3x8(HB9ew)/6.0;a84tu+=lURFQ;
S8tqs=DAwhk;
u4XFm+=a84tu*nwQQL+0.5*S8tqs*VNvlY(nwQQL)-b4jXr*qn3x8(nwQQL)/6.0;a84tu+=FYkEp;
S8tqs=0.0;
u4XFm+=a84tu*cbBNZ;
a84tu=a84tu;S8tqs=0.0;
u4XFm+=a84tu*FP7H_+0.5*S8tqs*VNvlY(FP7H_)+b4jXr*qn3x8(FP7H_)/6.0;a84tu+=eacZD;
S8tqs=DAwhk;
u4XFm+=a84tu*LwKEC+0.5*S8tqs*VNvlY(LwKEC);
a84tu+=nIcav;S8tqs=DAwhk;
u4XFm+=a84tu*EcLHL+0.5*S8tqs*VNvlY(EcLHL)-b4jXr*qn3x8(EcLHL)/6.0;
return(u4XFm<=ldeCA);}


bool qPN_6::Tb7lI(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr,const 
double&rTEdP,const double&SynchronizationTime){
double FP7H_=0.0
,LwKEC=0.0
,EcLHL=0.0
,cbBNZ=0.0
,HB9ew=0.0
,nwQQL=0.0
,eacZD=0.0
,nIcav=0.0
,HMqUy=0.0
,lURFQ=0.0
,FYkEp=0.0
,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4;
FP7H_=(DAwhk-S8tqs)/b4jXr;eacZD=S8tqs*FP7H_+0.5*b4jXr*VNvlY(FP7H_);
EcLHL=HB9ew=nwQQL=DAwhk/b4jXr;HMqUy=lURFQ=FYkEp=0.5*VNvlY(DAwhk)/b4jXr;
nIcav=HcINC-a84tu-eacZD-HMqUy-lURFQ-FYkEp;LwKEC=nIcav/DAwhk;
cbBNZ=SynchronizationTime-rTEdP-FP7H_-LwKEC-EcLHL-HB9ew-nwQQL;
u4XFm+=a84tu*FP7H_+0.5*S8tqs*VNvlY(FP7H_)+b4jXr*qn3x8(FP7H_)/6.0;a84tu+=eacZD;
S8tqs=DAwhk;
u4XFm+=a84tu*LwKEC+0.5*S8tqs*VNvlY(LwKEC);
a84tu+=nIcav;S8tqs=DAwhk;
u4XFm+=a84tu*EcLHL+0.5*S8tqs*VNvlY(EcLHL)-b4jXr*qn3x8(EcLHL)/6.0;
a84tu+=HMqUy;S8tqs=0.0;
u4XFm+=a84tu*cbBNZ;
a84tu=a84tu;S8tqs=0.0;
u4XFm+=a84tu*HB9ew+0.5*S8tqs*VNvlY(HB9ew)+b4jXr*qn3x8(HB9ew)/6.0;a84tu+=lURFQ;
S8tqs=DAwhk;
u4XFm+=a84tu*nwQQL+0.5*S8tqs*VNvlY(nwQQL)-b4jXr*qn3x8(nwQQL)/6.0;return(u4XFm<=
ldeCA);}


bool qPN_6::pzbBp(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&b4jXr,const double&rTEdP,const double&
SynchronizationTime){
double HB9ew=0.0
,nwQQL=0.0
,sZyb6=0.0
,oTXmt=0.0
,lURFQ=0.0
,FYkEp=0.0
,L38Np=0.0;

HB9ew=(DAwhk-k6sf4)/b4jXr;lURFQ=k6sf4*HB9ew+0.5*b4jXr*VNvlY(HB9ew);
nwQQL=DAwhk/b4jXr;FYkEp=0.5*VNvlY(DAwhk)/b4jXr;
L38Np=pbQOc(b4jXr*(HcINC-AQzqu-lURFQ-FYkEp));
sZyb6=oTXmt=L38Np/b4jXr;return((rTEdP+HB9ew+nwQQL+sZyb6+oTXmt)>=
SynchronizationTime);}


bool qPN_6::Jpn7p(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&b4jXr,const double&rTEdP,const double&
SynchronizationTime){
double HB9ew=0.0
,nwQQL=0.0
,sZyb6=0.0
,oTXmt=0.0
,lURFQ=0.0
,FYkEp=0.0
,L38Np=0.0;

HB9ew=nwQQL=DAwhk/b4jXr;lURFQ=FYkEp=0.5*VNvlY(DAwhk)/b4jXr;
L38Np=pbQOc((VNvlY(k6sf4)+2.0*b4jXr*(HcINC-AQzqu-lURFQ-FYkEp))/2.0);
sZyb6=(L38Np-k6sf4)/b4jXr;
oTXmt=L38Np/b4jXr;return((rTEdP+sZyb6+oTXmt+HB9ew+nwQQL)>=SynchronizationTime);}


bool qPN_6::kh7Ln(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr,const 
double&rTEdP,const double&SynchronizationTime){
double sZyb6=0.0
,oTXmt=0.0
,cbBNZ=0.0
,HB9ew=0.0
,nwQQL=0.0
,E2jtU=0.0
,Tdb4k=0.0
,lURFQ=0.0
,FYkEp=0.0
,L38Np=0.0
,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4;
HB9ew=nwQQL=DAwhk/b4jXr;lURFQ=FYkEp=0.5*VNvlY(DAwhk)/b4jXr;
L38Np=pbQOc((VNvlY(S8tqs)+2.0*b4jXr*(HcINC-a84tu-lURFQ-FYkEp))/2.0);
sZyb6=(L38Np-S8tqs)/b4jXr;E2jtU=S8tqs*sZyb6+0.5*b4jXr*VNvlY(sZyb6);
oTXmt=L38Np/b4jXr;Tdb4k=0.5*VNvlY(L38Np)/b4jXr;
cbBNZ=SynchronizationTime-rTEdP-sZyb6-oTXmt-HB9ew-nwQQL;
u4XFm+=a84tu*sZyb6+0.5*S8tqs*VNvlY(sZyb6)+b4jXr*qn3x8(sZyb6)/6.0;a84tu+=E2jtU;
S8tqs=L38Np;
u4XFm+=a84tu*oTXmt+0.5*S8tqs*VNvlY(oTXmt)-b4jXr*qn3x8(oTXmt)/6.0;a84tu+=Tdb4k;
S8tqs=0.0;
u4XFm+=a84tu*cbBNZ;
a84tu=a84tu;S8tqs=0.0;
u4XFm+=a84tu*HB9ew+0.5*S8tqs*VNvlY(HB9ew)+b4jXr*qn3x8(HB9ew)/6.0;a84tu+=lURFQ;
S8tqs=DAwhk;
u4XFm+=a84tu*nwQQL+0.5*S8tqs*VNvlY(nwQQL)-b4jXr*qn3x8(nwQQL)/6.0;return(u4XFm<=
ldeCA);}


bool qPN_6::oyCnN(const double&k6sf4,const double&AQzqu,const double&HcINC,const
 double&iPzPj,const double&ldeCA,const double&b4jXr,const double&rTEdP,const 
double&SynchronizationTime){double sqGvl=0.0,TwAY8=0.0,mItn3=0.0,vp93e=0.0,laktM
=0.0,EvE6x=0.0,L62Wd=0.0,KSaj_=0.0,mdI9v=0.0,Lj91q=0.0
,cfQSX=0.0
,tpyUy=0.0
,TdPa7=0.0
,L38Np=0.0;

L38Np=(pbQOc(b4jXr)*pbQOc(VNvlY(k6sf4)/b4jXr-2.0*AQzqu+2.0*HcINC))/2.0;
Lj91q=(L38Np-k6sf4)/b4jXr;
cfQSX=tpyUy=TdPa7=L38Np/b4jXr;;if((rTEdP+Lj91q+cfQSX+tpyUy+TdPa7)<
SynchronizationTime){return(false);}sqGvl=0.25*k6sf4+0.25*(SynchronizationTime-
rTEdP)*b4jXr-0.25*pbQOc(3.0*VNvlY(k6sf4)-2.0*k6sf4*(SynchronizationTime-rTEdP)*
b4jXr+b4jXr*(8.0*(HcINC-AQzqu)-VNvlY(SynchronizationTime-rTEdP)*b4jXr));TwAY8=
0.25*k6sf4+0.25*(SynchronizationTime-rTEdP)*b4jXr+0.25*pbQOc(3.0*VNvlY(k6sf4)-
2.0*k6sf4*(SynchronizationTime-rTEdP)*b4jXr+b4jXr*(8.0*(HcINC-AQzqu)-VNvlY(
SynchronizationTime-rTEdP)*b4jXr));if(sqGvl<k6sf4){sqGvl=k6sf4;}if(TwAY8<k6sf4){
TwAY8=k6sf4;}laktM=k6sf4*k6sf4;KSaj_=TwAY8+sqGvl;mdI9v=TwAY8*TwAY8;EvE6x=sqGvl*
sqGvl;L62Wd=b4jXr*b4jXr;vp93e=iPzPj+(laktM*k6sf4/0.3e1-laktM*KSaj_-k6sf4*b4jXr*
AQzqu+mdI9v*TwAY8+0.2e1*mdI9v*sqGvl+EvE6x*sqGvl+0.2e1*KSaj_*b4jXr*AQzqu)/L62Wd;
if(vp93e>ldeCA){return(true);}mItn3=iPzPj+(laktM*k6sf4/0.3e1-laktM*KSaj_-k6sf4*
b4jXr*AQzqu+EvE6x*sqGvl+0.2e1*EvE6x*TwAY8+mdI9v*TwAY8+0.2e1*KSaj_*b4jXr*AQzqu)/
L62Wd;if(mItn3<ldeCA){return(true);}return(false);}


bool qPN_6::HvZ3F(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr,const 
double&rTEdP,const double&SynchronizationTime){
double HB9ew=0.0
,nwQQL=0.0
,cbBNZ=0.0
,tepSg=0.0
,YQBR5=0.0
,lURFQ=0.0
,FYkEp=0.0
,Znfhm=0.0
,YPfXT=0.0
,L38Np=0.0
,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4;
HB9ew=(DAwhk-S8tqs)/b4jXr;lURFQ=S8tqs*HB9ew+0.5*b4jXr*VNvlY(HB9ew);
nwQQL=DAwhk/b4jXr;FYkEp=0.5*VNvlY(DAwhk)/b4jXr;
L38Np=pbQOc(b4jXr*(HcINC-a84tu-lURFQ-FYkEp));
tepSg=YQBR5=L38Np/b4jXr;Znfhm=YPfXT=0.5*VNvlY(L38Np)/b4jXr;
cbBNZ=SynchronizationTime-rTEdP-HB9ew-nwQQL-tepSg-YQBR5;
u4XFm+=a84tu*HB9ew+0.5*S8tqs*VNvlY(HB9ew)+b4jXr*qn3x8(HB9ew)/6.0;a84tu+=lURFQ;
S8tqs=DAwhk;
u4XFm+=a84tu*nwQQL+0.5*S8tqs*VNvlY(nwQQL)-b4jXr*qn3x8(nwQQL)/6.0;a84tu+=FYkEp;
S8tqs=0.0;
u4XFm+=a84tu*cbBNZ;
a84tu=a84tu;S8tqs=0.0;
u4XFm+=a84tu*tepSg+0.5*S8tqs*VNvlY(tepSg)+b4jXr*qn3x8(tepSg)/6.0;a84tu+=Znfhm;
S8tqs=L38Np;
u4XFm+=a84tu*YQBR5+0.5*S8tqs*VNvlY(YQBR5)-b4jXr*qn3x8(YQBR5)/6.0;return(u4XFm<=
ldeCA);}


bool qPN_6::giVp2(const double&DAwhk,const double&AQzqu,const double&HcINC,const
 double&b4jXr,const double&rTEdP,const double&SynchronizationTime){
double UKSpI=0.0
,DVdae=0.0
,LIlfP=0.0
,zAA9v=0.0
,rGqvR=0.0
,WBuXo=0.0
,ri4IE=0.0
,vwyxx=0.0
,rT061=0.0
,Iqm8o=0.0;

UKSpI=DVdae=LIlfP=rGqvR=DAwhk/b4jXr;WBuXo=ri4IE=-0.5*VNvlY(DAwhk)/b4jXr;vwyxx=
Iqm8o=0.5*VNvlY(DAwhk)/b4jXr;
rT061=HcINC-AQzqu-WBuXo-ri4IE-vwyxx-Iqm8o;zAA9v=rT061/DAwhk;return((rTEdP+UKSpI+
DVdae+LIlfP+zAA9v+rGqvR)>=SynchronizationTime);}


bool qPN_6::OqKS2(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr,const 
double&rTEdP,const double&SynchronizationTime){
double WT1rZ=0.0
,PbPkG=0.0
,cbBNZ=0.0
,HB9ew=0.0
,zRNIW=0.0
,nwQQL=0.0
,XxdWk=0.0
,NIjjJ=0.0
,lURFQ=0.0
,Vsamh=0.0
,FYkEp=0.0
,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4;
WT1rZ=PbPkG=HB9ew=nwQQL=DAwhk/b4jXr;XxdWk=NIjjJ=-0.5*VNvlY(DAwhk)/b4jXr;lURFQ=
FYkEp=0.5*VNvlY(DAwhk)/b4jXr;
Vsamh=HcINC-a84tu-XxdWk-NIjjJ-lURFQ-FYkEp;zRNIW=Vsamh/DAwhk;
cbBNZ=SynchronizationTime-rTEdP-WT1rZ-PbPkG-HB9ew-zRNIW-nwQQL;
u4XFm+=a84tu*WT1rZ+0.5*S8tqs*VNvlY(WT1rZ)-b4jXr*qn3x8(WT1rZ)/6.0;a84tu+=XxdWk;
S8tqs=-DAwhk;
u4XFm+=a84tu*PbPkG+0.5*S8tqs*VNvlY(PbPkG)+b4jXr*qn3x8(PbPkG)/6.0;a84tu+=NIjjJ;
S8tqs=0.0;
u4XFm+=a84tu*cbBNZ;
a84tu=a84tu;S8tqs=0.0;
u4XFm+=a84tu*HB9ew+0.5*S8tqs*VNvlY(HB9ew)+b4jXr*qn3x8(HB9ew)/6.0;a84tu+=lURFQ;
S8tqs=DAwhk;
u4XFm+=a84tu*zRNIW+0.5*S8tqs*VNvlY(zRNIW);
a84tu+=Vsamh;S8tqs=DAwhk;
u4XFm+=a84tu*nwQQL+0.5*S8tqs*VNvlY(nwQQL)-b4jXr*qn3x8(nwQQL)/6.0;
return(u4XFm>=ldeCA);}


bool qPN_6::JUT5r(const double&k6sf4,const double&AQzqu,const double&HcINC,const
 double&b4jXr,const double&rTEdP,const double&SynchronizationTime){double EFQ3W=
0.0
,tepSg=0.0
,YQBR5=0.0
,NJ168=0.0
,L38Np=0.0;

EFQ3W=k6sf4/b4jXr;NJ168=0.5*VNvlY(k6sf4)/b4jXr;
L38Np=pbQOc(b4jXr*(HcINC-AQzqu-NJ168));
tepSg=YQBR5=L38Np/b4jXr;return((rTEdP+EFQ3W+tepSg+YQBR5)>=SynchronizationTime);}


bool qPN_6::zs6Zz(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&b4jXr,const double&rTEdP,const double&
SynchronizationTime){return(pzbBp(k6sf4,DAwhk,AQzqu,HcINC,b4jXr,rTEdP,
SynchronizationTime));}


bool qPN_6::agdJf(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr,const 
double&rTEdP,const double&SynchronizationTime){return(HvZ3F(k6sf4,DAwhk,AQzqu,
HcINC,iPzPj,ldeCA,b4jXr,rTEdP,SynchronizationTime));}


bool qPN_6::BavCO(const double&k6sf4,const double&AQzqu,const double&HcINC,const
 double&iPzPj,const double&ldeCA,const double&b4jXr,const double&rTEdP,const 
double&SynchronizationTime){double EFQ3W=0.0
,cbBNZ=0.0
,UKSpI=0.0
,DVdae=0.0
,NJ168=0.0
,WBuXo=0.0
,ri4IE=0.0
,L38Np=0.0
,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4;
EFQ3W=S8tqs/b4jXr;NJ168=S8tqs*EFQ3W-0.5*VNvlY(EFQ3W)*b4jXr;
L38Np=pbQOc(b4jXr*(HcINC-a84tu-NJ168));
UKSpI=DVdae=L38Np/b4jXr;WBuXo=ri4IE=0.5*VNvlY(L38Np)/b4jXr;
cbBNZ=SynchronizationTime-rTEdP-EFQ3W-UKSpI-DVdae;
u4XFm+=a84tu*EFQ3W+0.5*S8tqs*VNvlY(EFQ3W)-b4jXr*qn3x8(EFQ3W)/6.0;a84tu+=NJ168;
S8tqs=0.0;
u4XFm+=a84tu*cbBNZ;
a84tu=a84tu;S8tqs=0.0;
u4XFm+=a84tu*UKSpI+0.5*S8tqs*VNvlY(UKSpI)+b4jXr*qn3x8(UKSpI)/6.0;a84tu+=WBuXo;
S8tqs=L38Np;
u4XFm+=a84tu*DVdae+0.5*S8tqs*VNvlY(DVdae)-b4jXr*qn3x8(DVdae)/6.0;return(u4XFm<=
ldeCA);}


bool qPN_6::RPvj1(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&b4jXr,const double&rTEdP,const double&
SynchronizationTime){return(pzbBp(k6sf4,DAwhk,AQzqu,HcINC,b4jXr,rTEdP,
SynchronizationTime));}


bool qPN_6::LgS1l(const double&k6sf4,const double&AQzqu,const double&HcINC,const
 double&b4jXr){return(WBeIT(k6sf4,AQzqu,HcINC,b4jXr));}


bool qPN_6::zi2EW(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr,const 
double&rTEdP,const double&SynchronizationTime){return(HvZ3F(k6sf4,DAwhk,AQzqu,
HcINC,iPzPj,ldeCA,b4jXr,rTEdP,SynchronizationTime));}


bool qPN_6::K5JY6(const double&DAwhk,const double&AQzqu,const double&HcINC,const
 double&b4jXr,const double&rTEdP,const double&SynchronizationTime){
double HB9ew=0.0
,nwQQL=0.0
,HfmHV=0.0
,J6Aml=0.0
,lURFQ=0.0
,FYkEp=0.0
,L38Np=0.0
,a84tu=AQzqu,JMvCa=HcINC;
HB9ew=nwQQL=DAwhk/b4jXr;lURFQ=FYkEp=0.5*VNvlY(DAwhk)/b4jXr;
a84tu=-a84tu;JMvCa=-JMvCa;lURFQ=-lURFQ;FYkEp=-FYkEp;
L38Np=pbQOc(b4jXr*(JMvCa-a84tu-lURFQ-FYkEp));
a84tu=-a84tu;JMvCa=-JMvCa;lURFQ=-lURFQ;FYkEp=-FYkEp;L38Np=-L38Np;
HfmHV=J6Aml=(-L38Np)/b4jXr;return((rTEdP+HB9ew+nwQQL+HfmHV+J6Aml)>=
SynchronizationTime);}


bool qPN_6::xyzl1(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr,const 
double&rTEdP,const double&SynchronizationTime){
double sZyb6=0.0
,oTXmt=0.0
,cbBNZ=0.0
,HB9ew=0.0
,nwQQL=0.0
,E2jtU=0.0
,Tdb4k=0.0
,lURFQ=0.0
,FYkEp=0.0
,L38Np=0.0
,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4,JMvCa=HcINC;
HB9ew=nwQQL=DAwhk/b4jXr;lURFQ=FYkEp=0.5*VNvlY(DAwhk)/b4jXr;
JMvCa=-JMvCa;a84tu=-a84tu;lURFQ=-lURFQ;FYkEp=-FYkEp;
L38Np=pbQOc(b4jXr*(JMvCa-a84tu-lURFQ-FYkEp));
L38Np=-L38Np;JMvCa=-JMvCa;a84tu=-a84tu;lURFQ=-lURFQ;FYkEp=-FYkEp;
sZyb6=oTXmt=(-L38Np)/b4jXr;E2jtU=Tdb4k=-0.5*VNvlY(L38Np)/b4jXr;
cbBNZ=SynchronizationTime-rTEdP-sZyb6-oTXmt-HB9ew-nwQQL;
u4XFm+=a84tu*sZyb6+0.5*S8tqs*VNvlY(sZyb6)-b4jXr*qn3x8(sZyb6)/6.0;a84tu+=E2jtU;
S8tqs=L38Np;
u4XFm+=a84tu*oTXmt+0.5*S8tqs*VNvlY(oTXmt)+b4jXr*qn3x8(oTXmt)/6.0;a84tu+=Tdb4k;
S8tqs=0.0;
u4XFm+=a84tu*cbBNZ;
a84tu=a84tu;S8tqs=0.0;
u4XFm+=a84tu*HB9ew+0.5*S8tqs*VNvlY(HB9ew)+b4jXr*qn3x8(HB9ew)/6.0;a84tu+=lURFQ;
S8tqs=DAwhk;
u4XFm+=a84tu*nwQQL+0.5*S8tqs*VNvlY(nwQQL)-b4jXr*qn3x8(nwQQL)/6.0;return(u4XFm>=
ldeCA);}


bool qPN_6::dQv3M(const double&k6sf4,const double&AQzqu,const double&HcINC,const
 double&iPzPj,const double&ldeCA,const double&b4jXr,const double&hxixi,const 
double&rTEdP,const double&SynchronizationTime){
double tepSg=0.0
,YQBR5=0.0
,cbBNZ=0.0
,Znfhm=0.0
,YPfXT=0.0
,L38Np=0.0
,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4;
L38Np=pbQOc((VNvlY(S8tqs)+2.0*b4jXr*(HcINC-a84tu))/2.0);
tepSg=(L38Np-S8tqs)/b4jXr;Znfhm=S8tqs*tepSg+0.5*b4jXr*VNvlY(tepSg);
YQBR5=L38Np/b4jXr;YPfXT=0.5*VNvlY(L38Np)/b4jXr;
cbBNZ=SynchronizationTime-rTEdP-tepSg-YQBR5;
u4XFm+=a84tu*tepSg+0.5*S8tqs*VNvlY(tepSg)+b4jXr*qn3x8(tepSg)/6.0;a84tu+=Znfhm;
S8tqs=L38Np;
u4XFm+=a84tu*YQBR5+0.5*S8tqs*VNvlY(YQBR5)-b4jXr*qn3x8(YQBR5)/6.0;a84tu=HcINC;
S8tqs=0.0;
u4XFm+=a84tu*cbBNZ;
a84tu=HcINC;S8tqs=0.0;if(HcINC>0.0){return(u4XFm<=ldeCA+hxixi*EVXXF*(1.0-(hxixi-
HcINC)/hxixi));}else{return(u4XFm<=ldeCA);}}


bool qPN_6::kl7zT(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&b4jXr,const double&rTEdP,const double&
SynchronizationTime){
double HB9ew=0.0
,nwQQL=0.0
,nlydG=0.0
,Y9kH3=0.0
,L38Np=0.0
,a84tu=AQzqu,S8tqs=k6sf4,JMvCa=HcINC;
HB9ew=(DAwhk-S8tqs)/b4jXr;
a84tu+=S8tqs*HB9ew+0.5*b4jXr*VNvlY(HB9ew);S8tqs=DAwhk;
nwQQL=DAwhk/b4jXr;
a84tu+=S8tqs*nwQQL-0.5*b4jXr*VNvlY(nwQQL);S8tqs=0.0;

JMvCa=-JMvCa;a84tu=-a84tu;
L38Np=pbQOc(b4jXr*(JMvCa-a84tu));
JMvCa=-JMvCa;a84tu=-a84tu;L38Np=-L38Np;
nlydG=Y9kH3=(-L38Np)/b4jXr;return((rTEdP+HB9ew+nwQQL+nlydG+Y9kH3)>=
SynchronizationTime);}


bool qPN_6::TDuVs(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr,const 
double&rTEdP,const double&SynchronizationTime){
double HB9ew=0.0
,nwQQL=0.0
,cbBNZ=0.0
,nlydG=0.0
,Y9kH3=0.0
,lURFQ=0.0
,FYkEp=0.0
,RvMt3=0.0
,b_l5_=0.0
,L38Np=0.0
,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4,JMvCa=HcINC;
HB9ew=(DAwhk-S8tqs)/b4jXr;lURFQ=S8tqs*HB9ew+0.5*b4jXr*VNvlY(HB9ew);
nwQQL=DAwhk/b4jXr;FYkEp=0.5*VNvlY(DAwhk)/b4jXr;
JMvCa=-JMvCa;a84tu=-a84tu;lURFQ=-lURFQ;FYkEp=-FYkEp;
L38Np=pbQOc(b4jXr*(JMvCa-a84tu-lURFQ-FYkEp));
L38Np=-L38Np;JMvCa=-JMvCa;a84tu=-a84tu;lURFQ=-lURFQ;FYkEp=-FYkEp;
nlydG=Y9kH3=(-L38Np)/b4jXr;RvMt3=b_l5_=-0.5*VNvlY(L38Np)/b4jXr;
cbBNZ=SynchronizationTime-rTEdP-HB9ew-nwQQL-nlydG-Y9kH3;
u4XFm+=a84tu*HB9ew+0.5*S8tqs*VNvlY(HB9ew)+b4jXr*qn3x8(HB9ew)/6.0;a84tu+=lURFQ;
S8tqs=DAwhk;
u4XFm+=a84tu*nwQQL+0.5*S8tqs*VNvlY(nwQQL)-b4jXr*qn3x8(nwQQL)/6.0;
a84tu+=FYkEp;S8tqs=(0x14a2+2421-0x1e17);
u4XFm+=a84tu*cbBNZ;
a84tu=a84tu;S8tqs=0.0;
u4XFm+=a84tu*nlydG+0.5*S8tqs*VNvlY(nlydG)-b4jXr*qn3x8(nlydG)/6.0;a84tu+=RvMt3;
S8tqs=L38Np;
u4XFm+=a84tu*Y9kH3+0.5*S8tqs*VNvlY(Y9kH3)+b4jXr*qn3x8(Y9kH3)/6.0;return(u4XFm<=
ldeCA);}


bool qPN_6::dZdud(const double&k6sf4,const double&AQzqu,const double&HcINC,const
 double&b4jXr,const double&rTEdP,const double&SynchronizationTime){return(JUT5r(
k6sf4,AQzqu,HcINC,b4jXr,rTEdP,SynchronizationTime));}


bool qPN_6::qL2q7(const double&k6sf4,const double&AQzqu,const double&HcINC,const
 double&b4jXr){return(WBeIT(k6sf4,AQzqu,HcINC,b4jXr));}


bool qPN_6::c4qB9(const double&k6sf4,const double&AQzqu,const double&HcINC,const
 double&b4jXr,const double&rTEdP,const double&SynchronizationTime){return(tCg8u(
k6sf4,AQzqu,HcINC,b4jXr,rTEdP,SynchronizationTime));}


bool qPN_6::jCYe5(const double&k6sf4,const double&AQzqu,const double&HcINC,const
 double&iPzPj,const double&ldeCA,const double&b4jXr,const double&rTEdP,const 
double&SynchronizationTime){return(WxfSq(k6sf4,AQzqu,HcINC,iPzPj,ldeCA,b4jXr,
rTEdP,SynchronizationTime));}


bool qPN_6::W81G_(const double&k6sf4,const double&AQzqu,const double&HcINC,const
 double&iPzPj,const double&ldeCA,const double&b4jXr,const double&rTEdP,const 
double&SynchronizationTime){return(VC639(k6sf4,AQzqu,HcINC,iPzPj,ldeCA,b4jXr,
rTEdP,SynchronizationTime));}


bool qPN_6::EImlu(const double&k6sf4,const double&AQzqu,const double&HcINC,const
 double&iPzPj,const double&ldeCA,const double&b4jXr,const double&rTEdP,const 
double&SynchronizationTime){return(dqYWb(k6sf4,AQzqu,HcINC,iPzPj,ldeCA,b4jXr,
rTEdP,SynchronizationTime));}


bool qPN_6::yIJmk(const double&k6sf4,const double&AQzqu,const double&HcINC,const
 double&iPzPj,const double&ldeCA,const double&b4jXr,const double&rTEdP,const 
double&SynchronizationTime){
return(oyCnN(k6sf4,AQzqu,HcINC,iPzPj,ldeCA,b4jXr,rTEdP,SynchronizationTime));}


bool qPN_6::FJWy6(const double&k6sf4,const double&AQzqu,const double&HcINC,const
 double&iPzPj,const double&ldeCA,const double&b4jXr,const double&rTEdP,const 
double&SynchronizationTime){return(BavCO(k6sf4,AQzqu,HcINC,iPzPj,ldeCA,b4jXr,
rTEdP,SynchronizationTime));}


bool qPN_6::WBeIT(const double&k6sf4,const double&AQzqu,const double&HcINC,const
 double&b4jXr){
double Time=0.0
,vkjQy=0.0
,L38Np=0.0
,a84tu=AQzqu,S8tqs=k6sf4;vkjQy=S8tqs;
Time=S8tqs/b4jXr;a84tu+=S8tqs*Time-0.5*b4jXr*VNvlY(Time);S8tqs=0.0;
L38Np=pbQOc(b4jXr*(HcINC-a84tu));return(vkjQy>=L38Np);}


bool qPN_6::tA4N6(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&b4jXr){
return(jBDiW(k6sf4,DAwhk,AQzqu,HcINC,b4jXr));}


bool qPN_6::cM9j6(const double&k6sf4,const double&AQzqu,const double&HcINC,const
 double&iPzPj,const double&ldeCA,const double&b4jXr,const double&hxixi,const 
double&rTEdP,const double&SynchronizationTime){
double EFQ3W=0.0
,nlydG=0.0
,Y9kH3=0.0
,cbBNZ=0.0
,NJ168=0.0
,RvMt3=0.0
,b_l5_=0.0
,L38Np=0.0
,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4,JMvCa=HcINC;
EFQ3W=S8tqs/b4jXr;NJ168=0.5*VNvlY(S8tqs)/b4jXr;
JMvCa=-JMvCa;a84tu=-a84tu;NJ168=-NJ168;
L38Np=pbQOc(b4jXr*(JMvCa-a84tu-NJ168));
L38Np=-L38Np;JMvCa=-JMvCa;a84tu=-a84tu;NJ168=-NJ168;
nlydG=Y9kH3=(-L38Np)/b4jXr;RvMt3=b_l5_=-0.5*VNvlY(L38Np)/b4jXr;
cbBNZ=SynchronizationTime-rTEdP-EFQ3W-nlydG-Y9kH3;
u4XFm+=a84tu*EFQ3W+0.5*S8tqs*VNvlY(EFQ3W)-b4jXr*qn3x8(EFQ3W)/6.0;a84tu+=NJ168;
S8tqs=0.0;
u4XFm+=a84tu*nlydG+0.5*S8tqs*VNvlY(nlydG)-b4jXr*qn3x8(nlydG)/6.0;a84tu+=RvMt3;
S8tqs=L38Np;
u4XFm+=a84tu*Y9kH3+0.5*S8tqs*VNvlY(Y9kH3)+b4jXr*qn3x8(Y9kH3)/6.0;a84tu=JMvCa;
S8tqs=0.0;
u4XFm+=a84tu*cbBNZ;
a84tu=JMvCa;S8tqs=0.0;if(HcINC<0.0){return(u4XFm>=ldeCA-hxixi*EVXXF*(1.0-(hxixi+
HcINC)/hxixi));}else{return(u4XFm>=ldeCA);}}


bool qPN_6::FGVLA(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&b4jXr,const double&rTEdP,const double&
SynchronizationTime){
double WT1rZ=0.0
,PbPkG=0.0
,UKSpI=0.0
,DVdae=0.0
,XxdWk=0.0
,NIjjJ=0.0
,L38Np=0.0
,a84tu=AQzqu,S8tqs=k6sf4;
WT1rZ=PbPkG=DAwhk/b4jXr;XxdWk=NIjjJ=-0.5*VNvlY(DAwhk)/b4jXr;
a84tu+=XxdWk;S8tqs=-DAwhk;
a84tu+=NIjjJ;S8tqs=0.0;
L38Np=pbQOc(b4jXr*(HcINC-a84tu));
UKSpI=DVdae=L38Np/b4jXr;return((rTEdP+WT1rZ+PbPkG+UKSpI+DVdae)>=
SynchronizationTime);}


bool qPN_6::bxEA5(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr,const 
double&rTEdP,const double&SynchronizationTime){
double WT1rZ=0.0
,PbPkG=0.0
,cbBNZ=0.0
,tepSg=0.0
,YQBR5=0.0
,XxdWk=0.0
,NIjjJ=0.0
,Znfhm=0.0
,YPfXT=0.0
,L38Np=0.0
,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4;
WT1rZ=PbPkG=DAwhk/b4jXr;XxdWk=NIjjJ=-0.5*VNvlY(DAwhk)/b4jXr;
L38Np=pbQOc(b4jXr*(HcINC-a84tu-XxdWk-NIjjJ));
tepSg=YQBR5=L38Np/b4jXr;Znfhm=YPfXT=0.5*VNvlY(L38Np)/b4jXr;
cbBNZ=SynchronizationTime-rTEdP-WT1rZ-PbPkG-tepSg-YQBR5;
u4XFm+=a84tu*WT1rZ+0.5*S8tqs*VNvlY(WT1rZ)-b4jXr*qn3x8(WT1rZ)/6.0;a84tu+=XxdWk;
S8tqs=-DAwhk;
u4XFm+=a84tu*PbPkG+0.5*S8tqs*VNvlY(PbPkG)+b4jXr*qn3x8(PbPkG)/6.0;a84tu+=NIjjJ;
S8tqs=0.0;
u4XFm+=a84tu*cbBNZ;
a84tu=a84tu;S8tqs=0.0;
u4XFm+=a84tu*tepSg+0.5*S8tqs*VNvlY(tepSg)+b4jXr*qn3x8(tepSg)/6.0;a84tu+=Znfhm;
S8tqs=L38Np;
u4XFm+=a84tu*YQBR5+0.5*S8tqs*VNvlY(YQBR5)-b4jXr*qn3x8(YQBR5)/6.0;return(u4XFm>=
ldeCA);}


bool qPN_6::SBP18(const double&DAwhk,const double&AQzqu,const double&HcINC,const
 double&b4jXr,const double&rTEdP,const double&SynchronizationTime){
double WT1rZ=0.0
,un6jF=0.0
,PbPkG=0.0
,HB9ew=0.0
,nwQQL=0.0
,XxdWk=0.0
,a69lF=0.0
,NIjjJ=0.0
,lURFQ=0.0
,FYkEp=0.0;

WT1rZ=PbPkG=HB9ew=nwQQL=DAwhk/b4jXr;XxdWk=NIjjJ=-0.5*VNvlY(DAwhk)/b4jXr;lURFQ=
FYkEp=0.5*VNvlY(DAwhk)/b4jXr;
a69lF=HcINC-AQzqu-XxdWk-NIjjJ-lURFQ-FYkEp;un6jF=a69lF/(-DAwhk);return((rTEdP+
WT1rZ+un6jF+PbPkG+HB9ew+nwQQL)>=SynchronizationTime);}


bool qPN_6::R3xv6(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr,const 
double&rTEdP,const double&SynchronizationTime){
double WT1rZ=0.0
,un6jF=0.0
,PbPkG=0.0
,cbBNZ=0.0
,HB9ew=0.0
,nwQQL=0.0
,XxdWk=0.0
,a69lF=0.0
,NIjjJ=0.0
,lURFQ=0.0
,FYkEp=0.0
,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4;
WT1rZ=PbPkG=HB9ew=nwQQL=DAwhk/b4jXr;XxdWk=NIjjJ=-0.5*VNvlY(DAwhk)/b4jXr;lURFQ=
FYkEp=0.5*VNvlY(DAwhk)/b4jXr;
a69lF=HcINC-a84tu-XxdWk-NIjjJ-lURFQ-FYkEp;un6jF=a69lF/(-DAwhk);
cbBNZ=SynchronizationTime-rTEdP-WT1rZ-un6jF-PbPkG-HB9ew-nwQQL;
u4XFm+=a84tu*WT1rZ+0.5*S8tqs*VNvlY(WT1rZ)-b4jXr*qn3x8(WT1rZ)/6.0;a84tu+=XxdWk;
S8tqs=-DAwhk;
u4XFm+=a84tu*un6jF+0.5*S8tqs*VNvlY(un6jF);
a84tu+=a69lF;S8tqs=-DAwhk;
u4XFm+=a84tu*PbPkG+0.5*S8tqs*VNvlY(PbPkG)+b4jXr*qn3x8(PbPkG)/6.0;
a84tu+=NIjjJ;S8tqs=0.0;
u4XFm+=a84tu*cbBNZ;
a84tu=a84tu;S8tqs=0.0;
u4XFm+=a84tu*HB9ew+0.5*S8tqs*VNvlY(HB9ew)+b4jXr*qn3x8(HB9ew)/6.0;a84tu+=lURFQ;
S8tqs=DAwhk;
u4XFm+=a84tu*nwQQL+0.5*S8tqs*VNvlY(nwQQL)-b4jXr*qn3x8(nwQQL)/6.0;return(u4XFm>=
ldeCA);}


bool qPN_6::_kI1L(const double&k6sf4,const double&AQzqu,const double&HcINC,const
 double&iPzPj,const double&ldeCA,const double&b4jXr,const double&rTEdP,const 
double&SynchronizationTime){double EFQ3W=0.0
,cbBNZ=0.0
,UKSpI=0.0
,DVdae=0.0
,NJ168=0.0
,WBuXo=0.0
,ri4IE=0.0
,L38Np=0.0
,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4,JMvCa=HcINC;
EFQ3W=S8tqs/b4jXr;NJ168=S8tqs*EFQ3W-0.5*VNvlY(EFQ3W)*b4jXr;
JMvCa=-JMvCa;a84tu=-a84tu;NJ168=-NJ168;
L38Np=pbQOc(b4jXr*(JMvCa-a84tu-NJ168));
JMvCa=-JMvCa;a84tu=-a84tu;NJ168=-NJ168;L38Np=-L38Np;
UKSpI=DVdae=(-L38Np)/b4jXr;WBuXo=ri4IE=-0.5*VNvlY(L38Np)/b4jXr;
cbBNZ=SynchronizationTime-rTEdP-EFQ3W-UKSpI-DVdae;
u4XFm+=a84tu*EFQ3W+0.5*S8tqs*VNvlY(EFQ3W)-b4jXr*qn3x8(EFQ3W)/6.0;a84tu+=NJ168;
S8tqs=0.0;
u4XFm+=a84tu*cbBNZ;
a84tu=a84tu;S8tqs=0.0;
u4XFm+=a84tu*UKSpI+0.5*S8tqs*VNvlY(UKSpI)-b4jXr*qn3x8(UKSpI)/6.0;a84tu+=WBuXo;
S8tqs=L38Np;
u4XFm+=a84tu*DVdae+0.5*S8tqs*VNvlY(DVdae)+b4jXr*qn3x8(DVdae)/6.0;return(u4XFm>=
ldeCA);}


bool qPN_6::Yijm0(const double&AQzqu,const double&HcINC,const double&iPzPj,const
 double&ldeCA,const double&b4jXr,const double&rTEdP,const double&
SynchronizationTime){return(oyCnN(0.0,-AQzqu,-HcINC,-iPzPj,-ldeCA,b4jXr,rTEdP,
SynchronizationTime));}


bool qPN_6::WhfDp(const double&k6sf4,const double&AQzqu,const double&HcINC,const
 double&iPzPj,const double&ldeCA,const double&b4jXr,const double&rTEdP,const 
double&SynchronizationTime){return(oi7CN(k6sf4,AQzqu,HcINC,iPzPj,ldeCA,b4jXr,
rTEdP,SynchronizationTime));}


bool qPN_6::WwMkt(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&b4jXr){
return(wdgir(k6sf4,DAwhk,AQzqu,HcINC,b4jXr));}


bool qPN_6::yk_IR(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&b4jXr,const double&rTEdP,const double&
SynchronizationTime){
double sZyb6=0.0
,oTXmt=0.0
,WT1rZ=0.0
,PbPkG=0.0
,XxdWk=0.0
,NIjjJ=0.0
,L38Np=0.0;

WT1rZ=PbPkG=DAwhk/b4jXr;XxdWk=NIjjJ=-0.5*VNvlY(DAwhk)/b4jXr;
L38Np=pbQOc((VNvlY(k6sf4)+2.0*b4jXr*(HcINC-AQzqu-XxdWk-NIjjJ))/2.0);
sZyb6=(L38Np-k6sf4)/b4jXr;
oTXmt=L38Np/b4jXr;return((rTEdP+sZyb6+oTXmt+WT1rZ+PbPkG)>=SynchronizationTime);}


bool qPN_6::AWe7i(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr,const 
double&rTEdP,const double&SynchronizationTime){
double sZyb6=0.0
,oTXmt=0.0
,cbBNZ=0.0
,WT1rZ=0.0
,PbPkG=0.0
,E2jtU=0.0
,Tdb4k=0.0
,XxdWk=0.0
,NIjjJ=0.0
,CKkns=0.0
,L38Np=0.0
,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4;
WT1rZ=PbPkG=DAwhk/b4jXr;XxdWk=NIjjJ=-0.5*VNvlY(DAwhk)/b4jXr;
L38Np=pbQOc((VNvlY(S8tqs)+2.0*b4jXr*(HcINC-a84tu-XxdWk-NIjjJ))/2.0);
sZyb6=(L38Np-S8tqs)/b4jXr;E2jtU=S8tqs*sZyb6+0.5*b4jXr*VNvlY(sZyb6);
oTXmt=L38Np/b4jXr;Tdb4k=0.5*VNvlY(L38Np)/b4jXr;
CKkns=a84tu+E2jtU+Tdb4k;
cbBNZ=SynchronizationTime-rTEdP-sZyb6-oTXmt-WT1rZ-PbPkG;
u4XFm+=a84tu*sZyb6+0.5*S8tqs*VNvlY(sZyb6)+b4jXr*qn3x8(sZyb6)/6.0;a84tu+=E2jtU;
S8tqs=L38Np;
u4XFm+=a84tu*oTXmt+0.5*S8tqs*VNvlY(oTXmt)-b4jXr*qn3x8(oTXmt)/6.0;a84tu+=Tdb4k;
S8tqs=0.0;
u4XFm+=a84tu*cbBNZ;
a84tu=a84tu;S8tqs=0.0;
u4XFm+=a84tu*WT1rZ+0.5*S8tqs*VNvlY(WT1rZ)-b4jXr*qn3x8(WT1rZ)/6.0;a84tu+=XxdWk;
S8tqs=-DAwhk;
u4XFm+=a84tu*PbPkG+0.5*S8tqs*VNvlY(PbPkG)+b4jXr*qn3x8(PbPkG)/6.0;return(u4XFm<=
ldeCA);}


bool qPN_6::mtcVv(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&b4jXr,const double&rTEdP,const double&
SynchronizationTime){
double HB9ew=0.0
,nwQQL=0.0
,WT1rZ=0.0
,un6jF=0.0
,PbPkG=0.0
,XxdWk=0.0
,a69lF=0.0
,NIjjJ=0.0
,a84tu=AQzqu,S8tqs=k6sf4;
HB9ew=(DAwhk-S8tqs)/b4jXr;a84tu+=S8tqs*HB9ew+0.5*b4jXr*VNvlY(HB9ew);S8tqs=DAwhk;
nwQQL=DAwhk/b4jXr;a84tu+=S8tqs*nwQQL-0.5*b4jXr*VNvlY(nwQQL);S8tqs=0.0;
WT1rZ=PbPkG=DAwhk/b4jXr;XxdWk=NIjjJ=-0.5*VNvlY(DAwhk)/b4jXr;
a69lF=HcINC-a84tu-XxdWk-NIjjJ;un6jF=a69lF/(-DAwhk);return((rTEdP+HB9ew+nwQQL+
WT1rZ+un6jF+PbPkG)>=SynchronizationTime);}


bool qPN_6::vZCWF(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr,const 
double&rTEdP,const double&SynchronizationTime){
double HB9ew=0.0
,nwQQL=0.0
,cbBNZ=0.0
,MixVF=0.0
,Bczkr=0.0
,ynw_o=0.0
,lURFQ=0.0
,FYkEp=0.0
,tQ417=0.0
,eLwei=0.0
,k1KD8=0.0
,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4;
HB9ew=(DAwhk-S8tqs)/b4jXr;lURFQ=S8tqs*HB9ew+0.5*b4jXr*VNvlY(HB9ew);
nwQQL=DAwhk/b4jXr;FYkEp=0.5*VNvlY(DAwhk)/b4jXr;
MixVF=ynw_o=DAwhk/b4jXr;tQ417=k1KD8=-0.5*VNvlY(DAwhk)/b4jXr;
eLwei=HcINC-a84tu-lURFQ-FYkEp-tQ417-k1KD8;Bczkr=eLwei/(-DAwhk);
cbBNZ=SynchronizationTime-rTEdP-HB9ew-nwQQL-MixVF-Bczkr-ynw_o;
u4XFm+=a84tu*HB9ew+0.5*S8tqs*VNvlY(HB9ew)+b4jXr*qn3x8(HB9ew)/6.0;a84tu+=lURFQ;
S8tqs=DAwhk;
u4XFm+=a84tu*nwQQL+0.5*S8tqs*VNvlY(nwQQL)-b4jXr*qn3x8(nwQQL)/6.0;a84tu+=FYkEp;
S8tqs=0.0;
u4XFm+=a84tu*cbBNZ;
a84tu=a84tu;S8tqs=0.0;
u4XFm+=a84tu*MixVF+0.5*S8tqs*VNvlY(MixVF)-b4jXr*qn3x8(MixVF)/6.0;a84tu+=tQ417;
S8tqs=-DAwhk;
u4XFm+=a84tu*Bczkr+0.5*S8tqs*VNvlY(Bczkr);
a84tu+=eLwei;S8tqs=-DAwhk;
u4XFm+=a84tu*ynw_o+0.5*S8tqs*VNvlY(ynw_o)+b4jXr*qn3x8(ynw_o)/6.0;
return(u4XFm<=ldeCA);}


bool qPN_6::oJMn7(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr,const 
double&hxixi,const double&rTEdP,const double&SynchronizationTime){
double EFQ3W=0.0
,WT1rZ=0.0
,un6jF=0.0
,PbPkG=0.0
,cbBNZ=0.0
,NJ168=0.0
,XxdWk=0.0
,a69lF=0.0
,NIjjJ=0.0
,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4;
EFQ3W=S8tqs/b4jXr;NJ168=0.5*VNvlY(S8tqs)/b4jXr;
WT1rZ=PbPkG=DAwhk/b4jXr;XxdWk=NIjjJ=-0.5*VNvlY(DAwhk)/b4jXr;
a69lF=HcINC-a84tu-NJ168-XxdWk-NIjjJ;un6jF=a69lF/(-DAwhk);
cbBNZ=SynchronizationTime-rTEdP-EFQ3W-WT1rZ-un6jF-PbPkG;
u4XFm+=a84tu*EFQ3W+0.5*S8tqs*VNvlY(EFQ3W)-b4jXr*qn3x8(EFQ3W)/6.0;a84tu+=NJ168;
S8tqs=0.0;
u4XFm+=a84tu*WT1rZ+0.5*S8tqs*VNvlY(WT1rZ)-b4jXr*qn3x8(WT1rZ)/6.0;a84tu+=XxdWk;
S8tqs=-DAwhk;
u4XFm+=a84tu*un6jF+0.5*S8tqs*VNvlY(un6jF);
a84tu+=a69lF;S8tqs=-DAwhk;
u4XFm+=a84tu*PbPkG+0.5*S8tqs*VNvlY(PbPkG)+b4jXr*qn3x8(PbPkG)/6.0;
a84tu=HcINC;S8tqs=0.0;
u4XFm+=a84tu*cbBNZ;
a84tu=HcINC;S8tqs=0.0;if(HcINC<0.0){return(u4XFm>=ldeCA-hxixi*EVXXF*(1.0-(hxixi+
HcINC)/hxixi));}else{return(u4XFm>=ldeCA);}}


bool qPN_6::WQT1H(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr,const 
double&rTEdP,const double&SynchronizationTime){double EFQ3W=0.0
,cbBNZ=0.0
,LIlfP=0.0
,zAA9v=0.0
,rGqvR=0.0
,NJ168=0.0
,vwyxx=0.0
,a4lTx=0.0
,Iqm8o=0.0
,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4;
EFQ3W=S8tqs/b4jXr;NJ168=S8tqs*EFQ3W-0.5*VNvlY(EFQ3W)*b4jXr;
LIlfP=rGqvR=DAwhk/b4jXr;vwyxx=Iqm8o=-0.5*VNvlY(DAwhk)/b4jXr;
a4lTx=HcINC-a84tu-NJ168-vwyxx-Iqm8o;zAA9v=a4lTx/(-DAwhk);
cbBNZ=SynchronizationTime-rTEdP-EFQ3W-LIlfP-zAA9v-rGqvR;
u4XFm+=a84tu*EFQ3W+0.5*S8tqs*VNvlY(EFQ3W)-b4jXr*qn3x8(EFQ3W)/6.0;a84tu+=NJ168;
S8tqs=0.0;
u4XFm+=a84tu*cbBNZ;
a84tu=a84tu;S8tqs=0.0;
u4XFm+=a84tu*LIlfP+0.5*S8tqs*VNvlY(LIlfP)-b4jXr*qn3x8(LIlfP)/6.0;a84tu+=vwyxx;
S8tqs=-DAwhk;
u4XFm+=a84tu*zAA9v+0.5*S8tqs*VNvlY(zAA9v);
a84tu+=a4lTx;S8tqs=-DAwhk;
u4XFm+=a84tu*rGqvR+0.5*S8tqs*VNvlY(rGqvR)+b4jXr*qn3x8(rGqvR)/6.0;
return(u4XFm>=ldeCA);}


bool qPN_6::NVw3D(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&b4jXr){
double S8tqs=k6sf4,Time=0.0,a84tu=AQzqu;
Time=DAwhk/b4jXr;a84tu+=S8tqs*Time-0.5*b4jXr*VNvlY(Time);S8tqs=-DAwhk;


a84tu+=S8tqs*Time+0.5*b4jXr*VNvlY(Time);S8tqs=0.0;


a84tu+=S8tqs*Time-0.5*b4jXr*VNvlY(Time);S8tqs=-DAwhk;


a84tu+=S8tqs*Time+0.5*b4jXr*VNvlY(Time);return(a84tu>=HcINC);}


bool qPN_6::Xxly5(const double&DAwhk,const double&AQzqu,const double&HcINC,const
 double&b4jXr,const double&rTEdP,const double&SynchronizationTime){
double WT1rZ=0.0
,PbPkG=0.0
,MixVF=0.0
,Bczkr=0.0
,ynw_o=0.0
,XxdWk=0.0
,NIjjJ=0.0
,tQ417=0.0
,eLwei=0.0
,k1KD8=0.0;

WT1rZ=PbPkG=MixVF=ynw_o=DAwhk/b4jXr;XxdWk=NIjjJ=tQ417=k1KD8=-0.5*VNvlY(DAwhk)/
b4jXr;
eLwei=HcINC-AQzqu-XxdWk-NIjjJ-tQ417-k1KD8;Bczkr=eLwei/(-DAwhk);return((rTEdP+
WT1rZ+PbPkG+MixVF+Bczkr+ynw_o)>=SynchronizationTime);}


bool qPN_6::vSRz9(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr,const 
double&rTEdP,const double&SynchronizationTime){
double WT1rZ=0.0
,un6jF=0.0
,PbPkG=0.0
,sZyb6=0.0
,oTXmt=0.0
,XxdWk=0.0
,a69lF=0.0
,NIjjJ=0.0
,E2jtU=0.0
,Tdb4k=0.0
,L38Np=0.0
,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4;
L38Np=0.5*(-2.0*DAwhk+2.8284271247461903*pbQOc(1.0*VNvlY(DAwhk)+0.5*DAwhk*b4jXr*
rTEdP-0.5*DAwhk*b4jXr*SynchronizationTime+0.5*b4jXr*a84tu-0.5*b4jXr*HcINC));
WT1rZ=PbPkG=DAwhk/b4jXr;XxdWk=NIjjJ=-0.5*VNvlY(DAwhk)/b4jXr;
un6jF=(-4.0*DAwhk)/b4jXr-1.0*rTEdP+SynchronizationTime+(2.8284271247461903*pbQOc
(1.0*VNvlY(DAwhk)+0.5*DAwhk*b4jXr*rTEdP-0.5*DAwhk*b4jXr*SynchronizationTime+0.5*
b4jXr*a84tu-0.5*b4jXr*HcINC))/b4jXr;a69lF=(-DAwhk)*un6jF;
sZyb6=oTXmt=(-L38Np)/b4jXr;E2jtU=Tdb4k=-0.5*VNvlY(L38Np)/b4jXr;
u4XFm+=a84tu*WT1rZ+0.5*S8tqs*VNvlY(WT1rZ)-b4jXr*qn3x8(WT1rZ)/6.0;a84tu+=XxdWk;
S8tqs=-DAwhk;
u4XFm+=a84tu*un6jF+0.5*S8tqs*VNvlY(un6jF);
a84tu+=a69lF;S8tqs=-DAwhk;
u4XFm+=a84tu*PbPkG+0.5*S8tqs*VNvlY(PbPkG)+b4jXr*qn3x8(PbPkG)/6.0;
a84tu+=NIjjJ;S8tqs=0.0;
u4XFm+=a84tu*sZyb6+0.5*S8tqs*VNvlY(sZyb6)-b4jXr*qn3x8(sZyb6)/6.0;a84tu+=E2jtU;
S8tqs=L38Np;
u4XFm+=a84tu*oTXmt+0.5*S8tqs*VNvlY(oTXmt)+b4jXr*qn3x8(oTXmt)/6.0;return(u4XFm>=
ldeCA);}


bool qPN_6::_CTD7(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr,const 
double&rTEdP,const double&SynchronizationTime){
double sZyb6=0.0
,oTXmt=0.0
,WT1rZ=0.0
,un6jF=0.0
,PbPkG=0.0
,E2jtU=0.0
,Tdb4k=0.0
,XxdWk=0.0
,a69lF=0.0
,NIjjJ=0.0
,L38Np=0.0
,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4;
L38Np=0.5*(-2.0*DAwhk+2.8284271247461903*pbQOc(1.0*VNvlY(DAwhk)+0.5*DAwhk*b4jXr*
rTEdP-0.5*DAwhk*b4jXr*SynchronizationTime+0.5*b4jXr*a84tu-0.5*b4jXr*HcINC));
sZyb6=oTXmt=(-L38Np)/b4jXr;E2jtU=Tdb4k=-0.5*VNvlY(L38Np)/b4jXr;
WT1rZ=PbPkG=DAwhk/b4jXr;XxdWk=NIjjJ=-0.5*VNvlY(DAwhk)/b4jXr;
un6jF=(-4.0*DAwhk)/b4jXr-1.0*rTEdP+SynchronizationTime+(2.8284271247461903*pbQOc
(1.0*VNvlY(DAwhk)+0.5*DAwhk*b4jXr*rTEdP-0.5*DAwhk*b4jXr*SynchronizationTime+0.5*
b4jXr*a84tu-0.5*b4jXr*HcINC))/b4jXr;a69lF=(-DAwhk)*un6jF;
u4XFm+=a84tu*sZyb6+0.5*S8tqs*VNvlY(sZyb6)-b4jXr*qn3x8(sZyb6)/6.0;a84tu+=E2jtU;
S8tqs=L38Np;
u4XFm+=a84tu*oTXmt+0.5*S8tqs*VNvlY(oTXmt)+b4jXr*qn3x8(oTXmt)/6.0;a84tu+=Tdb4k;
S8tqs=0.0;
u4XFm+=a84tu*WT1rZ+0.5*S8tqs*VNvlY(WT1rZ)-b4jXr*qn3x8(WT1rZ)/6.0;a84tu+=XxdWk;
S8tqs=-DAwhk;
u4XFm+=a84tu*un6jF+0.5*S8tqs*VNvlY(un6jF);
a84tu+=a69lF;S8tqs=-DAwhk;
u4XFm+=a84tu*PbPkG+0.5*S8tqs*VNvlY(PbPkG)+b4jXr*qn3x8(PbPkG)/6.0;
return(u4XFm>=ldeCA);}


bool qPN_6::oi7CN(const double&k6sf4,const double&AQzqu,const double&HcINC,const
 double&iPzPj,const double&ldeCA,const double&b4jXr,const double&rTEdP,const 
double&SynchronizationTime){double fTK9I=0.0
,iqKv9=0.0
,cjYid=0.0
,FrEi2=0.0
,jmlSk=0.0
,LDLfE=0.0
,FBPho=0.0
,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4;
fTK9I=0.5*S8tqs+0.5*b4jXr*rTEdP-0.5*b4jXr*SynchronizationTime+0.25*b4jXr*pbQOc((
-4.*VNvlY(S8tqs))/VNvlY(b4jXr)+(8.0*S8tqs*rTEdP)/b4jXr+4.0*VNvlY(rTEdP)-(8.0*
S8tqs*SynchronizationTime)/b4jXr-8.0*rTEdP*SynchronizationTime+4.0*VNvlY(
SynchronizationTime)-(16.0*a84tu)/b4jXr+(16.0*HcINC)/b4jXr);
iqKv9=(S8tqs-fTK9I)/b4jXr;jmlSk=S8tqs*iqKv9-0.5*b4jXr*VNvlY(iqKv9);
FrEi2=(-fTK9I)/b4jXr;FBPho=-0.5*VNvlY(fTK9I)/b4jXr;
cjYid=SynchronizationTime-rTEdP-iqKv9-FrEi2;LDLfE=cjYid*fTK9I;
u4XFm+=a84tu*iqKv9+0.5*S8tqs*VNvlY(iqKv9)-b4jXr*qn3x8(iqKv9)/6.0;a84tu+=jmlSk;
S8tqs=fTK9I;
u4XFm+=a84tu*cjYid+0.5*S8tqs*VNvlY(cjYid);a84tu+=LDLfE;S8tqs=fTK9I;
u4XFm+=a84tu*FrEi2+0.5*S8tqs*VNvlY(FrEi2)+b4jXr*qn3x8(FrEi2)/6.0;return(u4XFm>=
ldeCA);}


bool qPN_6::fvLgK(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr,const 
double&rTEdP,const double&SynchronizationTime){double fTK9I=0.0
,WT1rZ=0.0
,FrEi2=0.0
,cjYid=0.0
,EFQ3W=0.0
,XxdWk=0.0
,FBPho=0.0
,LDLfE=0.0
,jCsL3=0.0
,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4;
fTK9I=(b4jXr*(a84tu-HcINC)-VNvlY(DAwhk))/(2.0*DAwhk-b4jXr*(SynchronizationTime-
rTEdP));
WT1rZ=(DAwhk+S8tqs)/b4jXr;XxdWk=S8tqs*WT1rZ-0.5*b4jXr*VNvlY(WT1rZ);
FrEi2=(fTK9I+DAwhk)/b4jXr;FBPho=-DAwhk*FrEi2+0.5*b4jXr*VNvlY(FrEi2);
EFQ3W=(-fTK9I)/b4jXr;jCsL3=-0.5*VNvlY(fTK9I)/b4jXr;
cjYid=SynchronizationTime-rTEdP-WT1rZ-FrEi2-EFQ3W;LDLfE=fTK9I*cjYid;
u4XFm+=a84tu*WT1rZ+0.5*S8tqs*VNvlY(WT1rZ)-b4jXr*qn3x8(WT1rZ)/6.0;a84tu+=XxdWk;
S8tqs=-DAwhk,
u4XFm+=a84tu*FrEi2+0.5*S8tqs*VNvlY(FrEi2)+b4jXr*qn3x8(FrEi2)/6.0;a84tu+=FBPho;
S8tqs=fTK9I;
u4XFm+=a84tu*cjYid+0.5*S8tqs*VNvlY(cjYid);a84tu+=LDLfE;S8tqs=fTK9I;
u4XFm+=a84tu*EFQ3W+0.5*S8tqs*VNvlY(EFQ3W)+b4jXr*qn3x8(EFQ3W)/6.0;return(u4XFm>=
ldeCA);}


bool qPN_6::tHR7F(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr,const 
double&rTEdP,const double&SynchronizationTime){double fTK9I=0.0
,iqKv9=0.0
,cjYid=0.0
,WT1rZ=0.0
,EFQ3W=0.0
,jmlSk=0.0
,LDLfE=0.0
,XxdWk=0.0
,jCsL3=0.0
,Yoeil=0.0,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4;Yoeil=SynchronizationTime-rTEdP;
fTK9I=(b4jXr*(a84tu-HcINC)-VNvlY(DAwhk))/(2.0*DAwhk-Yoeil*b4jXr);
iqKv9=(S8tqs-fTK9I)/b4jXr;jmlSk=S8tqs*iqKv9-0.5*b4jXr*VNvlY(iqKv9);
WT1rZ=(DAwhk+fTK9I)/b4jXr;XxdWk=fTK9I*WT1rZ-0.5*WT1rZ*(DAwhk+fTK9I);
EFQ3W=DAwhk/b4jXr;jCsL3=-0.5*VNvlY(DAwhk)/b4jXr;
cjYid=SynchronizationTime-rTEdP-iqKv9-WT1rZ-EFQ3W;LDLfE=fTK9I*cjYid;
u4XFm+=a84tu*iqKv9+0.5*S8tqs*VNvlY(iqKv9)-b4jXr*qn3x8(iqKv9)/6.0;a84tu+=jmlSk;
S8tqs=fTK9I;
u4XFm+=a84tu*cjYid+0.5*S8tqs*VNvlY(cjYid);a84tu+=LDLfE;S8tqs=fTK9I;
u4XFm+=a84tu*WT1rZ+0.5*S8tqs*VNvlY(WT1rZ)-b4jXr*qn3x8(WT1rZ)/6.0;a84tu+=XxdWk;
S8tqs=-DAwhk,
u4XFm+=a84tu*EFQ3W+0.5*S8tqs*VNvlY(EFQ3W)+b4jXr*qn3x8(EFQ3W)/6.0;return(u4XFm>=
ldeCA);}


bool qPN_6::m4BSO(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr,const 
double&rTEdP,const double&SynchronizationTime){
double WT1rZ=0.0
,PbPkG=0.0
,cbBNZ=0.0
,MixVF=0.0
,Bczkr=0.0
,ynw_o=0.0
,XxdWk=0.0
,NIjjJ=0.0
,tQ417=0.0
,eLwei=0.0
,k1KD8=0.0
,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4;
WT1rZ=PbPkG=MixVF=ynw_o=DAwhk/b4jXr;XxdWk=NIjjJ=tQ417=k1KD8=-0.5*VNvlY(DAwhk)/
b4jXr;
eLwei=HcINC-a84tu-XxdWk-NIjjJ-tQ417-k1KD8;Bczkr=eLwei/(-DAwhk);
cbBNZ=SynchronizationTime-rTEdP-WT1rZ-PbPkG-MixVF-Bczkr-ynw_o;
u4XFm+=a84tu*WT1rZ+0.5*S8tqs*VNvlY(WT1rZ)-b4jXr*qn3x8(WT1rZ)/6.0;a84tu+=XxdWk;
S8tqs=-DAwhk;
u4XFm+=a84tu*PbPkG+0.5*S8tqs*VNvlY(PbPkG)+b4jXr*qn3x8(PbPkG)/6.0;a84tu+=NIjjJ;
S8tqs=0.0;
u4XFm+=a84tu*cbBNZ;
a84tu=a84tu;S8tqs=0.0;
u4XFm+=a84tu*MixVF+0.5*S8tqs*VNvlY(MixVF)-b4jXr*qn3x8(MixVF)/6.0;a84tu+=tQ417;
S8tqs=-DAwhk;
u4XFm+=a84tu*Bczkr+0.5*S8tqs*VNvlY(Bczkr);
a84tu+=eLwei;S8tqs=-DAwhk;
u4XFm+=a84tu*ynw_o+0.5*S8tqs*VNvlY(ynw_o)+b4jXr*qn3x8(ynw_o)/6.0;
return(u4XFm>=ldeCA);}


bool qPN_6::Zr_Ql(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr,const 
double&rTEdP,const double&SynchronizationTime){
double MixVF=0.0
,Bczkr=0.0
,ynw_o=0.0
,cbBNZ=0.0
,WT1rZ=0.0
,PbPkG=0.0
,tQ417=0.0
,eLwei=0.0
,k1KD8=0.0
,XxdWk=0.0
,NIjjJ=0.0
,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4;
MixVF=ynw_o=WT1rZ=PbPkG=DAwhk/b4jXr;tQ417=k1KD8=XxdWk=NIjjJ=-0.5*VNvlY(DAwhk)/
b4jXr;
eLwei=HcINC-a84tu-tQ417-k1KD8-XxdWk-NIjjJ;Bczkr=eLwei/(-DAwhk);
cbBNZ=SynchronizationTime-rTEdP-MixVF-Bczkr-ynw_o-WT1rZ-PbPkG;
u4XFm+=a84tu*MixVF+0.5*S8tqs*VNvlY(MixVF)-b4jXr*qn3x8(MixVF)/6.0;a84tu+=tQ417;
S8tqs=-DAwhk;
u4XFm+=a84tu*Bczkr+0.5*S8tqs*VNvlY(Bczkr);
a84tu+=eLwei;S8tqs=-DAwhk;
u4XFm+=a84tu*ynw_o+0.5*S8tqs*VNvlY(ynw_o)+b4jXr*qn3x8(ynw_o)/6.0;
a84tu+=k1KD8;S8tqs=0.0;
u4XFm+=a84tu*cbBNZ;
a84tu=a84tu;S8tqs=0.0;
u4XFm+=a84tu*WT1rZ+0.5*S8tqs*VNvlY(WT1rZ)-b4jXr*qn3x8(WT1rZ)/6.0;a84tu+=XxdWk;
S8tqs=-DAwhk;
u4XFm+=a84tu*PbPkG+0.5*S8tqs*VNvlY(PbPkG)+b4jXr*qn3x8(PbPkG)/6.0;return(u4XFm>=
ldeCA);}


bool qPN_6::d6tlK(const double&DAwhk,const double&AQzqu,const double&HcINC,const
 double&b4jXr,const double&rTEdP,const double&SynchronizationTime){
double WT1rZ=0.0
,PbPkG=0.0
,sZyb6=0.0
,oTXmt=0.0
,XxdWk=0.0
,NIjjJ=0.0
,L38Np=0.0
,JMvCa=HcINC,a84tu=AQzqu;
WT1rZ=PbPkG=DAwhk/b4jXr;XxdWk=NIjjJ=-0.5*VNvlY(DAwhk)/b4jXr;
JMvCa=-JMvCa;a84tu=-a84tu;XxdWk=-XxdWk;NIjjJ=-NIjjJ;
L38Np=pbQOc(b4jXr*(JMvCa-a84tu-XxdWk-NIjjJ));
JMvCa=-JMvCa;a84tu=-a84tu;XxdWk=-XxdWk;NIjjJ=-NIjjJ;L38Np=-L38Np;
sZyb6=oTXmt=(-L38Np)/b4jXr;return((rTEdP+WT1rZ+PbPkG+sZyb6+oTXmt)>=
SynchronizationTime);}


bool qPN_6::mDvwC(const double&DAwhk,const double&AQzqu,const double&HcINC,const
 double&b4jXr,const double&rTEdP,const double&SynchronizationTime){
double sZyb6=0.0
,oTXmt=0.0
,WT1rZ=0.0
,PbPkG=0.0
,XxdWk=0.0
,NIjjJ=0.0
,L38Np=0.0,JMvCa=HcINC,a84tu=AQzqu;
WT1rZ=PbPkG=DAwhk/b4jXr;XxdWk=NIjjJ=-0.5*VNvlY(DAwhk)/b4jXr;
JMvCa=-JMvCa;a84tu=-a84tu;XxdWk=-XxdWk;NIjjJ=-NIjjJ;
L38Np=pbQOc(b4jXr*(JMvCa-a84tu-XxdWk-NIjjJ));
JMvCa=-JMvCa;a84tu=-a84tu;XxdWk=-XxdWk;NIjjJ=-NIjjJ;L38Np=-L38Np;
sZyb6=oTXmt=(-L38Np)/b4jXr;return((rTEdP+sZyb6+oTXmt+WT1rZ+PbPkG)>=
SynchronizationTime);}


bool qPN_6::x3iCv(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr,const 
double&rTEdP,const double&SynchronizationTime){
return(vSRz9(k6sf4,DAwhk,AQzqu,HcINC,iPzPj,ldeCA,b4jXr,rTEdP,SynchronizationTime
));}


bool qPN_6::VPqtN(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr,const 
double&rTEdP,const double&SynchronizationTime){
double sZyb6=0.0
,oTXmt=0.0
,cbBNZ=0.0
,WT1rZ=0.0
,PbPkG=0.0
,E2jtU=0.0
,Tdb4k=0.0
,XxdWk=0.0
,NIjjJ=0.0
,L38Np=0.0,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4,JMvCa=HcINC;
WT1rZ=PbPkG=DAwhk/b4jXr;XxdWk=NIjjJ=-0.5*VNvlY(DAwhk)/b4jXr;
JMvCa=-JMvCa;a84tu=-a84tu;XxdWk=-XxdWk;NIjjJ=-NIjjJ;
L38Np=pbQOc(b4jXr*(JMvCa-a84tu-XxdWk-NIjjJ));
JMvCa=-JMvCa;a84tu=-a84tu;XxdWk=-XxdWk;NIjjJ=-NIjjJ;
sZyb6=oTXmt=L38Np/b4jXr;E2jtU=Tdb4k=-0.5*VNvlY(L38Np)/b4jXr;
cbBNZ=SynchronizationTime-rTEdP-sZyb6-oTXmt-WT1rZ-PbPkG;
u4XFm+=a84tu*sZyb6+0.5*S8tqs*VNvlY(sZyb6)-b4jXr*qn3x8(sZyb6)/6.0;a84tu+=E2jtU;
S8tqs=-L38Np;
u4XFm+=a84tu*oTXmt+0.5*S8tqs*VNvlY(oTXmt)+b4jXr*qn3x8(oTXmt)/6.0;a84tu+=Tdb4k;
S8tqs=0.0;
u4XFm+=a84tu*cbBNZ;
a84tu=a84tu;S8tqs=0.0;
u4XFm+=a84tu*WT1rZ+0.5*S8tqs*VNvlY(WT1rZ)-b4jXr*qn3x8(WT1rZ)/6.0;a84tu+=XxdWk;
S8tqs=-DAwhk;
u4XFm+=a84tu*PbPkG+0.5*S8tqs*VNvlY(PbPkG)+b4jXr*qn3x8(PbPkG)/6.0;return(u4XFm>=
ldeCA);}


bool qPN_6::bCvNP(const double&AQzqu,const double&HcINC,const double&iPzPj,const
 double&ldeCA,const double&b4jXr,const double&rTEdP,const double&
SynchronizationTime){
return(Yijm0(AQzqu,HcINC,iPzPj,ldeCA,b4jXr,rTEdP,SynchronizationTime));}


bool qPN_6::pCBdf(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr,const 
double&rTEdP,const double&SynchronizationTime){
double WT1rZ=0.0
,PbPkG=0.0
,cbBNZ=0.0
,sZyb6=0.0
,oTXmt=0.0
,XxdWk=0.0
,NIjjJ=0.0
,E2jtU=0.0
,Tdb4k=0.0
,L38Np=0.0
,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4,JMvCa=HcINC;
WT1rZ=PbPkG=DAwhk/b4jXr;XxdWk=NIjjJ=-0.5*VNvlY(DAwhk)/b4jXr;
JMvCa=-JMvCa;a84tu=-a84tu;XxdWk=-XxdWk;NIjjJ=-NIjjJ;
L38Np=pbQOc(b4jXr*(JMvCa-a84tu-XxdWk-NIjjJ));
JMvCa=-JMvCa;a84tu=-a84tu;XxdWk=-XxdWk;NIjjJ=-NIjjJ;L38Np=-L38Np;
sZyb6=oTXmt=(-L38Np)/b4jXr;E2jtU=Tdb4k=-0.5*VNvlY(L38Np)/b4jXr;
cbBNZ=SynchronizationTime-rTEdP-WT1rZ-PbPkG-sZyb6-oTXmt;
u4XFm+=a84tu*WT1rZ+0.5*S8tqs*VNvlY(WT1rZ)-b4jXr*qn3x8(WT1rZ)/6.0;a84tu+=XxdWk;
S8tqs=-DAwhk;
u4XFm+=a84tu*PbPkG+0.5*S8tqs*VNvlY(PbPkG)+b4jXr*qn3x8(PbPkG)/6.0;a84tu+=NIjjJ;
S8tqs=0.0;
u4XFm+=a84tu*cbBNZ;
a84tu=a84tu;S8tqs=0.0;
u4XFm+=a84tu*sZyb6+0.5*S8tqs*VNvlY(sZyb6)-b4jXr*qn3x8(sZyb6)/6.0;a84tu+=E2jtU;
S8tqs=L38Np;
u4XFm+=a84tu*oTXmt+0.5*S8tqs*VNvlY(oTXmt)+b4jXr*qn3x8(oTXmt)/6.0;return(u4XFm>=
ldeCA);}
