
































#include <TypeIVRMLStep1Decisions.h>
#include <TypeIVRMLMath.h>
#include <TypeIVRMLDefinitions.h>
#include <TypeIVRMLStep1Profiles.h>
#include <TypeIVRMLABK.h>
#include <TypeIVRMLStep1RootFunctions.h>
#include <math.h>
#include <stdlib.h>








bool qPN_6::L5k8N(const double&k6sf4){return(k6sf4>=0.0);}


bool qPN_6::t3fQZ(const double&k6sf4,const double&DAwhk){return(k6sf4<=DAwhk);}


bool qPN_6::RNszr(const double&k6sf4,const double&AQzqu,const double&hxixi,const
 double&b4jXr){double a84tu=AQzqu,Time=0.0;Time=k6sf4/b4jXr;a84tu+=k6sf4*Time-
0.5*b4jXr*VNvlY(Time);return(a84tu<=hxixi);}


bool qPN_6::KbW6_(const double&AQzqu,const double&hxixi){return(AQzqu>=-hxixi);}


bool qPN_6::wKs7P(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&b4jXr){

double a84tu=AQzqu,Time=0.0;Time=(DAwhk-k6sf4)/b4jXr;
a84tu+=k6sf4*Time+0.5*b4jXr*VNvlY(Time);return(a84tu>-hxixi);}


bool qPN_6::mWwy9(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&b4jXr){return(P90PB(k6sf4,DAwhk,AQzqu,HcINC,b4jXr));}


bool qPN_6::bSu3N(const double&k6sf4,const double&AQzqu,const double&hxixi,const
 double&b4jXr){

double a84tu=AQzqu,S8tqs=k6sf4,Time=0.0,L38Np=0.0;

L38Np=pbQOc(VNvlY(S8tqs)-2.0*b4jXr*(hxixi+a84tu));
a84tu=-hxixi;S8tqs=L38Np;

Time=S8tqs/b4jXr;a84tu+=S8tqs*Time-0.5*b4jXr*VNvlY(Time);return(a84tu<=hxixi);}


bool qPN_6::y15i4(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&b4jXr){

double a84tu=AQzqu,S8tqs=k6sf4,Time=0.0;
a84tu=-hxixi;S8tqs=DAwhk;
Time=DAwhk/b4jXr;a84tu+=S8tqs*Time-0.5*b4jXr*VNvlY(Time);return(a84tu>hxixi);}


bool qPN_6::CpvSN(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&b4jXr){

double a84tu=AQzqu,S8tqs=k6sf4,Time=0.0;
Time=(DAwhk-S8tqs)/b4jXr;a84tu+=S8tqs*Time+0.5*b4jXr*VNvlY(Time);S8tqs=DAwhk;
Time=DAwhk/b4jXr;a84tu+=DAwhk*Time-0.5*b4jXr*VNvlY(Time);return(a84tu>hxixi);}


bool qPN_6::_3B72(const double&k6sf4,const double&AQzqu,const double&HcINC,const
 double&hxixi,const double&b4jXr){
double a84tu=AQzqu,S8tqs=k6sf4,Time=0.0;
Time=S8tqs/b4jXr;a84tu+=S8tqs*Time-0.5*b4jXr*VNvlY(Time);if(a84tu>hxixi){a84tu=
hxixi;}return(a84tu<=HcINC);}


bool qPN_6::P90PB(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&b4jXr){
double a84tu=AQzqu,S8tqs=k6sf4,Time=0.0;
Time=(DAwhk-S8tqs)/b4jXr;a84tu+=S8tqs*Time+0.5*b4jXr*VNvlY(Time);S8tqs=DAwhk;
Time=DAwhk/b4jXr;a84tu+=S8tqs*Time-0.5*b4jXr*VNvlY(Time);return(a84tu<=HcINC);}


bool qPN_6::kaAIh(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr){
double cjcAG=0.0
,StX1T=0.0
,m0JG3=0.0
,sddwV=0.0
,rT061=0.0
,y71cs=0.0
,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4;
cjcAG=(DAwhk-S8tqs)/b4jXr;
m0JG3=DAwhk/b4jXr;sddwV=S8tqs*cjcAG+0.5*b4jXr*VNvlY(cjcAG);y71cs=0.5*VNvlY(DAwhk
)/b4jXr;
rT061=HcINC-a84tu-sddwV-y71cs;StX1T=rT061/DAwhk;
u4XFm+=a84tu*cjcAG+0.5*S8tqs*VNvlY(cjcAG)+b4jXr*qn3x8(cjcAG)/6.0;a84tu+=sddwV;
S8tqs=DAwhk;
u4XFm+=a84tu*StX1T+0.5*S8tqs*VNvlY(StX1T);
a84tu+=rT061;S8tqs=DAwhk;
u4XFm+=a84tu*m0JG3+0.5*S8tqs*VNvlY(m0JG3)-b4jXr*qn3x8(m0JG3)/6.0;
return(u4XFm<=ldeCA);}


bool qPN_6::GYWO5(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&HcINC,const double&b4jXr){

double a84tu=AQzqu,S8tqs=k6sf4,Time=0.0;
a84tu=hxixi;S8tqs=0.0;
Time=DAwhk/b4jXr;a84tu+=S8tqs*Time-0.5*b4jXr*VNvlY(Time);S8tqs=-DAwhk;


a84tu+=S8tqs*Time+0.5*b4jXr*VNvlY(Time);return(a84tu<=HcINC);}


bool qPN_6::Io38i(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&HcINC,const double&iPzPj,const double&ldeCA,const 
double&b4jXr){

double Time=0.0
,cjcAG=0.0
,StX1T=0.0
,m0JG3=0.0
,sddwV=0.0
,rT061=0.0
,y71cs=0.0
,L38Np=0.0
,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4,JMvCa=HcINC;

cjcAG=(DAwhk-S8tqs)/b4jXr;
m0JG3=DAwhk/b4jXr;sddwV=S8tqs*cjcAG+0.5*b4jXr*VNvlY(cjcAG);y71cs=0.5*VNvlY(DAwhk
)/b4jXr;
rT061=hxixi-a84tu-sddwV-y71cs;StX1T=rT061/DAwhk;
u4XFm+=a84tu*cjcAG+0.5*S8tqs*VNvlY(cjcAG)+b4jXr*qn3x8(cjcAG)/6.0;a84tu+=sddwV;
S8tqs=DAwhk;
u4XFm+=a84tu*StX1T+0.5*S8tqs*VNvlY(StX1T);
a84tu+=rT061;S8tqs=DAwhk;
u4XFm+=a84tu*m0JG3+0.5*S8tqs*VNvlY(m0JG3)-b4jXr*qn3x8(m0JG3)/6.0;
a84tu=hxixi;S8tqs=0.0;

a84tu=-a84tu;JMvCa=-JMvCa;
L38Np=pbQOc(b4jXr*(JMvCa-a84tu));
a84tu=-a84tu;JMvCa=-JMvCa;L38Np=-L38Np;
Time=(-L38Np)/b4jXr;u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)-b4jXr*qn3x8(Time)/
6.0;a84tu+=S8tqs*Time-0.5*b4jXr*VNvlY(Time);S8tqs=L38Np;


u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)+b4jXr*qn3x8(Time)/6.0;return(u4XFm<=
ldeCA);}


bool qPN_6::PBnqj(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr){

double HB9ew=0.0
,StX1T=0.0
,nwQQL=0.0
,WT1rZ=0.0
,PbPkG=0.0
,lURFQ=0.0
,rT061=0.0
,FYkEp=0.0
,XxdWk=0.0
,NIjjJ=0.0
,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4;
HB9ew=(DAwhk-S8tqs)/b4jXr;
nwQQL=WT1rZ=PbPkG=DAwhk/b4jXr;lURFQ=S8tqs*HB9ew+0.5*b4jXr*VNvlY(HB9ew);FYkEp=0.5
*VNvlY(DAwhk)/b4jXr;XxdWk=NIjjJ=-0.5*VNvlY(DAwhk)/b4jXr;
rT061=HcINC-a84tu-lURFQ-FYkEp-XxdWk-NIjjJ;StX1T=rT061/DAwhk;
u4XFm+=a84tu*HB9ew+0.5*S8tqs*VNvlY(HB9ew)+b4jXr*qn3x8(HB9ew)/6.0;a84tu+=lURFQ;
S8tqs=DAwhk;
u4XFm+=a84tu*StX1T+0.5*S8tqs*VNvlY(StX1T);
a84tu+=rT061;S8tqs=DAwhk;
u4XFm+=a84tu*nwQQL+0.5*S8tqs*VNvlY(nwQQL)-b4jXr*qn3x8(nwQQL)/6.0;
a84tu+=FYkEp;S8tqs=0.0;
u4XFm+=a84tu*WT1rZ-b4jXr*qn3x8(WT1rZ)/6.0;a84tu+=XxdWk;S8tqs=-DAwhk;
u4XFm+=a84tu*PbPkG+0.5*S8tqs*VNvlY(PbPkG)+b4jXr*qn3x8(PbPkG)/6.0;
return(u4XFm<=ldeCA);}


bool qPN_6::xDxct(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&HcINC,const double&iPzPj,const double&ldeCA,const 
double&b4jXr){

double cjcAG=0.0
,StX1T=0.0
,m0JG3=0.0
,sddwV=0.0
,rT061=0.0
,y71cs=0.0
,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4;

cjcAG=(DAwhk-S8tqs)/b4jXr;
m0JG3=DAwhk/b4jXr;sddwV=S8tqs*cjcAG+0.5*b4jXr*VNvlY(cjcAG);y71cs=0.5*VNvlY(DAwhk
)/b4jXr;
rT061=hxixi-a84tu-sddwV-y71cs;StX1T=rT061/DAwhk;
u4XFm+=a84tu*cjcAG+0.5*S8tqs*VNvlY(cjcAG)+b4jXr*qn3x8(cjcAG)/6.0;a84tu+=sddwV;
S8tqs=DAwhk;
u4XFm+=a84tu*StX1T+0.5*S8tqs*VNvlY(StX1T);
a84tu+=rT061;S8tqs=DAwhk;
u4XFm+=a84tu*m0JG3+0.5*S8tqs*VNvlY(m0JG3)-b4jXr*qn3x8(m0JG3)/6.0;
a84tu=hxixi;S8tqs=0.0;

cjcAG=m0JG3=DAwhk/b4jXr;sddwV=y71cs=-0.5*VNvlY(DAwhk)/b4jXr;
rT061=HcINC-a84tu-sddwV-y71cs;StX1T=rT061/(-DAwhk);
u4XFm+=a84tu*cjcAG-b4jXr*qn3x8(cjcAG)/6.0;a84tu+=sddwV;S8tqs=-DAwhk;
u4XFm+=a84tu*StX1T+0.5*S8tqs*VNvlY(StX1T);
a84tu+=rT061;S8tqs=-DAwhk;
u4XFm+=a84tu*m0JG3+0.5*S8tqs*VNvlY(m0JG3)+b4jXr*qn3x8(m0JG3)/6.0;
return(u4XFm<=ldeCA);}


bool qPN_6::fwKhJ(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&b4jXr){
double Time=0.0,a84tu=AQzqu,S8tqs=k6sf4;
Time=S8tqs/b4jXr;a84tu+=S8tqs*Time-0.5*b4jXr*VNvlY(Time);S8tqs=0.0;
Time=DAwhk/b4jXr;a84tu+=S8tqs*Time+0.5*b4jXr*VNvlY(Time);S8tqs=DAwhk;


a84tu+=S8tqs*Time-0.5*b4jXr*VNvlY(Time);return(a84tu<=HcINC);}


bool qPN_6::GwY4E(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr){
double Time=0.0
,cjcAG=0.0
,StX1T=0.0
,m0JG3=0.0
,sddwV=0.0
,rT061=0.0
,y71cs=0.0
,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4;
Time=S8tqs/b4jXr;u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)-b4jXr*qn3x8(Time)/6.0;
a84tu+=S8tqs*Time-0.5*b4jXr*VNvlY(Time);S8tqs=0.0;

cjcAG=m0JG3=DAwhk/b4jXr;sddwV=y71cs=0.5*VNvlY(DAwhk)/b4jXr;
rT061=HcINC-a84tu-sddwV-y71cs;StX1T=rT061/DAwhk;
u4XFm+=a84tu*cjcAG+b4jXr*qn3x8(cjcAG)/6.0;a84tu+=sddwV;S8tqs=DAwhk;
u4XFm+=a84tu*StX1T+0.5*S8tqs*VNvlY(StX1T);
a84tu+=rT061;S8tqs=DAwhk;
u4XFm+=a84tu*m0JG3+0.5*S8tqs*VNvlY(m0JG3)-b4jXr*qn3x8(m0JG3)/6.0;
return(u4XFm<=ldeCA);}


bool qPN_6::w_3wx(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&b4jXr){

double Time=0.0,a84tu=AQzqu,S8tqs=k6sf4;
Time=DAwhk/b4jXr;a84tu+=S8tqs*Time-0.5*b4jXr*VNvlY(Time);S8tqs=-DAwhk;
Time=DAwhk/b4jXr;a84tu+=S8tqs*Time+0.5*b4jXr*VNvlY(Time);return(a84tu>=-hxixi);}


bool qPN_6::xmXMM(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr){
double UKSpI=0.0
,DVdae=0.0
,LIlfP=0.0
,StX1T=0.0
,rGqvR=0.0
,WBuXo=0.0
,ri4IE=0.0
,vwyxx=0.0
,rT061=0.0
,Iqm8o=0.0
,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4;
UKSpI=DVdae=LIlfP=rGqvR=DAwhk/b4jXr;WBuXo=ri4IE=-0.5*VNvlY(DAwhk)/b4jXr;vwyxx=
Iqm8o=0.5*VNvlY(DAwhk)/b4jXr;
rT061=HcINC-a84tu-WBuXo-ri4IE-vwyxx-Iqm8o;StX1T=rT061/DAwhk;
u4XFm+=a84tu*UKSpI-b4jXr*qn3x8(UKSpI)/6.0;a84tu+=WBuXo;S8tqs=-DAwhk;
u4XFm+=a84tu*DVdae+0.5*S8tqs*VNvlY(DVdae)+b4jXr*qn3x8(DVdae)/6.0;
a84tu+=ri4IE;S8tqs=0.0;
u4XFm+=a84tu*LIlfP+b4jXr*qn3x8(LIlfP)/6.0;a84tu+=vwyxx;S8tqs=DAwhk;
u4XFm+=a84tu*StX1T+0.5*S8tqs*VNvlY(StX1T);
a84tu+=rT061;S8tqs=DAwhk;
u4XFm+=a84tu*rGqvR+0.5*S8tqs*VNvlY(rGqvR)-b4jXr*qn3x8(rGqvR)/6.0;
return(u4XFm>=ldeCA);}


bool qPN_6::vp1ru(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&HcINC,const double&iPzPj,const double&ldeCA,const 
double&b4jXr){
double WT1rZ=0.0
,un6jF=0.0
,PbPkG=0.0
,HB9ew=0.0
,zRNIW=0.0
,nwQQL=0.0
,XxdWk=0.0
,a69lF=0.0
,NIjjJ=0.0
,lURFQ=0.0
,Vsamh=0.0
,FYkEp=0.0
,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4;
WT1rZ=PbPkG=HB9ew=nwQQL=DAwhk/b4jXr;XxdWk=NIjjJ=-0.5*VNvlY(DAwhk)/b4jXr;lURFQ=
FYkEp=0.5*VNvlY(DAwhk)/b4jXr;
a69lF=-hxixi-a84tu-XxdWk-NIjjJ;un6jF=a69lF/(-DAwhk);
Vsamh=HcINC-(-hxixi)-lURFQ-FYkEp;zRNIW=Vsamh/DAwhk;

u4XFm+=a84tu*WT1rZ-b4jXr*qn3x8(WT1rZ)/6.0;a84tu+=XxdWk;S8tqs=-DAwhk;
u4XFm+=a84tu*un6jF+0.5*S8tqs*VNvlY(un6jF);
a84tu+=a69lF;S8tqs=-DAwhk;
u4XFm+=a84tu*PbPkG+0.5*S8tqs*VNvlY(PbPkG)+b4jXr*qn3x8(PbPkG)/6.0;
a84tu=-hxixi;S8tqs=0.0;

u4XFm+=a84tu*HB9ew+b4jXr*qn3x8(HB9ew)/6.0;a84tu+=lURFQ;S8tqs=DAwhk;
u4XFm+=a84tu*zRNIW+0.5*S8tqs*VNvlY(zRNIW);
a84tu+=Vsamh;S8tqs=DAwhk;
u4XFm+=a84tu*nwQQL+0.5*S8tqs*VNvlY(nwQQL)-b4jXr*qn3x8(nwQQL)/6.0;
return(u4XFm>=ldeCA);}


bool qPN_6::CLbMG(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&HcINC,const double&iPzPj,const double&ldeCA,const 
double&b4jXr){
double Time=0.0
,cjcAG=0.0
,StX1T=0.0
,m0JG3=0.0
,sddwV=0.0
,rT061=0.0
,y71cs=0.0
,L38Np=0.0
,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4;

a84tu=-a84tu;
L38Np=pbQOc(b4jXr*(hxixi-a84tu));
a84tu=-a84tu;L38Np=-L38Np;
Time=(-L38Np)/b4jXr;u4XFm+=a84tu*Time-b4jXr*qn3x8(Time)/6.0;a84tu+=-0.5*b4jXr*
VNvlY(Time);S8tqs=L38Np;


u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)+b4jXr*qn3x8(Time)/6.0;a84tu=-hxixi;S8tqs
=0.0;

cjcAG=m0JG3=DAwhk/b4jXr;sddwV=y71cs=0.5*VNvlY(DAwhk)/b4jXr;
rT061=HcINC-a84tu-sddwV-y71cs;StX1T=rT061/DAwhk;
u4XFm+=a84tu*cjcAG+b4jXr*qn3x8(cjcAG)/6.0;a84tu+=sddwV;S8tqs=DAwhk;
u4XFm+=a84tu*StX1T+0.5*S8tqs*VNvlY(StX1T);
a84tu+=rT061;S8tqs=DAwhk;
u4XFm+=a84tu*m0JG3+0.5*S8tqs*VNvlY(m0JG3)-b4jXr*qn3x8(m0JG3)/6.0;
return(u4XFm>=ldeCA);}


bool qPN_6::DoM5j(const double&k6sf4,const double&AQzqu,const double&HcINC,const
 double&iPzPj,const double&ldeCA,const double&b4jXr){
double Time=0.0,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4,L38Np=0.0;
Time=S8tqs/b4jXr;u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)-b4jXr*qn3x8(Time)/6.0;
a84tu+=S8tqs*Time-0.5*b4jXr*VNvlY(Time);S8tqs=0.0;

L38Np=pbQOc(b4jXr*(HcINC-a84tu));
Time=L38Np/b4jXr;u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)+b4jXr*qn3x8(Time)/6.0;
a84tu+=S8tqs*Time+0.5*b4jXr*VNvlY(Time);S8tqs=L38Np;


u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)-b4jXr*qn3x8(Time)/6.0;return(u4XFm<=
ldeCA);}


bool qPN_6::aK8vC(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&HcINC,const double&iPzPj,const double&ldeCA,const 
double&b4jXr){return(r9BxJ(&l2be0,&sebzR,&lNzRf,k6sf4,DAwhk,AQzqu,hxixi,HcINC,
iPzPj,ldeCA,b4jXr));}


bool qPN_6::bkH4G(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&b4jXr){
return(w_3wx(k6sf4,DAwhk,AQzqu,hxixi,b4jXr));}


bool qPN_6::cxDC2(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr){
double Time=0.0
,cjcAG=0.0
,m0JG3=0.0
,sddwV=0.0
,y71cs=0.0
,L38Np=0.0
,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4,JMvCa=HcINC;
cjcAG=m0JG3=DAwhk/b4jXr;sddwV=y71cs=0.5*VNvlY(DAwhk)/b4jXr;
a84tu=-a84tu;JMvCa=-JMvCa;sddwV=-sddwV;y71cs=-y71cs;
L38Np=pbQOc(b4jXr*(JMvCa-a84tu-sddwV-y71cs));
a84tu=-a84tu;JMvCa=-JMvCa;sddwV=-sddwV;y71cs=-y71cs;L38Np=-L38Np;
Time=(-L38Np)/b4jXr;u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)-b4jXr*qn3x8(Time)/
6.0;a84tu+=S8tqs*Time-0.5*b4jXr*VNvlY(Time);S8tqs=L38Np;


u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)+b4jXr*qn3x8(Time)/6.0;a84tu+=S8tqs*Time+
0.5*b4jXr*VNvlY(Time);S8tqs=0.0;
u4XFm+=a84tu*cjcAG+b4jXr*qn3x8(cjcAG)/6.0;a84tu+=sddwV;S8tqs=DAwhk;
u4XFm+=a84tu*m0JG3+0.5*S8tqs*VNvlY(m0JG3)-b4jXr*qn3x8(m0JG3)/6.0;
return(u4XFm>=ldeCA);}


bool qPN_6::EqXsF(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&HcINC,const double&b4jXr){
double Time=0.0,a84tu=AQzqu,S8tqs=k6sf4;
a84tu=-hxixi;S8tqs=0.0;
Time=DAwhk/b4jXr;a84tu+=S8tqs*Time+0.5*b4jXr*VNvlY(Time);S8tqs=DAwhk;


a84tu+=S8tqs*Time-0.5*b4jXr*VNvlY(Time);return(a84tu>=HcINC);}


bool qPN_6::HErfB(const double&k6sf4,const double&AQzqu,const double&hxixi,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr){

double Time=0.0,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4,L38Np=0.0;

a84tu=-a84tu;
L38Np=pbQOc(b4jXr*(hxixi-a84tu));
a84tu=-a84tu;L38Np=-L38Np;
Time=(-L38Np)/b4jXr;u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)-b4jXr*qn3x8(Time)/
6.0;a84tu+=S8tqs*Time-0.5*b4jXr*VNvlY(Time);S8tqs=L38Np;


u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)+b4jXr*qn3x8(Time)/6.0;a84tu=-hxixi;S8tqs
=0.0;

L38Np=pbQOc(b4jXr*(HcINC-a84tu));
Time=L38Np/b4jXr;u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)+b4jXr*qn3x8(Time)/6.0;
a84tu+=S8tqs*Time+0.5*b4jXr*VNvlY(Time);S8tqs=L38Np;


u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)-b4jXr*qn3x8(Time)/6.0;return(u4XFm>=
ldeCA);}


bool qPN_6::RpIXV(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr){
return(cxDC2(k6sf4,DAwhk,AQzqu,HcINC,iPzPj,ldeCA,b4jXr));}


bool qPN_6::VCZrp(const double&k6sf4,const double&AQzqu,const double&HcINC,const
 double&iPzPj,const double&ldeCA,const double&b4jXr){
double Time=0.0,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4,L38Np=0.0;
L38Np=pbQOc((VNvlY(S8tqs)+2.0*b4jXr*(HcINC-a84tu))/2.0);
Time=(L38Np-S8tqs)/b4jXr;u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)+b4jXr*qn3x8(
Time)/6.0;a84tu+=S8tqs*Time+0.5*b4jXr*VNvlY(Time);S8tqs=L38Np;
Time=L38Np/b4jXr;u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)-b4jXr*qn3x8(Time)/6.0;
return(u4XFm<=ldeCA);}


bool qPN_6::m1HJ_(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&b4jXr){


double Time=0.0,a84tu=AQzqu,S8tqs=k6sf4;
Time=(DAwhk-S8tqs)/b4jXr;a84tu+=S8tqs*Time+0.5*b4jXr*VNvlY(Time);S8tqs=DAwhk;
Time=DAwhk/b4jXr;a84tu+=S8tqs*Time-0.5*b4jXr*VNvlY(Time);return(a84tu<=hxixi);}


bool qPN_6::Q4MLB(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr){

double Time=0.0,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4,JMvCa=HcINC,L38Np=0.0;
Time=(DAwhk-S8tqs)/b4jXr;u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)+b4jXr*qn3x8(
Time)/6.0;a84tu+=S8tqs*Time+0.5*b4jXr*VNvlY(Time);S8tqs=DAwhk;
Time=DAwhk/b4jXr;u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)-b4jXr*qn3x8(Time)/6.0;
a84tu+=S8tqs*Time-0.5*b4jXr*VNvlY(Time);S8tqs=0.0;

JMvCa=-JMvCa;a84tu=-a84tu;
L38Np=pbQOc(b4jXr*(JMvCa-a84tu));
JMvCa=-JMvCa;a84tu=-a84tu;L38Np=-L38Np;
Time=(-L38Np)/b4jXr;u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)-b4jXr*qn3x8(Time)/
6.0;a84tu+=S8tqs*Time-0.5*b4jXr*VNvlY(Time);S8tqs=L38Np;


u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)+b4jXr*qn3x8(Time)/6.0;return(u4XFm<=
ldeCA);}


bool qPN_6::JIwMI(const double&k6sf4,const double&AQzqu,const double&hxixi,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr){

double Time=0.0,L38Np=0.0,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4,JMvCa=HcINC;

L38Np=pbQOc((VNvlY(S8tqs)+2.0*b4jXr*(hxixi-a84tu))/2.0);
Time=(L38Np-S8tqs)/b4jXr;u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)+b4jXr*qn3x8(
Time)/6.0;a84tu+=S8tqs*Time+0.5*b4jXr*VNvlY(Time);S8tqs=L38Np;
Time=L38Np/b4jXr;u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)-b4jXr*qn3x8(Time)/6.0;
a84tu=hxixi;S8tqs=0.0;

JMvCa=-JMvCa;a84tu=-a84tu;
L38Np=pbQOc(b4jXr*(JMvCa-a84tu));
JMvCa=-JMvCa;a84tu=-a84tu;L38Np=-L38Np;
Time=(-L38Np)/b4jXr;u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)-b4jXr*qn3x8(Time)/
6.0;a84tu+=S8tqs*Time-0.5*b4jXr*VNvlY(Time);S8tqs=L38Np;
Time=(-L38Np)/b4jXr;u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)+b4jXr*qn3x8(Time)/
6.0;return(u4XFm<=ldeCA);}


bool qPN_6::KaAcO(const double&k6sf4,const double&AQzqu,const double&HcINC,const
 double&iPzPj,const double&ldeCA,const double&b4jXr){return(DoM5j(k6sf4,AQzqu,
HcINC,iPzPj,ldeCA,b4jXr));}


bool qPN_6::jBDiW(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&b4jXr){
double Time=0.0,a84tu=AQzqu,S8tqs=k6sf4;
Time=S8tqs/b4jXr;a84tu+=S8tqs*Time-0.5*b4jXr*VNvlY(Time);S8tqs=0.0;
Time=DAwhk/b4jXr;a84tu+=S8tqs*Time-0.5*b4jXr*VNvlY(Time);S8tqs=-DAwhk;


a84tu+=S8tqs*Time+0.5*b4jXr*VNvlY(Time);return(a84tu<=HcINC);}


bool qPN_6::XX0hr(const double&k6sf4,const double&AQzqu,const double&HcINC,const
 double&iPzPj,const double&ldeCA,const double&b4jXr){
double Time=0.0,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4,JMvCa=HcINC,L38Np=0.0;
Time=S8tqs/b4jXr;u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)-b4jXr*qn3x8(Time)/6.0;
a84tu+=S8tqs*Time-0.5*b4jXr*VNvlY(Time);S8tqs=0.0;

a84tu=-a84tu;JMvCa=-JMvCa;
L38Np=pbQOc(b4jXr*(JMvCa-a84tu));
L38Np=-L38Np;a84tu=-a84tu;JMvCa=-JMvCa;
Time=(-L38Np)/b4jXr;u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)-b4jXr*qn3x8(Time)/
6.0;a84tu+=S8tqs*Time-0.5*b4jXr*VNvlY(Time);S8tqs=L38Np;


u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)+b4jXr*qn3x8(Time)/6.0;return(u4XFm<=
ldeCA);}


bool qPN_6::wdgir(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&b4jXr){
double Time=0.0,a84tu=AQzqu,S8tqs=k6sf4;
Time=(DAwhk-S8tqs)/b4jXr;a84tu+=S8tqs*Time+0.5*b4jXr*VNvlY(Time);S8tqs=DAwhk;
Time=DAwhk/b4jXr;a84tu+=S8tqs*Time-0.5*b4jXr*VNvlY(Time);S8tqs=0.0;


a84tu+=S8tqs*Time-0.5*b4jXr*VNvlY(Time);S8tqs=-DAwhk;


a84tu+=S8tqs*Time+0.5*b4jXr*VNvlY(Time);return(a84tu<=HcINC);}


bool qPN_6::TvZOq(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&b4jXr){
return(m1HJ_(k6sf4,DAwhk,AQzqu,hxixi,b4jXr));}


bool qPN_6::bmLNj(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr){
double Time=0.0
,cjcAG=0.0
,m0JG3=0.0
,sddwV=0.0
,y71cs=0.0
,L38Np=0.0
,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4;
cjcAG=m0JG3=DAwhk/b4jXr;sddwV=y71cs=-0.5*VNvlY(DAwhk)/b4jXr;
L38Np=pbQOc((VNvlY(S8tqs)+2.0*b4jXr*(HcINC-a84tu-sddwV-y71cs))/2.0);
Time=(L38Np-S8tqs)/b4jXr;u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)+b4jXr*qn3x8(
Time)/6.0;a84tu+=S8tqs*Time+0.5*b4jXr*VNvlY(Time);S8tqs=L38Np;
Time=L38Np/b4jXr;u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)-b4jXr*qn3x8(Time)/6.0;
a84tu+=S8tqs*Time-0.5*b4jXr*VNvlY(Time);S8tqs=0.0;
u4XFm+=a84tu*cjcAG-b4jXr*qn3x8(cjcAG)/6.0;a84tu+=sddwV;S8tqs=-DAwhk;
u4XFm+=a84tu*m0JG3+0.5*S8tqs*VNvlY(m0JG3)+b4jXr*qn3x8(m0JG3)/6.0;
return(u4XFm<=ldeCA);}


bool qPN_6::jDqO_(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr){
double Time=0.0
,cjcAG=0.0
,StX1T=0.0
,m0JG3=0.0
,sddwV=0.0
,rT061=0.0
,y71cs=0.0
,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4;
Time=(DAwhk-S8tqs)/b4jXr;u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)+b4jXr*qn3x8(
Time)/6.0;a84tu+=S8tqs*Time+0.5*b4jXr*VNvlY(Time);S8tqs=DAwhk;
Time=DAwhk/b4jXr;u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)-b4jXr*qn3x8(Time)/6.0;
a84tu+=S8tqs*Time-0.5*b4jXr*VNvlY(Time);S8tqs=0.0;
cjcAG=m0JG3=DAwhk/b4jXr;sddwV=y71cs=-0.5*VNvlY(DAwhk)/b4jXr;
rT061=HcINC-a84tu-sddwV-y71cs;StX1T=rT061/(-DAwhk);

u4XFm+=a84tu*cjcAG-b4jXr*qn3x8(cjcAG)/6.0;a84tu+=sddwV;S8tqs=-DAwhk;
u4XFm+=a84tu*StX1T+0.5*S8tqs*VNvlY(StX1T);
a84tu+=rT061;S8tqs=-DAwhk;
u4XFm+=a84tu*m0JG3+0.5*S8tqs*VNvlY(m0JG3)+b4jXr*qn3x8(m0JG3)/6.0;
return(u4XFm<=ldeCA);}


bool qPN_6::A0AQ0(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&HcINC,const double&b4jXr){
double Time=0.0,a84tu=AQzqu,S8tqs=k6sf4;
a84tu=hxixi;S8tqs=0.0;
Time=DAwhk/b4jXr;a84tu+=S8tqs*Time-0.5*b4jXr*VNvlY(Time);S8tqs=-DAwhk;
Time=DAwhk/b4jXr;a84tu+=S8tqs*Time+0.5*b4jXr*VNvlY(Time);return(a84tu<=HcINC);}


bool qPN_6::xgMBW(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr){
return(bmLNj(k6sf4,DAwhk,AQzqu,HcINC,iPzPj,ldeCA,b4jXr));}


bool qPN_6::Ftk9o(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&HcINC,const double&iPzPj,const double&ldeCA,const 
double&b4jXr){

double Time=0.0
,cjcAG=0.0
,StX1T=0.0
,m0JG3=0.0
,sddwV=0.0
,rT061=0.0
,y71cs=0.0
,L38Np=0.0
,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4;

L38Np=pbQOc((VNvlY(S8tqs)+2.0*b4jXr*(hxixi-a84tu))/(0x1eac+1511-0x2491));
Time=(L38Np-S8tqs)/b4jXr;u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)+b4jXr*qn3x8(
Time)/6.0;a84tu+=S8tqs*Time+0.5*b4jXr*VNvlY(Time);S8tqs=L38Np;
Time=L38Np/b4jXr;u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)-b4jXr*qn3x8(Time)/6.0;
a84tu=hxixi;S8tqs=0.0;

cjcAG=m0JG3=DAwhk/b4jXr;sddwV=y71cs=-0.5*VNvlY(DAwhk)/b4jXr;
rT061=HcINC-a84tu-sddwV-y71cs;StX1T=rT061/(-DAwhk);
u4XFm+=a84tu*cjcAG-b4jXr*qn3x8(cjcAG)/6.0;a84tu+=sddwV;S8tqs=-DAwhk;
u4XFm+=a84tu*StX1T+0.5*S8tqs*VNvlY(StX1T);
a84tu+=rT061;S8tqs=-DAwhk;
u4XFm+=a84tu*m0JG3+0.5*S8tqs*VNvlY(m0JG3)+b4jXr*qn3x8(m0JG3)/6.0;
return(u4XFm<=ldeCA);}


bool qPN_6::n92gb(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&b4jXr){
return(w_3wx(k6sf4,DAwhk,AQzqu,hxixi,b4jXr));}


bool qPN_6::IORvp(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr){
double Time=0.0
,cjcAG=0.0
,m0JG3=0.0
,sddwV=0.0
,y71cs=0.0
,L38Np=0.0
,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4;
cjcAG=m0JG3=DAwhk/b4jXr;sddwV=y71cs=-0.5*VNvlY(DAwhk)/b4jXr;
u4XFm+=a84tu*cjcAG-b4jXr*qn3x8(cjcAG)/6.0;a84tu+=sddwV;S8tqs=-DAwhk;
u4XFm+=a84tu*m0JG3+0.5*S8tqs*VNvlY(m0JG3)+b4jXr*qn3x8(m0JG3)/6.0;
a84tu+=y71cs;S8tqs=0.0;
L38Np=pbQOc(b4jXr*(HcINC-a84tu));
Time=L38Np/b4jXr;u4XFm+=a84tu*Time+b4jXr*qn3x8(Time)/6.0;a84tu+=0.5*b4jXr*VNvlY(
Time);S8tqs=L38Np;


u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)-b4jXr*qn3x8(Time)/6.0;return(u4XFm>=
ldeCA);}


bool qPN_6::GKDou(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&HcINC,const double&b4jXr){
double Time=0.0,a84tu=AQzqu,S8tqs=k6sf4;
a84tu=-hxixi;S8tqs=0.0;
Time=DAwhk/b4jXr;a84tu+=S8tqs*Time+0.5*b4jXr*VNvlY(Time);S8tqs=DAwhk;


a84tu+=S8tqs*Time-0.5*b4jXr*VNvlY(Time);return(a84tu>=HcINC);}


bool qPN_6::hYWjA(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&HcINC,const double&iPzPj,const double&ldeCA,const 
double&b4jXr){
double Time=0.0
,cjcAG=0.0
,StX1T=0.0
,m0JG3=0.0
,sddwV=0.0
,rT061=0.0
,y71cs=0.0
,L38Np=0.0
,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4;

cjcAG=m0JG3=DAwhk/b4jXr;sddwV=y71cs=-0.5*VNvlY(DAwhk)/b4jXr;
rT061=-hxixi-a84tu-sddwV-y71cs;StX1T=rT061/(-DAwhk);
u4XFm+=a84tu*cjcAG-b4jXr*qn3x8(cjcAG)/6.0;a84tu+=sddwV;S8tqs=-DAwhk;
u4XFm+=a84tu*StX1T+0.5*S8tqs*VNvlY(StX1T);
a84tu+=rT061;S8tqs=-DAwhk;
u4XFm+=a84tu*m0JG3+0.5*S8tqs*VNvlY(m0JG3)+b4jXr*qn3x8(m0JG3)/6.0;
a84tu=-hxixi;S8tqs=0.0;

L38Np=pbQOc(b4jXr*(HcINC-a84tu));
Time=L38Np/b4jXr;u4XFm+=a84tu*Time+b4jXr*qn3x8(Time)/6.0;a84tu+=0.5*b4jXr*VNvlY(
Time);S8tqs=L38Np;


u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)-b4jXr*qn3x8(Time)/6.0;return(u4XFm>=
ldeCA);}


bool qPN_6::iHzcj(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr){
double WT1rZ=0.0
,un6jF=0.0
,PbPkG=0.0
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
u4XFm+=a84tu*WT1rZ-b4jXr*qn3x8(WT1rZ)/6.0;a84tu+=XxdWk;S8tqs=-DAwhk;
u4XFm+=a84tu*un6jF+0.5*S8tqs*VNvlY(un6jF);
a84tu+=a69lF;S8tqs=-DAwhk;
u4XFm+=a84tu*PbPkG+0.5*S8tqs*VNvlY(PbPkG)+b4jXr*qn3x8(PbPkG)/6.0;
a84tu+=NIjjJ;S8tqs=0.0;
u4XFm+=a84tu*HB9ew+b4jXr*qn3x8(HB9ew)/6.0;a84tu+=lURFQ;S8tqs=DAwhk;
u4XFm+=a84tu*nwQQL+0.5*S8tqs*VNvlY(nwQQL)-b4jXr*qn3x8(nwQQL)/6.0;
return(u4XFm>=ldeCA);}


bool qPN_6::t6UJh(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr){
double Time=0.0
,cjcAG=0.0
,StX1T=0.0
,m0JG3=0.0
,sddwV=0.0
,rT061=0.0
,y71cs=0.0
,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4;
Time=S8tqs/b4jXr;u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)-b4jXr*qn3x8(Time)/6.0;
a84tu+=S8tqs*Time-0.5*b4jXr*VNvlY(Time);S8tqs=0.0;

cjcAG=m0JG3=DAwhk/b4jXr;sddwV=y71cs=-0.5*VNvlY(DAwhk)/b4jXr;
rT061=HcINC-a84tu-sddwV-y71cs;StX1T=rT061/(-DAwhk);
u4XFm+=a84tu*cjcAG-b4jXr*qn3x8(cjcAG)/6.0;a84tu+=sddwV;S8tqs=-DAwhk;
u4XFm+=a84tu*StX1T+0.5*S8tqs*VNvlY(StX1T);
a84tu+=rT061;S8tqs=-DAwhk;
u4XFm+=a84tu*m0JG3+0.5*S8tqs*VNvlY(m0JG3)+b4jXr*qn3x8(m0JG3)/6.0;
return(u4XFm<=ldeCA);}


bool qPN_6::lK2zI(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&b4jXr){
return(m1HJ_(k6sf4,DAwhk,AQzqu,hxixi,b4jXr));}







bool qPN_6::f2bmD(const double&k6sf4){return(L5k8N(k6sf4));}


bool qPN_6::U1aEP(const double&k6sf4,const double&DAwhk){
return(t3fQZ(k6sf4,DAwhk));}


bool qPN_6::DlSXv(const double&k6sf4,const double&AQzqu,const double&hxixi,const
 double&b4jXr){
return(RNszr(k6sf4,AQzqu,hxixi,b4jXr));}


bool qPN_6::yTLKU(const double&AQzqu,const double&hxixi){
return(KbW6_(AQzqu,hxixi));}


bool qPN_6::sfoTf(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&b4jXr){

return(wKs7P(k6sf4,DAwhk,AQzqu,hxixi,b4jXr));}


bool qPN_6::PkiuE(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&b4jXr){return(P90PB(k6sf4,DAwhk,AQzqu,HcINC,b4jXr));}


bool qPN_6::vpolI(const double&k6sf4,const double&AQzqu,const double&hxixi,const
 double&b4jXr){

return(bSu3N(k6sf4,AQzqu,hxixi,b4jXr));}


bool qPN_6::tih30(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&b4jXr){

return(y15i4(k6sf4,DAwhk,AQzqu,hxixi,b4jXr));}


bool qPN_6::xpKgT(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&b4jXr){

return(CpvSN(k6sf4,DAwhk,AQzqu,hxixi,b4jXr));}


bool qPN_6::bPIxK(const double&k6sf4,const double&AQzqu,const double&HcINC,const
 double&b4jXr){
double Time=0.0,a84tu=AQzqu,S8tqs=k6sf4;
Time=S8tqs/b4jXr;a84tu+=S8tqs*Time-0.5*b4jXr*VNvlY(Time);return((m85bi(a84tu)==
m85bi(HcINC)));}


bool qPN_6::WwUXd(const double&k6sf4,const double&AQzqu,const double&b4jXr){
double Time=0.0,a84tu=AQzqu,S8tqs=k6sf4;
Time=S8tqs/b4jXr;a84tu+=S8tqs*Time-0.5*b4jXr*VNvlY(Time);return(a84tu<
(0x1212+219-0x12ed));}


bool qPN_6::nNVMq(const double&k6sf4){
return(L5k8N(k6sf4));}


bool qPN_6::N6oSo(const double&k6sf4,const double&AQzqu,const double&HcINC,const
 double&hxixi,const double&b4jXr){
return(_3B72(k6sf4,AQzqu,HcINC,hxixi,b4jXr));}


bool qPN_6::tYza7(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&b4jXr){

return(P90PB(k6sf4,DAwhk,AQzqu,HcINC,b4jXr));}


bool qPN_6::fs_IJ(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr){

double cjcAG=0.0
,StX1T=0.0
,m0JG3=0.0
,sddwV=0.0
,rT061=0.0
,y71cs=0.0
,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4;
cjcAG=(DAwhk-S8tqs)/b4jXr;
m0JG3=DAwhk/b4jXr;sddwV=S8tqs*cjcAG+0.5*b4jXr*VNvlY(cjcAG);y71cs=0.5*VNvlY(DAwhk
)/b4jXr;
rT061=HcINC-a84tu-sddwV-y71cs;StX1T=rT061/DAwhk;
u4XFm+=a84tu*cjcAG+0.5*S8tqs*VNvlY(cjcAG)+b4jXr*qn3x8(cjcAG)/6.0;a84tu+=sddwV;
S8tqs=DAwhk;
u4XFm+=a84tu*StX1T+0.5*S8tqs*VNvlY(StX1T);
a84tu+=rT061;S8tqs=DAwhk;
u4XFm+=a84tu*m0JG3+0.5*S8tqs*VNvlY(m0JG3)-b4jXr*qn3x8(m0JG3)/6.0;
return(u4XFm<=ldeCA+tmU_f+S9HIU*fabs(iPzPj-ldeCA));}


bool qPN_6::ZpPt0(const double&k6sf4,const double&AQzqu,const double&HcINC,const
 double&iPzPj,const double&ldeCA,const double&b4jXr){

double Time=0.0,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4,L38Np=0.0;
L38Np=pbQOc((VNvlY(S8tqs)+2.0*b4jXr*(HcINC-a84tu))/2.0);
Time=(L38Np-S8tqs)/b4jXr;u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)+b4jXr*qn3x8(
Time)/6.0;a84tu+=S8tqs*Time+0.5*b4jXr*VNvlY(Time);S8tqs=L38Np;
Time=L38Np/b4jXr;u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)-b4jXr*qn3x8(Time)/6.0;
return(u4XFm<=ldeCA+tmU_f+S9HIU*fabs(iPzPj-ldeCA));}


bool qPN_6::baMkR(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&b4jXr){

double Time=0.0,a84tu=AQzqu,S8tqs=k6sf4;
Time=(DAwhk+S8tqs)/b4jXr;a84tu+=S8tqs*Time-0.5*b4jXr*VNvlY(Time);S8tqs=-DAwhk;
Time=DAwhk/b4jXr;a84tu+=S8tqs*Time+0.5*b4jXr*VNvlY(Time);return(a84tu<=HcINC);}


bool qPN_6::PrcrY(const double&k6sf4,const double&AQzqu,const double&HcINC,const
 double&iPzPj,const double&ldeCA,const double&b4jXr){

double Time=0.0,L38Np=0.0,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4,JMvCa=HcINC;
S8tqs=-S8tqs;a84tu=-a84tu;JMvCa=-JMvCa;
L38Np=pbQOc((VNvlY(S8tqs)+(JMvCa-a84tu)*2.0*b4jXr)/2.0);
S8tqs=-S8tqs;a84tu=-a84tu;JMvCa=-JMvCa;L38Np=-L38Np;
Time=(S8tqs-L38Np)/b4jXr;u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)-b4jXr*qn3x8(
Time)/6.0;a84tu+=S8tqs*Time-0.5*b4jXr*VNvlY(Time);S8tqs=L38Np;
Time=(-L38Np)/b4jXr;u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)+b4jXr*qn3x8(Time)/
6.0;return(u4XFm>=ldeCA+tmU_f+S9HIU*fabs(iPzPj-ldeCA));}


bool qPN_6::d1__g(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr){

double cjcAG=0.0
,StX1T=0.0
,m0JG3=0.0
,sddwV=0.0
,rT061=0.0
,y71cs=0.0
,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4;
cjcAG=(DAwhk+S8tqs)/b4jXr;sddwV=S8tqs*cjcAG-0.5*b4jXr*VNvlY(cjcAG);
m0JG3=DAwhk/b4jXr;y71cs=-0.5*VNvlY(DAwhk)/b4jXr;
rT061=HcINC-a84tu-sddwV-y71cs;StX1T=rT061/(-DAwhk);
u4XFm+=a84tu*cjcAG+0.5*S8tqs*VNvlY(cjcAG)-b4jXr*qn3x8(cjcAG)/6.0;a84tu+=sddwV;
S8tqs=-DAwhk;
u4XFm+=a84tu*StX1T+0.5*S8tqs*VNvlY(StX1T);
a84tu+=rT061;S8tqs=-DAwhk;
u4XFm+=a84tu*m0JG3+0.5*S8tqs*VNvlY(m0JG3)+b4jXr*qn3x8(m0JG3)/6.0;
return(u4XFm>=ldeCA+tmU_f+S9HIU*fabs(iPzPj-ldeCA));}


bool qPN_6::eCGb6(const double&k6sf4,const double&AQzqu,const double&HcINC,const
 double&b4jXr){
double Time=0.0,a84tu=AQzqu,S8tqs=k6sf4;
Time=(-S8tqs)/b4jXr;a84tu+=S8tqs*Time+0.5*b4jXr*VNvlY(Time);return(a84tu>HcINC);
}







bool qPN_6::n61tF(const double&k6sf4){return(L5k8N(k6sf4));}


bool qPN_6::XX8Ub(const double&k6sf4,const double&DAwhk){
return(t3fQZ(k6sf4,DAwhk));}


bool qPN_6::ShcRT(const double&k6sf4,const double&AQzqu,const double&hxixi,const
 double&b4jXr){
return(RNszr(k6sf4,AQzqu,hxixi,b4jXr));}


bool qPN_6::dGXca(const double&AQzqu,const double&hxixi){
return(KbW6_(AQzqu,hxixi));}


bool qPN_6::ofJY_(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&b4jXr){

return(wKs7P(k6sf4,DAwhk,AQzqu,hxixi,b4jXr));}


bool qPN_6::P4hfD(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&b4jXr){return(P90PB(k6sf4,DAwhk,AQzqu,HcINC,b4jXr));}


bool qPN_6::CEEIA(const double&k6sf4,const double&AQzqu,const double&hxixi,const
 double&b4jXr){

return(bSu3N(k6sf4,AQzqu,hxixi,b4jXr));}


bool qPN_6::A1qgc(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&b4jXr){

return(y15i4(k6sf4,DAwhk,AQzqu,hxixi,b4jXr));}


bool qPN_6::XDwJW(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&b4jXr){

return(CpvSN(k6sf4,DAwhk,AQzqu,hxixi,b4jXr));}


bool qPN_6::TAQ09(const double&k6sf4,const double&AQzqu,const double&b4jXr){
return(WwUXd(k6sf4,AQzqu,b4jXr));}


bool qPN_6::NMj7d(const double&k6sf4){
return(L5k8N(k6sf4));}


bool qPN_6::IpwId(const double&k6sf4,const double&AQzqu,const double&HcINC,const
 double&hxixi,const double&b4jXr){
return(_3B72(k6sf4,AQzqu,HcINC,hxixi,b4jXr));}


bool qPN_6::xrxMX(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&b4jXr){
return(fwKhJ(k6sf4,DAwhk,AQzqu,HcINC,b4jXr));}


bool qPN_6::E2PXV(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr){
return(GwY4E(k6sf4,DAwhk,AQzqu,HcINC,iPzPj,ldeCA,b4jXr));}


bool qPN_6::DcUhz(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&b4jXr){
double Time=0.0,a84tu=AQzqu,S8tqs=k6sf4;
Time=(DAwhk+S8tqs)/b4jXr;a84tu+=S8tqs*Time-0.5*b4jXr*VNvlY(Time);S8tqs=-DAwhk;
Time=DAwhk/b4jXr;a84tu+=S8tqs*Time+0.5*b4jXr*VNvlY(Time);S8tqs=0.0;return(a84tu
>=(0xa36+119-0xaad));}


bool qPN_6::Ne1C8(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr){
return(xmXMM(k6sf4,DAwhk,AQzqu,HcINC,iPzPj,ldeCA,b4jXr));}


bool qPN_6::gqU3_(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&HcINC,const double&iPzPj,const double&ldeCA,const 
double&b4jXr){return(r9BxJ(&p4iJ9,&KsLO9,&S5k0a,-k6sf4,DAwhk,-AQzqu,hxixi,-HcINC
,-iPzPj,-ldeCA,b4jXr));}


bool qPN_6::RPg2y(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr){
double cjcAG=0.0
,StX1T=0.0
,m0JG3=0.0
,sddwV=0.0
,rT061=0.0
,y71cs=0.0
,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4;
cjcAG=(DAwhk+S8tqs)/b4jXr;sddwV=S8tqs*cjcAG-0.5*b4jXr*VNvlY(cjcAG);
m0JG3=DAwhk/b4jXr;y71cs=-0.5*VNvlY(DAwhk)/b4jXr;
rT061=(0x1641+1588-0x1c75)-a84tu-sddwV-y71cs;StX1T=rT061/(-DAwhk);
u4XFm+=a84tu*cjcAG+0.5*S8tqs*VNvlY(cjcAG)-b4jXr*qn3x8(cjcAG)/6.0;a84tu+=sddwV;
S8tqs=-DAwhk;
u4XFm+=a84tu*StX1T+0.5*S8tqs*VNvlY(StX1T);
a84tu+=rT061;S8tqs=-DAwhk;
u4XFm+=a84tu*m0JG3+0.5*S8tqs*VNvlY(m0JG3)+b4jXr*qn3x8(m0JG3)/6.0;
a84tu=0.0;S8tqs=0.0;

cjcAG=m0JG3=DAwhk/b4jXr;sddwV=y71cs=0.5*VNvlY(DAwhk)/b4jXr;
rT061=HcINC-(0x350+7100-0x1f0c)-sddwV-y71cs;StX1T=rT061/DAwhk;
u4XFm+=a84tu*cjcAG+0.5*S8tqs*VNvlY(cjcAG)+b4jXr*qn3x8(cjcAG)/6.0;a84tu+=sddwV;
S8tqs=DAwhk;
u4XFm+=a84tu*StX1T+0.5*S8tqs*VNvlY(StX1T);
a84tu+=rT061;S8tqs=DAwhk;
u4XFm+=a84tu*m0JG3+0.5*S8tqs*VNvlY(m0JG3)-b4jXr*qn3x8(m0JG3)/6.0;
return(u4XFm>=ldeCA);}


bool qPN_6::xOGeL(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&HcINC,const double&iPzPj,const double&ldeCA,const 
double&b4jXr){return(r9BxJ(&gPu6H,&RDM53,&F0eNd,-k6sf4,DAwhk,-AQzqu,hxixi,-HcINC
,-iPzPj,-ldeCA,b4jXr));}


bool qPN_6::fwSUt(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr){
double Time=0.0
,cjcAG=0.0
,StX1T=0.0
,m0JG3=0.0
,sddwV=0.0
,rT061=0.0
,y71cs=0.0
,L38Np=0.0
,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4;
S8tqs=-S8tqs;a84tu=-a84tu;
L38Np=pbQOc((VNvlY(S8tqs)+2.0*b4jXr*((0x11c6+4438-0x231c)-a84tu))/2.0);
S8tqs=-S8tqs;a84tu=-a84tu;L38Np=-L38Np;
Time=(-L38Np+S8tqs)/b4jXr;u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)-b4jXr*qn3x8(
Time)/6.0;a84tu+=S8tqs*Time-0.5*b4jXr*VNvlY(Time);S8tqs=L38Np;
Time=(-L38Np)/b4jXr;u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)+b4jXr*qn3x8(Time)/
6.0;a84tu=0.0;S8tqs=0.0;

cjcAG=m0JG3=DAwhk/b4jXr;sddwV=y71cs=0.5*VNvlY(DAwhk)/b4jXr;
rT061=HcINC-a84tu-sddwV-y71cs;StX1T=rT061/DAwhk;
u4XFm+=a84tu*cjcAG+0.5*S8tqs*VNvlY(cjcAG)+b4jXr*qn3x8(cjcAG)/6.0;a84tu+=sddwV;
S8tqs=DAwhk;
u4XFm+=a84tu*StX1T+0.5*S8tqs*VNvlY(StX1T);
a84tu+=rT061;S8tqs=DAwhk;
u4XFm+=a84tu*m0JG3+0.5*S8tqs*VNvlY(m0JG3)-b4jXr*qn3x8(m0JG3)/6.0;
return(u4XFm>=ldeCA);}


bool qPN_6::kLx4I(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&HcINC,const double&iPzPj,const double&ldeCA,const 
double&b4jXr){return(gqU3_(k6sf4,DAwhk,AQzqu,hxixi,HcINC,iPzPj,ldeCA,b4jXr));}


bool qPN_6::KUS7o(const double&k6sf4,const double&AQzqu,const double&HcINC,const
 double&iPzPj,const double&ldeCA,const double&b4jXr){
return(DoM5j(k6sf4,AQzqu,HcINC,iPzPj,ldeCA,b4jXr));}


bool qPN_6::R1dyY(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&b4jXr){
return(DcUhz(k6sf4,DAwhk,AQzqu,b4jXr));}


bool qPN_6::Sngn1(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr){
return(xmXMM(k6sf4,DAwhk,AQzqu,HcINC,iPzPj,ldeCA,b4jXr));}


bool qPN_6::K4n3U(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr){
return(cxDC2(k6sf4,DAwhk,AQzqu,HcINC,iPzPj,ldeCA,b4jXr));}


bool qPN_6::kg15G(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&HcINC,const double&iPzPj,const double&ldeCA,const 
double&b4jXr){return(c42JT(k6sf4,DAwhk,AQzqu,hxixi,HcINC,iPzPj,ldeCA,b4jXr));}


bool qPN_6::hvltn(const double&DAwhk,const double&HcINC,const double&b4jXr){
double HB9ew=0.0
,nwQQL=0.0
,lURFQ=0.0
,FYkEp=0.0
,EhlHW=0.0;

HB9ew=nwQQL=DAwhk/b4jXr;lURFQ=FYkEp=0.5*VNvlY(DAwhk)/b4jXr;EhlHW=HcINC-lURFQ-
FYkEp;return(EhlHW>(0x582+3306-0x126c));}


bool qPN_6::OyS6O(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr){return(
fwSUt(k6sf4,DAwhk,AQzqu,HcINC,iPzPj,ldeCA,b4jXr));}


bool qPN_6::lebHt(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&HcINC,const double&iPzPj,const double&ldeCA,const 
double&b4jXr){return(c42JT(k6sf4,DAwhk,AQzqu,hxixi,HcINC,iPzPj,ldeCA,b4jXr));}


bool qPN_6::om8ol(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&HcINC,const double&iPzPj,const double&ldeCA,const 
double&b4jXr){return(gqU3_(k6sf4,DAwhk,AQzqu,hxixi,HcINC,iPzPj,ldeCA,b4jXr));}


bool qPN_6::vnQ65(const double&k6sf4,const double&AQzqu,const double&HcINC,const
 double&iPzPj,const double&ldeCA,const double&b4jXr){
double Time=0.0
,L38Np=0.0
,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4;

S8tqs=-S8tqs;a84tu=-a84tu;
L38Np=pbQOc((VNvlY(S8tqs)+2.0*b4jXr*((0x11cf+1923-0x1952)-a84tu))/2.0);
S8tqs=-S8tqs;a84tu=-a84tu;L38Np=-L38Np;
Time=(-L38Np+S8tqs)/b4jXr;u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)-b4jXr*qn3x8(
Time)/6.0;a84tu+=S8tqs*Time-0.5*b4jXr*VNvlY(Time);S8tqs=L38Np;
Time=(-L38Np)/b4jXr;u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)+b4jXr*qn3x8(Time)/
6.0;a84tu=0.0;S8tqs=0.0;

L38Np=pbQOc((HcINC-(0x9bc+6654-0x23ba))*b4jXr);
Time=L38Np/b4jXr;u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)+b4jXr*qn3x8(Time)/6.0;
a84tu+=S8tqs*Time+0.5*b4jXr*VNvlY(Time);S8tqs=L38Np;


u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)-b4jXr*qn3x8(Time)/6.0;return(u4XFm>=
ldeCA);}


bool qPN_6::c42JT(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&HcINC,const double&iPzPj,const double&ldeCA,const 
double&b4jXr){return(r9BxJ(&Ryy_6,&tlbcQ,&QkEpP,-k6sf4,DAwhk,-AQzqu,hxixi,-HcINC
,-iPzPj,-ldeCA,b4jXr));}


bool qPN_6::g45vp(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&HcINC,const double&iPzPj,const double&ldeCA,const 
double&b4jXr){return(r9BxJ(&PmRIf,&HcX34,&JI0hH,k6sf4,DAwhk,AQzqu,hxixi,HcINC,
iPzPj,ldeCA,b4jXr));}


bool qPN_6::_DZiP(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&b4jXr){

double a84tu=AQzqu,S8tqs=k6sf4,Time=0.0;
Time=(DAwhk+S8tqs)/b4jXr;a84tu+=S8tqs*Time-0.5*b4jXr*VNvlY(Time);S8tqs=-DAwhk;
Time=DAwhk/b4jXr;a84tu+=S8tqs*Time+0.5*b4jXr*VNvlY(Time);return(a84tu>HcINC);}


bool qPN_6::Sqxp3(const double&DAwhk,const double&HcINC,const double&b4jXr){

double lURFQ=0.0
,FYkEp=0.0;

lURFQ=FYkEp=0.5*VNvlY(DAwhk)/b4jXr;return((HcINC-lURFQ-FYkEp)>=
(0xdd1+3344-0x1ae1));}


bool qPN_6::Hiv6b(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr){

double WT1rZ=0.0
,un6jF=0.0
,PbPkG=0.0
,HB9ew=0.0
,nwQQL=0.0
,XxdWk=0.0
,a69lF=0.0
,NIjjJ=0.0
,lURFQ=0.0
,FYkEp=0.0
,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4;
WT1rZ=(DAwhk+S8tqs)/b4jXr;XxdWk=S8tqs*WT1rZ-0.5*b4jXr*VNvlY(WT1rZ);
PbPkG=HB9ew=nwQQL=DAwhk/b4jXr;NIjjJ=-0.5*VNvlY(DAwhk)/b4jXr;lURFQ=FYkEp=0.5*
VNvlY(DAwhk)/b4jXr;
a69lF=HcINC-a84tu-XxdWk-NIjjJ-lURFQ-FYkEp;un6jF=a69lF/(-DAwhk);
u4XFm+=a84tu*WT1rZ+0.5*S8tqs*VNvlY(WT1rZ)-b4jXr*qn3x8(WT1rZ)/6.0;a84tu+=XxdWk;
S8tqs=-DAwhk;
u4XFm+=a84tu*un6jF+0.5*S8tqs*VNvlY(un6jF);
a84tu+=a69lF;S8tqs=-DAwhk;
u4XFm+=a84tu*PbPkG+0.5*S8tqs*VNvlY(PbPkG)+b4jXr*qn3x8(PbPkG)/6.0;
a84tu+=NIjjJ;S8tqs=0.0;
u4XFm+=a84tu*HB9ew+0.5*S8tqs*VNvlY(HB9ew)+b4jXr*qn3x8(HB9ew)/6.0;a84tu+=lURFQ;
S8tqs=DAwhk;
u4XFm+=a84tu*nwQQL+0.5*S8tqs*VNvlY(nwQQL)-b4jXr*qn3x8(nwQQL)/6.0;return(u4XFm>=
ldeCA);}


bool qPN_6::r2Pca(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&HcINC,const double&iPzPj,const double&ldeCA,const 
double&b4jXr){return(iwaWv(k6sf4,DAwhk,AQzqu,hxixi,HcINC,iPzPj,ldeCA,b4jXr));}


bool qPN_6::SEljH(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr){
double Time=0.0
,cjcAG=0.0
,StX1T=0.0
,m0JG3=0.0
,sddwV=0.0
,rT061=0.0
,y71cs=0.0
,L38Np=0.0
,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4;

cjcAG=(DAwhk+S8tqs)/b4jXr;sddwV=S8tqs*cjcAG-0.5*b4jXr*VNvlY(cjcAG);
m0JG3=DAwhk/b4jXr;y71cs=-0.5*VNvlY(DAwhk)/b4jXr;
rT061=(0x9af+761-0xca8)-a84tu-sddwV-y71cs;StX1T=rT061/(-DAwhk);
u4XFm+=a84tu*cjcAG+0.5*S8tqs*VNvlY(cjcAG)-b4jXr*qn3x8(cjcAG)/6.0;a84tu+=sddwV;
S8tqs=-DAwhk;
u4XFm+=a84tu*StX1T+0.5*S8tqs*VNvlY(StX1T);
a84tu+=rT061;S8tqs=-DAwhk;
u4XFm+=a84tu*m0JG3+0.5*S8tqs*VNvlY(m0JG3)+b4jXr*qn3x8(m0JG3)/6.0;
a84tu=0.0;S8tqs=0.0;

L38Np=pbQOc((HcINC-(0x91f+5169-0x1d50))*b4jXr);
Time=L38Np/b4jXr;u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)+b4jXr*qn3x8(Time)/6.0;
a84tu+=S8tqs*Time+0.5*b4jXr*VNvlY(Time);S8tqs=L38Np;


u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)-b4jXr*qn3x8(Time)/6.0;return(u4XFm>=
ldeCA);}


bool qPN_6::iwaWv(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&HcINC,const double&iPzPj,const double&ldeCA,const 
double&b4jXr){return(r9BxJ(&eOy_R,&p9gAB,&X79jJ,-k6sf4,DAwhk,-AQzqu,hxixi,-HcINC
,-iPzPj,-ldeCA,b4jXr));}


bool qPN_6::PMwsB(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&b4jXr){return(DcUhz(k6sf4,DAwhk,AQzqu,b4jXr));}


bool qPN_6::qO4bH(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr){

double Time=0.0
,L38Np=0.0
,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4;
Time=(DAwhk+S8tqs)/b4jXr;u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)-b4jXr*qn3x8(
Time)/6.0;a84tu+=S8tqs*Time-0.5*b4jXr*VNvlY(Time);S8tqs=-DAwhk;
Time=DAwhk/b4jXr;u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)+b4jXr*qn3x8(Time)/6.0;
a84tu+=S8tqs*Time+0.5*b4jXr*VNvlY(Time);S8tqs=0.0;

L38Np=pbQOc((HcINC-a84tu)*b4jXr);
Time=L38Np/b4jXr;u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)+b4jXr*qn3x8(Time)/6.0;
a84tu+=S8tqs*Time+0.5*b4jXr*VNvlY(Time);S8tqs=L38Np;


u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)-b4jXr*qn3x8(Time)/6.0;return(u4XFm>=
ldeCA);}


bool qPN_6::Oqm0M(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&HcINC,const double&iPzPj,const double&ldeCA,const 
double&b4jXr){return(c42JT(k6sf4,DAwhk,AQzqu,hxixi,HcINC,iPzPj,ldeCA,b4jXr));}


bool qPN_6::ay0sl(const double&k6sf4,const double&AQzqu,const double&HcINC,const
 double&b4jXr){
return(eCGb6(k6sf4,AQzqu,HcINC,b4jXr));}


bool qPN_6::uM3fA(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&b4jXr){
double Time=0.0
,a84tu=AQzqu,S8tqs=k6sf4;
Time=-S8tqs/b4jXr;a84tu+=0.5*S8tqs*Time;S8tqs=0.0;
Time=DAwhk/b4jXr;a84tu+=S8tqs*Time-0.5*b4jXr*VNvlY(Time);S8tqs=-DAwhk;


a84tu+=S8tqs*Time+0.5*b4jXr*VNvlY(Time);return(a84tu>HcINC);}


bool qPN_6::ltkxg(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr){
double Time=0.0
,cjcAG=0.0
,StX1T=0.0
,m0JG3=0.0
,sddwV=0.0
,rT061=0.0
,y71cs=0.0
,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4;
Time=-S8tqs/b4jXr;u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)+b4jXr*qn3x8(Time)/6.0;
a84tu+=S8tqs*Time+0.5*b4jXr*VNvlY(Time);S8tqs=0.0;

cjcAG=m0JG3=DAwhk/b4jXr;sddwV=y71cs=-0.5*VNvlY(DAwhk)/b4jXr;
rT061=HcINC-a84tu-sddwV-y71cs;StX1T=-rT061/DAwhk;
u4XFm+=a84tu*cjcAG-b4jXr*qn3x8(cjcAG)/6.0;a84tu+=sddwV;S8tqs=-DAwhk;
u4XFm+=a84tu*StX1T+0.5*S8tqs*VNvlY(StX1T);
a84tu+=rT061;S8tqs=-DAwhk;
u4XFm+=a84tu*m0JG3+0.5*S8tqs*VNvlY(m0JG3)+b4jXr*qn3x8(m0JG3)/6.0;
return(u4XFm>=ldeCA);}


bool qPN_6::D7yxU(const double&k6sf4,const double&AQzqu,const double&HcINC,const
 double&iPzPj,const double&ldeCA,const double&b4jXr){
double Time=0.0
,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4,L38Np=0.0;
Time=-S8tqs/b4jXr;u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)+b4jXr*qn3x8(Time)/6.0;
a84tu+=S8tqs*Time+0.5*b4jXr*VNvlY(Time);S8tqs=0.0;

L38Np=pbQOc(b4jXr*(a84tu-HcINC));
Time=L38Np/b4jXr;u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)-b4jXr*qn3x8(Time)/6.0;
a84tu+=S8tqs*Time-0.5*b4jXr*VNvlY(Time);S8tqs=-L38Np;


u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)+b4jXr*qn3x8(Time)/6.0;return(u4XFm>=
ldeCA);}


bool qPN_6::snMmx(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&HcINC,const double&iPzPj,const double&ldeCA,const 
double&b4jXr){return(r9BxJ(&PmRIf,&HcX34,&JI0hH,-k6sf4,DAwhk,-AQzqu,hxixi,-HcINC
,-iPzPj,-ldeCA,b4jXr));}


bool qPN_6::rLgKx(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&b4jXr){
double Time=0.0
,a84tu=AQzqu,S8tqs=k6sf4;
Time=-S8tqs/b4jXr;a84tu+=S8tqs*Time+0.5*b4jXr*VNvlY(Time);S8tqs=0.0;
Time=DAwhk/b4jXr;a84tu+=S8tqs*Time+0.5*b4jXr*VNvlY(Time);S8tqs=DAwhk;


a84tu+=S8tqs*Time-0.5*b4jXr*VNvlY(Time);return(a84tu>HcINC);}


bool qPN_6::EqPQU(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&b4jXr){
double Time=0.0
,a84tu=AQzqu,S8tqs=k6sf4;
Time=(DAwhk+S8tqs)/b4jXr;a84tu+=S8tqs*Time-0.5*b4jXr*VNvlY(Time);S8tqs=-DAwhk;
Time=DAwhk/b4jXr;a84tu+=S8tqs*Time+0.5*b4jXr*VNvlY(Time);S8tqs=0.0;


a84tu+=S8tqs*Time+0.5*b4jXr*VNvlY(Time);S8tqs=DAwhk;


a84tu+=S8tqs*Time-0.5*b4jXr*VNvlY(Time);return(a84tu>HcINC);}


bool qPN_6::TcZKW(const double&DAwhk,const double&HcINC,const double&b4jXr){


double lURFQ=0.0;
lURFQ=VNvlY(DAwhk)/b4jXr;return((HcINC-lURFQ)>(0x1a5+3078-0xdab));}


bool qPN_6::XlBm5(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr){
double Time=0.0
,cjcAG=0.0
,m0JG3=0.0
,sddwV=0.0
,y71cs=0.0
,L38Np=0.0
,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4;
cjcAG=m0JG3=DAwhk/b4jXr;sddwV=y71cs=0.5*VNvlY(DAwhk)/b4jXr;
L38Np=pbQOc((VNvlY(S8tqs)+2.0*VNvlY(DAwhk)+2.0*b4jXr*(a84tu-HcINC))/2.0);
Time=(S8tqs+L38Np)/b4jXr;u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)-b4jXr*qn3x8(
Time)/6.0;a84tu+=S8tqs*Time-0.5*b4jXr*VNvlY(Time);S8tqs=-L38Np;
Time=L38Np/b4jXr;u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)+b4jXr*qn3x8(Time)/6.0;
a84tu+=S8tqs*Time+0.5*b4jXr*VNvlY(Time);S8tqs=0.0;
u4XFm+=a84tu*cjcAG+b4jXr*qn3x8(cjcAG)/6.0;a84tu+=sddwV;S8tqs=+DAwhk;
u4XFm+=a84tu*m0JG3+0.5*S8tqs*VNvlY(m0JG3)-b4jXr*qn3x8(m0JG3)/6.0;
return(u4XFm>=ldeCA);}


bool qPN_6::l7hIc(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&HcINC,const double&iPzPj,const double&ldeCA,const 
double&b4jXr){return(c42JT(k6sf4,DAwhk,AQzqu,hxixi,HcINC,iPzPj,ldeCA,b4jXr));}


bool qPN_6::JvrEy(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&b4jXr){return(DcUhz(k6sf4,DAwhk,AQzqu,b4jXr));}


bool qPN_6::k64nO(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr){
double Time=0.0
,cjcAG=0.0
,StX1T=0.0
,m0JG3=0.0
,sddwV=0.0
,rT061=0.0
,y71cs=0.0
,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4;
Time=(DAwhk+S8tqs)/b4jXr;u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)-b4jXr*qn3x8(
Time)/6.0;a84tu+=S8tqs*Time-0.5*b4jXr*VNvlY(Time);S8tqs=-DAwhk;
Time=DAwhk/b4jXr;u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)+b4jXr*qn3x8(Time)/6.0;
a84tu+=S8tqs*Time+0.5*b4jXr*VNvlY(Time);S8tqs=0.0;
cjcAG=m0JG3=DAwhk/b4jXr;sddwV=y71cs=0.5*VNvlY(DAwhk)/b4jXr;
rT061=HcINC-a84tu-sddwV-y71cs;StX1T=rT061/DAwhk;
u4XFm+=a84tu*cjcAG+0.5*S8tqs*VNvlY(cjcAG)+b4jXr*qn3x8(cjcAG)/6.0;a84tu+=sddwV;
S8tqs=DAwhk;
u4XFm+=a84tu*StX1T+0.5*S8tqs*VNvlY(StX1T);
a84tu+=rT061;S8tqs=DAwhk;
u4XFm+=a84tu*m0JG3+0.5*S8tqs*VNvlY(m0JG3)-b4jXr*qn3x8(m0JG3)/6.0;
return(u4XFm>=ldeCA);}


bool qPN_6::DHN7R(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&HcINC,const double&iPzPj,const double&ldeCA,const 
double&b4jXr){return(gqU3_(k6sf4,DAwhk,AQzqu,hxixi,HcINC,iPzPj,ldeCA,b4jXr));}







bool qPN_6::ZsQgx(const double&k6sf4){return(L5k8N(k6sf4));}


bool qPN_6::PsauU(const double&k6sf4,const double&DAwhk){
return(t3fQZ(k6sf4,DAwhk));}


bool qPN_6::_JUe9(const double&k6sf4,const double&AQzqu,const double&hxixi,const
 double&b4jXr){
return(RNszr(k6sf4,AQzqu,hxixi,b4jXr));}


bool qPN_6::ZX6Y5(const double&AQzqu,const double&hxixi){
return(KbW6_(AQzqu,hxixi));}


bool qPN_6::ix0lH(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&b4jXr){

return(wKs7P(k6sf4,DAwhk,AQzqu,hxixi,b4jXr));}


bool qPN_6::vhEYc(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&b4jXr){return(P90PB(k6sf4,DAwhk,AQzqu,HcINC,b4jXr));}


bool qPN_6::kCbyv(const double&k6sf4,const double&AQzqu,const double&hxixi,const
 double&b4jXr){

return(bSu3N(k6sf4,AQzqu,hxixi,b4jXr));}


bool qPN_6::pjX4V(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&b4jXr){

return(y15i4(k6sf4,DAwhk,AQzqu,hxixi,b4jXr));}


bool qPN_6::huDFX(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&b4jXr){

return(CpvSN(k6sf4,DAwhk,AQzqu,hxixi,b4jXr));}


bool qPN_6::PWANe(const double&k6sf4,const double&AQzqu,const double&HcINC,const
 double&b4jXr){
double a84tu=AQzqu+0.5*VNvlY(k6sf4)/b4jXr;return((a84tu<0.0)&&(HcINC>0.0));}


bool qPN_6::eAfAf(const int PUMj7){return((PUMj7==qPN_6::CWXWH)||(PUMj7==qPN_6::
cvsfI)||(PUMj7==qPN_6::JSPnP)||(PUMj7==qPN_6::aBNZo)||(PUMj7==qPN_6::YItCk)||(
PUMj7==qPN_6::n1iUG));}


bool qPN_6::YIBe5(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&b4jXr){return(P90PB(k6sf4,DAwhk,AQzqu,HcINC,b4jXr));}


bool qPN_6::WHaCg(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&b4jXr){return(fwKhJ(k6sf4,DAwhk,AQzqu,HcINC,b4jXr));}


bool qPN_6::wJ_yV(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr){
double Time=0.0
,cjcAG=0.0
,StX1T=0.0
,m0JG3=0.0
,sddwV=0.0
,rT061=0.0
,y71cs=0.0
,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4;
Time=S8tqs/b4jXr;u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)-b4jXr*qn3x8(Time)/6.0;
a84tu+=S8tqs*Time-0.5*b4jXr*VNvlY(Time);S8tqs=0.0;

cjcAG=m0JG3=DAwhk/b4jXr;sddwV=y71cs=0.5*VNvlY(DAwhk)/b4jXr;
rT061=HcINC-a84tu-sddwV-y71cs;StX1T=rT061/DAwhk;
u4XFm+=a84tu*cjcAG+b4jXr*qn3x8(cjcAG)/6.0;a84tu+=sddwV;S8tqs=DAwhk;
u4XFm+=a84tu*StX1T+0.5*S8tqs*VNvlY(StX1T);
a84tu+=rT061;S8tqs=DAwhk;
u4XFm+=a84tu*m0JG3+0.5*S8tqs*VNvlY(m0JG3)-b4jXr*qn3x8(m0JG3)/6.0;
return(u4XFm+tmU_f+S9HIU*fabs(iPzPj-ldeCA)<=ldeCA);}




bool qPN_6::V66ya(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&HcINC,const double&iPzPj,const double&ldeCA,const 
double&b4jXr,double*P9_Ry,double*OFDtk){bool DBRdu=false;double Ct7O1=0.0,GxGT4=
0.0,UNEWL=0.0,eriTl=0.0,oe8Mk=tlPiC,kHxgd=tlPiC;if(!(DiYOH(n1iUG,k6sf4,DAwhk,
AQzqu,hxixi,HcINC,iPzPj,ldeCA,b4jXr))){if((P9_Ry!=NULL)&&(OFDtk!=NULL)){*P9_Ry=
tlPiC;*OFDtk=tlPiC;}return(false);}oe8Mk=ap3Lr(&l2be0,&sebzR,&lNzRf,&gAdE9,iPzPj
,ldeCA,AQzqu,hxixi,HcINC,k6sf4,DAwhk,b4jXr,true,n3ue4,0.0,&DBRdu,&Ct7O1,n1iUG);
if(!DBRdu){kHxgd=ap3Lr(&l2be0,&sebzR,&lNzRf,&gAdE9,iPzPj,ldeCA,AQzqu,hxixi,HcINC
,k6sf4,DAwhk,b4jXr,false,n3ue4,0.0,&DBRdu,&GxGT4,n1iUG);}if(!DBRdu){UNEWL=l2be0(
0.5*(Ct7O1+GxGT4),k6sf4,AQzqu,HcINC,iPzPj,ldeCA,b4jXr,DAwhk);eriTl=l2be0(0.5*
Ct7O1,k6sf4,AQzqu,HcINC,iPzPj,ldeCA,b4jXr,DAwhk);if((UNEWL>=tmU_f+S9HIU*fabs(
iPzPj-ldeCA))||(eriTl>=tmU_f+S9HIU*fabs(iPzPj-ldeCA))){if((P9_Ry!=NULL)&&(OFDtk
!=NULL)){*P9_Ry=oe8Mk;if(l2be0(0.5*GxGT4,k6sf4,AQzqu,HcINC,iPzPj,ldeCA,b4jXr,
DAwhk)<0.0){*OFDtk=kHxgd;}else{*OFDtk=tlPiC;}}return(true);}else{if((P9_Ry!=NULL
)&&(OFDtk!=NULL)){*P9_Ry=tlPiC;*OFDtk=tlPiC;}return(false);}}else{if((P9_Ry!=
NULL)&&(OFDtk!=NULL)){*P9_Ry=tlPiC;*OFDtk=tlPiC;}return(false);}}


bool qPN_6::fOpp3(const double&k6sf4,const double&AQzqu,const double&HcINC,const
 double&iPzPj,const double&ldeCA,const double&b4jXr){
double Time=0.0
,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4,L38Np=0.0;
Time=S8tqs/b4jXr;u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)-b4jXr*qn3x8(Time)/6.0;
a84tu+=S8tqs*Time-0.5*b4jXr*VNvlY(Time);S8tqs=0.0;

L38Np=pbQOc(b4jXr*(HcINC-a84tu));
Time=L38Np/b4jXr;u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)+b4jXr*qn3x8(Time)/6.0;
a84tu+=S8tqs*Time+0.5*b4jXr*VNvlY(Time);S8tqs=L38Np;


u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)-b4jXr*qn3x8(Time)/6.0;a84tu=HcINC;S8tqs=
0.0;return(u4XFm+tmU_f+S9HIU*fabs(iPzPj-ldeCA)<=ldeCA);}




bool qPN_6::TIjb6(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&HcINC,const double&iPzPj,const double&ldeCA,const 
double&b4jXr,double*P9_Ry,double*OFDtk){bool DBRdu=false,z9PIw=false;double 
Ct7O1=0.0,GxGT4=0.0,UNEWL=0.0,eriTl=0.0,ux9Gf=0.0,BhhXm=0.0,et6lt=0.0,qmYSr=0.0,
DR5JY=0.0,oe8Mk=tlPiC,kHxgd=tlPiC;if(!(DiYOH(n1iUG,k6sf4,DAwhk,AQzqu,hxixi,HcINC
,iPzPj,ldeCA,b4jXr))){if((P9_Ry!=NULL)&&(OFDtk!=NULL)){*P9_Ry=tlPiC;*OFDtk=tlPiC
;}return(false);}oe8Mk=ap3Lr(&l2be0,&sebzR,&lNzRf,&gAdE9,iPzPj,ldeCA,AQzqu,hxixi
,HcINC,k6sf4,DAwhk,b4jXr,true,n3ue4,0.0,&DBRdu,&Ct7O1,n1iUG);if(DBRdu){if((P9_Ry
!=NULL)&&(OFDtk!=NULL)){*P9_Ry=tlPiC;*OFDtk=tlPiC;}return(false);}
z9PIw=H_mqX(k6sf4,DAwhk,AQzqu,HcINC,iPzPj,ldeCA,b4jXr,&DR5JY);if(z9PIw){

ap3Lr(&PmRIf,&HcX34,&JI0hH,&UEOTz,iPzPj,ldeCA,AQzqu,hxixi,HcINC,k6sf4,DAwhk,
b4jXr,true,n3ue4,0.0,&DBRdu,&GxGT4,YItCk);BhhXm=0.5*(Ct7O1+GxGT4);if(BhhXm>DR5JY
){UNEWL=l2be0(BhhXm,k6sf4,AQzqu,HcINC,iPzPj,ldeCA,b4jXr,DAwhk);}else{UNEWL=PmRIf
(BhhXm,k6sf4,AQzqu,HcINC,iPzPj,ldeCA,b4jXr,DAwhk);}if(UNEWL<0.0){if((P9_Ry!=NULL
)&&(OFDtk!=NULL)){*P9_Ry=tlPiC;*OFDtk=tlPiC;}return(false);}}
if(z9PIw){kHxgd=ap3Lr(&l2be0,&sebzR,&lNzRf,&gAdE9,iPzPj,ldeCA,AQzqu,hxixi,HcINC,
k6sf4,DAwhk,b4jXr,false,n3ue4,0.0,&DBRdu,&GxGT4,n1iUG);}else{kHxgd=ap3Lr(&PmRIf,
&HcX34,&JI0hH,&UEOTz,iPzPj,ldeCA,AQzqu,hxixi,HcINC,k6sf4,DAwhk,b4jXr,true,n3ue4,
0.0,&DBRdu,&GxGT4,YItCk);}if(!DBRdu){BhhXm=0.5*(Ct7O1+GxGT4);et6lt=0.5*Ct7O1;if(
BhhXm>DR5JY){UNEWL=l2be0(BhhXm,k6sf4,AQzqu,HcINC,iPzPj,ldeCA,b4jXr,DAwhk);}else{
UNEWL=PmRIf(BhhXm,k6sf4,AQzqu,HcINC,iPzPj,ldeCA,b4jXr,DAwhk);}if(et6lt>DR5JY){
eriTl=l2be0(et6lt,k6sf4,AQzqu,HcINC,iPzPj,ldeCA,b4jXr,DAwhk);}else{eriTl=PmRIf(
et6lt,k6sf4,AQzqu,HcINC,iPzPj,ldeCA,b4jXr,DAwhk);}if((UNEWL>=tmU_f+S9HIU*fabs(
iPzPj-ldeCA))||(eriTl>=tmU_f+S9HIU*fabs(iPzPj-ldeCA))){if((P9_Ry!=NULL)&&(OFDtk
!=NULL)){*P9_Ry=oe8Mk;qmYSr=0.5*GxGT4;if(qmYSr>DR5JY){ux9Gf=l2be0(qmYSr,k6sf4,
AQzqu,HcINC,iPzPj,ldeCA,b4jXr,DAwhk);}else{ux9Gf=PmRIf(qmYSr,k6sf4,AQzqu,HcINC,
iPzPj,ldeCA,b4jXr,DAwhk);}if(ux9Gf<0.0){*OFDtk=kHxgd;}else{*OFDtk=tlPiC;}}return
(true);}else{if((P9_Ry!=NULL)&&(OFDtk!=NULL)){*P9_Ry=tlPiC;*OFDtk=tlPiC;}return(
false);}}else{if((P9_Ry!=NULL)&&(OFDtk!=NULL)){*P9_Ry=tlPiC;*OFDtk=tlPiC;}return
(false);}}


bool qPN_6::CFnu3(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&HcINC,const double&iPzPj,const double&ldeCA,const 
double&b4jXr){return(g45vp(k6sf4,DAwhk,AQzqu,hxixi,HcINC,iPzPj,ldeCA,b4jXr));}


bool qPN_6::EVz5o(const double&k6sf4,const double&AQzqu,const double&HcINC,const
 double&iPzPj,const double&ldeCA,const double&b4jXr){return(fOpp3(k6sf4,AQzqu,
HcINC,iPzPj,ldeCA,b4jXr));}


bool qPN_6::H_mqX(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr,double*
eA7Xu){double xDFPd=iPzPj,jLiOf=AQzqu,Time=0.0;*eA7Xu=pbQOc(0.5*(VNvlY(k6sf4)+
2.0*VNvlY(DAwhk)+2.0*b4jXr*(AQzqu-HcINC)));Time=(k6sf4-(*eA7Xu))/b4jXr;if(Time<
0.0){Time=0.0;}xDFPd+=jLiOf*Time+0.5*k6sf4*VNvlY(Time)-b4jXr*qn3x8(Time)/6.0;
jLiOf+=k6sf4*Time-0.5*b4jXr*VNvlY(Time);Time=(DAwhk-(*eA7Xu))/b4jXr;if(Time<0.0)
{Time=0.0;}xDFPd+=jLiOf*Time+0.5*(*eA7Xu)*VNvlY(Time)+b4jXr*qn3x8(Time)/6.0;
jLiOf+=(*eA7Xu)*Time+0.5*b4jXr*VNvlY(Time);Time=DAwhk/b4jXr;xDFPd+=jLiOf*Time+
0.5*DAwhk*VNvlY(Time)-b4jXr*qn3x8(Time)/6.0;return(xDFPd<=ldeCA);}




bool qPN_6::EyW6C(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&HcINC,const double&iPzPj,const double&ldeCA,const 
double&b4jXr,double*P9_Ry,double*OFDtk){bool DBRdu=false;double Ct7O1=0.0,GxGT4=
0.0,UNEWL=0.0,eriTl=0.0,oe8Mk=tlPiC,kHxgd=tlPiC;if(!(DiYOH(YItCk,k6sf4,DAwhk,
AQzqu,hxixi,HcINC,iPzPj,ldeCA,b4jXr))){if((P9_Ry!=NULL)&&(OFDtk!=NULL)){*P9_Ry=
tlPiC;*OFDtk=tlPiC;}return(false);}oe8Mk=ap3Lr(&PmRIf,&HcX34,&JI0hH,&UEOTz,iPzPj
,ldeCA,AQzqu,hxixi,HcINC,k6sf4,DAwhk,b4jXr,true,n3ue4,0.0,&DBRdu,&Ct7O1,YItCk);
if(!DBRdu){kHxgd=ap3Lr(&PmRIf,&HcX34,&JI0hH,&UEOTz,iPzPj,ldeCA,AQzqu,hxixi,HcINC
,k6sf4,DAwhk,b4jXr,false,n3ue4,0.0,&DBRdu,&GxGT4,YItCk);}if(!DBRdu){UNEWL=PmRIf(
0.5*(Ct7O1+GxGT4),k6sf4,AQzqu,HcINC,iPzPj,ldeCA,b4jXr,DAwhk);eriTl=PmRIf(0.5*
Ct7O1,k6sf4,AQzqu,HcINC,iPzPj,ldeCA,b4jXr,DAwhk);if((UNEWL>=tmU_f+S9HIU*fabs(
iPzPj-ldeCA))||(eriTl>=tmU_f+S9HIU*fabs(iPzPj-ldeCA))){if((P9_Ry!=NULL)&&(OFDtk
!=NULL)){*P9_Ry=oe8Mk;if(PmRIf(0.5*GxGT4,k6sf4,AQzqu,HcINC,iPzPj,ldeCA,b4jXr,
DAwhk)<0.0){*OFDtk=kHxgd;}else{*OFDtk=tlPiC;}}return(true);}else{if((P9_Ry!=NULL
)&&(OFDtk!=NULL)){*P9_Ry=tlPiC;*OFDtk=tlPiC;}return(false);}}else{if((P9_Ry!=
NULL)&&(OFDtk!=NULL)){*P9_Ry=tlPiC;*OFDtk=tlPiC;}return(false);}}







bool qPN_6::X0vCV(const double&k6sf4){return(L5k8N(k6sf4));}


bool qPN_6::EcoOx(const double&k6sf4,const double&DAwhk){
return(t3fQZ(k6sf4,DAwhk));}


bool qPN_6::Pfc0r(const double&k6sf4,const double&AQzqu,const double&hxixi,const
 double&b4jXr){
return(RNszr(k6sf4,AQzqu,hxixi,b4jXr));}


bool qPN_6::VRv9s(const double&AQzqu,const double&hxixi){
return(KbW6_(AQzqu,hxixi));}


bool qPN_6::mzLXC(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&b4jXr){

return(wKs7P(k6sf4,DAwhk,AQzqu,hxixi,b4jXr));}


bool qPN_6::U5VcP(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&b4jXr){return(P90PB(k6sf4,DAwhk,AQzqu,HcINC,b4jXr));}


bool qPN_6::JqArP(const double&k6sf4,const double&AQzqu,const double&hxixi,const
 double&b4jXr){

return(bSu3N(k6sf4,AQzqu,hxixi,b4jXr));}


bool qPN_6::t8AAE(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&b4jXr){

return(y15i4(k6sf4,DAwhk,AQzqu,hxixi,b4jXr));}


bool qPN_6::Uxx21(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&b4jXr){

return(CpvSN(k6sf4,DAwhk,AQzqu,hxixi,b4jXr));}


bool qPN_6::pRuPw(const double&HcINC){return(HcINC<=0.0);}


bool qPN_6::X6bhN(const double&k6sf4){

return(L5k8N(k6sf4));}


bool qPN_6::Jx7uV(const int&CqtVJ,const int&oiD6O){return((CqtVJ==qPN_6::n1iUG)
||(CqtVJ==qPN_6::YItCk)||(oiD6O==qPN_6::n1iUG)||(oiD6O==qPN_6::YItCk));}


bool qPN_6::J0Fq0(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&b4jXr){
return(m1HJ_(k6sf4,DAwhk,AQzqu,hxixi,b4jXr));}


bool qPN_6::zhkRv(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&HcINC,const double&b4jXr){return(GYWO5(k6sf4,DAwhk,
AQzqu,hxixi,HcINC,b4jXr));}


bool qPN_6::Cr43r(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&HcINC,const double&iPzPj,const double&ldeCA,const 
double&b4jXr){
return(Io38i(k6sf4,DAwhk,AQzqu,hxixi,HcINC,iPzPj,ldeCA,b4jXr));}


bool qPN_6::BNLYk(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&HcINC,const double&iPzPj,const double&ldeCA,const 
double&b4jXr){return(r9BxJ(&eOy_R,&p9gAB,&X79jJ,k6sf4,DAwhk,AQzqu,hxixi,HcINC,
iPzPj,ldeCA,b4jXr));}


bool qPN_6::wlfXo(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&HcINC,const double&iPzPj,const double&ldeCA,const 
double&b4jXr){
return(xDxct(k6sf4,DAwhk,AQzqu,hxixi,HcINC,iPzPj,ldeCA,b4jXr));}


bool qPN_6::nAcyu(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&HcINC,const double&iPzPj,const double&ldeCA,const 
double&b4jXr){return(r9BxJ(&gPu6H,&RDM53,&F0eNd,k6sf4,DAwhk,AQzqu,hxixi,HcINC,
iPzPj,ldeCA,b4jXr));}


bool qPN_6::ADtkh(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&HcINC,const double&b4jXr){
return(wdgir(k6sf4,DAwhk,AQzqu,HcINC,b4jXr));}


bool qPN_6::mayMb(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&HcINC,const double&iPzPj,const double&ldeCA,const 
double&b4jXr){return(r9BxJ(&p4iJ9,&KsLO9,&S5k0a,k6sf4,DAwhk,AQzqu,hxixi,HcINC,
iPzPj,ldeCA,b4jXr));}


bool qPN_6::QEfD1(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&HcINC,const double&b4jXr){
return(A0AQ0(k6sf4,DAwhk,AQzqu,hxixi,HcINC,b4jXr));}


bool qPN_6::eltDG(const double&k6sf4,const double&AQzqu,const double&hxixi,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr){
return(JIwMI(k6sf4,AQzqu,hxixi,HcINC,iPzPj,ldeCA,b4jXr));}


bool qPN_6::QyAWc(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&HcINC,const double&iPzPj,const double&ldeCA,const 
double&b4jXr){
return(Ftk9o(k6sf4,DAwhk,AQzqu,hxixi,HcINC,iPzPj,ldeCA,b4jXr));}







bool qPN_6::qC4La(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&HcINC,const double&b4jXr,const bool&mZass){

double Time=0.0
,a84tu=AQzqu,S8tqs=k6sf4;
a84tu=hxixi;S8tqs=0.0;
Time=DAwhk/b4jXr;a84tu+=S8tqs*Time-0.5*b4jXr*VNvlY(Time);S8tqs=-DAwhk;


a84tu+=S8tqs*Time+0.5*b4jXr*VNvlY(Time);return(a84tu+rsTOM*((mZass)?(1.0):(-1.0)
)+gBilO*((mZass)?(1.0):(-1.0))*fabs(a84tu)<=HcINC);}


bool qPN_6::sg7EJ(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&HcINC,const double&iPzPj,const double&ldeCA,const 
double&b4jXr,const bool&mZass){

double Time=0.0
,cjcAG=0.0
,StX1T=0.0
,m0JG3=0.0
,sddwV=0.0
,rT061=0.0
,y71cs=0.0
,L38Np=0.0
,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4,JMvCa=HcINC;

cjcAG=(DAwhk-S8tqs)/b4jXr;
m0JG3=DAwhk/b4jXr;sddwV=S8tqs*cjcAG+0.5*b4jXr*VNvlY(cjcAG);y71cs=0.5*VNvlY(DAwhk
)/b4jXr;
rT061=hxixi-a84tu-sddwV-y71cs;StX1T=rT061/DAwhk;
u4XFm+=a84tu*cjcAG+0.5*S8tqs*VNvlY(cjcAG)+b4jXr*qn3x8(cjcAG)/6.0;a84tu+=sddwV;
S8tqs=DAwhk;
u4XFm+=a84tu*StX1T+0.5*S8tqs*VNvlY(StX1T);
a84tu+=rT061;S8tqs=DAwhk;
u4XFm+=a84tu*m0JG3+0.5*S8tqs*VNvlY(m0JG3)-b4jXr*qn3x8(m0JG3)/6.0;
a84tu=hxixi;S8tqs=0.0;

a84tu=-a84tu;JMvCa=-JMvCa;
L38Np=pbQOc(b4jXr*(JMvCa-a84tu));
a84tu=-a84tu;JMvCa=-JMvCa;L38Np=-L38Np;
Time=(-L38Np)/b4jXr;u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)-b4jXr*qn3x8(Time)/
6.0;a84tu+=S8tqs*Time-0.5*b4jXr*VNvlY(Time);S8tqs=L38Np;


u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)+b4jXr*qn3x8(Time)/6.0;return(u4XFm+rsTOM
*((mZass)?(1.0):(-1.0))+gBilO*((mZass)?(1.0):(-1.0))*fabs(u4XFm-ldeCA)<=ldeCA);}


bool qPN_6::cRSW1(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&HcINC,const double&iPzPj,const double&ldeCA,const 
double&b4jXr,const bool&mZass){

double cjcAG=0.0
,StX1T=0.0
,m0JG3=0.0
,sddwV=0.0
,rT061=0.0
,y71cs=0.0
,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4;

cjcAG=(DAwhk-S8tqs)/b4jXr;
m0JG3=DAwhk/b4jXr;sddwV=S8tqs*cjcAG+0.5*b4jXr*VNvlY(cjcAG);y71cs=0.5*VNvlY(DAwhk
)/b4jXr;
rT061=hxixi-a84tu-sddwV-y71cs;StX1T=rT061/DAwhk;
u4XFm+=a84tu*cjcAG+0.5*S8tqs*VNvlY(cjcAG)+b4jXr*qn3x8(cjcAG)/6.0;a84tu+=sddwV;
S8tqs=DAwhk;
u4XFm+=a84tu*StX1T+0.5*S8tqs*VNvlY(StX1T);
a84tu+=rT061;S8tqs=DAwhk;
u4XFm+=a84tu*m0JG3+0.5*S8tqs*VNvlY(m0JG3)-b4jXr*qn3x8(m0JG3)/6.0;
a84tu=hxixi;S8tqs=0.0;

cjcAG=m0JG3=DAwhk/b4jXr;sddwV=y71cs=-0.5*VNvlY(DAwhk)/b4jXr;
rT061=HcINC-a84tu-sddwV-y71cs;StX1T=rT061/(-DAwhk);
u4XFm+=a84tu*cjcAG-b4jXr*qn3x8(cjcAG)/6.0;a84tu+=sddwV;S8tqs=-DAwhk;
u4XFm+=a84tu*StX1T+0.5*S8tqs*VNvlY(StX1T);
a84tu+=rT061;S8tqs=-DAwhk;
u4XFm+=a84tu*m0JG3+0.5*S8tqs*VNvlY(m0JG3)+b4jXr*qn3x8(m0JG3)/6.0;
return(u4XFm+rsTOM*((mZass)?(1.0):(-1.0))+gBilO*((mZass)?(1.0):(-1.0))*fabs(
u4XFm-ldeCA)<=ldeCA);}


bool qPN_6::sD7wJ(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&b4jXr,const bool&mZass){


double a84tu=AQzqu,S8tqs=k6sf4,Time=0.0;
Time=(DAwhk-S8tqs)/b4jXr;a84tu+=S8tqs*Time+0.5*b4jXr*VNvlY(Time);S8tqs=DAwhk;
Time=DAwhk/b4jXr;a84tu+=S8tqs*Time-0.5*b4jXr*VNvlY(Time);return(a84tu+rsTOM*((
mZass)?(1.0):(-1.0))+gBilO*((mZass)?(1.0):(-1.0))*fabs(a84tu)<=hxixi);}


bool qPN_6::Yngfp(const double&k6sf4,const double&AQzqu,const double&hxixi,const
 double&HcINC,const double&iPzPj,const double&ldeCA,const double&b4jXr,const 
bool&mZass){

double u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4,JMvCa=HcINC,Time=0.0,L38Np=0.0;

L38Np=pbQOc((VNvlY(S8tqs)+2.0*b4jXr*(hxixi-a84tu))/2.0);
Time=(L38Np-S8tqs)/b4jXr;u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)+b4jXr*qn3x8(
Time)/6.0;a84tu+=S8tqs*Time+0.5*b4jXr*VNvlY(Time);S8tqs=L38Np;
Time=L38Np/b4jXr;u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)-b4jXr*qn3x8(Time)/6.0;
a84tu=hxixi;S8tqs=0.0;

JMvCa=-JMvCa;a84tu=-a84tu;
L38Np=pbQOc(b4jXr*(JMvCa-a84tu));
JMvCa=-JMvCa;a84tu=-a84tu;L38Np=-L38Np;
Time=(-L38Np)/b4jXr;u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)-b4jXr*qn3x8(Time)/
6.0;a84tu+=S8tqs*Time-0.5*b4jXr*VNvlY(Time);S8tqs=L38Np;
Time=(-L38Np)/b4jXr;u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)+b4jXr*qn3x8(Time)/
6.0;return(u4XFm+rsTOM*((mZass)?(1.0):(-1.0))+gBilO*((mZass)?(1.0):(-1.0))*fabs(
u4XFm-ldeCA)<=ldeCA);}


bool qPN_6::YqOu_(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&HcINC,const double&b4jXr,const bool&mZass){
double a84tu=AQzqu,S8tqs=k6sf4,Time=0.0;
a84tu=hxixi;S8tqs=0.0;
Time=DAwhk/b4jXr;a84tu+=S8tqs*Time-0.5*b4jXr*VNvlY(Time);S8tqs=-DAwhk;
Time=DAwhk/b4jXr;a84tu+=S8tqs*Time+0.5*b4jXr*VNvlY(Time);return(a84tu+rsTOM*((
mZass)?(1.0):(-1.0))+gBilO*((mZass)?(1.0):(-1.0))*fabs(a84tu)<=HcINC);}


bool qPN_6::TN2L4(const double&k6sf4,const double&DAwhk,const double&AQzqu,const
 double&hxixi,const double&HcINC,const double&iPzPj,const double&ldeCA,const 
double&b4jXr,const bool&mZass){

double Time=0.0
,cjcAG=0.0
,StX1T=0.0
,m0JG3=0.0
,sddwV=0.0
,rT061=0.0
,y71cs=0.0
,L38Np=0.0
,u4XFm=iPzPj,a84tu=AQzqu,S8tqs=k6sf4;
L38Np=pbQOc((VNvlY(S8tqs)+2.0*b4jXr*(hxixi-a84tu))/(0x9a8+2410-0x1310));
Time=(L38Np-S8tqs)/b4jXr;u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)+b4jXr*qn3x8(
Time)/6.0;a84tu+=S8tqs*Time+0.5*b4jXr*VNvlY(Time);S8tqs=L38Np;
Time=L38Np/b4jXr;u4XFm+=a84tu*Time+0.5*S8tqs*VNvlY(Time)-b4jXr*qn3x8(Time)/6.0;
a84tu=hxixi;S8tqs=0.0;

cjcAG=m0JG3=DAwhk/b4jXr;sddwV=y71cs=-0.5*VNvlY(DAwhk)/b4jXr;
rT061=HcINC-a84tu-sddwV-y71cs;StX1T=rT061/(-DAwhk);
u4XFm+=a84tu*cjcAG-b4jXr*qn3x8(cjcAG)/6.0;a84tu+=sddwV;S8tqs=-DAwhk;
u4XFm+=a84tu*StX1T+0.5*S8tqs*VNvlY(StX1T);
a84tu+=rT061;S8tqs=-DAwhk;
u4XFm+=a84tu*m0JG3+0.5*S8tqs*VNvlY(m0JG3)+b4jXr*qn3x8(m0JG3)/6.0;
return(u4XFm+rsTOM*((mZass)?(1.0):(-1.0))+gBilO*((mZass)?(1.0):(-1.0))*fabs(
u4XFm-ldeCA)<=ldeCA);}
