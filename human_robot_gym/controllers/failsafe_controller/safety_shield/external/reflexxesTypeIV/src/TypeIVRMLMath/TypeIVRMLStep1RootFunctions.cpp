



































#include <TypeIVRMLStep1RootFunctions.h>
#include <TypeIVRMLMath.h>
#include <TypeIVRMLDefinitions.h>
#include <TypeIVRMLStep1Decisions.h>
#include <math.h>
#ifdef WIN32
#pragma\
 warning( disable : 4100 )	// to suppress the C4100 compiler warning (unreferenced formal parameter)
#endif


void qPN_6::F0eNd(const double&x5Jdm,const double&EPTqx,const double&S7nLL,const
 double&usHjQ,const double&QCcTr,const double&A1yTX,double*MjTPS,double*ZEYjO){
if(wdgir(EPTqx,x5Jdm,S7nLL,usHjQ,QCcTr)){
*MjTPS=(VNvlY(EPTqx)+2.0*QCcTr*(usHjQ-S7nLL))/(2.0*x5Jdm*QCcTr);}else{*MjTPS=0.0
;}
*ZEYjO=(VNvlY(EPTqx)-2.0*VNvlY(x5Jdm)+2.0*QCcTr*(A1yTX-S7nLL))/(2.0*x5Jdm*QCcTr)
;if(*ZEYjO<0.0){*ZEYjO=0.0;}if(*MjTPS<0.0){*MjTPS=0.0;}m4lDi(MjTPS,ZEYjO);}

double qPN_6::gPu6H(const double&EvE6x,const double&EPTqx,const double&S7nLL,
const double&usHjQ,const double&zFb7J,const double&gBbH5,const double&QCcTr,
const double&x5Jdm){double u7ROa=0.0,laktM=0.0,V3sFS=0.0,yPvHT=0.0,eFtGc=0.0,
ttbPQ=0.0,OV_Bz=0.0,mdI9v=0.0,Au08l=0.0;laktM=EPTqx*EPTqx;V3sFS=laktM*laktM;
mdI9v=QCcTr*S7nLL;Au08l=x5Jdm*x5Jdm;OV_Bz=Au08l*Au08l;ttbPQ=QCcTr*QCcTr;u7ROa=
EvE6x*EvE6x;yPvHT=S7nLL*S7nLL;eFtGc=usHjQ*usHjQ;return(
0.41666666666666666666666666666666666667e-1*(0.3e1*V3sFS+0.8e1*laktM*EPTqx*x5Jdm
-0.24e2*EPTqx*x5Jdm*mdI9v-0.6e1*laktM*(0.7e1*Au08l+0.4e1*x5Jdm*QCcTr*EvE6x+0.2e1
*mdI9v)+0.48e2*OV_Bz+0.72e2*Au08l*x5Jdm*QCcTr*EvE6x+0.24e2*x5Jdm*ttbPQ*(zFb7J-
0.1e1*gBbH5+0.2e1*EvE6x*S7nLL)+0.12e2*Au08l*QCcTr*(0.2e1*QCcTr*u7ROa+0.7e1*S7nLL
+usHjQ)+0.12e2*ttbPQ*(yPvHT-0.1e1*eFtGc))/x5Jdm/ttbPQ);}

double qPN_6::RDM53(const double&EvE6x,const double&EPTqx,const double&S7nLL,
const double&usHjQ,const double&zFb7J,const double&gBbH5,const double&QCcTr,
const double&x5Jdm){double ZBiss=0.0,laktM=0.0;laktM=EPTqx*EPTqx;ZBiss=x5Jdm*
x5Jdm;return((-0.1e1*laktM+0.3e1*ZBiss+0.2e1*x5Jdm*QCcTr*EvE6x+0.2e1*QCcTr*S7nLL
)/QCcTr);}

double qPN_6::TFyD5(const double&EvE6x,const double&EPTqx,const double&S7nLL,
const double&usHjQ,const double&zFb7J,const double&gBbH5,const double&QCcTr,
const double&x5Jdm){double laktM=0.0,KSaj_=0.0;laktM=EPTqx*EPTqx;KSaj_=x5Jdm*
x5Jdm;return(-0.5e0*(laktM+0.2e1*EPTqx*x5Jdm-0.8e1*KSaj_-0.4e1*x5Jdm*QCcTr*EvE6x
-0.2e1*QCcTr*S7nLL+0.2e1*QCcTr*usHjQ)/x5Jdm/QCcTr);}

void qPN_6::X79jJ(const double&x5Jdm,const double&EPTqx,const double&S7nLL,const
 double&usHjQ,const double&QCcTr,const double&A1yTX,double*PM8Ak,double*PnXnX){
double C98HB=0.0;C98HB=S7nLL+(VNvlY(x5Jdm)-0.5*VNvlY(EPTqx))/QCcTr;
if(C98HB>usHjQ){
*PnXnX=-pbQOc(QCcTr*(C98HB-usHjQ));}else{*PnXnX=0.0;}
*PM8Ak=-pbQOc(QCcTr*(A1yTX-usHjQ));if(*PM8Ak<-x5Jdm){*PM8Ak=-x5Jdm;}if(*PnXnX>
0.0){*PnXnX=0.0;}m4lDi(PM8Ak,PnXnX);return;}

double qPN_6::eOy_R(const double&ju5eQ,const double&EPTqx,const double&S7nLL,
const double&usHjQ,const double&zFb7J,const double&gBbH5,const double&QCcTr,
const double&x5Jdm){double laktM=0.0,mdI9v=0.0,Au08l=0.0,FvoS1=0.0,V3sFS=0.0,
fYdvb=0.0,rgvw_=0.0,Ttalj=0.0,Tmhm2=0.0;laktM=EPTqx*EPTqx;V3sFS=laktM*laktM;
mdI9v=QCcTr*S7nLL;Au08l=x5Jdm*x5Jdm;Ttalj=ju5eQ*ju5eQ;Tmhm2=Ttalj*Ttalj;FvoS1=
QCcTr*QCcTr;fYdvb=S7nLL*S7nLL;rgvw_=usHjQ*usHjQ;return(
0.41666666666666666666666666666667e-1*(-0.3e1*V3sFS+0.8e1*laktM*EPTqx*x5Jdm-
0.24e2*EPTqx*x5Jdm*mdI9v-0.6e1*laktM*(Au08l-0.2e1*mdI9v)+0.12e2*Tmhm2+0.24e2*
Ttalj*QCcTr*usHjQ-0.24e2*x5Jdm*(Ttalj*ju5eQ+FvoS1*(-0.1e1*zFb7J+gBbH5)+0.2e1*
ju5eQ*QCcTr*usHjQ)+0.12e2*FvoS1*(-0.1e1*fYdvb+rgvw_)+0.12e2*Au08l*(Ttalj+QCcTr*(
S7nLL+usHjQ)))/x5Jdm/FvoS1);}

double qPN_6::p9gAB(const double&ju5eQ,const double&EPTqx,const double&S7nLL,
const double&usHjQ,const double&zFb7J,const double&gBbH5,const double&QCcTr,
const double&x5Jdm){double laktM=0.0,KSaj_=0.0,r9J9t=0.0;laktM=x5Jdm*x5Jdm;KSaj_
=ju5eQ*ju5eQ;r9J9t=QCcTr*QCcTr;return((0.1e1*laktM*ju5eQ-0.3e1*x5Jdm*KSaj_+0.2e1
*KSaj_*ju5eQ-0.2e1*x5Jdm*QCcTr*usHjQ+0.2e1*ju5eQ*QCcTr*usHjQ)/x5Jdm/r9J9t);}

double qPN_6::R1laJ(const double&ju5eQ,const double&EPTqx,const double&S7nLL,
const double&usHjQ,const double&zFb7J,const double&gBbH5,const double&QCcTr,
const double&x5Jdm){double KSaj_=0.0,mdI9v=0.0,laktM=0.0;laktM=EPTqx*EPTqx;KSaj_
=x5Jdm*x5Jdm;mdI9v=ju5eQ*ju5eQ;return(0.5e0*(laktM-0.2e1*EPTqx*x5Jdm+0.2e1*KSaj_
-0.4e1*x5Jdm*ju5eQ+0.2e1*mdI9v-0.2e1*QCcTr*S7nLL+0.2e1*QCcTr*usHjQ)/x5Jdm/QCcTr)
;}

void qPN_6::S5k0a(const double&x5Jdm,const double&EPTqx,const double&S7nLL,const
 double&usHjQ,const double&QCcTr,const double&A1yTX,double*WLFWK,double*Hksa7){
double x6XvO=0.0;x6XvO=S7nLL+(VNvlY(EPTqx)-2.0*VNvlY(x5Jdm))/(2.0*QCcTr);if(
x6XvO>usHjQ){*WLFWK=EPTqx;}else{
*WLFWK=pbQOc(0.5*(VNvlY(EPTqx)+2.0*VNvlY(x5Jdm)+2.0*QCcTr*(usHjQ-S7nLL)));}
*Hksa7=pbQOc(0.5*(VNvlY(EPTqx)+2.0*QCcTr*(A1yTX-S7nLL)));if(*Hksa7>x5Jdm){*Hksa7
=x5Jdm;}if(*WLFWK<0.0){*WLFWK=0.0;}m4lDi(WLFWK,Hksa7);return;}

double qPN_6::p4iJ9(const double&cCJ5e,const double&EPTqx,const double&S7nLL,
const double&usHjQ,const double&zFb7J,const double&gBbH5,const double&QCcTr,
const double&x5Jdm){double lmPpg=0.0,laktM=0.0,V3sFS=0.0,u7ROa=0.0,VA2lx=0.0,
Au08l=0.0,L62Wd=0.0,nDRrN=0.0,mdI9v=0.0;laktM=EPTqx*EPTqx;V3sFS=laktM*laktM;
mdI9v=QCcTr*S7nLL;Au08l=x5Jdm*x5Jdm;L62Wd=cCJ5e*cCJ5e;nDRrN=L62Wd*L62Wd;VA2lx=
QCcTr*QCcTr;u7ROa=S7nLL*S7nLL;lmPpg=usHjQ*usHjQ;return(
0.41666666666666666666666666666667e-1*(0.3e1*V3sFS+0.8e1*laktM*EPTqx*x5Jdm-
0.24e2*EPTqx*x5Jdm*mdI9v-0.6e1*laktM*(Au08l+0.4e1*x5Jdm*cCJ5e+0.2e1*L62Wd+0.2e1*
mdI9v)+0.12e2*nDRrN+0.24e2*L62Wd*QCcTr*S7nLL+0.24e2*x5Jdm*(L62Wd*cCJ5e+VA2lx*(
zFb7J-0.1e1*gBbH5)+0.2e1*cCJ5e*QCcTr*S7nLL)+0.12e2*VA2lx*(u7ROa-0.1e1*lmPpg)+
0.12e2*Au08l*(L62Wd+QCcTr*(S7nLL+usHjQ)))/x5Jdm/VA2lx);}

double qPN_6::KsLO9(const double&cCJ5e,const double&EPTqx,const double&S7nLL,
const double&usHjQ,const double&zFb7J,const double&gBbH5,const double&QCcTr,
const double&x5Jdm){double laktM=0.0,FvoS1=0.0,IyC_V=0.0,LQB6k=0.0;laktM=EPTqx*
EPTqx;LQB6k=x5Jdm*x5Jdm;IyC_V=cCJ5e*cCJ5e;FvoS1=QCcTr*QCcTr;return((laktM*(-
0.1e1*x5Jdm-0.1e1*cCJ5e)+0.1e1*LQB6k*cCJ5e+0.3e1*x5Jdm*IyC_V+0.2e1*IyC_V*cCJ5e+
0.2e1*x5Jdm*QCcTr*S7nLL+0.2e1*cCJ5e*QCcTr*S7nLL)/x5Jdm/FvoS1);}

double qPN_6::s8ZpY(const double&cCJ5e,const double&EPTqx,const double&S7nLL,
const double&usHjQ,const double&zFb7J,const double&gBbH5,const double&QCcTr,
const double&x5Jdm){double laktM=0.0,KSaj_=0.0,mdI9v=0.0;laktM=EPTqx*EPTqx;KSaj_
=x5Jdm*x5Jdm;mdI9v=cCJ5e*cCJ5e;return(-0.5e0*(laktM+0.2e1*EPTqx*x5Jdm-0.2e1*
KSaj_-0.4e1*x5Jdm*cCJ5e-0.2e1*mdI9v-0.2e1*QCcTr*S7nLL+0.2e1*QCcTr*usHjQ)/x5Jdm/
QCcTr);}

void qPN_6::QkEpP(const double&x5Jdm,const double&EPTqx,const double&S7nLL,const
 double&usHjQ,const double&QCcTr,const double&A1yTX,double*WLFWK,double*Hksa7){
double QvNr6=0.0,vqSSF=0.0,gr7ls=0.0;QvNr6=S7nLL+0.5*VNvlY(EPTqx)/QCcTr;if(QvNr6
>usHjQ){*WLFWK=EPTqx;}else{
*WLFWK=pbQOc(0.5*(VNvlY(EPTqx)+2.0*QCcTr*(usHjQ-S7nLL)));}
vqSSF=pbQOc(0.5*(VNvlY(QCcTr)*(VNvlY(EPTqx)+2.0*VNvlY(x5Jdm))+qn3x8(QCcTr)*(
usHjQ-S7nLL)))/QCcTr;if(vqSSF>x5Jdm){vqSSF=x5Jdm;}
gr7ls=pbQOc(0.5*(VNvlY(EPTqx)+2.0*QCcTr*(A1yTX-S7nLL)));if(gr7ls>x5Jdm){gr7ls=
x5Jdm;}if(gr7ls<vqSSF){*Hksa7=gr7ls;}else{*Hksa7=vqSSF;}if(*WLFWK<0.0){*WLFWK=
0.0;}m4lDi(WLFWK,Hksa7);return;}

double qPN_6::Ryy_6(const double&cCJ5e,const double&EPTqx,const double&S7nLL,
const double&usHjQ,const double&zFb7J,const double&gBbH5,const double&QCcTr,
const double&x5Jdm){double Ttalj=0.0,laktM=0.0,IyC_V=0.0,lmPpg=0.0;laktM=EPTqx*
EPTqx;IyC_V=cCJ5e*cCJ5e;Ttalj=pbQOc(-0.2e1*laktM+0.4e1*IyC_V+0.4e1*QCcTr*S7nLL-
0.4e1*QCcTr*usHjQ);lmPpg=QCcTr*QCcTr;return(
0.8333333333333333333333333333333333333333333333333e-1*(0.4e1*laktM*EPTqx-0.12e2
*EPTqx*QCcTr*S7nLL-0.3e1*laktM*(0.4e1*cCJ5e+Ttalj)+0.12e2*IyC_V*cCJ5e+0.24e2*
cCJ5e*QCcTr*S7nLL+0.6e1*QCcTr*(0.2e1*QCcTr*(zFb7J-0.1e1*gBbH5)+(S7nLL+usHjQ)*
Ttalj)+0.6e1*IyC_V*Ttalj)/lmPpg);}

double qPN_6::tlbcQ(const double&cCJ5e,const double&EPTqx,const double&S7nLL,
const double&usHjQ,const double&zFb7J,const double&gBbH5,const double&QCcTr,
const double&x5Jdm){double ttbPQ=0.0,laktM=0.0,KSaj_=0.0,IyC_V=0.0,EvE6x=0.0,
Tmhm2=0.0;laktM=cCJ5e*cCJ5e;KSaj_=cCJ5e*QCcTr;IyC_V=EPTqx*EPTqx;EvE6x=QCcTr*
S7nLL;Tmhm2=pbQOc(-0.2e1*IyC_V+0.4e1*laktM+0.4e1*EvE6x-0.4e1*QCcTr*usHjQ);ttbPQ=
QCcTr*QCcTr;return((0.6e1*laktM*cCJ5e+0.6e1*KSaj_*S7nLL-0.2e1*KSaj_*usHjQ+0.3e1*
laktM*Tmhm2+0.2e1*EvE6x*Tmhm2+IyC_V*(-0.3e1*cCJ5e-0.1e1*Tmhm2))/ttbPQ/Tmhm2);}

double qPN_6::QKzoG(const double&cCJ5e,const double&EPTqx,const double&S7nLL,
const double&usHjQ,const double&zFb7J,const double&gBbH5,const double&QCcTr,
const double&x5Jdm){double EvE6x=0.0,ZBiss=0.0,e0vvg=0.0;ZBiss=EPTqx*EPTqx;e0vvg
=cCJ5e*cCJ5e;EvE6x=pbQOc(-0.2e1*ZBiss+0.4e1*e0vvg+0.4e1*QCcTr*S7nLL-0.4e1*QCcTr*
usHjQ);return((-0.1e1*EPTqx+0.2e1*cCJ5e+EvE6x)/QCcTr);}

void qPN_6::lNzRf(const double&x5Jdm,const double&EPTqx,const double&S7nLL,const
 double&usHjQ,const double&QCcTr,const double&A1yTX,double*WLFWK,double*Hksa7){
double r9dGs=0.0;*Hksa7=EPTqx;r9dGs=S7nLL+(0.5*VNvlY(EPTqx)+VNvlY(x5Jdm))/QCcTr;
if(r9dGs<usHjQ){*WLFWK=0.0;}else{
*WLFWK=pbQOc(0.5*(VNvlY(EPTqx)+2.0*VNvlY(x5Jdm)+2.0*QCcTr*(S7nLL-usHjQ)));}if(*
WLFWK<0.0){*WLFWK=0.0;}if(*Hksa7<0.0){*Hksa7=0.0;}m4lDi(WLFWK,Hksa7);return;}

double qPN_6::l2be0(const double&cCJ5e,const double&EPTqx,const double&S7nLL,
const double&usHjQ,const double&zFb7J,const double&gBbH5,const double&QCcTr,
const double&x5Jdm){double VA2lx=0.0,laktM=0.0,V3sFS=0.0,u7ROa=0.0,lmPpg=0.0,
Au08l=0.0,nDRrN=0.0,L62Wd=0.0,mdI9v=0.0;laktM=EPTqx*EPTqx;V3sFS=laktM*laktM;
mdI9v=QCcTr*S7nLL;Au08l=x5Jdm*x5Jdm;L62Wd=cCJ5e*cCJ5e;nDRrN=L62Wd*L62Wd;VA2lx=
QCcTr*QCcTr;u7ROa=S7nLL*S7nLL;lmPpg=usHjQ*usHjQ;return(
0.416666666666666666666666666666666667e-1*(-0.3e1*V3sFS+0.8e1*laktM*EPTqx*x5Jdm+
0.24e2*EPTqx*x5Jdm*mdI9v+0.6e1*laktM*(Au08l-0.4e1*x5Jdm*cCJ5e+0.2e1*L62Wd-0.2e1*
mdI9v)-0.12e2*nDRrN+0.24e2*L62Wd*QCcTr*S7nLL+0.24e2*x5Jdm*(L62Wd*cCJ5e+VA2lx*(
zFb7J-0.1e1*gBbH5)-0.2e1*cCJ5e*QCcTr*S7nLL)-0.12e2*VA2lx*(u7ROa-0.1e1*lmPpg)-
0.12e2*Au08l*(L62Wd-0.1e1*QCcTr*(S7nLL+usHjQ)))/x5Jdm/VA2lx);}

double qPN_6::sebzR(const double&cCJ5e,const double&EPTqx,const double&S7nLL,
const double&usHjQ,const double&zFb7J,const double&gBbH5,const double&QCcTr,
const double&x5Jdm){double laktM=0.0,KSaj_=0.0,IyC_V=0.0,FvoS1=0.0;laktM=x5Jdm*
x5Jdm;KSaj_=cCJ5e*cCJ5e;IyC_V=EPTqx*EPTqx;FvoS1=QCcTr*QCcTr;return((-0.1e1*laktM
*cCJ5e+0.3e1*x5Jdm*KSaj_-0.2e1*KSaj_*cCJ5e+IyC_V*(-0.1e1*x5Jdm+0.1e1*cCJ5e)-
0.2e1*x5Jdm*QCcTr*S7nLL+0.2e1*cCJ5e*QCcTr*S7nLL)/x5Jdm/FvoS1);}

double qPN_6::BOuKO(const double&cCJ5e,const double&EPTqx,const double&S7nLL,
const double&usHjQ,const double&zFb7J,const double&gBbH5,const double&QCcTr,
const double&x5Jdm){double laktM=0.0,L62Wd=0.0,ZBiss=0.0,BIVJE=0.0;laktM=EPTqx*
EPTqx;ZBiss=x5Jdm*x5Jdm;BIVJE=cCJ5e*cCJ5e;L62Wd=QCcTr*QCcTr;return((0.1e1*laktM-
0.1e1*ZBiss+0.6e1*x5Jdm*cCJ5e-0.6e1*BIVJE+0.2e1*QCcTr*S7nLL)/x5Jdm/L62Wd);}

double qPN_6::gAdE9(const double&cCJ5e,const double&EPTqx,const double&S7nLL,
const double&usHjQ,const double&zFb7J,const double&gBbH5,const double&QCcTr,
const double&x5Jdm){double e0vvg=0.0,IyC_V=0.0,laktM=0.0;laktM=EPTqx*EPTqx;e0vvg
=x5Jdm*x5Jdm;IyC_V=cCJ5e*cCJ5e;return(0.5e0*(-0.1e1*laktM+0.2e1*EPTqx*x5Jdm+
0.2e1*e0vvg-0.4e1*x5Jdm*cCJ5e+0.2e1*IyC_V-0.2e1*QCcTr*S7nLL+0.2e1*QCcTr*usHjQ)/
x5Jdm/QCcTr);}

void qPN_6::JI0hH(const double&x5Jdm,const double&EPTqx,const double&S7nLL,const
 double&usHjQ,const double&QCcTr,const double&A1yTX,double*WLFWK,double*Hksa7){
double C98HB=0.0;C98HB=S7nLL+(VNvlY(x5Jdm)-0.5*VNvlY(EPTqx))/QCcTr;if(C98HB>
usHjQ){*Hksa7=EPTqx;}else{
*Hksa7=pbQOc(0.5*(VNvlY(EPTqx)+2.0*VNvlY(x5Jdm)+2.0*QCcTr*(S7nLL-usHjQ)));}*
WLFWK=0.0;m4lDi(WLFWK,Hksa7);return;}

double qPN_6::PmRIf(const double&cCJ5e,const double&EPTqx,const double&S7nLL,
const double&usHjQ,const double&zFb7J,const double&gBbH5,const double&QCcTr,
const double&x5Jdm){double lmPpg=0.0,laktM=0.0,IyC_V=0.0,Ttalj=0.0;laktM=EPTqx*
EPTqx;IyC_V=cCJ5e*cCJ5e;Ttalj=pbQOc(-0.2e1*laktM+0.4e1*IyC_V-0.4e1*QCcTr*S7nLL+
0.4e1*QCcTr*usHjQ);lmPpg=QCcTr*QCcTr;return(
0.8333333333333333333333333333333333333333e-1*(0.4e1*laktM*EPTqx+0.12e2*EPTqx*
QCcTr*S7nLL+0.3e1*laktM*(-0.4e1*cCJ5e+Ttalj)+0.12e2*IyC_V*cCJ5e-0.24e2*cCJ5e*
QCcTr*S7nLL-0.6e1*IyC_V*Ttalj+0.6e1*QCcTr*(0.2e1*QCcTr*(zFb7J-0.1e1*gBbH5)+(
S7nLL+usHjQ)*Ttalj))/lmPpg);}

double qPN_6::HcX34(const double&cCJ5e,const double&EPTqx,const double&S7nLL,
const double&usHjQ,const double&zFb7J,const double&gBbH5,const double&QCcTr,
const double&x5Jdm){double ttbPQ=0.0,laktM=0.0,IyC_V=0.0,Tmhm2=0.0,KSaj_=0.0,
EvE6x=0.0;laktM=cCJ5e*cCJ5e;KSaj_=cCJ5e*QCcTr;IyC_V=EPTqx*EPTqx;EvE6x=QCcTr*
S7nLL;Tmhm2=pbQOc(-0.2e1*IyC_V+0.4e1*laktM-0.4e1*EvE6x+0.4e1*QCcTr*usHjQ);ttbPQ=
QCcTr*QCcTr;return((-0.6e1*laktM*cCJ5e+0.6e1*KSaj_*S7nLL-0.2e1*KSaj_*usHjQ+0.3e1
*laktM*Tmhm2-0.2e1*EvE6x*Tmhm2+IyC_V*(0.3e1*cCJ5e-0.1e1*Tmhm2))/ttbPQ/Tmhm2);}

double qPN_6::IxTds(const double&cCJ5e,const double&EPTqx,const double&S7nLL,
const double&usHjQ,const double&zFb7J,const double&gBbH5,const double&QCcTr,
const double&x5Jdm){double laktM=0.0,yP0aV=0.0,Tmhm2=0.0,UXXxS=0.0,V3sFS=0.0,
nDRrN=0.0,xJInj=0.0,NX4hf=0.0,LOUhW=0.0,KSaj_=0.0,jbIj4=0.0,e0vvg=0.0;laktM=
EPTqx*EPTqx;V3sFS=laktM*laktM;KSaj_=cCJ5e*cCJ5e;e0vvg=KSaj_*KSaj_;yP0aV=QCcTr*
S7nLL;Tmhm2=QCcTr*usHjQ;UXXxS=-0.2e1*laktM+0.4e1*KSaj_-0.4e1*yP0aV+0.4e1*Tmhm2;
nDRrN=pbQOc(UXXxS);xJInj=QCcTr*QCcTr;NX4hf=S7nLL*S7nLL;LOUhW=usHjQ*usHjQ;jbIj4=
pbQOc(UXXxS);return((-0.6e1*V3sFS-0.48e2*e0vvg+KSaj_*QCcTr*(0.72e2*S7nLL-0.72e2*
usHjQ)+0.24e2*KSaj_*cCJ5e*nDRrN+cCJ5e*QCcTr*(-0.24e2*S7nLL+0.24e2*usHjQ)*nDRrN+
xJInj*(-0.24e2*NX4hf+0.32e2*S7nLL*usHjQ-0.8e1*LOUhW)+laktM*(0.36e2*KSaj_-0.24e2*
yP0aV+0.16e2*Tmhm2-0.12e2*cCJ5e*nDRrN))/xJInj/jbIj4/UXXxS);}

double qPN_6::UEOTz(const double&cCJ5e,const double&EPTqx,const double&S7nLL,
const double&usHjQ,const double&zFb7J,const double&gBbH5,const double&QCcTr,
const double&x5Jdm){double Au08l=0.0,V3sFS=0.0,KSaj_=0.0;V3sFS=EPTqx*EPTqx;KSaj_
=cCJ5e*cCJ5e;Au08l=pbQOc(-0.2e1*V3sFS+0.4e1*KSaj_-0.4e1*QCcTr*S7nLL+0.4e1*QCcTr*
usHjQ);return((EPTqx-0.2e1*cCJ5e+Au08l)/QCcTr);}
