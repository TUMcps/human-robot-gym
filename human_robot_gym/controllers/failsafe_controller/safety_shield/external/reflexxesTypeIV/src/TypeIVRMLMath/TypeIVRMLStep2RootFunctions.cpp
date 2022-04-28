


































#include <TypeIVRMLStep2RootFunctions.h>
#include <TypeIVRMLMath.h>
#include <TypeIVRMLDefinitions.h>
#include <TypeIVRMLStep1Decisions.h>
#ifdef WIN32
#pragma\
 warning( disable : 4100 )	// to suppress the C4100 compiler warning (unreferenced formal parameter)
#endif


void qPN_6::pOAGI(const double&x5Jdm,const double&olNu8,const double&EPTqx,const
 double&S7nLL,const double&usHjQ,const double&zFb7J,const double&gBbH5,const 
double&QCcTr,double*MjTPS,double*ZEYjO){double olV6T=0.0;*MjTPS=0.0;
*ZEYjO=-(2.0*VNvlY(x5Jdm)-VNvlY(EPTqx)+2.0*QCcTr*(S7nLL-usHjQ))/(2.0*x5Jdm*QCcTr
);olV6T=olNu8-(2.0*x5Jdm-EPTqx)/QCcTr;if(*ZEYjO<0.0){*ZEYjO=0.0;}if(*ZEYjO>olV6T
){*ZEYjO=olV6T;}m4lDi(MjTPS,ZEYjO);return;}

double qPN_6::XzpgJ(const double&EvE6x,const double&olNu8,const double&EPTqx,
const double&S7nLL,const double&usHjQ,const double&zFb7J,const double&gBbH5,
const double&QCcTr,const double&x5Jdm){double FvoS1=0.0,NQbjZ=0.0,NX4hf=0.0,
kqQbP=0.0,OV_Bz=0.0,U8OnA=0.0,YU5aA=0.0,iUVx1=0.0,u5peh=0.0,yPvHT=0.0,eFtGc=0.0,
Qogm3=0.0,KuHU1=0.0,ehCpZ=0.0,TwXRm=0.0,t5GrQ=0.0,yP0aV=0.0,r9J9t=0.0,UXXxS=0.0,
laktM=0.0,V3sFS=0.0,LOUhW=0.0,rDoKh=0.0;laktM=EPTqx*EPTqx;V3sFS=laktM*laktM;
t5GrQ=x5Jdm*x5Jdm;yP0aV=x5Jdm*QCcTr;r9J9t=olNu8*olNu8;UXXxS=r9J9t*QCcTr;OV_Bz=
EvE6x*EvE6x;U8OnA=QCcTr*OV_Bz;FvoS1=0.4e1*S7nLL;NQbjZ=0.4e1*usHjQ;NX4hf=t5GrQ*
x5Jdm;kqQbP=olNu8*QCcTr;LOUhW=QCcTr*EvE6x;iUVx1=kqQbP*EvE6x;u5peh=QCcTr*QCcTr;
yPvHT=0.1e1*gBbH5;eFtGc=EvE6x*S7nLL;Qogm3=olNu8*usHjQ;ehCpZ=EvE6x*usHjQ;TwXRm=
0.1e1*ehCpZ;YU5aA=t5GrQ*t5GrQ;KuHU1=pow(S7nLL-0.1e1*usHjQ,0.2e1);rDoKh=(EPTqx-
0.2e1*x5Jdm+QCcTr*(olNu8-0.1e1*EvE6x));if(rDoKh==0.0){rDoKh=T68ug;}return(
0.41666666666666666666666666666666666667e-1*(0.5e1*V3sFS-0.4e1*laktM*EPTqx*(
0.4e1*x5Jdm+QCcTr*(olNu8+0.2e1*EvE6x))+0.6e1*laktM*(0.4e1*t5GrQ+0.4e1*yP0aV*
EvE6x+QCcTr*(-0.1e1*UXXxS+U8OnA-FvoS1+NQbjZ))-0.12e2*EPTqx*(0.2e1*NX4hf+t5GrQ*(-
0.2e1*kqQbP+0.3e1*LOUhW)+yP0aV*(-0.2e1*iUVx1+U8OnA-FvoS1+NQbjZ)-0.2e1*u5peh*(
zFb7J-yPvHT+eFtGc+Qogm3-TwXRm))+0.12e2*YU5aA+0.24e2*NX4hf*QCcTr*(-0.1e1*olNu8+
EvE6x)+0.12e2*t5GrQ*QCcTr*(UXXxS-0.3e1*iUVx1+U8OnA-0.2e1*S7nLL+0.2e1*usHjQ)+
0.12e2*x5Jdm*u5peh*(-0.4e1*zFb7J+0.4e1*gBbH5+UXXxS*EvE6x-0.1e1*kqQbP*OV_Bz-0.2e1
*eFtGc-0.4e1*Qogm3+0.2e1*ehCpZ)+0.12e2*u5peh*(KuHU1+UXXxS*(S7nLL+usHjQ)+0.2e1*
kqQbP*(zFb7J-yPvHT-TwXRm)+LOUhW*(-0.2e1*zFb7J+0.2e1*gBbH5-0.1e1*eFtGc+ehCpZ)))/
u5peh/rDoKh);}

void qPN_6::WqY2Q(const double&x5Jdm,const double&olNu8,const double&EPTqx,const
 double&S7nLL,const double&usHjQ,const double&zFb7J,const double&gBbH5,const 
double&QCcTr,double*WLFWK,double*Hksa7){double AoAJ_=0.0,BrdFf=0.0;
*Hksa7=pbQOc((double)(0.5*(VNvlY(EPTqx*QCcTr)+2.0*qn3x8(QCcTr)*(usHjQ-S7nLL))))/
QCcTr;BrdFf=0.5*(EPTqx+olNu8*QCcTr);if(*Hksa7>BrdFf){*Hksa7=BrdFf;}if(*Hksa7>
x5Jdm){*Hksa7=x5Jdm;}
AoAJ_=(2.0*EPTqx*olNu8*QCcTr-VNvlY(EPTqx))/(2.0*QCcTr);if(AoAJ_>(usHjQ-S7nLL)){*
WLFWK=EPTqx;}else{
*WLFWK=0.5*(EPTqx+olNu8*QCcTr-pbQOc((double)(2.0*EPTqx*olNu8*QCcTr+VNvlY(olNu8*
QCcTr)+4.0*QCcTr*(S7nLL-usHjQ)-VNvlY(EPTqx))));}if(*WLFWK<0.0){*WLFWK=0.0;}m4lDi
(WLFWK,Hksa7);return;}

double qPN_6::e4nH0(const double&cCJ5e,const double&olNu8,const double&EPTqx,
const double&S7nLL,const double&usHjQ,const double&zFb7J,const double&gBbH5,
const double&QCcTr,const double&x5Jdm){double CncTH=0.0,EvE6x=0.0,t5GrQ=0.0,
kqQbP=0.0,tmhSw=0.0,LOUhW=0.0,laktM=0.0,U8OnA=0.0,LCqju=0.0,V3sFS=0.0,LQB6k=0.0,
zS8C8=0.0,rDoKh=0.0;laktM=EPTqx*EPTqx;V3sFS=laktM*laktM;LQB6k=olNu8*QCcTr;CncTH=
cCJ5e*cCJ5e;EvE6x=olNu8*olNu8;t5GrQ=EvE6x*QCcTr;U8OnA=CncTH*cCJ5e;LCqju=QCcTr*
QCcTr;kqQbP=0.1e1*gBbH5;LOUhW=zFb7J-kqQbP+olNu8*usHjQ;tmhSw=CncTH*CncTH;zS8C8=
pow(S7nLL-0.1e1*usHjQ,0.2e1);rDoKh=(EPTqx-0.2e1*cCJ5e+LQB6k);if(rDoKh==0.0){
rDoKh=T68ug;}return(0.416666666666666666666666666666666666666666666667e-1*(0.5e1
*V3sFS-0.4e1*laktM*EPTqx*(0.4e1*cCJ5e+LQB6k)+0.6e1*laktM*(0.4e1*CncTH-0.1e1*
QCcTr*(t5GrQ+0.4e1*S7nLL-0.4e1*usHjQ))-0.24e2*EPTqx*(U8OnA-0.1e1*CncTH*olNu8*
QCcTr+0.2e1*cCJ5e*QCcTr*(-0.1e1*S7nLL+usHjQ)-0.1e1*LCqju*LOUhW)+0.12e2*tmhSw-
0.24e2*U8OnA*olNu8*QCcTr+0.12e2*CncTH*QCcTr*(t5GrQ-0.2e1*S7nLL+0.2e1*usHjQ)-
0.48e2*cCJ5e*LCqju*LOUhW+0.12e2*LCqju*(0.2e1*LQB6k*(zFb7J-kqQbP)+zS8C8+t5GrQ*(
S7nLL+usHjQ)))/LCqju/rDoKh);}

void qPN_6::d_rMF(const double&x5Jdm,const double&olNu8,const double&EPTqx,const
 double&S7nLL,const double&usHjQ,const double&zFb7J,const double&gBbH5,const 
double&QCcTr,double*c66rw,double*EOOJT){double rDoKh=0.0;rDoKh=(2.0*EPTqx-4.0*
x5Jdm+2.0*olNu8*QCcTr);if(rDoKh==0.0){rDoKh=T68ug;}
*EOOJT=(VNvlY(EPTqx)-2.0*VNvlY(x5Jdm)+2.0*QCcTr*(usHjQ-S7nLL))/rDoKh;if(*EOOJT>
x5Jdm){*EOOJT=x5Jdm;}*c66rw=EPTqx;m4lDi(c66rw,EOOJT);return;}

double qPN_6::MvKFA(const double&JWb5x,const double&olNu8,const double&EPTqx,
const double&S7nLL,const double&usHjQ,const double&zFb7J,const double&gBbH5,
const double&QCcTr,const double&x5Jdm){double VA2lx=0.0,laktM=0.0,xJInj=0.0,
Tmhm2=0.0,V3sFS=0.0,UXXxS=0.0,KSaj_=0.0,RqHYU=0.0,iEiah=0.0,kqQbP=0.0,yP0aV=0.0,
mdI9v=0.0,rDoKh=0.0;laktM=EPTqx*EPTqx;V3sFS=laktM*laktM;KSaj_=laktM*EPTqx;mdI9v=
0.1e1*gBbH5;yP0aV=pow(S7nLL-0.1e1*usHjQ,0.2e1);Tmhm2=x5Jdm*x5Jdm;UXXxS=-0.1e1*
S7nLL+usHjQ;VA2lx=0.2e1*x5Jdm*olNu8*QCcTr;xJInj=Tmhm2-VA2lx+0.2e1*QCcTr*UXXxS;
kqQbP=JWb5x*JWb5x;iEiah=olNu8*olNu8;RqHYU=QCcTr*QCcTr;rDoKh=(JWb5x-0.1e1*x5Jdm);
if(rDoKh==0.0){rDoKh=wNuTo;}return(-0.41666666666666666666666666666666666667e-1*
(0.3e1*V3sFS-0.4e1*KSaj_*x5Jdm+0.12e2*QCcTr*(0.2e1*x5Jdm*QCcTr*(zFb7J-mdI9v+
olNu8*S7nLL)+QCcTr*yP0aV+Tmhm2*UXXxS)+0.6e1*laktM*xJInj+0.6e1*kqQbP*(laktM-0.2e1
*EPTqx*x5Jdm+0.2e1*Tmhm2-VA2lx-0.2e1*QCcTr*S7nLL+0.2e1*QCcTr*usHjQ)-0.4e1*JWb5x*
(0.2e1*KSaj_-0.3e1*laktM*x5Jdm+0.3e1*EPTqx*xJInj+0.3e1*QCcTr*(Tmhm2*olNu8-0.1e1*
x5Jdm*iEiah*QCcTr+0.2e1*QCcTr*(zFb7J-mdI9v+olNu8*usHjQ))))/rDoKh/RqHYU);}

void qPN_6::VrX3P(const double&x5Jdm,const double&olNu8,const double&EPTqx,const
 double&S7nLL,const double&usHjQ,const double&zFb7J,const double&gBbH5,const 
double&QCcTr,double*WLFWK,double*Hksa7){double BrdFf=0.0;
*Hksa7=EPTqx+pbQOc((double)((0.5*QCcTr)*(2.0*(usHjQ-S7nLL)+VNvlY(EPTqx)/QCcTr-
2.0*EPTqx*olNu8)));BrdFf=0.5*(EPTqx+olNu8*QCcTr);if(*Hksa7>BrdFf){*Hksa7=BrdFf;}
if(*Hksa7>x5Jdm){*Hksa7=x5Jdm;}
*WLFWK=0.5*(EPTqx+olNu8*QCcTr-pbQOc((double)(2.0*EPTqx*olNu8*QCcTr+VNvlY(olNu8*
QCcTr)+4.0*QCcTr*(S7nLL-usHjQ)-VNvlY(EPTqx))));if(*WLFWK<0.0){*WLFWK=0.0;}m4lDi(
WLFWK,Hksa7);return;}

double qPN_6::UeTUI(const double&cCJ5e,const double&olNu8,const double&EPTqx,
const double&S7nLL,const double&usHjQ,const double&zFb7J,const double&gBbH5,
const double&QCcTr,const double&x5Jdm){double laktM=0.0,V3sFS=0.0,KSaj_=0.0,
e0vvg=0.0,LQB6k=0.0,CncTH=0.0,EvE6x=0.0,Ttalj=0.0,FvoS1=0.0,tAdoV=0.0,yPvHT=0.0,
rDoKh=0.0;laktM=cCJ5e*cCJ5e;V3sFS=laktM*laktM;KSaj_=EPTqx*EPTqx;e0vvg=KSaj_*
KSaj_;LQB6k=KSaj_*EPTqx;CncTH=olNu8*olNu8;EvE6x=QCcTr*QCcTr;Ttalj=olNu8*QCcTr;
FvoS1=0.1e1*gBbH5;tAdoV=pow(S7nLL-0.1e1*usHjQ,0.2e1);yPvHT=CncTH*QCcTr;rDoKh=(-
0.2e1*cCJ5e+EPTqx+Ttalj);if(rDoKh==0.0){rDoKh=T68ug;}return(-
0.41666666666666666666666666666666666667e-1*(0.12e2*V3sFS+e0vvg+0.4e1*LQB6k*
olNu8*QCcTr+0.6e1*KSaj_*CncTH*EvE6x-0.24e2*laktM*cCJ5e*(EPTqx+Ttalj)-0.8e1*cCJ5e
*(LQB6k+0.3e1*KSaj_*olNu8*QCcTr-0.6e1*EvE6x*(zFb7J-FvoS1+olNu8*S7nLL))-0.24e2*
EPTqx*EvE6x*(zFb7J-FvoS1+olNu8*usHjQ)-0.12e2*EvE6x*(0.2e1*Ttalj*(zFb7J-FvoS1)-
0.1e1*tAdoV+yPvHT*(S7nLL+usHjQ))+0.12e2*laktM*(0.2e1*KSaj_+0.2e1*EPTqx*olNu8*
QCcTr+QCcTr*(yPvHT-0.2e1*S7nLL+0.2e1*usHjQ)))/EvE6x/rDoKh);}

void qPN_6::viKDU(const double&x5Jdm,const double&olNu8,const double&EPTqx,const
 double&S7nLL,const double&usHjQ,const double&zFb7J,const double&gBbH5,const 
double&QCcTr,double*WLFWK,double*Hksa7){double t5GrQ=0.0,L62Wd=0.0,Ttalj=0.0,
laktM=0.0,BIVJE=0.0,ZBiss=0.0,LQB6k=0.0,OV_Bz=0.0,r9J9t=0.0,Au08l=0.0,ZmUm9=0.0,
CVZTL=0.0,MypxW=0.0,N1yDI=0.0,rDoKh=0.0;

laktM=EPTqx*EPTqx;ZBiss=olNu8*QCcTr;rDoKh=(0.4e1*EPTqx+0.4e1*ZBiss);if(rDoKh==
0.0){LQB6k=tlPiC;}else{LQB6k=0.1e1/rDoKh;}BIVJE=laktM*LQB6k;Au08l=EPTqx*olNu8*
QCcTr*LQB6k;t5GrQ=olNu8*olNu8;L62Wd=QCcTr*QCcTr;Ttalj=t5GrQ*L62Wd*LQB6k;r9J9t=
QCcTr*S7nLL*LQB6k;OV_Bz=QCcTr*usHjQ*LQB6k;ZmUm9=EPTqx-0.1e1*BIVJE-0.2e1*Au08l+
Ttalj-0.4e1*r9J9t+0.4e1*OV_Bz;MypxW=-(0.5e0*EPTqx-0.5e0*ZBiss-0.10e1*BIVJE-
0.20e1*Au08l+0.10e1*Ttalj-0.40e1*r9J9t+0.40e1*OV_Bz);CVZTL=0.5*(EPTqx+olNu8*
QCcTr);N1yDI=0.5*(EPTqx-olNu8*QCcTr);if(N1yDI>0.0){N1yDI=0.0;}if(ZmUm9>CVZTL){
ZmUm9=CVZTL;}if(ZmUm9>x5Jdm){ZmUm9=x5Jdm;}if(MypxW<N1yDI){MypxW=N1yDI;}if(MypxW<
-x5Jdm){MypxW=-x5Jdm;}
if(wdgir(EPTqx,x5Jdm,S7nLL,usHjQ,QCcTr)){
*Hksa7=ZmUm9;if((0.5*VNvlY(EPTqx)/QCcTr)>(usHjQ-S7nLL)){*WLFWK=EPTqx;}else{
*WLFWK=(1.0/((double)pbQOc((double)2.0)))*((double)pbQOc(VNvlY(EPTqx)+2.0*(usHjQ
-S7nLL)*QCcTr));}}else{
*WLFWK=EPTqx;*Hksa7=(1.0/((double)pbQOc((double)2.0)))*((double)pbQOc(VNvlY(
EPTqx)+2.0*VNvlY(MypxW)+2.0*(usHjQ-S7nLL)*QCcTr));}if(*WLFWK<EPTqx){*WLFWK=EPTqx
;}m4lDi(WLFWK,Hksa7);return;}

double qPN_6::naN2E(const double&cCJ5e,const double&olNu8,const double&EPTqx,
const double&S7nLL,const double&usHjQ,const double&zFb7J,const double&gBbH5,
const double&QCcTr,const double&x5Jdm){double nDRrN=0.0,IyC_V=0.0,ttbPQ=0.0,
xJInj=0.0,L62Wd=0.0,Ttalj=0.0,laktM=0.0,e0vvg=0.0,tAdoV=0.0;laktM=EPTqx*EPTqx;
e0vvg=cCJ5e*cCJ5e;IyC_V=QCcTr*QCcTr;L62Wd=QCcTr*S7nLL;Ttalj=QCcTr*usHjQ;nDRrN=
pbQOc(IyC_V*(-0.1e1*laktM+0.2e1*e0vvg+0.2e1*L62Wd-0.2e1*Ttalj));ttbPQ=pbQOc(-
0.2e1*laktM+0.4e1*e0vvg+0.4e1*L62Wd-0.4e1*Ttalj);xJInj=-0.8e1*olNu8*IyC_V+
0.42426406871192851464e1*nDRrN+QCcTr*ttbPQ;tAdoV=S7nLL-0.1e1*usHjQ;return(
0.208333333333333333333333333333333333333333e-1*(-0.8e1*laktM*EPTqx*QCcTr+0.48e2
*EPTqx*e0vvg*QCcTr+0.3e1*xJInj*laktM-0.48e2*e0vvg*cCJ5e*QCcTr-0.6e1*QCcTr*(-
0.8e1*IyC_V*(zFb7J-0.1e1*gBbH5+olNu8*S7nLL)+QCcTr*tAdoV*ttbPQ+
0.42426406871192851464e1*nDRrN*tAdoV)-0.6e1*e0vvg*xJInj)/IyC_V/QCcTr);}

void qPN_6::H1iwP(const double&x5Jdm,const double&olNu8,const double&EPTqx,const
 double&S7nLL,const double&usHjQ,const double&zFb7J,const double&gBbH5,const 
double&QCcTr,double*WLFWK,double*Hksa7){double TR9di=0.0,shsBT=0.0,sqGvl=0.0,
TwAY8=0.0,Lj91q=0.0
,cfQSX=0.0
,tpyUy=0.0
,TdPa7=0.0
,L38Np=0.0
,n6gL7=0.0,Ue0wt=0.0,wJKjp=0.0,Qzpol=0.0,BrdFf=0.0;
TR9di=pbQOc((double)(0.5*(1.0*VNvlY(EPTqx)-2.0*QCcTr*S7nLL+2.0*QCcTr*usHjQ)));
BrdFf=0.5*(EPTqx+olNu8*QCcTr);if(TR9di>BrdFf){TR9di=BrdFf;}
if(TR9di>x5Jdm){*Hksa7=x5Jdm;}else{*Hksa7=TR9di;}




shsBT=(0.5*VNvlY(EPTqx)+VNvlY(x5Jdm))/QCcTr;if(shsBT>(usHjQ-S7nLL)){*WLFWK=EPTqx
;}else{*WLFWK=pbQOc((double)(0.5*(VNvlY(EPTqx)-2.0*VNvlY(x5Jdm)-2.0*QCcTr*S7nLL+
2.0*QCcTr*usHjQ)));}if(*WLFWK<EPTqx){*WLFWK=EPTqx;}
L38Np=(pbQOc(QCcTr)*pbQOc(VNvlY(EPTqx)/QCcTr-2.0*S7nLL+2.0*usHjQ))/2.0;
Lj91q=(L38Np-EPTqx)/QCcTr;
cfQSX=tpyUy=TdPa7=L38Np/QCcTr;;if((Lj91q+cfQSX+tpyUy+TdPa7)>olNu8){sqGvl=0.25*
EPTqx+0.25*olNu8*QCcTr-0.25*pbQOc(3.0*VNvlY(EPTqx)-2.0*EPTqx*olNu8*QCcTr+QCcTr*(
8.0*(usHjQ-S7nLL)-VNvlY(olNu8)*QCcTr));TwAY8=0.25*EPTqx+0.25*olNu8*QCcTr+0.25*
pbQOc(3.0*VNvlY(EPTqx)-2.0*EPTqx*olNu8*QCcTr+QCcTr*(8.0*(usHjQ-S7nLL)-VNvlY(
olNu8)*QCcTr));if(sqGvl<EPTqx){sqGvl=EPTqx;}if(TwAY8<EPTqx){TwAY8=EPTqx;}n6gL7=
EQBeY(*WLFWK,olNu8,EPTqx,S7nLL,usHjQ,zFb7J,gBbH5,QCcTr,x5Jdm);Ue0wt=EQBeY(sqGvl,
olNu8,EPTqx,S7nLL,usHjQ,zFb7J,gBbH5,QCcTr,x5Jdm);wJKjp=EQBeY(TwAY8,olNu8,EPTqx,
S7nLL,usHjQ,zFb7J,gBbH5,QCcTr,x5Jdm);Qzpol=EQBeY(*Hksa7,olNu8,EPTqx,S7nLL,usHjQ,
zFb7J,gBbH5,QCcTr,x5Jdm);if(((fabs(n6gL7)<fabs(Ue0wt))&&(fabs(n6gL7)<fabs(wJKjp)
)&&(fabs(n6gL7)<fabs(Qzpol)))||((fabs(Ue0wt)<fabs(n6gL7))&&(fabs(Ue0wt)<fabs(
wJKjp))&&(fabs(Ue0wt)<fabs(Qzpol)))){*Hksa7=sqGvl;}else{*WLFWK=TwAY8;}}m4lDi(
WLFWK,Hksa7);return;}

double qPN_6::EQBeY(const double&cCJ5e,const double&olNu8,const double&EPTqx,
const double&S7nLL,const double&usHjQ,const double&zFb7J,const double&gBbH5,
const double&QCcTr,const double&x5Jdm){double KSaj_=0.0,yP0aV=0.0,Tmhm2=0.0,
laktM=0.0,lmPpg=0.0;laktM=EPTqx*EPTqx;KSaj_=cCJ5e*cCJ5e;yP0aV=pbQOc(laktM-0.2e1*
KSaj_-0.2e1*QCcTr*S7nLL+0.2e1*QCcTr*usHjQ);Tmhm2=-0.2e1*olNu8*QCcTr+
0.14142135623730950488e1*yP0aV;lmPpg=QCcTr*QCcTr;return(
0.8333333333333333333333333333333333333333e-1*(-0.2e1*laktM*EPTqx+0.12e2*EPTqx*
KSaj_+0.3e1*laktM*Tmhm2-0.12e2*KSaj_*cCJ5e-0.6e1*KSaj_*Tmhm2-0.6e1*QCcTr*(-0.2e1
*QCcTr*(zFb7J-0.1e1*gBbH5+olNu8*S7nLL)+0.14142135623730950488e1*(S7nLL-0.1e1*
usHjQ)*yP0aV))/lmPpg);}

void qPN_6::Zn7EB(const double&x5Jdm,const double&olNu8,const double&EPTqx,const
 double&S7nLL,const double&usHjQ,const double&zFb7J,const double&gBbH5,const 
double&QCcTr,double*WLFWK,double*Hksa7){double shsBT=0.0,V3sFS=0.0,L62Wd=0.0,
BrdFf=0.0;




shsBT=(0.5*VNvlY(EPTqx)-VNvlY(x5Jdm))/QCcTr;if(shsBT>(usHjQ-S7nLL)){*WLFWK=EPTqx
;}else{*WLFWK=pbQOc((double)(0.5*(1.0*VNvlY(EPTqx)+2.0*VNvlY(x5Jdm)-2.0*QCcTr*
S7nLL+2.0*QCcTr*usHjQ)));}



V3sFS=EPTqx*EPTqx;L62Wd=pbQOc(0.1e1*V3sFS+0.2e1*EPTqx*x5Jdm+0.2e1*x5Jdm*olNu8*
QCcTr-0.2e1*QCcTr*S7nLL+0.2e1*QCcTr*usHjQ);*Hksa7=-0.1e1*x5Jdm+pbQOc(0.5)*L62Wd;
BrdFf=0.5*(EPTqx+(olNu8-2.0*x5Jdm/QCcTr)*QCcTr);if(BrdFf>x5Jdm){BrdFf=x5Jdm;}if(
*Hksa7>BrdFf){*Hksa7=BrdFf;}if(*WLFWK<0.0){*WLFWK=0.0;}m4lDi(WLFWK,Hksa7);return
;}

double qPN_6::AW_QQ(const double&cCJ5e,const double&olNu8,const double&EPTqx,
const double&S7nLL,const double&usHjQ,const double&zFb7J,const double&gBbH5,
const double&QCcTr,const double&x5Jdm){double mdI9v=0.0,Au08l=0.0,SwlHG=0.0,
t5GrQ=0.0,iUVx1=0.0,Ttalj=0.0,Tmhm2=0.0,laktM=0.0,V3sFS=0.0,tAdoV=0.0,TwXRm=0.0,
iEiah=0.0;laktM=EPTqx*EPTqx;V3sFS=laktM*laktM;mdI9v=cCJ5e*cCJ5e;Au08l=mdI9v*
mdI9v;t5GrQ=mdI9v*QCcTr;Ttalj=QCcTr*QCcTr;Tmhm2=S7nLL*S7nLL;iUVx1=usHjQ*usHjQ;
tAdoV=x5Jdm*x5Jdm;TwXRm=QCcTr*S7nLL;iEiah=QCcTr*usHjQ;SwlHG=-0.125e0*V3sFS-
0.16666666666666666666666666666666666667e0*laktM*EPTqx*x5Jdm+0.1e1*EPTqx*x5Jdm*
mdI9v-0.5e0*Au08l-0.1e1*t5GrQ*S7nLL-0.5e0*Tmhm2*Ttalj+x5Jdm*(-0.1e1*mdI9v*cCJ5e+
0.1e1*mdI9v*olNu8*QCcTr+Ttalj*(0.1e1*zFb7J-0.1e1*gBbH5+0.1e1*olNu8*S7nLL))+0.1e1
*t5GrQ*usHjQ+0.1e1*Ttalj*S7nLL*usHjQ-0.5e0*Ttalj*iUVx1+laktM*(0.25e0*tAdoV+0.5e0
*mdI9v-0.5e0*x5Jdm*olNu8*QCcTr+0.5e0*TwXRm-0.5e0*iEiah)+tAdoV*(-0.5e0*mdI9v-
0.5e0*TwXRm+0.5e0*iEiah);return(SwlHG/x5Jdm/Ttalj);}

void qPN_6::ZJOyu(const double&x5Jdm,const double&olNu8,const double&EPTqx,const
 double&S7nLL,const double&usHjQ,const double&zFb7J,const double&gBbH5,const 
double&QCcTr,double*WLFWK,double*Hksa7){bool xSyqv=false,h0Qso=false;double 
EzU9y=0.0,sEdcq=0.0,LmWyu=0.0,p7P_H=0.0,vxTtH=0.0,V3sFS=0.0,ZBiss=0.0,laktM=0.0,
KSaj_=0.0,yP0aV=0.0,e0vvg=0.0,LQB6k=0.0,mdI9v=0.0,BrdFf=0.0;







laktM=EPTqx*EPTqx;V3sFS=0.1e1/QCcTr;e0vvg=x5Jdm*x5Jdm;EzU9y=-0.5e0*laktM*V3sFS+
0.2e1*e0vvg*V3sFS;if(EzU9y>(usHjQ-S7nLL)){xSyqv=false;laktM=EPTqx*EPTqx;ZBiss=
x5Jdm*x5Jdm;p7P_H=pbQOc((double)(0.5*(0.1e1*laktM-0.2e1*ZBiss-0.2e1*QCcTr*S7nLL+
0.2e1*QCcTr*usHjQ)));}else{xSyqv=true;p7P_H=x5Jdm;}


if(xSyqv){




laktM=0.1e1/QCcTr;KSaj_=EPTqx*EPTqx;e0vvg=0.1e1/x5Jdm;sEdcq=-0.1e1*EPTqx*laktM+
0.5e0*KSaj_*e0vvg*laktM+0.2e1*x5Jdm*laktM-0.1e1*S7nLL*e0vvg+0.1e1*usHjQ*e0vvg;if
(sEdcq>olNu8){h0Qso=true;}else{h0Qso=false;}}else{laktM=0.1e1/QCcTr;LQB6k=EPTqx*
EPTqx;mdI9v=x5Jdm*x5Jdm;yP0aV=pbQOc(0.1e1*LQB6k-0.2e1*mdI9v-0.2e1*QCcTr*S7nLL+
0.2e1*QCcTr*usHjQ);LmWyu=-0.1e1*EPTqx*laktM+0.2e1*x5Jdm*laktM+pbQOc((double)2.0)
*yP0aV*laktM;if(LmWyu>olNu8){h0Qso=true;}else{h0Qso=false;}}if(h0Qso){laktM=
EPTqx*EPTqx;e0vvg=x5Jdm*x5Jdm;yP0aV=pbQOc((double)(0.5*(0.1e1*laktM-0.2e1*EPTqx*
x5Jdm+0.4e1*e0vvg-0.2e1*x5Jdm*olNu8*QCcTr-0.2e1*QCcTr*S7nLL+0.2e1*QCcTr*usHjQ)))
;vxTtH=x5Jdm-yP0aV;}else{vxTtH=x5Jdm;}BrdFf=0.5*(EPTqx+(olNu8-2.0*x5Jdm/QCcTr)*
QCcTr);if(BrdFf>x5Jdm){BrdFf=x5Jdm;}if(p7P_H<x5Jdm){*Hksa7=p7P_H;}else{*Hksa7=
x5Jdm;}if(vxTtH<*Hksa7){*Hksa7=vxTtH;}if(*Hksa7>BrdFf){*Hksa7=BrdFf;}*WLFWK=
EPTqx;if(*WLFWK<0.0){*WLFWK=0.0;}m4lDi(WLFWK,Hksa7);return;}

double qPN_6::TGgrw(const double&cCJ5e,const double&olNu8,const double&EPTqx,
const double&S7nLL,const double&usHjQ,const double&zFb7J,const double&gBbH5,
const double&QCcTr,const double&x5Jdm){double laktM=0.0,V3sFS=0.0,mdI9v=0.0,
Au08l=0.0,t5GrQ=0.0,Ttalj=0.0,Tmhm2=0.0,iUVx1=0.0,tAdoV=0.0,u5peh=0.0,eFtGc=0.0,
ehCpZ=0.0,qoEJO=0.0;laktM=EPTqx*EPTqx;V3sFS=laktM*laktM;mdI9v=cCJ5e*cCJ5e;Au08l=
mdI9v*mdI9v;t5GrQ=mdI9v*QCcTr;Ttalj=QCcTr*QCcTr;Tmhm2=S7nLL*S7nLL;iUVx1=usHjQ*
usHjQ;tAdoV=x5Jdm*x5Jdm;u5peh=0.5e0*mdI9v;eFtGc=0.5e0*QCcTr*S7nLL;ehCpZ=0.5e0*
QCcTr*usHjQ;qoEJO=0.125e0*V3sFS-0.16666666666666666666666666666666667e0*laktM*
EPTqx*x5Jdm+0.1e1*EPTqx*x5Jdm*mdI9v+0.5e0*Au08l+0.1e1*t5GrQ*S7nLL+0.5e0*Tmhm2*
Ttalj+x5Jdm*(-0.1e1*mdI9v*cCJ5e+0.1e1*mdI9v*olNu8*QCcTr+Ttalj*(0.1e1*zFb7J-0.1e1
*gBbH5+0.1e1*olNu8*S7nLL))-0.1e1*t5GrQ*usHjQ-0.1e1*Ttalj*S7nLL*usHjQ+0.5e0*Ttalj
*iUVx1+tAdoV*(-u5peh-eFtGc+ehCpZ)+laktM*(0.25e0*tAdoV-u5peh-0.5e0*x5Jdm*olNu8*
QCcTr-eFtGc+ehCpZ);return(qoEJO/x5Jdm/Ttalj);}

void qPN_6::uS_9X(const double&x5Jdm,const double&olNu8,const double&EPTqx,const
 double&S7nLL,const double&usHjQ,const double&zFb7J,const double&gBbH5,const 
double&QCcTr,double*PM8Ak,double*PnXnX){double Ig3wi=0.0,CytxK=0.0;Ig3wi=(VNvlY(
x5Jdm)-0.5*VNvlY(EPTqx))/QCcTr;if(Ig3wi>(usHjQ-S7nLL)){*PnXnX=-pbQOc((double)((
Ig3wi-usHjQ+S7nLL)*QCcTr));}else{*PnXnX=0.0;}



*PM8Ak=x5Jdm-pbQOc((double)(EPTqx*(-0.5*EPTqx+x5Jdm)+QCcTr*(x5Jdm*olNu8+S7nLL-
usHjQ)));CytxK=-0.5*(olNu8-(2.0*x5Jdm-EPTqx)/QCcTr)*QCcTr;if(CytxK<-x5Jdm){CytxK
=-x5Jdm;}if(*PM8Ak<CytxK){*PM8Ak=CytxK;}
if(*PM8Ak>*PnXnX){*PM8Ak=*PnXnX;}if(*PM8Ak>0.0){*PM8Ak=0.0;}m4lDi(PM8Ak,PnXnX);
return;}

double qPN_6::pXGYp(const double&ju5eQ,const double&olNu8,const double&EPTqx,
const double&S7nLL,const double&usHjQ,const double&zFb7J,const double&gBbH5,
const double&QCcTr,const double&x5Jdm){double mdI9v=0.0,wTpPJ=0.0,R4DaO=0.0,
laktM=0.0,ZBiss=0.0,KSaj_=0.0,BIVJE=0.0,EvE6x=0.0,FvoS1=0.0,suxJ2=0.0,ttbPQ=0.0,
u5peh=0.0,iEiah=0.0;laktM=EPTqx*EPTqx;ZBiss=QCcTr*QCcTr;KSaj_=0.1e1/ZBiss;BIVJE=
laktM*laktM;mdI9v=0.1e1/x5Jdm;EvE6x=ju5eQ*ju5eQ;FvoS1=QCcTr*S7nLL;suxJ2=QCcTr*
usHjQ;ttbPQ=-0.5e0*EvE6x+0.5e0*FvoS1-0.5e0*suxJ2;u5peh=EvE6x*EvE6x;iEiah=S7nLL*
S7nLL;R4DaO=usHjQ*usHjQ;wTpPJ=0.33333333333333333333333333333333333333e0*laktM*
EPTqx*KSaj_-0.125e0*BIVJE*mdI9v*KSaj_+0.1e1*EvE6x*ju5eQ*KSaj_+0.1e1*EvE6x*olNu8/
QCcTr+zFb7J-0.1e1*gBbH5+olNu8*usHjQ+x5Jdm*ttbPQ*KSaj_+EPTqx*(0.1e1*EvE6x-0.1e1*
FvoS1+0.1e1*suxJ2)*KSaj_+laktM*(-0.25e0*x5Jdm*KSaj_+ttbPQ*mdI9v*KSaj_)+(-0.5e0*
u5peh+EvE6x*QCcTr*(0.1e1*S7nLL-0.1e1*usHjQ)+ZBiss*(-0.5e0*iEiah+0.1e1*S7nLL*
usHjQ-0.5e0*R4DaO))*mdI9v*KSaj_;return(wTpPJ);}

void qPN_6::gG0XJ(const double&x5Jdm,const double&olNu8,const double&EPTqx,const
 double&S7nLL,const double&usHjQ,const double&zFb7J,const double&gBbH5,const 
double&QCcTr,double*MjTPS,double*ZEYjO){double ENArr=0.0,olV6T=0.0;

*ZEYjO=(VNvlY(EPTqx)/0.4e1+EPTqx*x5Jdm/0.2e1-0.2e1*VNvlY(x5Jdm)+x5Jdm*QCcTr*
olNu8/0.2e1-QCcTr*S7nLL/0.2e1+QCcTr*usHjQ/0.2e1)/x5Jdm/QCcTr;olV6T=olNu8-(4.0*
x5Jdm-EPTqx)/QCcTr;if(*ZEYjO>olV6T){*ZEYjO=olV6T;}

ENArr=0.5*VNvlY(EPTqx)/QCcTr;if((S7nLL-ENArr)<=(usHjQ)){*MjTPS=(usHjQ-S7nLL+
ENArr)/x5Jdm;}else{*MjTPS=0.0;}if(*MjTPS<0.0){*MjTPS=0.0;}m4lDi(MjTPS,ZEYjO);
return;}

double qPN_6::FmO3l(const double&EvE6x,const double&olNu8,const double&EPTqx,
const double&S7nLL,const double&usHjQ,const double&zFb7J,const double&gBbH5,
const double&QCcTr,const double&x5Jdm){double VA2lx=0.0,ehCpZ=0.0,mdI9v=0.0,
laktM=0.0,V3sFS=0.0,PW4id=0.0,Au08l=0.0,L62Wd=0.0,SwlHG=0.0,BIVJE=0.0;laktM=
EPTqx*EPTqx;V3sFS=laktM*laktM;BIVJE=x5Jdm*x5Jdm;mdI9v=BIVJE*BIVJE;Au08l=olNu8*
QCcTr;L62Wd=QCcTr*EvE6x;VA2lx=EvE6x*EvE6x;ehCpZ=QCcTr*QCcTr;SwlHG=S7nLL*S7nLL;
PW4id=usHjQ*usHjQ;return((-0.125e0*V3sFS-
0.16666666666666666666666666666666666667e0*laktM*EPTqx*x5Jdm-0.2e1*mdI9v+BIVJE*
x5Jdm*(0.1e1*Au08l-0.3e1*L62Wd)+EPTqx*BIVJE*(0.1e1*x5Jdm+0.1e1*L62Wd)+BIVJE*
QCcTr*(0.1e1*Au08l*EvE6x-0.1e1*QCcTr*VA2lx-0.15e1*S7nLL+0.15e1*usHjQ)+laktM*(
0.75e0*BIVJE-0.5e0*x5Jdm*olNu8*QCcTr+0.5e0*x5Jdm*QCcTr*EvE6x+0.5e0*QCcTr*S7nLL-
0.5e0*QCcTr*usHjQ)+x5Jdm*ehCpZ*(0.1e1*zFb7J-0.1e1*gBbH5+0.1e1*olNu8*S7nLL-0.1e1*
EvE6x*S7nLL+0.1e1*EvE6x*usHjQ)+ehCpZ*(-0.5e0*SwlHG+0.1e1*S7nLL*usHjQ-0.5e0*PW4id
))/x5Jdm/ehCpZ);}

void qPN_6::lWVnt(const double&x5Jdm,const double&olNu8,const double&EPTqx,const
 double&S7nLL,const double&usHjQ,const double&zFb7J,const double&gBbH5,const 
double&QCcTr,double*PM8Ak,double*PnXnX){double ndHbt=0.0,wWHPO=0.0;*PM8Ak=0.0;
ndHbt=(-0.5e0*VNvlY(EPTqx)+EPTqx*x5Jdm+x5Jdm*(-0.2e1*x5Jdm+0.1e1*olNu8*QCcTr))/
QCcTr;if(ndHbt<(usHjQ-S7nLL)){
*PnXnX=x5Jdm-pbQOc((double)(2.0*(0.25*VNvlY(EPTqx)-0.5*EPTqx*x5Jdm+VNvlY(x5Jdm)-
0.5*x5Jdm*olNu8*QCcTr-0.5*QCcTr*S7nLL+0.5*QCcTr*usHjQ)));}else{*PnXnX=x5Jdm;}
wWHPO=0.5*(olNu8-(2.0*x5Jdm-EPTqx)/QCcTr)*QCcTr;if(wWHPO>x5Jdm){wWHPO=x5Jdm;}if(
*PnXnX>wWHPO){*PnXnX=wWHPO;}m4lDi(PM8Ak,PnXnX);return;}

double qPN_6::Ck2un(const double&ju5eQ,const double&olNu8,const double&EPTqx,
const double&S7nLL,const double&usHjQ,const double&zFb7J,const double&gBbH5,
const double&QCcTr,const double&x5Jdm){double iEiah=0.0,laktM=0.0,R4DaO=0.0,
FvoS1=0.0,u5peh=0.0,suxJ2=0.0,ttbPQ=0.0,ZBiss=0.0,KSaj_=0.0,BIVJE=0.0,mdI9v=0.0,
EvE6x=0.0,wTpPJ=0.0;laktM=EPTqx*EPTqx;ZBiss=QCcTr*QCcTr;KSaj_=0.1e1/ZBiss;BIVJE=
laktM*laktM;mdI9v=0.1e1/x5Jdm;EvE6x=ju5eQ*ju5eQ;FvoS1=QCcTr*S7nLL;suxJ2=QCcTr*
usHjQ;ttbPQ=0.5e0*EvE6x+0.5e0*FvoS1-0.5e0*suxJ2;u5peh=EvE6x*EvE6x;iEiah=S7nLL*
S7nLL;R4DaO=usHjQ*usHjQ;wTpPJ=0.3333333333333333333333333333333333333333e0*laktM
*EPTqx*KSaj_-0.125e0*BIVJE*mdI9v*KSaj_+0.1e1*EvE6x*ju5eQ*KSaj_-0.1e1*EvE6x*olNu8
/QCcTr+zFb7J-0.1e1*gBbH5+olNu8*usHjQ+x5Jdm*ttbPQ*KSaj_+EPTqx*(-0.1e1*EvE6x-0.1e1
*FvoS1+0.1e1*suxJ2)*KSaj_+laktM*(-0.25e0*x5Jdm*KSaj_+ttbPQ*mdI9v*KSaj_)+(-0.5e0*
u5peh+EvE6x*QCcTr*(-0.1e1*S7nLL+0.1e1*usHjQ)+ZBiss*(-0.5e0*iEiah+0.1e1*S7nLL*
usHjQ-0.5e0*R4DaO))*mdI9v*KSaj_;return(wTpPJ);}

void qPN_6::hBNRM(const double&x5Jdm,const double&olNu8,const double&EPTqx,const
 double&S7nLL,const double&usHjQ,const double&zFb7J,const double&gBbH5,const 
double&QCcTr,double*MjTPS,double*ZEYjO){double vQ9KK=0.0,rDoKh=0.0,olV6T=0.0;
rDoKh=(0.5*(x5Jdm-EPTqx)+EPTqx);if(rDoKh==0.0){rDoKh=T68ug;}vQ9KK=usHjQ-S7nLL-
1.5*VNvlY(x5Jdm)/QCcTr+(x5Jdm-EPTqx)/QCcTr*rDoKh;olV6T=olNu8-(4.0*x5Jdm-EPTqx)/
QCcTr;*MjTPS=0.0;*ZEYjO=vQ9KK/x5Jdm;if(*ZEYjO>olV6T){*ZEYjO=olV6T;}m4lDi(MjTPS,
ZEYjO);return;}

double qPN_6::Cjc4b(const double&EvE6x,const double&olNu8,const double&EPTqx,
const double&S7nLL,const double&usHjQ,const double&zFb7J,const double&gBbH5,
const double&QCcTr,const double&x5Jdm){double BIVJE=0.0,laktM=0.0,V3sFS=0.0,
mdI9v=0.0,Au08l=0.0,L62Wd=0.0,yPvHT=0.0,qoEJO=0.0,zS8C8=0.0;laktM=EPTqx*EPTqx;
V3sFS=laktM*laktM;BIVJE=x5Jdm*x5Jdm;mdI9v=BIVJE*BIVJE;Au08l=olNu8*QCcTr;L62Wd=
QCcTr*EvE6x;yPvHT=QCcTr*QCcTr;qoEJO=S7nLL*S7nLL;zS8C8=usHjQ*usHjQ;return((
0.125e0*V3sFS-0.166666666666666666666666666666667e0*laktM*EPTqx*x5Jdm-0.1e1*
mdI9v+BIVJE*x5Jdm*(0.1e1*Au08l-0.1e1*L62Wd)+EPTqx*BIVJE*(0.1e1*x5Jdm+0.1e1*L62Wd
)+BIVJE*QCcTr*(0.1e1*Au08l*EvE6x+0.5e0*S7nLL-0.5e0*usHjQ)+laktM*(-0.25e0*BIVJE-
0.5e0*x5Jdm*olNu8*QCcTr-0.5e0*x5Jdm*QCcTr*EvE6x-0.5e0*QCcTr*S7nLL+0.5e0*QCcTr*
usHjQ)+x5Jdm*yPvHT*(0.1e1*zFb7J-0.1e1*gBbH5+0.1e1*olNu8*S7nLL+0.1e1*EvE6x*S7nLL-
0.1e1*EvE6x*usHjQ)+yPvHT*(0.5e0*qoEJO-0.1e1*S7nLL*usHjQ+0.5e0*zS8C8))/x5Jdm/
yPvHT);}

void qPN_6::UREKg(const double&x5Jdm,const double&olNu8,const double&EPTqx,const
 double&S7nLL,const double&usHjQ,const double&zFb7J,const double&gBbH5,const 
double&QCcTr,double*c66rw,double*EOOJT){double KSaj_=0.0,t5GrQ=0.0,L62Wd=0.0,
OV_Bz=0.0,eA7Xu=0.0,SVqfm=0.0,mK9dq=0.0,rDoKh=0.0;
SVqfm=(EPTqx+2.0*pbQOc(0.5*(2.0*QCcTr*(usHjQ-S7nLL)-VNvlY(EPTqx))))/QCcTr;if(
SVqfm<olNu8){
eA7Xu=pbQOc(0.5*(VNvlY(EPTqx)-2.0*QCcTr*(usHjQ-S7nLL)));if(eA7Xu>x5Jdm){
KSaj_=EPTqx*EPTqx;t5GrQ=olNu8*olNu8;L62Wd=QCcTr*QCcTr;OV_Bz=pbQOc(-KSaj_+0.4e1*
EPTqx*x5Jdm-0.2e1*EPTqx*olNu8*QCcTr-0.4e1*x5Jdm*olNu8*QCcTr+t5GrQ*L62Wd-0.4e1*
QCcTr*S7nLL+0.4e1*QCcTr*usHjQ);*c66rw=EPTqx/0.2e1+x5Jdm-olNu8*QCcTr/0.2e1+OV_Bz/
0.2e1;}else{*c66rw=0.0;}}else{

rDoKh=(4.0*(EPTqx-olNu8*QCcTr));if(rDoKh==0.0){rDoKh=T68ug;}*c66rw=(3.0*VNvlY(
EPTqx)-2.0*EPTqx*olNu8*QCcTr+VNvlY(olNu8)*VNvlY(QCcTr)+4.0*QCcTr*(S7nLL-usHjQ))/
rDoKh;}mK9dq=EPTqx*olNu8-0.5*VNvlY(EPTqx)/QCcTr;if(mK9dq<=(usHjQ-S7nLL)){*EOOJT=
EPTqx;}else{rDoKh=(2.0*(EPTqx-olNu8*QCcTr));if(rDoKh==0.0){rDoKh=T68ug;}
*EOOJT=(2.0*QCcTr*(S7nLL-usHjQ)+VNvlY(EPTqx))/rDoKh;}if(*c66rw<0.0){*c66rw=0.0;}
m4lDi(c66rw,EOOJT);return;}

double qPN_6::IUU3u(const double&JWb5x,const double&olNu8,const double&EPTqx,
const double&S7nLL,const double&usHjQ,const double&zFb7J,const double&gBbH5,
const double&QCcTr,const double&x5Jdm){double tAdoV=0.0,laktM=0.0,V3sFS=0.0,
ZBiss=0.0,NQbjZ=0.0,CCBY_=0.0,r9J9t=0.0,NX4hf=0.0,Au08l=0.0,je6YR=0.0,VA2lx=0.0,
U8OnA=0.0,K9egE=0.0,LQB6k=0.0;laktM=JWb5x*JWb5x;V3sFS=QCcTr*QCcTr;ZBiss=0.1e1/
V3sFS;LQB6k=0.1e1/QCcTr;Au08l=EPTqx*EPTqx;r9J9t=JWb5x*EPTqx;U8OnA=JWb5x*olNu8*
QCcTr;NQbjZ=QCcTr*S7nLL;VA2lx=QCcTr*usHjQ;NX4hf=pbQOc((0.2e1*r9J9t-0.1e1*Au08l-
0.2e1*U8OnA-0.2e1*NQbjZ+0.2e1*VA2lx)*ZBiss);tAdoV=pbQOc(0.225e1*r9J9t-0.1125e1*
Au08l-0.225e1*U8OnA-0.225e1*NQbjZ+0.225e1*VA2lx);K9egE=QCcTr*NX4hf;CCBY_=olNu8*
olNu8;je6YR=laktM*(0.5e0*ZBiss*EPTqx-0.5e0*LQB6k*olNu8)-
0.1666666666666666666666666666666667e0*Au08l*EPTqx*ZBiss+zFb7J-0.1e1*gBbH5+0.1e1
*olNu8*S7nLL-0.17677669529663689318e0*S7nLL*NX4hf+0.17677669529663689318e0*usHjQ
*NX4hf-0.5e0*S7nLL*tAdoV*LQB6k+0.5e0*usHjQ*tAdoV*LQB6k+Au08l*(0.5e0*olNu8*QCcTr-
0.88388347648318446592e-1*K9egE-0.25e0*tAdoV)*ZBiss+JWb5x*(0.5e0*CCBY_-0.1e1*
S7nLL*LQB6k+0.1e1*usHjQ*LQB6k+olNu8*(-0.1e1*EPTqx-0.17677669529663689318e0*K9egE
-0.5e0*tAdoV)*LQB6k+EPTqx*(0.17677669529663689318e0*K9egE+0.5e0*tAdoV)*ZBiss);
return(je6YR);}

void qPN_6::tmMnv(const double&x5Jdm,const double&olNu8,const double&EPTqx,const
 double&S7nLL,const double&usHjQ,const double&zFb7J,const double&gBbH5,const 
double&QCcTr,double*c66rw,double*EOOJT){double r9dGs=0.0,aTBHx=0.0,QBRYe=0.0,
e0vvg=0.0,yP0aV=0.0,Ttalj=0.0,NQbjZ=0.0,p9L5z=0.0;r9dGs=(0.5*VNvlY(EPTqx)+VNvlY(
x5Jdm))/QCcTr;if(r9dGs>(usHjQ-S7nLL)){
*c66rw=x5Jdm-pbQOc(0.5*VNvlY(EPTqx)-EPTqx*x5Jdm+x5Jdm*olNu8*QCcTr+QCcTr*(S7nLL-
usHjQ));}else{aTBHx=(EPTqx+x5Jdm)/QCcTr+((-0.5*VNvlY(EPTqx))/QCcTr-S7nLL+usHjQ)/
x5Jdm;if(aTBHx>olNu8){
*c66rw=x5Jdm-pbQOc(0.5*VNvlY(EPTqx)-EPTqx*x5Jdm+x5Jdm*olNu8*QCcTr+QCcTr*(S7nLL-
usHjQ));}else{*c66rw=0.0;}}
QBRYe=EPTqx*olNu8+(0.5*VNvlY(EPTqx)-2.0*EPTqx*x5Jdm+VNvlY(x5Jdm))/QCcTr;if(QBRYe
<(usHjQ-S7nLL)){*EOOJT=EPTqx;}else{
e0vvg=EPTqx*EPTqx;yP0aV=olNu8*olNu8;Ttalj=QCcTr*QCcTr;NQbjZ=pbQOc(-0.1e1*e0vvg+
0.4e1*EPTqx*x5Jdm-0.2e1*EPTqx*olNu8*QCcTr-0.4e1*x5Jdm*olNu8*QCcTr+0.1e1*yP0aV*
Ttalj-0.4e1*QCcTr*S7nLL+0.4e1*QCcTr*usHjQ);*EOOJT=0.50e0*EPTqx+0.1e1*x5Jdm-
0.50e0*olNu8*QCcTr+0.50e0*NQbjZ;}p9L5z=0.5*(EPTqx+2.0*x5Jdm-olNu8*QCcTr);if(
p9L5z<0.0){p9L5z=0.0;}if(*c66rw<p9L5z){*c66rw=p9L5z;}m4lDi(c66rw,EOOJT);return;}

double qPN_6::viC08(const double&JWb5x,const double&olNu8,const double&EPTqx,
const double&S7nLL,const double&usHjQ,const double&zFb7J,const double&gBbH5,
const double&QCcTr,const double&x5Jdm){double CncTH=0.0,V3sFS=0.0,G7uFI=0.0,
KSaj_=0.0,iUVx1=0.0,Au08l=0.0,xJInj=0.0,BIVJE=0.0,TwXRm=0.0,UXXxS=0.0,laktM=0.0,
ttbPQ=0.0,suxJ2=0.0,rDoKh=0.0;laktM=EPTqx*EPTqx;V3sFS=laktM*laktM;KSaj_=laktM*
EPTqx;BIVJE=JWb5x*JWb5x;CncTH=x5Jdm*olNu8;Au08l=0.1e1*usHjQ;UXXxS=0.1e1*gBbH5;
suxJ2=pow(S7nLL-Au08l,0.2e1);ttbPQ=x5Jdm*x5Jdm;xJInj=-0.1e1*S7nLL+usHjQ;iUVx1=
ttbPQ-0.2e1*CncTH*QCcTr+0.2e1*QCcTr*xJInj;TwXRm=olNu8*olNu8;G7uFI=QCcTr*QCcTr;
rDoKh=(JWb5x-0.1e1*x5Jdm);if(rDoKh==0.0){rDoKh=T68ug;}return(
0.4166666666666666666666666666666666667e-1*(-0.3e1*V3sFS+0.4e1*KSaj_*x5Jdm-0.6e1
*BIVJE*(laktM-0.2e1*EPTqx*x5Jdm+0.2e1*QCcTr*(CncTH+S7nLL-Au08l))-0.12e2*QCcTr*(
0.2e1*x5Jdm*QCcTr*(zFb7J-UXXxS+olNu8*S7nLL)+QCcTr*suxJ2+ttbPQ*xJInj)+0.6e1*laktM
*iUVx1+0.4e1*JWb5x*(0.2e1*KSaj_-0.3e1*laktM*x5Jdm-0.3e1*EPTqx*iUVx1+0.3e1*QCcTr*
(ttbPQ*olNu8-0.1e1*x5Jdm*TwXRm*QCcTr+0.2e1*QCcTr*(zFb7J-UXXxS+olNu8*usHjQ))))/
rDoKh/G7uFI);}
