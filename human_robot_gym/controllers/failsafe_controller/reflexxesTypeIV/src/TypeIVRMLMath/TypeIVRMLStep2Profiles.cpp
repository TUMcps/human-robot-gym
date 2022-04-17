



































#include <TypeIVRMLStep2Profiles.h>
#include <TypeIVRMLPolynomial.h>
#include <TypeIVRMLStep2RootFunctions.h>
#include <TypeIVRMLMath.h>
#include <TypeIVRMLDefinitions.h>
#include <TypeIVRMLABK.h>
#include <math.h>


void qPN_6::vMaVh(const double&rTEdP,const double&SynchronizationTime,const 
double&iPzPj,const double&ldeCA,const double&AQzqu,const double&HcINC,const 
double&k6sf4,const double&DAwhk,const double&b4jXr,XkwFr*_2E29,const bool&U7RiH,
bool*DBRdu){oivu3(rTEdP,SynchronizationTime,-iPzPj,-ldeCA,-AQzqu,-HcINC,-k6sf4,
DAwhk,b4jXr,_2E29,!U7RiH,DBRdu);return;}

void qPN_6::ekDcv(const double&rTEdP,const double&SynchronizationTime,const 
double&iPzPj,const double&ldeCA,const double&AQzqu,const double&HcINC,const 
double&k6sf4,const double&DAwhk,const double&b4jXr,XkwFr*_2E29,const bool&U7RiH,
bool*DBRdu){XbOe0(rTEdP,SynchronizationTime,-iPzPj,-ldeCA,-AQzqu,-HcINC,-k6sf4,
DAwhk,b4jXr,_2E29,!U7RiH,DBRdu);return;}

void qPN_6::NVhNa(const double&rTEdP,const double&SynchronizationTime,const 
double&iPzPj,const double&ldeCA,const double&AQzqu,const double&HcINC,const 
double&k6sf4,const double&DAwhk,const double&b4jXr,XkwFr*_2E29,const bool&U7RiH,
bool*DBRdu){double ifRIn=0.0,TfLg_=iPzPj,EgDTv=ldeCA,JRB5m=AQzqu,EI730=HcINC,
jEwtH=k6sf4,Z4lSz=rTEdP,JWb5x=0.0,Kaclx=0.0,EvE6x=0.0,FvoS1=0.0,LOUhW=0.0,Qogm3=
0.0,xek11=0.0,ZaFFa=0.0,_HBf3=0.0,T5yjB=0.0,OLj8b=0.0,Ixvmb=0.0,zlPa_=0.0,g7nwu=
0.0,mpXRu=0.0,xHUUb=0.0,hxN2g=0.0,jE9zt=0.0,fjgkf=0.0,jfI7z=0.0,gvv1d=0.0,AF2wW=
0.0,Yoeil=0.0,XLg6q=0.0,inQml=0.0,D0ol4=0.0;

if(jEwtH>DAwhk){jEwtH=DAwhk;}Yoeil=SynchronizationTime-Z4lSz;ifRIn=Q8oIQ(&tmMnv,
&viC08,TfLg_,EgDTv,JRB5m,EI730,jEwtH,DAwhk,b4jXr,Yoeil);


JWb5x=ifRIn;if(JWb5x<0.0){JWb5x=0.0;}if(JWb5x>jEwtH){JWb5x=jEwtH;}
Kaclx=(jEwtH-JWb5x)/b4jXr;xek11=jEwtH*Kaclx-0.5*b4jXr*VNvlY(Kaclx);Ixvmb=JRB5m*
Kaclx+0.5*jEwtH*VNvlY(Kaclx)-b4jXr*qn3x8(Kaclx)/6.0;if(fabs(JWb5x-DAwhk)<MFWCZ){
EvE6x=0.0;}else{
EvE6x=(2.0*VNvlY(JWb5x)-VNvlY(jEwtH)-4.0*JWb5x*DAwhk+2.0*jEwtH*DAwhk+2.0*VNvlY(
DAwhk)-2.0*DAwhk*Yoeil*b4jXr+2.0*b4jXr*(EI730-JRB5m))/(2.0*b4jXr*(JWb5x-DAwhk));
}if(EvE6x>(Yoeil-(jEwtH-2.0*JWb5x+2.0*DAwhk)/b4jXr)){EvE6x=Yoeil-(jEwtH-2.0*
JWb5x+2.0*DAwhk)/b4jXr;}if(EvE6x<0.0){EvE6x=0.0;}ZaFFa=EvE6x*JWb5x;zlPa_=(JRB5m+
xek11)*EvE6x+0.5*JWb5x*VNvlY(EvE6x);
FvoS1=(DAwhk-JWb5x)/b4jXr;_HBf3=JWb5x*FvoS1+0.5*b4jXr*VNvlY(FvoS1);g7nwu=(JRB5m+
xek11+ZaFFa)*FvoS1+0.5*JWb5x*VNvlY(FvoS1)+b4jXr*qn3x8(FvoS1)/6.0;
Qogm3=DAwhk/b4jXr;OLj8b=0.5*VNvlY(DAwhk)/b4jXr;

T5yjB=EI730-JRB5m-xek11-ZaFFa-_HBf3-OLj8b;if(T5yjB<0.0){T5yjB=0.0;}LOUhW=T5yjB/
DAwhk;if(LOUhW<0.0){LOUhW=0.0;}mpXRu=(JRB5m+xek11+ZaFFa+_HBf3)*LOUhW+0.5*DAwhk*
VNvlY(LOUhW);
xHUUb=(JRB5m+xek11+ZaFFa+_HBf3+T5yjB)*Qogm3+0.5*DAwhk*VNvlY(Qogm3)-b4jXr*qn3x8(
Qogm3)/6.0;


if(m85bi(JRB5m)==m85bi(JRB5m+xek11)){XLg6q+=fabs(Ixvmb);}else{D0ol4=jEwtH/b4jXr-
pbQOc(VNvlY(jEwtH/b4jXr)+2.0*JRB5m/b4jXr);if(D0ol4<0.0){D0ol4=0.0;}if(D0ol4>
Kaclx){D0ol4=Kaclx;}XLg6q+=fabs(JRB5m*D0ol4+0.5*jEwtH*VNvlY(D0ol4)-b4jXr*qn3x8(
D0ol4)/6.0)+fabs(0.5*(jEwtH-b4jXr*D0ol4)*VNvlY(Kaclx-D0ol4)-b4jXr*qn3x8(Kaclx-
D0ol4)/6.0);}
if(m85bi(JRB5m+xek11)==m85bi(JRB5m+xek11+ZaFFa)){XLg6q+=fabs(zlPa_);}else{D0ol4=
fabs((JRB5m+xek11)/JWb5x);XLg6q+=fabs(0.5*JWb5x*VNvlY(D0ol4))+fabs(0.5*JWb5x*
VNvlY(EvE6x-D0ol4));}
if(m85bi(JRB5m+xek11+ZaFFa)==m85bi(JRB5m+xek11+ZaFFa+_HBf3)){XLg6q+=fabs(g7nwu);
}else{D0ol4=-JWb5x/b4jXr+pbQOc(VNvlY(JWb5x/b4jXr)-2.0*(JRB5m+xek11+ZaFFa)/b4jXr)
;if(D0ol4<0.0){D0ol4=0.0;}if(D0ol4>FvoS1){D0ol4=FvoS1;}XLg6q+=fabs((JRB5m+xek11+
ZaFFa)*D0ol4+0.5*JWb5x*VNvlY(D0ol4)+b4jXr*qn3x8(D0ol4)/6.0)+fabs(0.5*(JWb5x+
b4jXr*D0ol4)*VNvlY(FvoS1-D0ol4)+b4jXr*qn3x8(FvoS1-D0ol4)/6.0);}
if(m85bi(JRB5m+xek11+ZaFFa+_HBf3)==m85bi(JRB5m+xek11+ZaFFa+_HBf3+T5yjB)){XLg6q+=
fabs(mpXRu);}else{D0ol4=fabs((JRB5m+xek11+ZaFFa+_HBf3)/DAwhk);XLg6q+=fabs(0.5*
DAwhk*VNvlY(D0ol4))+fabs(0.5*DAwhk*VNvlY(LOUhW-D0ol4));}
if(m85bi(JRB5m+xek11+ZaFFa+_HBf3+T5yjB)==m85bi(EI730)){XLg6q+=fabs(xHUUb);}else{
D0ol4=DAwhk/b4jXr-pbQOc(VNvlY(DAwhk/b4jXr)+2.0*(JRB5m+xek11+ZaFFa+_HBf3+T5yjB)/
b4jXr);if(D0ol4<0.0){D0ol4=0.0;}if(D0ol4>Qogm3){D0ol4=Qogm3;}XLg6q+=fabs((JRB5m+
xek11+ZaFFa+_HBf3+T5yjB)*D0ol4+0.5*DAwhk*VNvlY(D0ol4)-b4jXr*qn3x8(D0ol4)/6.0)+
fabs(0.5*(DAwhk-b4jXr*D0ol4)*VNvlY(Qogm3-D0ol4)-b4jXr*qn3x8(Qogm3-D0ol4)/6.0);}
inQml=fabs(xek11)+fabs(ZaFFa)+fabs(_HBf3)+fabs(T5yjB)+fabs(OLj8b);


AF2wW=Kaclx+EvE6x+FvoS1+LOUhW+Qogm3;jfI7z=Ixvmb+zlPa_+g7nwu+mpXRu+xHUUb;gvv1d=
xek11+ZaFFa+_HBf3+T5yjB+OLj8b;if(AF2wW!=0.0){fjgkf=SynchronizationTime-Z4lSz-
AF2wW;if(fabs(fjgkf)>(kED7o*(SynchronizationTime-Z4lSz)+PZBAL)){*DBRdu=true;}}if
(jfI7z!=0.0){hxN2g=EgDTv-TfLg_-jfI7z;if(fabs(hxN2g)>(fabs(Cmmsw*XLg6q)+oUhjD)){*
DBRdu=true;}}if(gvv1d!=0.0){jE9zt=EI730-JRB5m-gvv1d;if(fabs(jE9zt)>(c_4Sr*fabs((
fabs(EI730)>fabs(inQml))?(EI730):(inQml)))+Uq_ZW){*DBRdu=true;}}



if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ((b4jXr/6.0),(0.5*(-jEwtH)),(-JRB5m),(
-TfLg_),(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(b4jXr*0.5),(-jEwtH),(-
JRB5m),(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,b4jXr,(-jEwtH),(Z4lSz))
;}else{_2E29->fCxBi[_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),(0.5*jEwtH),JRB5m,TfLg_,(
Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,((-b4jXr)*0.5),jEwtH,JRB5m,(Z4lSz))
;_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,-b4jXr,jEwtH,(Z4lSz));}_2E29->FfhZi[
_2E29->djbCD]=(Z4lSz)+Kaclx;_2E29->djbCD++;
Z4lSz+=(Kaclx);TfLg_+=Ixvmb;JRB5m+=xek11;jEwtH=JWb5x;

if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0,(0.5*(-JWb5x)),(-JRB5m),(-TfLg_),
(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,-JWb5x,(-JRB5m),(Z4lSz));_2E29
->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,-JWb5x,(Z4lSz));}else{_2E29->fCxBi[_2E29
->djbCD].jGQqQ(0.0,(0.5*JWb5x),JRB5m,TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].
jGQqQ(0.0,0.0,JWb5x,JRB5m,(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,
JWb5x,(Z4lSz));}_2E29->FfhZi[_2E29->djbCD]=(Z4lSz)+EvE6x;_2E29->djbCD++;
Z4lSz+=(EvE6x);TfLg_+=zlPa_;JRB5m+=ZaFFa;jEwtH=JWb5x;

if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),(0.5*(-JWb5x)),(-JRB5m
),(-TfLg_),(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*(-b4jXr)),-JWb5x,(
-JRB5m),(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,-b4jXr,-JWb5x,(Z4lSz))
;}else{_2E29->fCxBi[_2E29->djbCD].jGQqQ((b4jXr/6.0),(0.5*JWb5x),JRB5m,TfLg_,(
Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*b4jXr),JWb5x,JRB5m,(Z4lSz));
_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,b4jXr,JWb5x,(Z4lSz));}_2E29->FfhZi[
_2E29->djbCD]=(Z4lSz)+FvoS1;_2E29->djbCD++;
Z4lSz+=(FvoS1);TfLg_+=g7nwu;JRB5m+=_HBf3;jEwtH=DAwhk;

if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0,(0.5*(-DAwhk)),-JRB5m,-TfLg_,(
Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,-DAwhk,-JRB5m,(Z4lSz));_2E29->
NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,-DAwhk,(Z4lSz));}else{_2E29->fCxBi[_2E29->
djbCD].jGQqQ(0.0,(0.5*DAwhk),JRB5m,TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].
jGQqQ(0.0,0.0,DAwhk,JRB5m,(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,
DAwhk,(Z4lSz));}_2E29->FfhZi[_2E29->djbCD]=(Z4lSz)+LOUhW;_2E29->djbCD++;
Z4lSz+=(LOUhW);TfLg_+=mpXRu;JRB5m+=T5yjB;jEwtH=DAwhk;

if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ((b4jXr/6.0),(0.5*(-DAwhk)),-JRB5m,-
TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*b4jXr),-DAwhk,-JRB5m,(
Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,b4jXr,-DAwhk,(Z4lSz));}else{
_2E29->fCxBi[_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),(0.5*DAwhk),JRB5m,TfLg_,(Z4lSz))
;_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*(-b4jXr)),DAwhk,JRB5m,(Z4lSz));_2E29
->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,-b4jXr,DAwhk,(Z4lSz));}_2E29->FfhZi[_2E29->
djbCD]=(Z4lSz)+Qogm3;_2E29->djbCD++;
Z4lSz+=(Qogm3);TfLg_+=xHUUb;JRB5m+=OLj8b;jEwtH=0.0;
if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0,0.0,-JRB5m,-TfLg_,(Z4lSz));_2E29
->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,-JRB5m,(Z4lSz));_2E29->NYWGt[_2E29->
djbCD].jGQqQ(0.0,0.0,0.0,0.0,(Z4lSz));}else{_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0
,0.0,JRB5m,TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,JRB5m,(
Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,0.0,(Z4lSz));}_2E29->FfhZi[
_2E29->djbCD]=(Z4lSz)+tlPiC;_2E29->djbCD++;return;}

void qPN_6::FAtnb(const double&rTEdP,const double&SynchronizationTime,const 
double&iPzPj,const double&ldeCA,const double&AQzqu,const double&HcINC,const 
double&k6sf4,const double&DAwhk,const double&b4jXr,XkwFr*_2E29,const bool&U7RiH,
bool*DBRdu){double ifRIn=0.0,TfLg_=iPzPj,EgDTv=ldeCA,JRB5m=AQzqu,EI730=HcINC,
jEwtH=k6sf4,Z4lSz=rTEdP,JWb5x=0.0,RIZdZ=0.0,Kaclx=0.0,EvE6x=0.0,FvoS1=0.0,LOUhW=
0.0,xek11=0.0,ZaFFa=0.0,_HBf3=0.0,T5yjB=0.0,Ixvmb=0.0,zlPa_=0.0,g7nwu=0.0,mpXRu=
0.0,hxN2g=0.0,jE9zt=0.0,fjgkf=0.0,jfI7z=0.0,gvv1d=0.0,AF2wW=0.0,Yoeil=0.0,XLg6q=
0.0,inQml=0.0,D0ol4=0.0;

if(jEwtH>DAwhk){jEwtH=DAwhk;}Yoeil=SynchronizationTime-Z4lSz;ifRIn=Q8oIQ(&UREKg,
&IUU3u,TfLg_,EgDTv,JRB5m,EI730,jEwtH,DAwhk,b4jXr,Yoeil);


JWb5x=ifRIn;if(JWb5x<0.0){JWb5x=0.0;}if(JWb5x>jEwtH){JWb5x=jEwtH;}
Kaclx=(jEwtH-JWb5x)/b4jXr;xek11=jEwtH*Kaclx-0.5*b4jXr*VNvlY(Kaclx);Ixvmb=JRB5m*
Kaclx+0.5*jEwtH*VNvlY(Kaclx)-b4jXr*qn3x8(Kaclx)/6.0;
RIZdZ=JWb5x+pbQOc((double)(0.5*(2.0*JWb5x*jEwtH-VNvlY(jEwtH)-2.0*JWb5x*Yoeil*
b4jXr+2.0*b4jXr*(EI730-JRB5m))));if(RIZdZ<JWb5x){RIZdZ=JWb5x;}
EvE6x=2.0*(-b4jXr*(0.5*jEwtH-0.5*Yoeil*b4jXr)-b4jXr*pbQOc(JWb5x*jEwtH-0.5*VNvlY(
jEwtH)-JWb5x*Yoeil*b4jXr-b4jXr*JRB5m+b4jXr*EI730))/VNvlY(b4jXr);if(EvE6x<0.0){
EvE6x=0.0;}ZaFFa=EvE6x*JWb5x;zlPa_=(JRB5m+xek11)*EvE6x+0.5*JWb5x*VNvlY(EvE6x);
FvoS1=(RIZdZ-JWb5x)/b4jXr;_HBf3=JWb5x*FvoS1+0.5*b4jXr*VNvlY(FvoS1);g7nwu=(JRB5m+
xek11+ZaFFa)*FvoS1+0.5*JWb5x*VNvlY(FvoS1)+b4jXr*qn3x8(FvoS1)/6.0;
LOUhW=RIZdZ/b4jXr;T5yjB=0.5*VNvlY(RIZdZ)/b4jXr;mpXRu=(JRB5m+xek11+ZaFFa+_HBf3)*
LOUhW+0.5*RIZdZ*VNvlY(LOUhW)-b4jXr*qn3x8(LOUhW)/6.0;


if(m85bi(JRB5m)==m85bi(JRB5m+xek11)){XLg6q+=fabs(Ixvmb);}else{D0ol4=jEwtH/b4jXr-
pbQOc(VNvlY(jEwtH/b4jXr)+2.0*JRB5m/b4jXr);if(D0ol4<0.0){D0ol4=0.0;}if(D0ol4>
Kaclx){D0ol4=Kaclx;}XLg6q+=fabs(JRB5m*D0ol4+0.5*jEwtH*VNvlY(D0ol4)-b4jXr*qn3x8(
D0ol4)/6.0)+fabs(0.5*(jEwtH-b4jXr*D0ol4)*VNvlY(Kaclx-D0ol4)-b4jXr*qn3x8(Kaclx-
D0ol4)/6.0);}
if(m85bi(JRB5m+xek11)==m85bi(JRB5m+xek11+ZaFFa)){XLg6q+=fabs(zlPa_);}else{D0ol4=
fabs((JRB5m+xek11)/JWb5x);XLg6q+=fabs(0.5*JWb5x*VNvlY(D0ol4))+fabs(0.5*JWb5x*
VNvlY(EvE6x-D0ol4));}
if(m85bi(JRB5m+xek11+ZaFFa)==m85bi(JRB5m+xek11+ZaFFa+_HBf3)){XLg6q+=fabs(g7nwu);
}else{D0ol4=-JWb5x/b4jXr+pbQOc(VNvlY(JWb5x/b4jXr)-2.0*(JRB5m+xek11+ZaFFa)/b4jXr)
;if(D0ol4<0.0){D0ol4=0.0;}if(D0ol4>FvoS1){D0ol4=FvoS1;}XLg6q+=fabs((JRB5m+xek11+
ZaFFa)*D0ol4+0.5*JWb5x*VNvlY(D0ol4)+b4jXr*qn3x8(D0ol4)/6.0)+fabs(0.5*(JWb5x+
b4jXr*D0ol4)*VNvlY(FvoS1-D0ol4)+b4jXr*qn3x8(FvoS1-D0ol4)/6.0);}
if(m85bi(JRB5m+xek11+ZaFFa+_HBf3)==m85bi(EI730)){XLg6q+=fabs(mpXRu);}else{D0ol4=
RIZdZ/b4jXr-pbQOc(VNvlY(RIZdZ/b4jXr)+2.0*(JRB5m+xek11+ZaFFa+_HBf3)/b4jXr);if(
D0ol4<0.0){D0ol4=0.0;}if(D0ol4>LOUhW){D0ol4=LOUhW;}XLg6q+=fabs((JRB5m+xek11+
ZaFFa+_HBf3)*D0ol4+0.5*RIZdZ*VNvlY(D0ol4)-b4jXr*qn3x8(D0ol4)/6.0)+fabs(0.5*(
RIZdZ-b4jXr*D0ol4)*VNvlY(LOUhW-D0ol4)-b4jXr*qn3x8(LOUhW-D0ol4)/6.0);}
inQml=fabs(xek11)+fabs(ZaFFa)+fabs(_HBf3)+fabs(T5yjB);


AF2wW=Kaclx+EvE6x+FvoS1+LOUhW;jfI7z=Ixvmb+zlPa_+g7nwu+mpXRu;gvv1d=xek11+ZaFFa+
_HBf3+T5yjB;if(AF2wW!=0.0){fjgkf=SynchronizationTime-Z4lSz-AF2wW;if(fabs(fjgkf)>
(kED7o*(SynchronizationTime-Z4lSz)+PZBAL)){*DBRdu=true;}}if(jfI7z!=0.0){hxN2g=
EgDTv-TfLg_-jfI7z;if(fabs(hxN2g)>(fabs(Cmmsw*XLg6q)+oUhjD)){*DBRdu=true;}}if(
gvv1d!=0.0){jE9zt=EI730-JRB5m-gvv1d;if(fabs(jE9zt)>(c_4Sr*fabs((fabs(EI730)>fabs
(inQml))?(EI730):(inQml)))+Uq_ZW){*DBRdu=true;}}



if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ((b4jXr/6.0),(0.5*(-jEwtH)),(-JRB5m),(
-TfLg_),(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(b4jXr*0.5),(-jEwtH),(-
JRB5m),(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,b4jXr,(-jEwtH),(Z4lSz))
;}else{_2E29->fCxBi[_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),(0.5*jEwtH),JRB5m,TfLg_,(
Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,((-b4jXr)*0.5),jEwtH,JRB5m,(Z4lSz))
;_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,-b4jXr,jEwtH,(Z4lSz));}_2E29->FfhZi[
_2E29->djbCD]=(Z4lSz)+Kaclx;_2E29->djbCD++;
Z4lSz+=(Kaclx);TfLg_+=Ixvmb;JRB5m+=xek11;jEwtH=JWb5x;

if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0,(0.5*(-JWb5x)),(-JRB5m),(-TfLg_),
(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,-JWb5x,(-JRB5m),(Z4lSz));_2E29
->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,-JWb5x,(Z4lSz));}else{_2E29->fCxBi[_2E29
->djbCD].jGQqQ(0.0,(0.5*JWb5x),JRB5m,TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].
jGQqQ(0.0,0.0,JWb5x,JRB5m,(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,
JWb5x,(Z4lSz));}_2E29->FfhZi[_2E29->djbCD]=(Z4lSz)+EvE6x;_2E29->djbCD++;
Z4lSz+=(EvE6x);TfLg_+=zlPa_;JRB5m+=ZaFFa;jEwtH=JWb5x;

if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),(0.5*(-JWb5x)),(-JRB5m
),(-TfLg_),(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*(-b4jXr)),-JWb5x,(
-JRB5m),(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,-b4jXr,-JWb5x,(Z4lSz))
;}else{_2E29->fCxBi[_2E29->djbCD].jGQqQ((b4jXr/6.0),(0.5*JWb5x),JRB5m,TfLg_,(
Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*b4jXr),JWb5x,JRB5m,(Z4lSz));
_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,b4jXr,JWb5x,(Z4lSz));}_2E29->FfhZi[
_2E29->djbCD]=(Z4lSz)+FvoS1;_2E29->djbCD++;
Z4lSz+=(FvoS1);TfLg_+=g7nwu;JRB5m+=_HBf3;jEwtH=RIZdZ;

if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ((b4jXr/6.0),(0.5*(-RIZdZ)),-JRB5m,-
TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*b4jXr),-RIZdZ,-JRB5m,(
Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,b4jXr,-RIZdZ,(Z4lSz));}else{
_2E29->fCxBi[_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),(0.5*RIZdZ),JRB5m,TfLg_,(Z4lSz))
;_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*(-b4jXr)),RIZdZ,JRB5m,(Z4lSz));_2E29
->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,-b4jXr,RIZdZ,(Z4lSz));}_2E29->FfhZi[_2E29->
djbCD]=(Z4lSz)+LOUhW;_2E29->djbCD++;
Z4lSz+=(LOUhW);TfLg_+=mpXRu;JRB5m+=T5yjB;jEwtH=0.0;
if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0,0.0,-JRB5m,-TfLg_,(Z4lSz));_2E29
->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,-JRB5m,(Z4lSz));_2E29->NYWGt[_2E29->
djbCD].jGQqQ(0.0,0.0,0.0,0.0,(Z4lSz));}else{_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0
,0.0,JRB5m,TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,JRB5m,(
Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,0.0,(Z4lSz));}_2E29->FfhZi[
_2E29->djbCD]=(Z4lSz)+tlPiC;_2E29->djbCD++;return;}

void qPN_6::MpPJf(const double&rTEdP,const double&SynchronizationTime,const 
double&iPzPj,const double&ldeCA,const double&AQzqu,const double&HcINC,const 
double&k6sf4,const double&DAwhk,const double&b4jXr,XkwFr*_2E29,const bool&U7RiH,
bool*DBRdu){xZ4vy(rTEdP,SynchronizationTime,-iPzPj,-ldeCA,-AQzqu,-HcINC,-k6sf4,
DAwhk,b4jXr,_2E29,!U7RiH,DBRdu);return;}

void qPN_6::ScS5d(const double&rTEdP,const double&SynchronizationTime,const 
double&iPzPj,const double&ldeCA,const double&AQzqu,const double&HcINC,const 
double&k6sf4,const double&DAwhk,const double&b4jXr,XkwFr*_2E29,const bool&U7RiH,
bool*DBRdu){XYtRl(rTEdP,SynchronizationTime,-iPzPj,-ldeCA,-AQzqu,-HcINC,-k6sf4,
DAwhk,b4jXr,_2E29,!U7RiH,DBRdu);return;}

void qPN_6::efaQ4(const double&rTEdP,const double&SynchronizationTime,const 
double&iPzPj,const double&ldeCA,const double&AQzqu,const double&HcINC,const 
double&k6sf4,const double&DAwhk,const double&b4jXr,XkwFr*_2E29,const bool&U7RiH,
bool*DBRdu){UCfFY(rTEdP,SynchronizationTime,-iPzPj,-ldeCA,-AQzqu,-HcINC,-k6sf4,
DAwhk,b4jXr,_2E29,!U7RiH,DBRdu);return;}

void qPN_6::OJDhQ(const double&rTEdP,const double&SynchronizationTime,const 
double&iPzPj,const double&ldeCA,const double&AQzqu,const double&HcINC,const 
double&k6sf4,const double&DAwhk,const double&b4jXr,XkwFr*_2E29,const bool&U7RiH,
bool*DBRdu){oLIt_(rTEdP,SynchronizationTime,-iPzPj,-ldeCA,-AQzqu,-HcINC,-k6sf4,
DAwhk,b4jXr,_2E29,!U7RiH,DBRdu);return;}

void qPN_6::Xw0BW(const double&rTEdP,const double&SynchronizationTime,const 
double&iPzPj,const double&ldeCA,const double&AQzqu,const double&HcINC,const 
double&k6sf4,const double&DAwhk,const double&b4jXr,XkwFr*_2E29,const bool&U7RiH,
bool*DBRdu){serOm(rTEdP,SynchronizationTime,-iPzPj,-ldeCA,-AQzqu,-HcINC,-k6sf4,
DAwhk,b4jXr,_2E29,!U7RiH,DBRdu);return;}

void qPN_6::itUTt(const double&rTEdP,const double&SynchronizationTime,const 
double&iPzPj,const double&ldeCA,const double&AQzqu,const double&HcINC,const 
double&k6sf4,const double&DAwhk,const double&b4jXr,XkwFr*_2E29,const bool&U7RiH,
bool*DBRdu){wFe1S(rTEdP,SynchronizationTime,-iPzPj,-ldeCA,-AQzqu,-HcINC,-k6sf4,
DAwhk,b4jXr,_2E29,!U7RiH,DBRdu);return;}

void qPN_6::sKplj(const double&rTEdP,const double&SynchronizationTime,const 
double&iPzPj,const double&ldeCA,const double&AQzqu,const double&HcINC,const 
double&k6sf4,const double&DAwhk,const double&b4jXr,XkwFr*_2E29,const bool&U7RiH,
bool*DBRdu){RWVKm(rTEdP,SynchronizationTime,-iPzPj,-ldeCA,-AQzqu,-HcINC,-k6sf4,
DAwhk,b4jXr,_2E29,!U7RiH,DBRdu);return;}

void qPN_6::aN2Eq(const double&rTEdP,const double&SynchronizationTime,const 
double&iPzPj,const double&ldeCA,const double&AQzqu,const double&HcINC,const 
double&k6sf4,const double&DAwhk,const double&b4jXr,XkwFr*_2E29,const bool&U7RiH,
bool*DBRdu){PJn2j(rTEdP,SynchronizationTime,-iPzPj,-ldeCA,-AQzqu,-HcINC,-k6sf4,
DAwhk,b4jXr,_2E29,!U7RiH,DBRdu);return;}

void qPN_6::hMy1N(const double&rTEdP,const double&SynchronizationTime,const 
double&iPzPj,const double&ldeCA,const double&AQzqu,const double&HcINC,const 
double&k6sf4,const double&DAwhk,const double&b4jXr,XkwFr*_2E29,const bool&U7RiH,
bool*DBRdu){ww9Hd(rTEdP,SynchronizationTime,-iPzPj,-ldeCA,-AQzqu,-HcINC,-k6sf4,
DAwhk,b4jXr,_2E29,!U7RiH,DBRdu);return;}

void qPN_6::ahzDs(const double&rTEdP,const double&SynchronizationTime,const 
double&iPzPj,const double&ldeCA,const double&AQzqu,const double&HcINC,const 
double&k6sf4,const double&DAwhk,const double&b4jXr,XkwFr*_2E29,const bool&U7RiH,
bool*DBRdu){gOgPf(rTEdP,SynchronizationTime,-iPzPj,-ldeCA,-AQzqu,-HcINC,-k6sf4,
DAwhk,b4jXr,_2E29,!U7RiH,DBRdu);return;}

void qPN_6::oivu3(const double&rTEdP,const double&SynchronizationTime,const 
double&iPzPj,const double&ldeCA,const double&AQzqu,const double&HcINC,const 
double&k6sf4,const double&DAwhk,const double&b4jXr,XkwFr*_2E29,const bool&U7RiH,
bool*DBRdu){double ifRIn=0.0,TfLg_=iPzPj,EgDTv=ldeCA,JRB5m=AQzqu,EI730=HcINC,
jEwtH=k6sf4,Z4lSz=rTEdP,JWb5x=0.0,Kaclx=0.0,EvE6x=0.0,FvoS1=0.0,LOUhW=0.0,Qogm3=
0.0,xek11=0.0,ZaFFa=0.0,_HBf3=0.0,T5yjB=0.0,OLj8b=0.0,Ixvmb=0.0,zlPa_=0.0,g7nwu=
0.0,mpXRu=0.0,xHUUb=0.0,hxN2g=0.0,jE9zt=0.0,fjgkf=0.0,jfI7z=0.0,gvv1d=0.0,AF2wW=
0.0,Yoeil=0.0,rDoKh=0.0,XLg6q=0.0,inQml=0.0,D0ol4=0.0;

if(jEwtH>DAwhk){jEwtH=DAwhk;}Yoeil=SynchronizationTime-Z4lSz;ifRIn=Q8oIQ(&d_rMF,
&MvKFA,TfLg_,EgDTv,JRB5m,EI730,jEwtH,DAwhk,b4jXr,Yoeil);


JWb5x=ifRIn;if(JWb5x<jEwtH){JWb5x=jEwtH;}if(JWb5x>DAwhk){JWb5x=DAwhk;}
Kaclx=(JWb5x-jEwtH)/b4jXr;xek11=jEwtH*Kaclx+0.5*b4jXr*VNvlY(Kaclx);Ixvmb=JRB5m*
Kaclx+0.5*jEwtH*VNvlY(Kaclx)+b4jXr*qn3x8(Kaclx)/6.0;if(fabs(DAwhk-JWb5x)<MFWCZ){
EvE6x=0.0;}else{rDoKh=(2.0*JWb5x-2.0*DAwhk);
EvE6x=jEwtH/b4jXr-(2.0*JWb5x*jEwtH)/(rDoKh*b4jXr)+(1.0*VNvlY(jEwtH))/(rDoKh*
b4jXr)-(2.0*DAwhk)/b4jXr+(4.0*JWb5x*DAwhk)/(rDoKh*b4jXr)-(2.0*VNvlY(DAwhk))/(
rDoKh*b4jXr)-1.0*(Z4lSz)+(2.0*JWb5x*(Z4lSz))/rDoKh+SynchronizationTime-(2.0*
JWb5x*SynchronizationTime)/rDoKh-(2.0*JRB5m)/rDoKh+(2.0*EI730)/rDoKh;}if(EvE6x<
0.0){EvE6x=0.0;}if(EvE6x>(Yoeil-(2.0*DAwhk-jEwtH)/b4jXr)){EvE6x=Yoeil-(2.0*DAwhk
-jEwtH)/b4jXr;}ZaFFa=EvE6x*JWb5x;zlPa_=(JRB5m+xek11)*EvE6x+0.5*JWb5x*VNvlY(EvE6x
);
FvoS1=(DAwhk-JWb5x)/b4jXr;_HBf3=JWb5x*FvoS1+0.5*b4jXr*VNvlY(FvoS1);g7nwu=(JRB5m+
xek11+ZaFFa)*FvoS1+0.5*JWb5x*VNvlY(FvoS1)+b4jXr*qn3x8(FvoS1)/6.0;
Qogm3=DAwhk/b4jXr;OLj8b=0.5*VNvlY(DAwhk)/b4jXr;

T5yjB=EI730-JRB5m-xek11-ZaFFa-_HBf3-OLj8b;if(T5yjB<0.0){T5yjB=0.0;}LOUhW=T5yjB/
DAwhk;if(LOUhW<0.0){LOUhW=0.0;}mpXRu=(JRB5m+xek11+ZaFFa+_HBf3)*LOUhW+0.5*DAwhk*
VNvlY(LOUhW);
xHUUb=(JRB5m+xek11+ZaFFa+_HBf3+T5yjB)*Qogm3+0.5*DAwhk*VNvlY(Qogm3)-b4jXr*qn3x8(
Qogm3)/6.0;


if(m85bi(JRB5m)==m85bi(JRB5m+xek11)){XLg6q+=fabs(Ixvmb);}else{D0ol4=-jEwtH/b4jXr
+pbQOc(VNvlY(jEwtH/b4jXr)-2.0*JRB5m/b4jXr);if(D0ol4<0.0){D0ol4=0.0;}if(D0ol4>
Kaclx){D0ol4=Kaclx;}XLg6q+=fabs(JRB5m*D0ol4+0.5*jEwtH*VNvlY(D0ol4)+b4jXr*qn3x8(
D0ol4)/6.0)+fabs(0.5*(jEwtH-b4jXr*D0ol4)*VNvlY(Kaclx-D0ol4)+b4jXr*qn3x8(Kaclx-
D0ol4)/6.0);}
if(m85bi(JRB5m+xek11)==m85bi(JRB5m+xek11+ZaFFa)){XLg6q+=fabs(zlPa_);}else{D0ol4=
fabs((JRB5m+xek11)/JWb5x);XLg6q+=fabs(0.5*JWb5x*VNvlY(D0ol4))+fabs(0.5*JWb5x*
VNvlY(EvE6x-D0ol4));}
if(m85bi(JRB5m+xek11+ZaFFa)==m85bi(JRB5m+xek11+ZaFFa+_HBf3)){XLg6q+=fabs(g7nwu);
}else{D0ol4=-JWb5x/b4jXr+pbQOc(VNvlY(JWb5x/b4jXr)-2.0*(JRB5m+xek11+ZaFFa)/b4jXr)
;if(D0ol4<0.0){D0ol4=0.0;}if(D0ol4>FvoS1){D0ol4=FvoS1;}XLg6q+=fabs((JRB5m+xek11+
ZaFFa)*D0ol4+0.5*JWb5x*VNvlY(D0ol4)+b4jXr*qn3x8(D0ol4)/6.0)+fabs(0.5*(JWb5x+
b4jXr*D0ol4)*VNvlY(FvoS1-D0ol4)+b4jXr*qn3x8(FvoS1-D0ol4)/6.0);}
if(m85bi(JRB5m+xek11+ZaFFa+_HBf3)==m85bi(JRB5m+xek11+ZaFFa+_HBf3+T5yjB)){XLg6q+=
fabs(mpXRu);}else{D0ol4=fabs((JRB5m+xek11+ZaFFa+_HBf3)/DAwhk);XLg6q+=fabs(0.5*
DAwhk*VNvlY(D0ol4))+fabs(0.5*DAwhk*VNvlY(LOUhW-D0ol4));}
if(m85bi(JRB5m+xek11+ZaFFa+_HBf3+T5yjB)==m85bi(EI730)){XLg6q+=fabs(xHUUb);}else{
D0ol4=DAwhk/b4jXr-pbQOc(VNvlY(DAwhk/b4jXr)+2.0*(JRB5m+xek11+ZaFFa+_HBf3+T5yjB)/
b4jXr);if(D0ol4<0.0){D0ol4=0.0;}if(D0ol4>Qogm3){D0ol4=Qogm3;}XLg6q+=fabs((JRB5m+
xek11+ZaFFa+_HBf3+T5yjB)*D0ol4+0.5*DAwhk*VNvlY(D0ol4)-b4jXr*qn3x8(D0ol4)/6.0)+
fabs(0.5*(DAwhk-b4jXr*D0ol4)*VNvlY(Qogm3-D0ol4)-b4jXr*qn3x8(Qogm3-D0ol4)/6.0);}
inQml=fabs(xek11)+fabs(ZaFFa)+fabs(_HBf3)+fabs(T5yjB)+fabs(OLj8b);


AF2wW=Kaclx+EvE6x+FvoS1+LOUhW+Qogm3;jfI7z=Ixvmb+zlPa_+g7nwu+mpXRu+xHUUb;gvv1d=
xek11+ZaFFa+_HBf3+T5yjB+OLj8b;if(AF2wW!=0.0){fjgkf=SynchronizationTime-Z4lSz-
AF2wW;if(fabs(fjgkf)>(kED7o*(SynchronizationTime-Z4lSz)+PZBAL)){*DBRdu=true;}}if
(jfI7z!=0.0){hxN2g=EgDTv-TfLg_-jfI7z;if(fabs(hxN2g)>(fabs(Cmmsw*XLg6q)+oUhjD)){*
DBRdu=true;}}if(gvv1d!=0.0){jE9zt=EI730-JRB5m-gvv1d;if(fabs(jE9zt)>(c_4Sr*fabs((
fabs(EI730)>fabs(inQml))?(EI730):(inQml)))+Uq_ZW){*DBRdu=true;}}



if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),(0.5*(-jEwtH)),(-JRB5m
),(-TfLg_),(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,((-b4jXr)*0.5),(-jEwtH)
,(-JRB5m),(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,(-b4jXr),(-jEwtH),(
Z4lSz));}else{_2E29->fCxBi[_2E29->djbCD].jGQqQ((b4jXr/6.0),(0.5*jEwtH),JRB5m,
TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(b4jXr*0.5),jEwtH,JRB5m,(
Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,b4jXr,jEwtH,(Z4lSz));}_2E29->
FfhZi[_2E29->djbCD]=(Z4lSz)+Kaclx;_2E29->djbCD++;
Z4lSz+=(Kaclx);TfLg_+=Ixvmb;JRB5m+=xek11;jEwtH=JWb5x;

if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0,(0.5*(-JWb5x)),(-JRB5m),(-TfLg_),
(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,-JWb5x,(-JRB5m),(Z4lSz));_2E29
->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,-JWb5x,(Z4lSz));}else{_2E29->fCxBi[_2E29
->djbCD].jGQqQ(0.0,(0.5*JWb5x),JRB5m,TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].
jGQqQ(0.0,0.0,JWb5x,JRB5m,(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,
JWb5x,(Z4lSz));}_2E29->FfhZi[_2E29->djbCD]=(Z4lSz)+EvE6x;_2E29->djbCD++;
Z4lSz+=(EvE6x);TfLg_+=zlPa_;JRB5m+=ZaFFa;jEwtH=JWb5x;

if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),(0.5*(-JWb5x)),(-JRB5m
),(-TfLg_),(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*(-b4jXr)),-JWb5x,(
-JRB5m),(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,-b4jXr,-JWb5x,(Z4lSz))
;}else{_2E29->fCxBi[_2E29->djbCD].jGQqQ((b4jXr/6.0),(0.5*JWb5x),JRB5m,TfLg_,(
Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*b4jXr),JWb5x,JRB5m,(Z4lSz));
_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,b4jXr,JWb5x,(Z4lSz));}_2E29->FfhZi[
_2E29->djbCD]=(Z4lSz)+FvoS1;_2E29->djbCD++;
Z4lSz+=(FvoS1);TfLg_+=g7nwu;JRB5m+=_HBf3;jEwtH=DAwhk;

if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0,(0.5*(-DAwhk)),-JRB5m,-TfLg_,(
Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,-DAwhk,-JRB5m,(Z4lSz));_2E29->
NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,-DAwhk,(Z4lSz));}else{_2E29->fCxBi[_2E29->
djbCD].jGQqQ(0.0,(0.5*DAwhk),JRB5m,TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].
jGQqQ(0.0,0.0,DAwhk,JRB5m,(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,
DAwhk,(Z4lSz));}_2E29->FfhZi[_2E29->djbCD]=(Z4lSz)+LOUhW;_2E29->djbCD++;
Z4lSz+=(LOUhW);TfLg_+=mpXRu;JRB5m+=T5yjB;jEwtH=DAwhk;

if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ((b4jXr/6.0),(0.5*(-DAwhk)),-JRB5m,-
TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*b4jXr),-DAwhk,-JRB5m,(
Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,b4jXr,-DAwhk,(Z4lSz));}else{
_2E29->fCxBi[_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),(0.5*DAwhk),JRB5m,TfLg_,(Z4lSz))
;_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*(-b4jXr)),DAwhk,JRB5m,(Z4lSz));_2E29
->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,-b4jXr,DAwhk,(Z4lSz));}_2E29->FfhZi[_2E29->
djbCD]=(Z4lSz)+Qogm3;_2E29->djbCD++;
Z4lSz+=(Qogm3);TfLg_+=xHUUb;JRB5m+=OLj8b;jEwtH=0.0;
if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0,0.0,-JRB5m,-TfLg_,(Z4lSz));_2E29
->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,-JRB5m,(Z4lSz));_2E29->NYWGt[_2E29->
djbCD].jGQqQ(0.0,0.0,0.0,0.0,(Z4lSz));}else{_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0
,0.0,JRB5m,TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,JRB5m,(
Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,0.0,(Z4lSz));}_2E29->FfhZi[
_2E29->djbCD]=(Z4lSz)+tlPiC;_2E29->djbCD++;return;}

void qPN_6::XbOe0(const double&rTEdP,const double&SynchronizationTime,const 
double&iPzPj,const double&ldeCA,const double&AQzqu,const double&HcINC,const 
double&k6sf4,const double&DAwhk,const double&b4jXr,XkwFr*_2E29,const bool&U7RiH,
bool*DBRdu){double ifRIn=0.0,TfLg_=iPzPj,EgDTv=ldeCA,JRB5m=AQzqu,EI730=HcINC,
jEwtH=k6sf4,Z4lSz=rTEdP,JWb5x=0.0,wmBRB=0.0,Kaclx=0.0,EvE6x=0.0,FvoS1=0.0,LOUhW=
0.0,xek11=0.0,ZaFFa=0.0,_HBf3=0.0,T5yjB=0.0,Ixvmb=0.0,zlPa_=0.0,g7nwu=0.0,mpXRu=
0.0,hxN2g=0.0,jE9zt=0.0,fjgkf=0.0,jfI7z=0.0,gvv1d=0.0,AF2wW=0.0,Yoeil=0.0,XLg6q=
0.0,inQml=0.0,D0ol4=0.0;

if(jEwtH>DAwhk){jEwtH=DAwhk;}Yoeil=SynchronizationTime-Z4lSz;ifRIn=Q8oIQ(&VrX3P,
&UeTUI,TfLg_,EgDTv,JRB5m,EI730,jEwtH,DAwhk,b4jXr,Yoeil);

wmBRB=ifRIn;if(wmBRB<jEwtH){wmBRB=jEwtH;}if(fabs(jEwtH+Yoeil*b4jXr-2.0*wmBRB)<
MFWCZ){
JWb5x=jEwtH;}else{
JWb5x=(VNvlY(jEwtH)-2.0*VNvlY(wmBRB)-2.0*b4jXr*JRB5m+2.0*b4jXr*EI730)/(2.0*(
jEwtH-2.0*wmBRB+Yoeil*b4jXr));if(JWb5x<jEwtH){JWb5x=jEwtH;}}if(JWb5x>DAwhk){
JWb5x=DAwhk;}
Kaclx=(JWb5x-jEwtH)/b4jXr;xek11=jEwtH*Kaclx+0.5*b4jXr*VNvlY(Kaclx);Ixvmb=JRB5m*
Kaclx+0.5*jEwtH*VNvlY(Kaclx)+b4jXr*qn3x8(Kaclx)/6.0;
EvE6x=(-2.0*JWb5x)/b4jXr+jEwtH/b4jXr-1.0*(Z4lSz)+SynchronizationTime-(2.0*pbQOc(
1.0*VNvlY(JWb5x)-1.0*JWb5x*jEwtH+0.5*VNvlY(jEwtH)+1.0*JWb5x*b4jXr*(Z4lSz)-1.0*
JWb5x*b4jXr*SynchronizationTime-1.0*b4jXr*JRB5m+1.0*b4jXr*EI730))/b4jXr;if(EvE6x
<0.0){EvE6x=0.0;}if(EvE6x>(Yoeil-(2.0*wmBRB-jEwtH)/b4jXr)){EvE6x=(Yoeil-(2.0*
wmBRB-jEwtH)/b4jXr);}ZaFFa=EvE6x*JWb5x;zlPa_=(JRB5m+xek11)*EvE6x+0.5*JWb5x*VNvlY
(EvE6x);
FvoS1=(wmBRB-JWb5x)/b4jXr;_HBf3=JWb5x*FvoS1+0.5*b4jXr*VNvlY(FvoS1);g7nwu=(JRB5m+
xek11+ZaFFa)*FvoS1+0.5*JWb5x*VNvlY(FvoS1)+b4jXr*qn3x8(FvoS1)/6.0;
LOUhW=wmBRB/b4jXr;T5yjB=0.5*VNvlY(wmBRB)/b4jXr;mpXRu=(JRB5m+xek11+ZaFFa+_HBf3)*
LOUhW+0.5*wmBRB*VNvlY(LOUhW)-b4jXr*qn3x8(LOUhW)/6.0;


if(m85bi(JRB5m)==m85bi(JRB5m+xek11)){XLg6q+=fabs(Ixvmb);}else{D0ol4=-jEwtH/b4jXr
+pbQOc(VNvlY(jEwtH/b4jXr)-2.0*JRB5m/b4jXr);if(D0ol4<0.0){D0ol4=0.0;}if(D0ol4>
Kaclx){D0ol4=Kaclx;}XLg6q+=fabs(JRB5m*D0ol4+0.5*jEwtH*VNvlY(D0ol4)-b4jXr*qn3x8(
D0ol4)/6.0)+fabs(0.5*(jEwtH-b4jXr*D0ol4)*VNvlY(Kaclx-D0ol4)-b4jXr*qn3x8(Kaclx-
D0ol4)/6.0);}
if(m85bi(JRB5m+xek11)==m85bi(JRB5m+xek11+ZaFFa)){XLg6q+=fabs(zlPa_);}else{D0ol4=
fabs((JRB5m+xek11)/JWb5x);XLg6q+=fabs(0.5*JWb5x*VNvlY(D0ol4))+fabs(0.5*JWb5x*
VNvlY(EvE6x-D0ol4));}
if(m85bi(JRB5m+xek11+ZaFFa)==m85bi(JRB5m+xek11+ZaFFa+_HBf3)){XLg6q+=fabs(g7nwu);
}else{D0ol4=-JWb5x/b4jXr+pbQOc(VNvlY(JWb5x/b4jXr)-2.0*(JRB5m+xek11+ZaFFa)/b4jXr)
;if(D0ol4<0.0){D0ol4=0.0;}if(D0ol4>FvoS1){D0ol4=FvoS1;}XLg6q+=fabs((JRB5m+xek11+
ZaFFa)*D0ol4+0.5*JWb5x*VNvlY(D0ol4)+b4jXr*qn3x8(D0ol4)/6.0)+fabs(0.5*(JWb5x+
b4jXr*D0ol4)*VNvlY(FvoS1-D0ol4)+b4jXr*qn3x8(FvoS1-D0ol4)/6.0);}
if(m85bi(JRB5m+xek11+ZaFFa+_HBf3)==m85bi(EI730)){XLg6q+=fabs(mpXRu);}else{D0ol4=
wmBRB/b4jXr-pbQOc(VNvlY(wmBRB/b4jXr)+2.0*(JRB5m+xek11+ZaFFa+_HBf3)/b4jXr);if(
D0ol4<0.0){D0ol4=0.0;}if(D0ol4>LOUhW){D0ol4=LOUhW;}XLg6q+=fabs((JRB5m+xek11+
ZaFFa+_HBf3)*D0ol4+0.5*wmBRB*VNvlY(D0ol4)-b4jXr*qn3x8(D0ol4)/6.0)+fabs(0.5*(
wmBRB-b4jXr*D0ol4)*VNvlY(LOUhW-D0ol4)-b4jXr*qn3x8(LOUhW-D0ol4)/6.0);}
inQml=fabs(xek11)+fabs(ZaFFa)+fabs(_HBf3)+fabs(T5yjB);


AF2wW=Kaclx+EvE6x+FvoS1+LOUhW;jfI7z=Ixvmb+zlPa_+g7nwu+mpXRu;gvv1d=xek11+ZaFFa+
_HBf3+T5yjB;if(AF2wW!=0.0){fjgkf=SynchronizationTime-Z4lSz-AF2wW;if(fabs(fjgkf)>
(kED7o*(SynchronizationTime-Z4lSz)+PZBAL)){*DBRdu=true;}}if(jfI7z!=0.0){hxN2g=
EgDTv-TfLg_-jfI7z;if(fabs(hxN2g)>(fabs(Cmmsw*XLg6q)+oUhjD)){*DBRdu=true;}}if(
gvv1d!=0.0){jE9zt=EI730-JRB5m-gvv1d;if(fabs(jE9zt)>(c_4Sr*fabs((fabs(EI730)>fabs
(inQml))?(EI730):(inQml)))+Uq_ZW){*DBRdu=true;}}



if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),(0.5*(-jEwtH)),(-JRB5m
),(-TfLg_),(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,((-b4jXr)*0.5),(-jEwtH)
,(-JRB5m),(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,(-b4jXr),(-jEwtH),(
Z4lSz));}else{_2E29->fCxBi[_2E29->djbCD].jGQqQ((b4jXr/6.0),(0.5*jEwtH),JRB5m,
TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(b4jXr*0.5),jEwtH,JRB5m,(
Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,b4jXr,jEwtH,(Z4lSz));}_2E29->
FfhZi[_2E29->djbCD]=(Z4lSz)+Kaclx;_2E29->djbCD++;
Z4lSz+=(Kaclx);TfLg_+=Ixvmb;JRB5m+=xek11;jEwtH=JWb5x;

if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0,(0.5*(-JWb5x)),(-JRB5m),(-TfLg_),
(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,-JWb5x,(-JRB5m),(Z4lSz));_2E29
->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,-JWb5x,(Z4lSz));}else{_2E29->fCxBi[_2E29
->djbCD].jGQqQ(0.0,(0.5*JWb5x),JRB5m,TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].
jGQqQ(0.0,0.0,JWb5x,JRB5m,(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,
JWb5x,(Z4lSz));}_2E29->FfhZi[_2E29->djbCD]=(Z4lSz)+EvE6x;_2E29->djbCD++;
Z4lSz+=(EvE6x);TfLg_+=zlPa_;JRB5m+=ZaFFa;jEwtH=JWb5x;

if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),(0.5*(-JWb5x)),(-JRB5m
),(-TfLg_),(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*(-b4jXr)),-JWb5x,(
-JRB5m),(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,-b4jXr,-JWb5x,(Z4lSz))
;}else{_2E29->fCxBi[_2E29->djbCD].jGQqQ((b4jXr/6.0),(0.5*JWb5x),JRB5m,TfLg_,(
Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*b4jXr),JWb5x,JRB5m,(Z4lSz));
_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,b4jXr,JWb5x,(Z4lSz));}_2E29->FfhZi[
_2E29->djbCD]=(Z4lSz)+FvoS1;_2E29->djbCD++;
Z4lSz+=(FvoS1);TfLg_+=g7nwu;JRB5m+=_HBf3;jEwtH=wmBRB;

if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ((b4jXr/6.0),(0.5*(-wmBRB)),-JRB5m,-
TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*b4jXr),-wmBRB,-JRB5m,(
Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,b4jXr,-wmBRB,(Z4lSz));}else{
_2E29->fCxBi[_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),(0.5*wmBRB),JRB5m,TfLg_,(Z4lSz))
;_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*(-b4jXr)),wmBRB,JRB5m,(Z4lSz));_2E29
->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,-b4jXr,wmBRB,(Z4lSz));}_2E29->FfhZi[_2E29->
djbCD]=(Z4lSz)+LOUhW;_2E29->djbCD++;
Z4lSz+=(LOUhW);TfLg_+=mpXRu;JRB5m+=T5yjB;jEwtH=0.0;
if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0,0.0,-JRB5m,-TfLg_,(Z4lSz));_2E29
->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,-JRB5m,(Z4lSz));_2E29->NYWGt[_2E29->
djbCD].jGQqQ(0.0,0.0,0.0,0.0,(Z4lSz));}else{_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0
,0.0,JRB5m,TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,JRB5m,(
Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,0.0,(Z4lSz));}_2E29->FfhZi[
_2E29->djbCD]=(Z4lSz)+tlPiC;_2E29->djbCD++;return;}

void qPN_6::xZ4vy(const double&rTEdP,const double&SynchronizationTime,const 
double&iPzPj,const double&ldeCA,const double&AQzqu,const double&HcINC,const 
double&k6sf4,const double&DAwhk,const double&b4jXr,XkwFr*_2E29,const bool&U7RiH,
bool*DBRdu){double ifRIn=0.0,TfLg_=iPzPj,EgDTv=ldeCA,JRB5m=AQzqu,EI730=HcINC,
jEwtH=k6sf4,Z4lSz=rTEdP,JWb5x=0.0,Kaclx=0.0,EvE6x=0.0,FvoS1=0.0,LOUhW=0.0,Qogm3=
0.0,xek11=0.0,ZaFFa=0.0,_HBf3=0.0,T5yjB=0.0,OLj8b=0.0,Ixvmb=0.0,zlPa_=0.0,g7nwu=
0.0,mpXRu=0.0,xHUUb=0.0,hxN2g=0.0,jE9zt=0.0,fjgkf=0.0,jfI7z=0.0,gvv1d=0.0,AF2wW=
0.0,Yoeil=0.0,XLg6q=0.0,inQml=0.0,D0ol4=0.0;

if(jEwtH>DAwhk){jEwtH=DAwhk;}Yoeil=SynchronizationTime-Z4lSz;ifRIn=Q8oIQ(&pOAGI,
&XzpgJ,TfLg_,EgDTv,JRB5m,EI730,jEwtH,DAwhk,b4jXr,Yoeil);

EvE6x=ifRIn;if(EvE6x<0.0){EvE6x=0.0;}
Kaclx=(DAwhk-jEwtH)/b4jXr;xek11=jEwtH*Kaclx+0.5*b4jXr*VNvlY(Kaclx);Ixvmb=JRB5m*
Kaclx+0.5*jEwtH*VNvlY(Kaclx)+b4jXr*qn3x8(Kaclx)/6.0;
ZaFFa=EvE6x*DAwhk;zlPa_=(JRB5m+xek11)*EvE6x+0.5*DAwhk*VNvlY(EvE6x);if(fabs(jEwtH
-2.0*DAwhk-b4jXr*EvE6x-b4jXr*(Z4lSz)+b4jXr*SynchronizationTime)<MFWCZ){

JWb5x=DAwhk;}else{JWb5x=(b4jXr*((1.0*VNvlY(jEwtH))/b4jXr-(2.0*VNvlY(DAwhk))/
b4jXr-2.0*DAwhk*EvE6x-2.0*JRB5m+2.0*EI730))/(2.0*jEwtH-4.0*DAwhk-2.0*b4jXr*EvE6x
-2.0*b4jXr*(Z4lSz)+2.0*b4jXr*SynchronizationTime);if(JWb5x<=0.0){JWb5x=T68ug;}if
(JWb5x>DAwhk){JWb5x=DAwhk;}}
FvoS1=(DAwhk-JWb5x)/b4jXr;_HBf3=DAwhk*FvoS1-0.5*b4jXr*VNvlY(FvoS1);g7nwu=(JRB5m+
xek11+ZaFFa)*FvoS1+0.5*DAwhk*VNvlY(FvoS1)-b4jXr*qn3x8(FvoS1)/6.0;
Qogm3=JWb5x/b4jXr;OLj8b=0.5*VNvlY(JWb5x)/b4jXr;

T5yjB=EI730-JRB5m-xek11-ZaFFa-_HBf3-OLj8b;if(T5yjB<0.0){T5yjB=0.0;}if(fabs(JWb5x
)<MFWCZ){LOUhW=SynchronizationTime-Z4lSz-Kaclx-EvE6x-FvoS1-JWb5x/b4jXr;}else{
LOUhW=T5yjB/JWb5x;}if(LOUhW<0.0){LOUhW=0.0;}mpXRu=(JRB5m+xek11+ZaFFa+_HBf3)*
LOUhW+0.5*JWb5x*VNvlY(LOUhW);
xHUUb=(JRB5m+xek11+ZaFFa+_HBf3+T5yjB)*Qogm3+0.5*JWb5x*VNvlY(Qogm3)-b4jXr*qn3x8(
Qogm3)/6.0;


if(m85bi(JRB5m)==m85bi(JRB5m+xek11)){XLg6q+=fabs(Ixvmb);}else{D0ol4=-jEwtH/b4jXr
+pbQOc(VNvlY(jEwtH/b4jXr)-2.0*JRB5m/b4jXr);if(D0ol4<0.0){D0ol4=0.0;}if(D0ol4>
Kaclx){D0ol4=Kaclx;}XLg6q+=fabs(JRB5m*D0ol4+0.5*jEwtH*VNvlY(D0ol4)+b4jXr*qn3x8(
D0ol4)/6.0)+fabs(0.5*(jEwtH-b4jXr*D0ol4)*VNvlY(Kaclx-D0ol4)+b4jXr*qn3x8(Kaclx-
D0ol4)/6.0);}
if(m85bi(JRB5m+xek11)==m85bi(JRB5m+xek11+ZaFFa)){XLg6q+=fabs(zlPa_);}else{D0ol4=
fabs((JRB5m+xek11)/DAwhk);XLg6q+=fabs(0.5*DAwhk*VNvlY(D0ol4))+fabs(0.5*DAwhk*
VNvlY(EvE6x-D0ol4));}
if(m85bi(JRB5m+xek11+ZaFFa)==m85bi(JRB5m+xek11+ZaFFa+_HBf3)){XLg6q+=fabs(g7nwu);
}else{D0ol4=DAwhk/b4jXr-pbQOc(VNvlY(DAwhk/b4jXr)+2.0*(JRB5m+xek11+ZaFFa)/b4jXr);
if(D0ol4<0.0){D0ol4=0.0;}if(D0ol4>FvoS1){D0ol4=FvoS1;}XLg6q+=fabs((JRB5m+xek11+
ZaFFa)*D0ol4+0.5*DAwhk*VNvlY(D0ol4)-b4jXr*qn3x8(D0ol4)/6.0)+fabs(0.5*(DAwhk-
b4jXr*D0ol4)*VNvlY(FvoS1-D0ol4)-b4jXr*qn3x8(FvoS1-D0ol4)/6.0);}
if(m85bi(JRB5m+xek11+ZaFFa+_HBf3)==m85bi(JRB5m+xek11+ZaFFa+_HBf3+T5yjB)){XLg6q+=
fabs(mpXRu);}else{D0ol4=fabs((JRB5m+xek11+ZaFFa+_HBf3)/JWb5x);XLg6q+=fabs(0.5*
JWb5x*VNvlY(D0ol4))+fabs(0.5*JWb5x*VNvlY(LOUhW-D0ol4));}
if(m85bi(JRB5m+xek11+ZaFFa+_HBf3+T5yjB)==m85bi(EI730)){XLg6q+=fabs(xHUUb);}else{
D0ol4=JWb5x/b4jXr-pbQOc(VNvlY(JWb5x/b4jXr)+2.0*(JRB5m+xek11+ZaFFa+_HBf3+T5yjB)/
b4jXr);if(D0ol4<0.0){D0ol4=0.0;}if(D0ol4>Qogm3){D0ol4=Qogm3;}XLg6q+=fabs((JRB5m+
xek11+ZaFFa+_HBf3+T5yjB)*D0ol4+0.5*JWb5x*VNvlY(D0ol4)-b4jXr*qn3x8(D0ol4)/6.0)+
fabs(0.5*(JWb5x-b4jXr*D0ol4)*VNvlY(Qogm3-D0ol4)-b4jXr*qn3x8(Qogm3-D0ol4)/6.0);}
inQml=fabs(xek11)+fabs(ZaFFa)+fabs(_HBf3)+fabs(T5yjB)+fabs(OLj8b);


AF2wW=Kaclx+EvE6x+FvoS1+LOUhW+Qogm3;jfI7z=Ixvmb+zlPa_+g7nwu+mpXRu+xHUUb;gvv1d=
xek11+ZaFFa+_HBf3+T5yjB+OLj8b;if(AF2wW!=0.0){fjgkf=SynchronizationTime-Z4lSz-
AF2wW;if(fabs(fjgkf)>(kED7o*(SynchronizationTime-Z4lSz)+PZBAL)){*DBRdu=true;}}if
(jfI7z!=0.0){hxN2g=EgDTv-TfLg_-jfI7z;if(fabs(hxN2g)>(fabs(Cmmsw*XLg6q)+oUhjD)){*
DBRdu=true;}}if(gvv1d!=0.0){jE9zt=EI730-JRB5m-gvv1d;if(fabs(jE9zt)>(c_4Sr*fabs((
fabs(EI730)>fabs(inQml))?(EI730):(inQml)))+Uq_ZW){*DBRdu=true;}}



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
Z4lSz+=(FvoS1);TfLg_+=g7nwu;JRB5m+=_HBf3;jEwtH=JWb5x;

if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0,(-0.5*JWb5x),(-JRB5m),(-TfLg_),(
Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,-JWb5x,(-JRB5m),(Z4lSz));_2E29
->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,-JWb5x,(Z4lSz));}else{_2E29->fCxBi[_2E29
->djbCD].jGQqQ(0.0,(0.5*JWb5x),JRB5m,TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].
jGQqQ(0.0,0.0,JWb5x,JRB5m,(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,
JWb5x,(Z4lSz));}_2E29->FfhZi[_2E29->djbCD]=(Z4lSz)+LOUhW;_2E29->djbCD++;
Z4lSz+=(LOUhW);TfLg_+=mpXRu;JRB5m+=T5yjB;jEwtH=JWb5x;

if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(((b4jXr)/6.0),(-0.5*JWb5x),(-JRB5m),(
-TfLg_),(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*b4jXr),-JWb5x,(-JRB5m
),(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,b4jXr,-JWb5x,(Z4lSz));}else{
_2E29->fCxBi[_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),(0.5*JWb5x),JRB5m,TfLg_,(Z4lSz))
;_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*(-b4jXr)),JWb5x,JRB5m,(Z4lSz));_2E29
->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,-b4jXr,JWb5x,(Z4lSz));}_2E29->FfhZi[_2E29->
djbCD]=(Z4lSz)+Qogm3;_2E29->djbCD++;
Z4lSz+=(Qogm3);TfLg_+=xHUUb;JRB5m+=OLj8b;jEwtH=0.0;
if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0,0.0,-JRB5m,-TfLg_,(Z4lSz));_2E29
->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,-JRB5m,(Z4lSz));_2E29->NYWGt[_2E29->
djbCD].jGQqQ(0.0,0.0,0.0,0.0,(Z4lSz));}else{_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0
,0.0,JRB5m,TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,JRB5m,(
Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,0.0,(Z4lSz));}_2E29->FfhZi[
_2E29->djbCD]=(Z4lSz)+tlPiC;_2E29->djbCD++;return;}

void qPN_6::oLIt_(const double&rTEdP,const double&SynchronizationTime,const 
double&iPzPj,const double&ldeCA,const double&AQzqu,const double&HcINC,const 
double&k6sf4,const double&DAwhk,const double&b4jXr,XkwFr*_2E29,const bool&U7RiH,
bool*DBRdu){double ifRIn=0.0,TfLg_=iPzPj,EgDTv=ldeCA,JRB5m=AQzqu,EI730=HcINC,
jEwtH=k6sf4,Z4lSz=rTEdP,Kaclx=0.0,EvE6x=0.0,FvoS1=0.0,LOUhW=0.0,Qogm3=0.0,CCBY_=
0.0,X6X_j=0.0,xek11=0.0,ZaFFa=0.0,_HBf3=0.0,T5yjB=0.0,OLj8b=0.0,CnAKY=0.0,oxEkR=
0.0,Ixvmb=0.0,zlPa_=0.0,g7nwu=0.0,mpXRu=0.0,xHUUb=0.0,_qvll=0.0,deZtJ=0.0,hxN2g=
0.0,jE9zt=0.0,fjgkf=0.0,jfI7z=0.0,gvv1d=0.0,AF2wW=0.0,Yoeil=0.0,rDoKh=0.0,XLg6q=
0.0,inQml=0.0,D0ol4=0.0;

if(jEwtH>DAwhk){jEwtH=DAwhk;}Yoeil=SynchronizationTime-Z4lSz;ifRIn=Q8oIQ(&gG0XJ,
&FmO3l,TfLg_,EgDTv,JRB5m,EI730,jEwtH,DAwhk,b4jXr,Yoeil);

EvE6x=ifRIn;if(EvE6x<0.0){EvE6x=0.0;}
Kaclx=(DAwhk-jEwtH)/b4jXr;xek11=jEwtH*Kaclx+0.5*b4jXr*VNvlY(Kaclx);Ixvmb=JRB5m*
Kaclx+0.5*jEwtH*VNvlY(Kaclx)+b4jXr*qn3x8(Kaclx)/6.0;
ZaFFa=EvE6x*DAwhk;zlPa_=(JRB5m+xek11)*EvE6x+0.5*DAwhk*VNvlY(EvE6x);
FvoS1=DAwhk/b4jXr;_HBf3=0.5*VNvlY(DAwhk)/b4jXr;g7nwu=(JRB5m+xek11+ZaFFa)*FvoS1+
0.5*DAwhk*VNvlY(FvoS1)-b4jXr*qn3x8(FvoS1)/6.0;

T5yjB=0.0;

Qogm3=FvoS1;OLj8b=-_HBf3;xHUUb=(JRB5m+xek11+ZaFFa+_HBf3+T5yjB)*Qogm3-b4jXr*qn3x8
(Qogm3)/6.0;
X6X_j=FvoS1;oxEkR=-_HBf3;

CnAKY=EI730-JRB5m-xek11-ZaFFa-_HBf3-T5yjB-OLj8b-oxEkR;CCBY_=CnAKY/(-DAwhk);_qvll
=(JRB5m+xek11+ZaFFa+_HBf3+T5yjB+OLj8b)*CCBY_-0.5*DAwhk*VNvlY(CCBY_);
deZtJ=(JRB5m+xek11+ZaFFa+_HBf3+T5yjB+OLj8b+CnAKY)*X6X_j-0.5*DAwhk*VNvlY(X6X_j)+
b4jXr*qn3x8(X6X_j)/6.0;
mpXRu=EgDTv-TfLg_-Ixvmb-zlPa_-g7nwu-xHUUb-_qvll-deZtJ;if((fabs(JRB5m+xek11+ZaFFa
+_HBf3)<Nm0g6)||(fabs(mpXRu)<Nm0g6)){LOUhW=SynchronizationTime-Z4lSz-Kaclx-EvE6x
-FvoS1-Qogm3-CCBY_-X6X_j;}else{rDoKh=(JRB5m+xek11+ZaFFa+_HBf3);if(rDoKh==0.0){
LOUhW=SynchronizationTime-Z4lSz-Kaclx-EvE6x-FvoS1-Qogm3-CCBY_-X6X_j;}else{LOUhW=
mpXRu/rDoKh;}}if(LOUhW<0.0){LOUhW=0.0;}


if(m85bi(JRB5m)==m85bi(JRB5m+xek11)){XLg6q+=fabs(Ixvmb);}else{D0ol4=-jEwtH/b4jXr
+pbQOc(VNvlY(jEwtH/b4jXr)-2.0*JRB5m/b4jXr);if(D0ol4<0.0){D0ol4=0.0;}if(D0ol4>
Kaclx){D0ol4=Kaclx;}XLg6q+=fabs(JRB5m*D0ol4+0.5*jEwtH*VNvlY(D0ol4)+b4jXr*qn3x8(
D0ol4)/6.0)+fabs(0.5*(jEwtH-b4jXr*D0ol4)*VNvlY(Kaclx-D0ol4)+b4jXr*qn3x8(Kaclx-
D0ol4)/6.0);}
if(m85bi(JRB5m+xek11)==m85bi(JRB5m+xek11+ZaFFa)){XLg6q+=fabs(zlPa_);}else{D0ol4=
fabs((JRB5m+xek11)/DAwhk);XLg6q+=fabs(0.5*DAwhk*VNvlY(D0ol4))+fabs(0.5*DAwhk*
VNvlY(EvE6x-D0ol4));}
if(m85bi(JRB5m+xek11+ZaFFa)==m85bi(JRB5m+xek11+ZaFFa+_HBf3)){XLg6q+=fabs(g7nwu);
}else{D0ol4=DAwhk/b4jXr-pbQOc(VNvlY(DAwhk/b4jXr)+2.0*(JRB5m+xek11+ZaFFa)/b4jXr);
if(D0ol4<0.0){D0ol4=0.0;}if(D0ol4>FvoS1){D0ol4=FvoS1;}XLg6q+=fabs((JRB5m+xek11+
ZaFFa)*D0ol4+0.5*DAwhk*VNvlY(D0ol4)-b4jXr*qn3x8(D0ol4)/6.0)+fabs(0.5*(DAwhk-
b4jXr*D0ol4)*VNvlY(FvoS1-D0ol4)-b4jXr*qn3x8(FvoS1-D0ol4)/6.0);}
XLg6q+=fabs(mpXRu);
if(m85bi(JRB5m+xek11+ZaFFa+_HBf3+T5yjB)==m85bi(JRB5m+xek11+ZaFFa+_HBf3+T5yjB+
OLj8b)){XLg6q+=fabs(xHUUb);}else{D0ol4=pbQOc(2.0*(JRB5m+xek11+ZaFFa+_HBf3+T5yjB)
/b4jXr);if(D0ol4>Qogm3){D0ol4=Qogm3;}XLg6q+=fabs((JRB5m+xek11+ZaFFa+_HBf3+T5yjB)
*D0ol4-b4jXr*qn3x8(D0ol4)/6.0)+fabs(0.5*(-b4jXr*D0ol4)*VNvlY(Qogm3-D0ol4)-b4jXr*
qn3x8(Qogm3-D0ol4)/6.0);}
if(m85bi(JRB5m+xek11+ZaFFa+_HBf3+T5yjB+OLj8b)==m85bi(JRB5m+xek11+ZaFFa+_HBf3+
T5yjB+OLj8b+CnAKY)){XLg6q+=fabs(_qvll);}else{D0ol4=fabs((JRB5m+xek11+ZaFFa+_HBf3
+T5yjB+OLj8b)/DAwhk);XLg6q+=fabs(0.5*DAwhk*VNvlY(D0ol4))+fabs(0.5*DAwhk*VNvlY(
CCBY_-D0ol4));}
if(m85bi(JRB5m+xek11+ZaFFa+_HBf3+T5yjB+OLj8b+CnAKY)==m85bi(EI730)){XLg6q+=fabs(
deZtJ);}else{D0ol4=DAwhk/b4jXr-pbQOc(VNvlY(DAwhk/b4jXr)-2.0*(JRB5m+xek11+ZaFFa+
_HBf3+T5yjB+OLj8b+CnAKY)/b4jXr);if(D0ol4<0.0){D0ol4=0.0;}if(D0ol4>X6X_j){D0ol4=
X6X_j;}XLg6q+=fabs((JRB5m+xek11+ZaFFa+_HBf3+T5yjB+OLj8b+CnAKY)*D0ol4-0.5*DAwhk*
VNvlY(D0ol4)+b4jXr*qn3x8(D0ol4)/6.0)+fabs(0.5*(-DAwhk+b4jXr*D0ol4)*VNvlY(X6X_j-
D0ol4)+b4jXr*qn3x8(X6X_j-D0ol4)/6.0);}
inQml=fabs(xek11)+fabs(ZaFFa)+fabs(_HBf3)+fabs(T5yjB)+fabs(OLj8b)+fabs(CnAKY)+
fabs(oxEkR);


AF2wW=Kaclx+EvE6x+FvoS1+LOUhW+Qogm3+CCBY_+X6X_j;jfI7z=Ixvmb+zlPa_+g7nwu+mpXRu+
xHUUb+_qvll+deZtJ;gvv1d=xek11+ZaFFa+_HBf3+T5yjB+OLj8b+CnAKY+oxEkR;if(AF2wW!=0.0)
{fjgkf=SynchronizationTime-Z4lSz-AF2wW;if(fabs(fjgkf)>(kED7o*(
SynchronizationTime-Z4lSz)+PZBAL)){*DBRdu=true;}}if(jfI7z!=0.0){hxN2g=EgDTv-
TfLg_-jfI7z;if(fabs(hxN2g)>(fabs(Cmmsw*XLg6q)+oUhjD)){*DBRdu=true;}}if(gvv1d!=
0.0){jE9zt=EI730-JRB5m-gvv1d;if(fabs(jE9zt)>(c_4Sr*fabs((fabs(EI730)>fabs(inQml)
)?(EI730):(inQml)))+Uq_ZW){*DBRdu=true;}}



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
Z4lSz+=(FvoS1);TfLg_+=g7nwu;JRB5m+=_HBf3;jEwtH=(0x57c+608-0x7dc);

if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0,0.0,(-JRB5m),(-TfLg_),(Z4lSz));
_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,(-JRB5m),(Z4lSz));_2E29->NYWGt[
_2E29->djbCD].jGQqQ(0.0,0.0,0.0,0.0,(Z4lSz));}else{_2E29->fCxBi[_2E29->djbCD].
jGQqQ(0.0,0.0,JRB5m,TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,
JRB5m,(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,0.0,(Z4lSz));}_2E29
->FfhZi[_2E29->djbCD]=(Z4lSz)+LOUhW;_2E29->djbCD++;
Z4lSz+=(LOUhW);TfLg_+=mpXRu;JRB5m+=T5yjB;jEwtH=(0x1f4b+595-0x219e);

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
_2E29->djbCD]=(Z4lSz)+tlPiC;_2E29->djbCD++;return;}

void qPN_6::serOm(const double&rTEdP,const double&SynchronizationTime,const 
double&iPzPj,const double&ldeCA,const double&AQzqu,const double&HcINC,const 
double&k6sf4,const double&DAwhk,const double&b4jXr,XkwFr*_2E29,const bool&U7RiH,
bool*DBRdu){double ifRIn=0.0,TfLg_=iPzPj,EgDTv=ldeCA,JRB5m=AQzqu,EI730=HcINC,
jEwtH=k6sf4,Z4lSz=rTEdP,RIZdZ=0.0,Kaclx=0.0,EvE6x=0.0,FvoS1=0.0,LOUhW=0.0,Qogm3=
0.0,CCBY_=0.0,xek11=0.0,ZaFFa=0.0,_HBf3=0.0,T5yjB=0.0,OLj8b=0.0,CnAKY=0.0,Ixvmb=
0.0,zlPa_=0.0,g7nwu=0.0,mpXRu=0.0,xHUUb=0.0,_qvll=0.0,hxN2g=0.0,jE9zt=0.0,fjgkf=
0.0,jfI7z=0.0,gvv1d=0.0,AF2wW=0.0,Yoeil=0.0,rDoKh=0.0,XLg6q=0.0,inQml=0.0,D0ol4=
0.0;

if(jEwtH>DAwhk){jEwtH=DAwhk;}Yoeil=SynchronizationTime-Z4lSz;ifRIn=Q8oIQ(&uS_9X,
&pXGYp,TfLg_,EgDTv,JRB5m,EI730,jEwtH,DAwhk,b4jXr,Yoeil);

RIZdZ=ifRIn;if(RIZdZ>0.0){RIZdZ=0.0;}
Kaclx=(DAwhk-jEwtH)/b4jXr;xek11=jEwtH*Kaclx+0.5*b4jXr*VNvlY(Kaclx);Ixvmb=JRB5m*
Kaclx+0.5*jEwtH*VNvlY(Kaclx)+b4jXr*qn3x8(Kaclx)/6.0;
FvoS1=DAwhk/b4jXr;_HBf3=0.5*VNvlY(DAwhk)/b4jXr;


T5yjB=0.0;

Qogm3=(-RIZdZ)/b4jXr;OLj8b=-0.5*VNvlY(RIZdZ)/b4jXr;

CCBY_=Qogm3;CnAKY=OLj8b;

ZaFFa=EI730-JRB5m-xek11-_HBf3-T5yjB-OLj8b-CnAKY;if(ZaFFa<0.0){ZaFFa=0.0;}EvE6x=
ZaFFa/DAwhk;zlPa_=(JRB5m+xek11)*EvE6x+0.5*DAwhk*VNvlY(EvE6x);
g7nwu=(JRB5m+xek11+ZaFFa)*FvoS1+0.5*DAwhk*VNvlY(FvoS1)-b4jXr*qn3x8(FvoS1)/6.0;
xHUUb=(JRB5m+xek11+ZaFFa+_HBf3+T5yjB)*Qogm3-b4jXr*qn3x8(Qogm3)/6.0;
_qvll=(JRB5m+xek11+ZaFFa+_HBf3+T5yjB+OLj8b)*CCBY_+0.5*RIZdZ*VNvlY(CCBY_)+b4jXr*
qn3x8(CCBY_)/6.0;
mpXRu=EgDTv-TfLg_-Ixvmb-zlPa_-g7nwu-xHUUb-_qvll;if((fabs(JRB5m+xek11+ZaFFa+_HBf3
)<Nm0g6)||(fabs(mpXRu)<Nm0g6)){LOUhW=SynchronizationTime-Z4lSz-Kaclx-EvE6x-FvoS1
-Qogm3-CCBY_;}else{rDoKh=(JRB5m+xek11+ZaFFa+_HBf3);if(rDoKh==0.0){LOUhW=
SynchronizationTime-Z4lSz-Kaclx-EvE6x-FvoS1-Qogm3-CCBY_;}else{LOUhW=mpXRu/rDoKh;
}}if(LOUhW<0.0){LOUhW=0.0;}


if(m85bi(JRB5m)==m85bi(JRB5m+xek11)){XLg6q+=fabs(Ixvmb);}else{D0ol4=-jEwtH/b4jXr
+pbQOc(VNvlY(jEwtH/b4jXr)-2.0*JRB5m/b4jXr);if(D0ol4<0.0){D0ol4=0.0;}if(D0ol4>
Kaclx){D0ol4=Kaclx;}XLg6q+=fabs(JRB5m*D0ol4+0.5*jEwtH*VNvlY(D0ol4)+b4jXr*qn3x8(
D0ol4)/6.0)+fabs(0.5*(jEwtH-b4jXr*D0ol4)*VNvlY(Kaclx-D0ol4)+b4jXr*qn3x8(Kaclx-
D0ol4)/6.0);}
if(m85bi(JRB5m+xek11)==m85bi(JRB5m+xek11+ZaFFa)){XLg6q+=fabs(zlPa_);}else{D0ol4=
fabs((JRB5m+xek11)/DAwhk);XLg6q+=fabs(0.5*DAwhk*VNvlY(D0ol4))+fabs(0.5*DAwhk*
VNvlY(EvE6x-D0ol4));}
if(m85bi(JRB5m+xek11+ZaFFa)==m85bi(JRB5m+xek11+ZaFFa+_HBf3)){XLg6q+=fabs(g7nwu);
}else{D0ol4=DAwhk/b4jXr-pbQOc(VNvlY(DAwhk/b4jXr)+2.0*(JRB5m+xek11+ZaFFa)/b4jXr);
if(D0ol4<0.0){D0ol4=0.0;}if(D0ol4>FvoS1){D0ol4=FvoS1;}XLg6q+=fabs((JRB5m+xek11+
ZaFFa)*D0ol4+0.5*DAwhk*VNvlY(D0ol4)-b4jXr*qn3x8(D0ol4)/6.0)+fabs(0.5*(DAwhk-
b4jXr*D0ol4)*VNvlY(FvoS1-D0ol4)-b4jXr*qn3x8(FvoS1-D0ol4)/6.0);}
XLg6q+=fabs(mpXRu);
if(m85bi(JRB5m+xek11+ZaFFa+_HBf3+T5yjB)==m85bi(JRB5m+xek11+ZaFFa+_HBf3+T5yjB+
OLj8b)){XLg6q+=fabs(xHUUb);}else{D0ol4=pbQOc(2.0*(JRB5m+xek11+ZaFFa+_HBf3+T5yjB)
/b4jXr);if(D0ol4>Qogm3){D0ol4=Qogm3;}XLg6q+=fabs((JRB5m+xek11+ZaFFa+_HBf3+T5yjB)
*D0ol4-b4jXr*qn3x8(D0ol4)/6.0)+fabs(0.5*(-b4jXr*D0ol4)*VNvlY(Qogm3-D0ol4)-b4jXr*
qn3x8(Qogm3-D0ol4)/6.0);}
if(m85bi(JRB5m+xek11+ZaFFa+_HBf3+T5yjB+OLj8b)==m85bi(EI730)){XLg6q+=fabs(_qvll);
}else{D0ol4=RIZdZ/b4jXr-pbQOc(VNvlY(RIZdZ/b4jXr)-2.0*(JRB5m+xek11+ZaFFa+_HBf3+
T5yjB+OLj8b)/b4jXr);if(D0ol4<0.0){D0ol4=0.0;}if(D0ol4>CCBY_){D0ol4=CCBY_;}XLg6q
+=fabs((JRB5m+xek11+ZaFFa+_HBf3+T5yjB+OLj8b)*D0ol4+0.5*RIZdZ*VNvlY(D0ol4)+b4jXr*
qn3x8(D0ol4)/6.0)+fabs(0.5*(RIZdZ+b4jXr*D0ol4)*VNvlY(CCBY_-D0ol4)+b4jXr*qn3x8(
CCBY_-D0ol4)/6.0);}
inQml=fabs(xek11)+fabs(ZaFFa)+fabs(_HBf3)+fabs(T5yjB)+fabs(OLj8b)+fabs(CnAKY);


AF2wW=Kaclx+EvE6x+FvoS1+LOUhW+Qogm3+CCBY_;jfI7z=Ixvmb+zlPa_+g7nwu+mpXRu+xHUUb+
_qvll;gvv1d=xek11+ZaFFa+_HBf3+T5yjB+OLj8b+CnAKY;if(AF2wW!=0.0){fjgkf=
SynchronizationTime-Z4lSz-AF2wW;if(fabs(fjgkf)>(kED7o*(SynchronizationTime-Z4lSz
)+PZBAL)){*DBRdu=true;}}if(jfI7z!=0.0){hxN2g=EgDTv-TfLg_-jfI7z;if(fabs(hxN2g)>(
fabs(Cmmsw*XLg6q)+oUhjD)){*DBRdu=true;}}if(gvv1d!=0.0){jE9zt=EI730-JRB5m-gvv1d;
if(fabs(jE9zt)>(c_4Sr*fabs((fabs(EI730)>fabs(inQml))?(EI730):(inQml)))+Uq_ZW){*
DBRdu=true;}}



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
Z4lSz+=(FvoS1);TfLg_+=g7nwu;JRB5m+=_HBf3;jEwtH=(0x1629+4167-0x2670);

if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0,0.0,(-JRB5m),(-TfLg_),(Z4lSz));
_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,(-JRB5m),(Z4lSz));_2E29->NYWGt[
_2E29->djbCD].jGQqQ(0.0,0.0,0.0,0.0,(Z4lSz));}else{_2E29->fCxBi[_2E29->djbCD].
jGQqQ(0.0,0.0,JRB5m,TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,
JRB5m,(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,0.0,(Z4lSz));}_2E29
->FfhZi[_2E29->djbCD]=(Z4lSz)+LOUhW;_2E29->djbCD++;
Z4lSz+=(LOUhW);TfLg_+=mpXRu;JRB5m+=T5yjB;jEwtH=(0x12c6+4001-0x2267);

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
_2E29->djbCD]=(Z4lSz)+tlPiC;_2E29->djbCD++;return;}

void qPN_6::XYtRl(const double&rTEdP,const double&SynchronizationTime,const 
double&iPzPj,const double&ldeCA,const double&AQzqu,const double&HcINC,const 
double&k6sf4,const double&DAwhk,const double&b4jXr,XkwFr*_2E29,const bool&U7RiH,
bool*DBRdu){double ifRIn=0.0,TfLg_=iPzPj,EgDTv=ldeCA,JRB5m=AQzqu,EI730=HcINC,
jEwtH=k6sf4,Z4lSz=rTEdP,Kaclx=0.0,EvE6x=0.0,FvoS1=0.0,LOUhW=0.0,Qogm3=0.0,CCBY_=
0.0,X6X_j=0.0,xek11=0.0,ZaFFa=0.0,_HBf3=0.0,T5yjB=0.0,OLj8b=0.0,CnAKY=0.0,oxEkR=
0.0,Ixvmb=0.0,zlPa_=0.0,g7nwu=0.0,mpXRu=0.0,xHUUb=0.0,_qvll=0.0,deZtJ=0.0,hxN2g=
0.0,jE9zt=0.0,fjgkf=0.0,jfI7z=0.0,gvv1d=0.0,AF2wW=0.0,Yoeil=0.0,rDoKh=0.0,XLg6q=
0.0,inQml=0.0,D0ol4=0.0;

if(jEwtH>DAwhk){jEwtH=DAwhk;}Yoeil=SynchronizationTime-Z4lSz;ifRIn=Q8oIQ(&hBNRM,
&Cjc4b,TfLg_,EgDTv,JRB5m,EI730,jEwtH,DAwhk,b4jXr,Yoeil);

EvE6x=ifRIn;if(EvE6x<0.0){EvE6x=0.0;}
Kaclx=(DAwhk-jEwtH)/b4jXr;xek11=jEwtH*Kaclx+0.5*b4jXr*VNvlY(Kaclx);Ixvmb=JRB5m*
Kaclx+0.5*jEwtH*VNvlY(Kaclx)+b4jXr*qn3x8(Kaclx)/6.0;
ZaFFa=EvE6x*DAwhk;zlPa_=(JRB5m+xek11)*EvE6x+0.5*DAwhk*VNvlY(EvE6x);
FvoS1=DAwhk/b4jXr;_HBf3=0.5*VNvlY(DAwhk)/b4jXr;g7nwu=(JRB5m+xek11+ZaFFa)*FvoS1+
0.5*DAwhk*VNvlY(FvoS1)-b4jXr*qn3x8(FvoS1)/6.0;

T5yjB=0.0;

Qogm3=FvoS1;OLj8b=_HBf3;xHUUb=(JRB5m+xek11+ZaFFa+_HBf3+T5yjB)*Qogm3+b4jXr*qn3x8(
Qogm3)/6.0;
X6X_j=FvoS1;oxEkR=_HBf3;

CnAKY=EI730-JRB5m-xek11-ZaFFa-_HBf3-T5yjB-OLj8b-oxEkR;if(CnAKY<0.0){CnAKY=0.0;}
CCBY_=CnAKY/DAwhk;_qvll=(JRB5m+xek11+ZaFFa+_HBf3+T5yjB+OLj8b)*CCBY_+0.5*DAwhk*
VNvlY(CCBY_);
deZtJ=(JRB5m+xek11+ZaFFa+_HBf3+T5yjB+OLj8b+CnAKY)*X6X_j+0.5*DAwhk*VNvlY(X6X_j)-
b4jXr*qn3x8(X6X_j)/6.0;
mpXRu=EgDTv-TfLg_-Ixvmb-zlPa_-g7nwu-xHUUb-_qvll-deZtJ;if((fabs(JRB5m+xek11+ZaFFa
+_HBf3)<Nm0g6)||(fabs(mpXRu)<Nm0g6)){LOUhW=SynchronizationTime-Z4lSz-Kaclx-EvE6x
-FvoS1-Qogm3-CCBY_-X6X_j;}else{rDoKh=(JRB5m+xek11+ZaFFa+_HBf3);if(rDoKh==0.0){
LOUhW=SynchronizationTime-Z4lSz-Kaclx-EvE6x-FvoS1-Qogm3-CCBY_-X6X_j;}else{LOUhW=
mpXRu/rDoKh;}}if(LOUhW<0.0){LOUhW=0.0;}


if(m85bi(JRB5m)==m85bi(JRB5m+xek11)){XLg6q+=fabs(Ixvmb);}else{D0ol4=-jEwtH/b4jXr
+pbQOc(VNvlY(jEwtH/b4jXr)-2.0*JRB5m/b4jXr);if(D0ol4<0.0){D0ol4=0.0;}if(D0ol4>
Kaclx){D0ol4=Kaclx;}XLg6q+=fabs(JRB5m*D0ol4+0.5*jEwtH*VNvlY(D0ol4)+b4jXr*qn3x8(
D0ol4)/6.0)+fabs(0.5*(jEwtH-b4jXr*D0ol4)*VNvlY(Kaclx-D0ol4)+b4jXr*qn3x8(Kaclx-
D0ol4)/6.0);}
if(m85bi(JRB5m+xek11)==m85bi(JRB5m+xek11+ZaFFa)){XLg6q+=fabs(zlPa_);}else{D0ol4=
fabs((JRB5m+xek11)/DAwhk);XLg6q+=fabs(0.5*DAwhk*VNvlY(D0ol4))+fabs(0.5*DAwhk*
VNvlY(EvE6x-D0ol4));}
if(m85bi(JRB5m+xek11+ZaFFa)==m85bi(JRB5m+xek11+ZaFFa+_HBf3)){XLg6q+=fabs(g7nwu);
}else{D0ol4=DAwhk/b4jXr-pbQOc(VNvlY(DAwhk/b4jXr)+2.0*(JRB5m+xek11+ZaFFa)/b4jXr);
if(D0ol4<0.0){D0ol4=0.0;}if(D0ol4>FvoS1){D0ol4=FvoS1;}XLg6q+=fabs((JRB5m+xek11+
ZaFFa)*D0ol4+0.5*DAwhk*VNvlY(D0ol4)-b4jXr*qn3x8(D0ol4)/6.0)+fabs(0.5*(DAwhk-
b4jXr*D0ol4)*VNvlY(FvoS1-D0ol4)-b4jXr*qn3x8(FvoS1-D0ol4)/6.0);}
XLg6q+=fabs(mpXRu);
if(m85bi(JRB5m+xek11+ZaFFa+_HBf3+T5yjB)==m85bi(JRB5m+xek11+ZaFFa+_HBf3+T5yjB+
OLj8b)){XLg6q+=fabs(xHUUb);}else{D0ol4=pbQOc(-2.0*(JRB5m+xek11+ZaFFa+_HBf3+T5yjB
)/b4jXr);if(D0ol4>Qogm3){D0ol4=Qogm3;}XLg6q+=fabs((JRB5m+xek11+ZaFFa+_HBf3+T5yjB
)*D0ol4+b4jXr*qn3x8(D0ol4)/6.0)+fabs(0.5*(b4jXr*D0ol4)*VNvlY(Qogm3-D0ol4)+b4jXr*
qn3x8(Qogm3-D0ol4)/6.0);}
if(m85bi(JRB5m+xek11+ZaFFa+_HBf3+T5yjB+OLj8b)==m85bi(JRB5m+xek11+ZaFFa+_HBf3+
T5yjB+OLj8b+CnAKY)){XLg6q+=fabs(_qvll);}else{D0ol4=fabs((JRB5m+xek11+ZaFFa+_HBf3
+T5yjB+OLj8b)/DAwhk);XLg6q+=fabs(0.5*DAwhk*VNvlY(D0ol4))+fabs(0.5*DAwhk*VNvlY(
CCBY_-D0ol4));}
if(m85bi(JRB5m+xek11+ZaFFa+_HBf3+T5yjB+OLj8b+CnAKY)==m85bi(EI730)){XLg6q+=fabs(
deZtJ);}else{D0ol4=DAwhk/b4jXr-pbQOc(VNvlY(DAwhk/b4jXr)+2.0*(JRB5m+xek11+ZaFFa+
_HBf3+T5yjB+OLj8b+CnAKY)/b4jXr);if(D0ol4<0.0){D0ol4=0.0;}if(D0ol4>X6X_j){D0ol4=
X6X_j;}XLg6q+=fabs((JRB5m+xek11+ZaFFa+_HBf3+T5yjB+OLj8b+CnAKY)*D0ol4+0.5*DAwhk*
VNvlY(D0ol4)-b4jXr*qn3x8(D0ol4)/6.0)+fabs(0.5*(DAwhk-b4jXr*D0ol4)*VNvlY(X6X_j-
D0ol4)-b4jXr*qn3x8(X6X_j-D0ol4)/6.0);}
inQml=fabs(xek11)+fabs(ZaFFa)+fabs(_HBf3)+fabs(T5yjB)+fabs(OLj8b)+fabs(CnAKY)+
fabs(oxEkR);


AF2wW=Kaclx+EvE6x+FvoS1+LOUhW+Qogm3+CCBY_+X6X_j;jfI7z=Ixvmb+zlPa_+g7nwu+mpXRu+
xHUUb+_qvll+deZtJ;gvv1d=xek11+ZaFFa+_HBf3+T5yjB+OLj8b+CnAKY+oxEkR;if(AF2wW!=0.0)
{fjgkf=SynchronizationTime-Z4lSz-AF2wW;if(fabs(fjgkf)>(kED7o*(
SynchronizationTime-Z4lSz)+PZBAL)){*DBRdu=true;}}if(jfI7z!=0.0){hxN2g=EgDTv-
TfLg_-jfI7z;if(fabs(hxN2g)>(fabs(Cmmsw*XLg6q)+oUhjD)){*DBRdu=true;}}if(gvv1d!=
0.0){jE9zt=EI730-JRB5m-gvv1d;if(fabs(jE9zt)>(c_4Sr*fabs((fabs(EI730)>fabs(inQml)
)?(EI730):(inQml)))+Uq_ZW){*DBRdu=true;}}



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
Z4lSz+=(FvoS1);TfLg_+=g7nwu;JRB5m+=_HBf3;jEwtH=(0x232b+900-0x26af);

if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0,0.0,(-JRB5m),(-TfLg_),(Z4lSz));
_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,(-JRB5m),(Z4lSz));_2E29->NYWGt[
_2E29->djbCD].jGQqQ(0.0,0.0,0.0,0.0,(Z4lSz));}else{_2E29->fCxBi[_2E29->djbCD].
jGQqQ(0.0,0.0,JRB5m,TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,
JRB5m,(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,0.0,(Z4lSz));}_2E29
->FfhZi[_2E29->djbCD]=(Z4lSz)+LOUhW;_2E29->djbCD++;
Z4lSz+=(LOUhW);TfLg_+=mpXRu;JRB5m+=T5yjB;jEwtH=(0xeb+4455-0x1252);

if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),0.0,(-JRB5m),(-TfLg_),
(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*(-b4jXr)),0.0,(-JRB5m),(Z4lSz
));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,-b4jXr,0.0,(Z4lSz));}else{_2E29->
fCxBi[_2E29->djbCD].jGQqQ(((b4jXr)/6.0),0.0,JRB5m,TfLg_,(Z4lSz));_2E29->diBqY[
_2E29->djbCD].jGQqQ(0.0,(0.5*(b4jXr)),0.0,JRB5m,(Z4lSz));_2E29->NYWGt[_2E29->
djbCD].jGQqQ(0.0,0.0,b4jXr,0.0,(Z4lSz));}_2E29->FfhZi[_2E29->djbCD]=(Z4lSz)+
Qogm3;_2E29->djbCD++;
Z4lSz+=(Qogm3);TfLg_+=xHUUb;JRB5m+=OLj8b;jEwtH=DAwhk;

if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0,(0.5*(-DAwhk)),-JRB5m,-TfLg_,(
Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,-DAwhk,-JRB5m,(Z4lSz));_2E29->
NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,-DAwhk,(Z4lSz));}else{_2E29->fCxBi[_2E29->
djbCD].jGQqQ(0.0,(0.5*DAwhk),JRB5m,TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].
jGQqQ(0.0,0.0,DAwhk,JRB5m,(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,
DAwhk,(Z4lSz));}_2E29->FfhZi[_2E29->djbCD]=(Z4lSz)+CCBY_;_2E29->djbCD++;
Z4lSz+=(CCBY_);TfLg_+=_qvll;JRB5m+=CnAKY;jEwtH=DAwhk;

if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ((b4jXr/6.0),(0.5*(-DAwhk)),-JRB5m,-
TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*b4jXr),-DAwhk,-JRB5m,(
Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,b4jXr,-DAwhk,(Z4lSz));}else{
_2E29->fCxBi[_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),(0.5*DAwhk),JRB5m,TfLg_,(Z4lSz))
;_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*(-b4jXr)),DAwhk,JRB5m,(Z4lSz));_2E29
->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,-b4jXr,DAwhk,(Z4lSz));}_2E29->FfhZi[_2E29->
djbCD]=(Z4lSz)+X6X_j;_2E29->djbCD++;
Z4lSz+=(X6X_j);TfLg_+=deZtJ;JRB5m+=oxEkR;jEwtH=0.0;
if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0,0.0,-JRB5m,-TfLg_,(Z4lSz));_2E29
->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,-JRB5m,(Z4lSz));_2E29->NYWGt[_2E29->
djbCD].jGQqQ(0.0,0.0,0.0,0.0,(Z4lSz));}else{_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0
,0.0,JRB5m,TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,JRB5m,(
Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,0.0,(Z4lSz));}_2E29->FfhZi[
_2E29->djbCD]=(Z4lSz)+tlPiC;_2E29->djbCD++;return;}

void qPN_6::UCfFY(const double&rTEdP,const double&SynchronizationTime,const 
double&iPzPj,const double&ldeCA,const double&AQzqu,const double&HcINC,const 
double&k6sf4,const double&DAwhk,const double&b4jXr,XkwFr*_2E29,const bool&U7RiH,
bool*DBRdu){double ifRIn=0.0,TfLg_=iPzPj,EgDTv=ldeCA,JRB5m=AQzqu,EI730=HcINC,
jEwtH=k6sf4,Z4lSz=rTEdP,RIZdZ=0.0,Kaclx=0.0,EvE6x=0.0,FvoS1=0.0,LOUhW=0.0,Qogm3=
0.0,CCBY_=0.0,xek11=0.0,ZaFFa=0.0,_HBf3=0.0,T5yjB=0.0,OLj8b=0.0,CnAKY=0.0,Ixvmb=
0.0,zlPa_=0.0,g7nwu=0.0,mpXRu=0.0,xHUUb=0.0,_qvll=0.0,hxN2g=0.0,jE9zt=0.0,fjgkf=
0.0,jfI7z=0.0,gvv1d=0.0,AF2wW=0.0,Yoeil=0.0,rDoKh=0.0,XLg6q=0.0,inQml=0.0,D0ol4=
0.0;

if(jEwtH>DAwhk){jEwtH=DAwhk;}Yoeil=SynchronizationTime-Z4lSz;ifRIn=Q8oIQ(&lWVnt,
&Ck2un,TfLg_,EgDTv,JRB5m,EI730,jEwtH,DAwhk,b4jXr,Yoeil);

RIZdZ=ifRIn;if(RIZdZ<0.0){RIZdZ=0.0;}
Kaclx=(DAwhk-jEwtH)/b4jXr;xek11=jEwtH*Kaclx+0.5*b4jXr*VNvlY(Kaclx);Ixvmb=JRB5m*
Kaclx+0.5*jEwtH*VNvlY(Kaclx)+b4jXr*qn3x8(Kaclx)/6.0;
FvoS1=DAwhk/b4jXr;_HBf3=0.5*VNvlY(DAwhk)/b4jXr;


T5yjB=0.0;

Qogm3=RIZdZ/b4jXr;OLj8b=0.5*VNvlY(RIZdZ)/b4jXr;

CCBY_=Qogm3;CnAKY=OLj8b;

ZaFFa=EI730-JRB5m-xek11-_HBf3-T5yjB-OLj8b-CnAKY;if(ZaFFa<0.0){ZaFFa=0.0;}EvE6x=
ZaFFa/DAwhk;zlPa_=(JRB5m+xek11)*EvE6x+0.5*DAwhk*VNvlY(EvE6x);
g7nwu=(JRB5m+xek11+ZaFFa)*FvoS1+0.5*DAwhk*VNvlY(FvoS1)-b4jXr*qn3x8(FvoS1)/6.0;
xHUUb=(JRB5m+xek11+ZaFFa+_HBf3+T5yjB)*Qogm3+b4jXr*qn3x8(Qogm3)/6.0;
_qvll=(JRB5m+xek11+ZaFFa+_HBf3+T5yjB+OLj8b)*CCBY_+0.5*RIZdZ*VNvlY(CCBY_)-b4jXr*
qn3x8(CCBY_)/6.0;
mpXRu=EgDTv-TfLg_-Ixvmb-zlPa_-g7nwu-xHUUb-_qvll;if((fabs(JRB5m+xek11+ZaFFa+_HBf3
)<Nm0g6)||(fabs(mpXRu)<Nm0g6)){LOUhW=SynchronizationTime-Z4lSz-Kaclx-EvE6x-FvoS1
-Qogm3-CCBY_;}else{rDoKh=(JRB5m+xek11+ZaFFa+_HBf3);if(rDoKh==0.0){LOUhW=
SynchronizationTime-Z4lSz-Kaclx-EvE6x-FvoS1-Qogm3-CCBY_;}else{LOUhW=mpXRu/rDoKh;
}}if(LOUhW<0.0){LOUhW=0.0;}


if(m85bi(JRB5m)==m85bi(JRB5m+xek11)){XLg6q+=fabs(Ixvmb);}else{D0ol4=-jEwtH/b4jXr
+pbQOc(VNvlY(jEwtH/b4jXr)-2.0*JRB5m/b4jXr);if(D0ol4<0.0){D0ol4=0.0;}if(D0ol4>
Kaclx){D0ol4=Kaclx;}XLg6q+=fabs(JRB5m*D0ol4+0.5*jEwtH*VNvlY(D0ol4)+b4jXr*qn3x8(
D0ol4)/6.0)+fabs(0.5*(jEwtH-b4jXr*D0ol4)*VNvlY(Kaclx-D0ol4)+b4jXr*qn3x8(Kaclx-
D0ol4)/6.0);}
if(m85bi(JRB5m+xek11)==m85bi(JRB5m+xek11+ZaFFa)){XLg6q+=fabs(zlPa_);}else{D0ol4=
fabs((JRB5m+xek11)/DAwhk);XLg6q+=fabs(0.5*DAwhk*VNvlY(D0ol4))+fabs(0.5*DAwhk*
VNvlY(EvE6x-D0ol4));}
if(m85bi(JRB5m+xek11+ZaFFa)==m85bi(JRB5m+xek11+ZaFFa+_HBf3)){XLg6q+=fabs(g7nwu);
}else{D0ol4=DAwhk/b4jXr-pbQOc(VNvlY(DAwhk/b4jXr)+2.0*(JRB5m+xek11+ZaFFa)/b4jXr);
if(D0ol4<0.0){D0ol4=0.0;}if(D0ol4>FvoS1){D0ol4=FvoS1;}XLg6q+=fabs((JRB5m+xek11+
ZaFFa)*D0ol4+0.5*DAwhk*VNvlY(D0ol4)-b4jXr*qn3x8(D0ol4)/6.0)+fabs(0.5*(DAwhk-
b4jXr*D0ol4)*VNvlY(FvoS1-D0ol4)-b4jXr*qn3x8(FvoS1-D0ol4)/6.0);}
XLg6q+=fabs(mpXRu);
if(m85bi(JRB5m+xek11+ZaFFa+_HBf3+T5yjB)==m85bi(JRB5m+xek11+ZaFFa+_HBf3+T5yjB+
OLj8b)){XLg6q+=fabs(xHUUb);}else{D0ol4=pbQOc(-2.0*(JRB5m+xek11+ZaFFa+_HBf3+T5yjB
)/b4jXr);if(D0ol4>Qogm3){D0ol4=Qogm3;}XLg6q+=fabs((JRB5m+xek11+ZaFFa+_HBf3+T5yjB
)*D0ol4+b4jXr*qn3x8(D0ol4)/6.0)+fabs(0.5*(b4jXr*D0ol4)*VNvlY(Qogm3-D0ol4)+b4jXr*
qn3x8(Qogm3-D0ol4)/6.0);}
if(m85bi(JRB5m+xek11+ZaFFa+_HBf3+T5yjB+OLj8b)==m85bi(EI730)){XLg6q+=fabs(_qvll);
}else{D0ol4=RIZdZ/b4jXr-pbQOc(VNvlY(RIZdZ/b4jXr)+2.0*(JRB5m+xek11+ZaFFa+_HBf3+
T5yjB+OLj8b+CnAKY)/b4jXr);if(D0ol4<0.0){D0ol4=0.0;}if(D0ol4>CCBY_){D0ol4=CCBY_;}
XLg6q+=fabs((JRB5m+xek11+ZaFFa+_HBf3+T5yjB+OLj8b+CnAKY)*D0ol4+0.5*RIZdZ*VNvlY(
D0ol4)-b4jXr*qn3x8(D0ol4)/6.0)+fabs(0.5*(RIZdZ-b4jXr*D0ol4)*VNvlY(CCBY_-D0ol4)-
b4jXr*qn3x8(CCBY_-D0ol4)/6.0);}
inQml=fabs(xek11)+fabs(ZaFFa)+fabs(_HBf3)+fabs(T5yjB)+fabs(OLj8b)+fabs(CnAKY);


AF2wW=Kaclx+EvE6x+FvoS1+LOUhW+Qogm3+CCBY_;jfI7z=Ixvmb+zlPa_+g7nwu+mpXRu+xHUUb+
_qvll;gvv1d=xek11+ZaFFa+_HBf3+T5yjB+OLj8b+CnAKY;if(AF2wW!=0.0){fjgkf=
SynchronizationTime-Z4lSz-AF2wW;if(fabs(fjgkf)>(kED7o*(SynchronizationTime-Z4lSz
)+PZBAL)){*DBRdu=true;}}if(jfI7z!=0.0){hxN2g=EgDTv-TfLg_-jfI7z;if(fabs(hxN2g)>(
fabs(Cmmsw*XLg6q)+oUhjD)){*DBRdu=true;}}if(gvv1d!=0.0){jE9zt=EI730-JRB5m-gvv1d;
if(fabs(jE9zt)>(c_4Sr*fabs((fabs(EI730)>fabs(inQml))?(EI730):(inQml)))+Uq_ZW){*
DBRdu=true;}}



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
Z4lSz+=(FvoS1);TfLg_+=g7nwu;JRB5m+=_HBf3;jEwtH=(0x134b+1177-0x17e4);

if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0,0.0,(-JRB5m),(-TfLg_),(Z4lSz));
_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,(-JRB5m),(Z4lSz));_2E29->NYWGt[
_2E29->djbCD].jGQqQ(0.0,0.0,0.0,0.0,(Z4lSz));}else{_2E29->fCxBi[_2E29->djbCD].
jGQqQ(0.0,0.0,JRB5m,TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,
JRB5m,(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,0.0,(Z4lSz));}_2E29
->FfhZi[_2E29->djbCD]=(Z4lSz)+LOUhW;_2E29->djbCD++;
Z4lSz+=(LOUhW);TfLg_+=mpXRu;JRB5m+=T5yjB;jEwtH=(0x7da+4342-0x18d0);

if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),0.0,(-JRB5m),(-TfLg_),
(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*(-b4jXr)),0.0,(-JRB5m),(Z4lSz
));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,-b4jXr,0.0,(Z4lSz));}else{_2E29->
fCxBi[_2E29->djbCD].jGQqQ((b4jXr/6.0),0.0,JRB5m,TfLg_,(Z4lSz));_2E29->diBqY[
_2E29->djbCD].jGQqQ(0.0,(0.5*b4jXr),0.0,JRB5m,(Z4lSz));_2E29->NYWGt[_2E29->djbCD
].jGQqQ(0.0,0.0,b4jXr,0.0,(Z4lSz));}_2E29->FfhZi[_2E29->djbCD]=(Z4lSz)+Qogm3;
_2E29->djbCD++;
Z4lSz+=(Qogm3);TfLg_+=xHUUb;JRB5m+=OLj8b;jEwtH=RIZdZ;

if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ((b4jXr/6.0),(0.5*(-RIZdZ)),-JRB5m,-
TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*b4jXr),-RIZdZ,-JRB5m,(
Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,b4jXr,-RIZdZ,(Z4lSz));}else{
_2E29->fCxBi[_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),(0.5*RIZdZ),JRB5m,TfLg_,(Z4lSz))
;_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*(-b4jXr)),RIZdZ,JRB5m,(Z4lSz));_2E29
->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,-b4jXr,RIZdZ,(Z4lSz));}_2E29->FfhZi[_2E29->
djbCD]=(Z4lSz)+CCBY_;_2E29->djbCD++;
Z4lSz+=(CCBY_);TfLg_+=_qvll;JRB5m+=CnAKY;jEwtH=0.0;
if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0,0.0,-JRB5m,-TfLg_,(Z4lSz));_2E29
->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,-JRB5m,(Z4lSz));_2E29->NYWGt[_2E29->
djbCD].jGQqQ(0.0,0.0,0.0,0.0,(Z4lSz));}else{_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0
,0.0,JRB5m,TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,JRB5m,(
Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,0.0,(Z4lSz));}_2E29->FfhZi[
_2E29->djbCD]=(Z4lSz)+tlPiC;_2E29->djbCD++;return;}

void qPN_6::wFe1S(const double&rTEdP,const double&SynchronizationTime,const 
double&iPzPj,const double&ldeCA,const double&AQzqu,const double&HcINC,const 
double&k6sf4,const double&DAwhk,const double&b4jXr,XkwFr*_2E29,const bool&U7RiH,
bool*DBRdu){double ifRIn=0.0,TfLg_=iPzPj,EgDTv=ldeCA,JRB5m=AQzqu,EI730=HcINC,
jEwtH=k6sf4,Z4lSz=rTEdP,wmBRB=0.0,JWb5x=0.0,Kaclx=0.0,EvE6x=0.0,FvoS1=0.0,LOUhW=
0.0,xek11=0.0,ZaFFa=0.0,_HBf3=0.0,T5yjB=0.0,Ixvmb=0.0,zlPa_=0.0,g7nwu=0.0,mpXRu=
0.0,hxN2g=0.0,jE9zt=0.0,fjgkf=0.0,jfI7z=0.0,gvv1d=0.0,AF2wW=0.0,Yoeil=0.0,XLg6q=
0.0,inQml=0.0,D0ol4=0.0;

if(jEwtH>DAwhk){jEwtH=DAwhk;}Yoeil=SynchronizationTime-Z4lSz;ifRIn=Q8oIQ(&WqY2Q,
&e4nH0,TfLg_,EgDTv,JRB5m,EI730,jEwtH,DAwhk,b4jXr,Yoeil);


wmBRB=ifRIn;if(wmBRB<jEwtH){wmBRB=jEwtH;}
Kaclx=(wmBRB-jEwtH)/b4jXr;xek11=jEwtH*Kaclx+0.5*b4jXr*VNvlY(Kaclx);Ixvmb=JRB5m*
Kaclx+0.5*jEwtH*VNvlY(Kaclx)+b4jXr*qn3x8(Kaclx)/6.0;if(fabs(jEwtH-2.0*wmBRB-
b4jXr*(Z4lSz)+b4jXr*SynchronizationTime)<MFWCZ){

JWb5x=0.0;}else{
JWb5x=(b4jXr*((1.0*VNvlY(jEwtH))/b4jXr-(2.0*VNvlY(wmBRB))/b4jXr-2.0*JRB5m+2.0*
EI730))/(2.0*jEwtH-4.0*wmBRB-2.0*b4jXr*(Z4lSz)+2.0*b4jXr*SynchronizationTime);if
(JWb5x<0.0){JWb5x=T68ug;}if(JWb5x>wmBRB){JWb5x=wmBRB;}}
EvE6x=(wmBRB-JWb5x)/b4jXr;ZaFFa=wmBRB*EvE6x-0.5*b4jXr*VNvlY(EvE6x);zlPa_=(JRB5m+
xek11)*EvE6x+0.5*wmBRB*VNvlY(EvE6x)-b4jXr*qn3x8(EvE6x)/6.0;
LOUhW=JWb5x/b4jXr;T5yjB=0.5*VNvlY(JWb5x)/b4jXr;

_HBf3=EI730-JRB5m-xek11-ZaFFa-T5yjB;if(_HBf3<0.0){_HBf3=0.0;}if(JWb5x<MFWCZ){
FvoS1=Yoeil-(2.0*wmBRB-jEwtH)/b4jXr;}else{FvoS1=_HBf3/JWb5x;}if(FvoS1<0.0){FvoS1
=0.0;}g7nwu=(JRB5m+xek11+ZaFFa)*FvoS1+0.5*JWb5x*VNvlY(FvoS1);
mpXRu=(JRB5m+xek11+ZaFFa+_HBf3)*LOUhW+0.5*JWb5x*VNvlY(LOUhW)-b4jXr*qn3x8(LOUhW)/
6.0;


if(m85bi(JRB5m)==m85bi(JRB5m+xek11)){XLg6q+=fabs(Ixvmb);}else{D0ol4=-jEwtH/b4jXr
+pbQOc(VNvlY(jEwtH/b4jXr)-2.0*JRB5m/b4jXr);if(D0ol4<0.0){D0ol4=0.0;}if(D0ol4>
Kaclx){D0ol4=Kaclx;}XLg6q+=fabs(JRB5m*D0ol4+0.5*jEwtH*VNvlY(D0ol4)+b4jXr*qn3x8(
D0ol4)/6.0)+fabs(0.5*(jEwtH-b4jXr*D0ol4)*VNvlY(Kaclx-D0ol4)+b4jXr*qn3x8(Kaclx-
D0ol4)/6.0);}
if(m85bi(JRB5m+xek11)==m85bi(JRB5m+xek11+ZaFFa)){XLg6q+=fabs(zlPa_);}else{D0ol4=
wmBRB/b4jXr-pbQOc(VNvlY(wmBRB/b4jXr)+2.0*(JRB5m+xek11)/b4jXr);if(D0ol4<0.0){
D0ol4=0.0;}if(D0ol4>EvE6x){D0ol4=EvE6x;}XLg6q+=fabs((JRB5m+xek11)*D0ol4+0.5*
wmBRB*VNvlY(D0ol4)-b4jXr*qn3x8(D0ol4)/6.0)+fabs(0.5*(wmBRB-b4jXr*D0ol4)*VNvlY(
EvE6x-D0ol4)-b4jXr*qn3x8(EvE6x-D0ol4)/6.0);}
if(m85bi(JRB5m+xek11+ZaFFa)==m85bi(JRB5m+xek11+ZaFFa+_HBf3)){XLg6q+=fabs(g7nwu);
}else{D0ol4=fabs((JRB5m+xek11+ZaFFa)/JWb5x);XLg6q+=fabs(0.5*JWb5x*VNvlY(D0ol4))+
fabs(0.5*JWb5x*VNvlY(FvoS1-D0ol4));}
if(m85bi(JRB5m+xek11+ZaFFa+_HBf3)==m85bi(EI730)){XLg6q+=fabs(mpXRu);}else{D0ol4=
JWb5x/b4jXr-pbQOc(VNvlY(JWb5x/b4jXr)+2.0*(JRB5m+xek11+ZaFFa+_HBf3)/b4jXr);if(
D0ol4<0.0){D0ol4=0.0;}if(D0ol4>LOUhW){D0ol4=LOUhW;}XLg6q+=fabs((JRB5m+xek11+
ZaFFa+_HBf3)*D0ol4+0.5*JWb5x*VNvlY(D0ol4)-b4jXr*qn3x8(D0ol4)/6.0)+fabs(0.5*(
JWb5x-b4jXr*D0ol4)*VNvlY(LOUhW-D0ol4)-b4jXr*qn3x8(LOUhW-D0ol4)/6.0);}
inQml=fabs(xek11)+fabs(ZaFFa)+fabs(_HBf3)+fabs(T5yjB);


AF2wW=Kaclx+EvE6x+FvoS1+LOUhW;jfI7z=Ixvmb+zlPa_+g7nwu+mpXRu;gvv1d=fabs(xek11)+
fabs(ZaFFa)+fabs(_HBf3)+fabs(T5yjB);if(AF2wW!=0.0){fjgkf=SynchronizationTime-
Z4lSz-AF2wW;if(fabs(fjgkf)>(kED7o*(SynchronizationTime-Z4lSz)+PZBAL)){*DBRdu=
true;}}if(jfI7z!=0.0){hxN2g=EgDTv-TfLg_-jfI7z;if(fabs(hxN2g)>(fabs(Cmmsw*XLg6q)+
oUhjD)){*DBRdu=true;}}if(gvv1d!=0.0){jE9zt=EI730-JRB5m-gvv1d;if(fabs(jE9zt)>(
c_4Sr*fabs((fabs(EI730)>fabs(inQml))?(EI730):(inQml)))+Uq_ZW){*DBRdu=true;}}



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
Z4lSz+=(EvE6x);TfLg_+=zlPa_;JRB5m+=ZaFFa;jEwtH=JWb5x;


if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0,(-0.5*JWb5x),(-JRB5m),(-TfLg_),(
Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,-JWb5x,(-JRB5m),(Z4lSz));_2E29
->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,-JWb5x,(Z4lSz));}else{_2E29->fCxBi[_2E29
->djbCD].jGQqQ(0.0,(0.5*JWb5x),JRB5m,TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].
jGQqQ(0.0,0.0,JWb5x,JRB5m,(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,
JWb5x,(Z4lSz));}_2E29->FfhZi[_2E29->djbCD]=(Z4lSz)+FvoS1;_2E29->djbCD++;
Z4lSz+=(FvoS1);TfLg_+=g7nwu;JRB5m+=_HBf3;jEwtH=JWb5x;


if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ((b4jXr/6.0),(-0.5*JWb5x),-JRB5m,-
TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*b4jXr),-JWb5x,-JRB5m,(
Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,b4jXr,-JWb5x,(Z4lSz));}else{
_2E29->fCxBi[_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),(0.5*JWb5x),JRB5m,TfLg_,(Z4lSz))
;_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*(-b4jXr)),JWb5x,JRB5m,(Z4lSz));_2E29
->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,-b4jXr,JWb5x,(Z4lSz));}_2E29->FfhZi[_2E29->
djbCD]=(Z4lSz)+LOUhW;_2E29->djbCD++;
Z4lSz+=(LOUhW);TfLg_+=mpXRu;JRB5m+=T5yjB;jEwtH=0.0;
if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0,0.0,-JRB5m,-TfLg_,(Z4lSz));_2E29
->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,-JRB5m,(Z4lSz));_2E29->NYWGt[_2E29->
djbCD].jGQqQ(0.0,0.0,0.0,0.0,(Z4lSz));}else{_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0
,0.0,JRB5m,TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,JRB5m,(
Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,0.0,(Z4lSz));}_2E29->FfhZi[
_2E29->djbCD]=(Z4lSz)+tlPiC;_2E29->djbCD++;return;}

void qPN_6::ww9Hd(const double&rTEdP,const double&SynchronizationTime,const 
double&iPzPj,const double&ldeCA,const double&AQzqu,const double&HcINC,const 
double&k6sf4,const double&DAwhk,const double&b4jXr,XkwFr*_2E29,const bool&U7RiH,
bool*DBRdu){double ifRIn=0.0,TfLg_=iPzPj,EgDTv=ldeCA,JRB5m=AQzqu,EI730=HcINC,
jEwtH=k6sf4,Z4lSz=rTEdP,wmBRB=0.0,Kaclx=0.0,EvE6x=0.0,FvoS1=0.0,LOUhW=0.0,Qogm3=
0.0,CCBY_=0.0,xek11=0.0,ZaFFa=0.0,_HBf3=0.0,T5yjB=0.0,OLj8b=0.0,CnAKY=0.0,Ixvmb=
0.0,zlPa_=0.0,g7nwu=0.0,mpXRu=0.0,xHUUb=0.0,_qvll=0.0,hxN2g=0.0,jE9zt=0.0,fjgkf=
0.0,jfI7z=0.0,gvv1d=0.0,AF2wW=0.0,Yoeil=0.0,rDoKh=0.0,XLg6q=0.0,inQml=0.0,D0ol4=
0.0;

if(jEwtH>DAwhk){jEwtH=DAwhk;}Yoeil=SynchronizationTime-Z4lSz;ifRIn=Q8oIQ(&Zn7EB,
&AW_QQ,TfLg_,EgDTv,JRB5m,EI730,jEwtH,DAwhk,b4jXr,Yoeil);


wmBRB=ifRIn;if(wmBRB<jEwtH){wmBRB=jEwtH;}
Kaclx=(wmBRB-jEwtH)/b4jXr;xek11=jEwtH*Kaclx+0.5*b4jXr*VNvlY(Kaclx);Ixvmb=JRB5m*
Kaclx+0.5*jEwtH*VNvlY(Kaclx)+b4jXr*qn3x8(Kaclx)/6.0;
EvE6x=wmBRB/b4jXr;ZaFFa=0.5*VNvlY(wmBRB)/b4jXr;zlPa_=(JRB5m+xek11)*EvE6x+0.5*
wmBRB*VNvlY(EvE6x)-b4jXr*qn3x8(EvE6x)/6.0;

_HBf3=0.0;

LOUhW=DAwhk/b4jXr;T5yjB=-0.5*VNvlY(DAwhk)/b4jXr;mpXRu=(JRB5m+xek11+ZaFFa+_HBf3)*
LOUhW-b4jXr*qn3x8(LOUhW)/6.0;
CCBY_=LOUhW;CnAKY=T5yjB;

OLj8b=EI730-JRB5m-xek11-ZaFFa-_HBf3-T5yjB-CnAKY;if(OLj8b>0.0){CnAKY=0.0;}Qogm3=
OLj8b/(-DAwhk);xHUUb=(JRB5m+xek11+ZaFFa+_HBf3+T5yjB)*Qogm3+0.5*(-DAwhk)*VNvlY(
Qogm3);
_qvll=(JRB5m+xek11+ZaFFa+_HBf3+T5yjB+OLj8b)*CCBY_+0.5*(-DAwhk)*VNvlY(CCBY_)+
b4jXr*qn3x8(CCBY_)/6.0;
g7nwu=EgDTv-TfLg_-Ixvmb-zlPa_-mpXRu-xHUUb-_qvll;if((fabs(JRB5m+xek11+ZaFFa)<
Nm0g6)||(fabs(g7nwu)<Nm0g6)){FvoS1=SynchronizationTime-Z4lSz-Kaclx-EvE6x-LOUhW-
Qogm3-CCBY_;}else{rDoKh=(JRB5m+xek11+ZaFFa);if(rDoKh==0.0){FvoS1=
SynchronizationTime-Z4lSz-Kaclx-EvE6x-LOUhW-Qogm3-CCBY_;}else{FvoS1=g7nwu/rDoKh;
}}if(FvoS1<0.0){FvoS1=0.0;}


if(m85bi(JRB5m)==m85bi(JRB5m+xek11)){XLg6q+=fabs(Ixvmb);}else{D0ol4=-jEwtH/b4jXr
+pbQOc(VNvlY(jEwtH/b4jXr)-2.0*JRB5m/b4jXr);if(D0ol4<0.0){D0ol4=0.0;}if(D0ol4>
Kaclx){D0ol4=Kaclx;}XLg6q+=fabs(JRB5m*D0ol4+0.5*jEwtH*VNvlY(D0ol4)+b4jXr*qn3x8(
D0ol4)/6.0)+fabs(0.5*(jEwtH-b4jXr*D0ol4)*VNvlY(Kaclx-D0ol4)+b4jXr*qn3x8(Kaclx-
D0ol4)/6.0);}
if(m85bi(JRB5m+xek11)==m85bi(JRB5m+xek11+ZaFFa)){XLg6q+=fabs(zlPa_);}else{D0ol4=
wmBRB/b4jXr-pbQOc(VNvlY(wmBRB/b4jXr)+2.0*(JRB5m+xek11)/b4jXr);if(D0ol4<0.0){
D0ol4=0.0;}if(D0ol4>EvE6x){D0ol4=EvE6x;}XLg6q+=fabs((JRB5m+xek11)*D0ol4+0.5*
wmBRB*VNvlY(D0ol4)-b4jXr*qn3x8(D0ol4)/6.0)+fabs(0.5*(wmBRB-b4jXr*D0ol4)*VNvlY(
EvE6x-D0ol4)-b4jXr*qn3x8(EvE6x-D0ol4)/6.0);}
XLg6q+=fabs(g7nwu);
if(m85bi(JRB5m+xek11+ZaFFa+_HBf3)==m85bi(JRB5m+xek11+ZaFFa+_HBf3+T5yjB)){XLg6q+=
fabs(mpXRu);}else{D0ol4=pbQOc(2.0*(JRB5m+xek11+ZaFFa+_HBf3)/b4jXr);if(D0ol4>
LOUhW){D0ol4=LOUhW;}XLg6q+=fabs((JRB5m+xek11+ZaFFa+_HBf3)*D0ol4-b4jXr*qn3x8(
D0ol4)/6.0)+fabs(0.5*(-b4jXr*D0ol4)*VNvlY(LOUhW-D0ol4)-b4jXr*qn3x8(LOUhW-D0ol4)/
6.0);}
if(m85bi(JRB5m+xek11+ZaFFa+_HBf3+T5yjB)==m85bi(JRB5m+xek11+ZaFFa+_HBf3+T5yjB+
OLj8b)){XLg6q+=fabs(xHUUb);}else{D0ol4=fabs((JRB5m+xek11+ZaFFa+_HBf3+T5yjB)/
DAwhk);XLg6q+=fabs(0.5*DAwhk*VNvlY(D0ol4))+fabs(0.5*DAwhk*VNvlY(Qogm3-D0ol4));}
if(m85bi(JRB5m+xek11+ZaFFa+_HBf3+T5yjB+OLj8b)==m85bi(EI730)){XLg6q+=fabs(_qvll);
}else{D0ol4=DAwhk/b4jXr-pbQOc(VNvlY(DAwhk/b4jXr)-2.0*(JRB5m+xek11+ZaFFa+_HBf3+
T5yjB+OLj8b)/b4jXr);if(D0ol4<0.0){D0ol4=0.0;}if(D0ol4>CCBY_){D0ol4=CCBY_;}XLg6q
+=fabs((JRB5m+xek11+ZaFFa+_HBf3+T5yjB+OLj8b)*D0ol4-0.5*DAwhk*VNvlY(D0ol4)+b4jXr*
qn3x8(D0ol4)/6.0)+fabs(0.5*(-DAwhk+b4jXr*D0ol4)*VNvlY(CCBY_-D0ol4)+b4jXr*qn3x8(
CCBY_-D0ol4)/6.0);}
inQml=fabs(xek11)+fabs(ZaFFa)+fabs(_HBf3)+fabs(T5yjB)+fabs(OLj8b)+fabs(CnAKY);


AF2wW=Kaclx+EvE6x+FvoS1+LOUhW+Qogm3+CCBY_;jfI7z=Ixvmb+zlPa_+g7nwu+mpXRu+xHUUb+
_qvll;gvv1d=xek11+ZaFFa+_HBf3+T5yjB+OLj8b+CnAKY;if(AF2wW!=0.0){fjgkf=
SynchronizationTime-Z4lSz-AF2wW;if(fabs(fjgkf)>(kED7o*(SynchronizationTime-Z4lSz
)+PZBAL)){*DBRdu=true;}}if(jfI7z!=0.0){hxN2g=EgDTv-TfLg_-jfI7z;if(fabs(hxN2g)>(
fabs(Cmmsw*XLg6q)+oUhjD)){*DBRdu=true;}}if(gvv1d!=0.0){jE9zt=EI730-JRB5m-gvv1d;
if(fabs(jE9zt)>(c_4Sr*fabs((fabs(EI730)>fabs(inQml))?(EI730):(inQml)))+Uq_ZW){*
DBRdu=true;}}



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
_2E29->djbCD]=(Z4lSz)+tlPiC;_2E29->djbCD++;return;}

void qPN_6::gOgPf(const double&rTEdP,const double&SynchronizationTime,const 
double&iPzPj,const double&ldeCA,const double&AQzqu,const double&HcINC,const 
double&k6sf4,const double&DAwhk,const double&b4jXr,XkwFr*_2E29,const bool&U7RiH,
bool*DBRdu){double ifRIn=0.0,TfLg_=iPzPj,EgDTv=ldeCA,JRB5m=AQzqu,EI730=HcINC,
jEwtH=k6sf4,Z4lSz=rTEdP,wmBRB=0.0,RIZdZ=0.0,Kaclx=0.0,EvE6x=0.0,FvoS1=0.0,LOUhW=
0.0,Qogm3=0.0,xek11=0.0,ZaFFa=0.0,_HBf3=0.0,T5yjB=0.0,OLj8b=0.0,Ixvmb=0.0,zlPa_=
0.0,g7nwu=0.0,mpXRu=0.0,xHUUb=0.0,hxN2g=0.0,jE9zt=0.0,fjgkf=0.0,jfI7z=0.0,gvv1d=
0.0,AF2wW=0.0,Yoeil=0.0,rDoKh=0.0,XLg6q=0.0,inQml=0.0,D0ol4=0.0;

if(jEwtH>DAwhk){jEwtH=DAwhk;}Yoeil=SynchronizationTime-Z4lSz;ifRIn=Q8oIQ(&viKDU,
&naN2E,TfLg_,EgDTv,JRB5m,EI730,jEwtH,DAwhk,b4jXr,Yoeil);


wmBRB=ifRIn;if(wmBRB<jEwtH){wmBRB=jEwtH;}
Kaclx=(wmBRB-jEwtH)/b4jXr;xek11=jEwtH*Kaclx+0.5*b4jXr*VNvlY(Kaclx);Ixvmb=JRB5m*
Kaclx+0.5*jEwtH*VNvlY(Kaclx)+b4jXr*qn3x8(Kaclx)/6.0;
EvE6x=wmBRB/b4jXr;ZaFFa=0.5*VNvlY(wmBRB)/b4jXr;zlPa_=(JRB5m+xek11)*EvE6x+0.5*
wmBRB*VNvlY(EvE6x)-b4jXr*qn3x8(EvE6x)/6.0;

_HBf3=0.0;

RIZdZ=-pbQOc((-EI730+JRB5m+xek11+ZaFFa+_HBf3)*b4jXr);
LOUhW=(-RIZdZ)/b4jXr;;T5yjB=-0.5*VNvlY(RIZdZ)/b4jXr;mpXRu=(JRB5m+xek11+ZaFFa+
_HBf3)*LOUhW-b4jXr*qn3x8(LOUhW)/6.0;
Qogm3=LOUhW;OLj8b=T5yjB;xHUUb=(JRB5m+xek11+ZaFFa+_HBf3+T5yjB)*Qogm3+0.5*(RIZdZ)*
VNvlY(Qogm3)+b4jXr*qn3x8(Qogm3)/6.0;
g7nwu=EgDTv-TfLg_-Ixvmb-zlPa_-mpXRu-xHUUb;if((fabs(JRB5m+xek11+ZaFFa)<Nm0g6)||(
fabs(g7nwu)<Nm0g6)){FvoS1=SynchronizationTime-Z4lSz-Kaclx-EvE6x-LOUhW-Qogm3;}
else{rDoKh=(JRB5m+xek11+ZaFFa);if(rDoKh==0.0){FvoS1=SynchronizationTime-Z4lSz-
Kaclx-EvE6x-LOUhW-Qogm3;}else{FvoS1=g7nwu/rDoKh;}}if(FvoS1<0.0){FvoS1=0.0;}


if(m85bi(JRB5m)==m85bi(JRB5m+xek11)){XLg6q+=fabs(Ixvmb);}else{D0ol4=-jEwtH/b4jXr
+pbQOc(VNvlY(jEwtH/b4jXr)-2.0*JRB5m/b4jXr);if(D0ol4<0.0){D0ol4=0.0;}if(D0ol4>
Kaclx){D0ol4=Kaclx;}XLg6q+=fabs(JRB5m*D0ol4+0.5*jEwtH*VNvlY(D0ol4)+b4jXr*qn3x8(
D0ol4)/6.0)+fabs(0.5*(jEwtH-b4jXr*D0ol4)*VNvlY(Kaclx-D0ol4)+b4jXr*qn3x8(Kaclx-
D0ol4)/6.0);}
if(m85bi(JRB5m+xek11)==m85bi(JRB5m+xek11+ZaFFa)){XLg6q+=fabs(zlPa_);}else{D0ol4=
wmBRB/b4jXr-pbQOc(VNvlY(wmBRB/b4jXr)+2.0*(JRB5m+xek11)/b4jXr);if(D0ol4<0.0){
D0ol4=0.0;}if(D0ol4>EvE6x){D0ol4=EvE6x;}XLg6q+=fabs((JRB5m+xek11)*D0ol4+0.5*
wmBRB*VNvlY(D0ol4)-b4jXr*qn3x8(D0ol4)/6.0)+fabs(0.5*(wmBRB-b4jXr*D0ol4)*VNvlY(
EvE6x-D0ol4)-b4jXr*qn3x8(EvE6x-D0ol4)/6.0);}
XLg6q+=fabs(g7nwu);
if(m85bi(JRB5m+xek11+ZaFFa+_HBf3)==m85bi(JRB5m+xek11+ZaFFa+_HBf3+T5yjB)){XLg6q+=
fabs(mpXRu);}else{D0ol4=pbQOc(2.0*(JRB5m+xek11+ZaFFa+_HBf3)/b4jXr);if(D0ol4>
LOUhW){D0ol4=LOUhW;}XLg6q+=fabs((JRB5m+xek11+ZaFFa+_HBf3)*D0ol4-b4jXr*qn3x8(
D0ol4)/6.0)+fabs(0.5*(-b4jXr*D0ol4)*VNvlY(LOUhW-D0ol4)-b4jXr*qn3x8(LOUhW-D0ol4)/
6.0);}
if(m85bi(JRB5m+xek11+ZaFFa+_HBf3+T5yjB)==m85bi(EI730)){XLg6q+=fabs(xHUUb);}else{
D0ol4=-RIZdZ/b4jXr-pbQOc(VNvlY(RIZdZ/b4jXr)-2.0*(JRB5m+xek11+ZaFFa+_HBf3+T5yjB)/
b4jXr);if(D0ol4<0.0){D0ol4=0.0;}if(D0ol4>Qogm3){D0ol4=Qogm3;}XLg6q+=fabs((JRB5m+
xek11+ZaFFa+_HBf3+T5yjB)*D0ol4+0.5*RIZdZ*VNvlY(D0ol4)+b4jXr*qn3x8(D0ol4)/6.0)+
fabs(0.5*(RIZdZ+b4jXr*D0ol4)*VNvlY(Qogm3-D0ol4)+b4jXr*qn3x8(Qogm3-D0ol4)/6.0);}
inQml=fabs(xek11)+fabs(ZaFFa)+fabs(_HBf3)+fabs(T5yjB)+fabs(OLj8b);


AF2wW=Kaclx+EvE6x+FvoS1+LOUhW+Qogm3;jfI7z=Ixvmb+zlPa_+g7nwu+mpXRu+xHUUb;gvv1d=
xek11+ZaFFa+_HBf3+T5yjB+OLj8b;if(AF2wW!=0.0){fjgkf=SynchronizationTime-Z4lSz-
AF2wW;if(fabs(fjgkf)>(kED7o*(SynchronizationTime-Z4lSz)+PZBAL)){*DBRdu=true;}}if
(jfI7z!=0.0){hxN2g=EgDTv-TfLg_-jfI7z;if(fabs(hxN2g)>(fabs(Cmmsw*XLg6q)+oUhjD)){*
DBRdu=true;}}if(gvv1d!=0.0){jE9zt=EI730-JRB5m-gvv1d;if(fabs(jE9zt)>(c_4Sr*fabs((
fabs(EI730)>fabs(inQml))?(EI730):(inQml)))+Uq_ZW){*DBRdu=true;}}



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
Z4lSz+=(EvE6x);TfLg_+=zlPa_;JRB5m+=ZaFFa;jEwtH=(0xc57+5581-0x2224);


if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0,0.0,(-JRB5m),(-TfLg_),(Z4lSz));
_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,(-JRB5m),(Z4lSz));_2E29->NYWGt[
_2E29->djbCD].jGQqQ(0.0,0.0,0.0,0.0,(Z4lSz));}else{_2E29->fCxBi[_2E29->djbCD].
jGQqQ(0.0,0.0,JRB5m,TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,
JRB5m,(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,0.0,(Z4lSz));}_2E29
->FfhZi[_2E29->djbCD]=(Z4lSz)+FvoS1;_2E29->djbCD++;
Z4lSz+=(FvoS1);TfLg_+=g7nwu;JRB5m+=_HBf3;jEwtH=(0x9b1+6850-0x2473);


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
_2E29->djbCD]=(Z4lSz)+tlPiC;_2E29->djbCD++;return;}

void qPN_6::RWVKm(const double&rTEdP,const double&SynchronizationTime,const 
double&iPzPj,const double&ldeCA,const double&AQzqu,const double&HcINC,const 
double&k6sf4,const double&DAwhk,const double&b4jXr,XkwFr*_2E29,const bool&U7RiH,
bool*DBRdu){double ifRIn=0.0,TfLg_=iPzPj,EgDTv=ldeCA,JRB5m=AQzqu,EI730=HcINC,
jEwtH=k6sf4,Z4lSz=rTEdP,wmBRB=0.0,Kaclx=0.0,EvE6x=0.0,FvoS1=0.0,LOUhW=0.0,Qogm3=
0.0,CCBY_=0.0,xek11=0.0,ZaFFa=0.0,_HBf3=0.0,T5yjB=0.0,OLj8b=0.0,CnAKY=0.0,Ixvmb=
0.0,zlPa_=0.0,g7nwu=0.0,mpXRu=0.0,xHUUb=0.0,_qvll=0.0,hxN2g=0.0,jE9zt=0.0,fjgkf=
0.0,jfI7z=0.0,gvv1d=0.0,AF2wW=0.0,Yoeil=0.0,rDoKh=0.0,XLg6q=0.0,inQml=0.0,D0ol4=
0.0;

if(jEwtH>DAwhk){jEwtH=DAwhk;}Yoeil=SynchronizationTime-Z4lSz;ifRIn=Q8oIQ(&ZJOyu,
&TGgrw,TfLg_,EgDTv,JRB5m,EI730,jEwtH,DAwhk,b4jXr,Yoeil);


wmBRB=ifRIn;if(wmBRB<jEwtH){wmBRB=jEwtH;}
Kaclx=(wmBRB-jEwtH)/b4jXr;xek11=jEwtH*Kaclx+0.5*b4jXr*VNvlY(Kaclx);Ixvmb=JRB5m*
Kaclx+0.5*jEwtH*VNvlY(Kaclx)+b4jXr*qn3x8(Kaclx)/6.0;
EvE6x=wmBRB/b4jXr;ZaFFa=0.5*VNvlY(wmBRB)/b4jXr;zlPa_=(JRB5m+xek11)*EvE6x+0.5*
wmBRB*VNvlY(EvE6x)-b4jXr*qn3x8(EvE6x)/6.0;

_HBf3=0.0;

LOUhW=DAwhk/b4jXr;;T5yjB=0.5*VNvlY(DAwhk)/b4jXr;mpXRu=(JRB5m+xek11+ZaFFa+_HBf3)*
LOUhW+b4jXr*qn3x8(LOUhW)/6.0;
CCBY_=LOUhW;CnAKY=T5yjB;

OLj8b=EI730-JRB5m-xek11-ZaFFa-_HBf3-T5yjB-CnAKY;if(OLj8b<0.0){OLj8b=0.0;}Qogm3=
OLj8b/DAwhk;xHUUb=(JRB5m+xek11+ZaFFa+_HBf3+T5yjB)*Qogm3+0.5*DAwhk*VNvlY(Qogm3);
_qvll=(JRB5m+xek11+ZaFFa+_HBf3+T5yjB+OLj8b)*CCBY_+0.5*DAwhk*VNvlY(CCBY_)-b4jXr*
qn3x8(CCBY_)/6.0;
g7nwu=EgDTv-TfLg_-Ixvmb-zlPa_-mpXRu-xHUUb-_qvll;if((fabs(JRB5m+xek11+ZaFFa)<
Nm0g6)||(fabs(g7nwu)<Nm0g6)){FvoS1=SynchronizationTime-Z4lSz-Kaclx-EvE6x-LOUhW-
Qogm3-CCBY_;}else{rDoKh=(JRB5m+xek11+ZaFFa);if(rDoKh==0.0){FvoS1=
SynchronizationTime-Z4lSz-Kaclx-EvE6x-LOUhW-Qogm3-CCBY_;}else{FvoS1=g7nwu/rDoKh;
}}if(FvoS1<0.0){FvoS1=0.0;}


if(m85bi(JRB5m)==m85bi(JRB5m+xek11)){XLg6q+=fabs(Ixvmb);}else{D0ol4=-jEwtH/b4jXr
+pbQOc(VNvlY(jEwtH/b4jXr)-2.0*JRB5m/b4jXr);if(D0ol4<0.0){D0ol4=0.0;}if(D0ol4>
Kaclx){D0ol4=Kaclx;}XLg6q+=fabs(JRB5m*D0ol4+0.5*jEwtH*VNvlY(D0ol4)+b4jXr*qn3x8(
D0ol4)/6.0)+fabs(0.5*(jEwtH-b4jXr*D0ol4)*VNvlY(Kaclx-D0ol4)+b4jXr*qn3x8(Kaclx-
D0ol4)/6.0);}
if(m85bi(JRB5m+xek11)==m85bi(JRB5m+xek11+ZaFFa)){XLg6q+=fabs(zlPa_);}else{D0ol4=
wmBRB/b4jXr-pbQOc(VNvlY(wmBRB/b4jXr)+2.0*(JRB5m+xek11)/b4jXr);if(D0ol4<0.0){
D0ol4=0.0;}if(D0ol4>EvE6x){D0ol4=EvE6x;}XLg6q+=fabs((JRB5m+xek11)*D0ol4+0.5*
wmBRB*VNvlY(D0ol4)-b4jXr*qn3x8(D0ol4)/6.0)+fabs(0.5*(wmBRB-b4jXr*D0ol4)*VNvlY(
EvE6x-D0ol4)-b4jXr*qn3x8(EvE6x-D0ol4)/6.0);}
XLg6q+=fabs(g7nwu);
if(m85bi(JRB5m+xek11+ZaFFa+_HBf3)==m85bi(JRB5m+xek11+ZaFFa+_HBf3+T5yjB)){XLg6q+=
fabs(mpXRu);}else{D0ol4=pbQOc(-2.0*(JRB5m+xek11+ZaFFa+_HBf3)/b4jXr);if(D0ol4>
LOUhW){D0ol4=LOUhW;}XLg6q+=fabs((JRB5m+xek11+ZaFFa+_HBf3)*D0ol4+b4jXr*qn3x8(
D0ol4)/6.0)+fabs(0.5*(b4jXr*D0ol4)*VNvlY(LOUhW-D0ol4)+b4jXr*qn3x8(LOUhW-D0ol4)/
6.0);}
if(m85bi(JRB5m+xek11+ZaFFa+_HBf3+T5yjB)==m85bi(JRB5m+xek11+ZaFFa+_HBf3+T5yjB+
OLj8b)){XLg6q+=fabs(xHUUb);}else{D0ol4=fabs((JRB5m+xek11+ZaFFa+_HBf3+T5yjB)/
DAwhk);XLg6q+=fabs(0.5*DAwhk*VNvlY(D0ol4))+fabs(0.5*DAwhk*VNvlY(Qogm3-D0ol4));}
if(m85bi(JRB5m+xek11+ZaFFa+_HBf3+T5yjB+OLj8b)==m85bi(EI730)){XLg6q+=fabs(_qvll);
}else{D0ol4=DAwhk/b4jXr-pbQOc(VNvlY(DAwhk/b4jXr)+2.0*(JRB5m+xek11+ZaFFa+_HBf3+
T5yjB+OLj8b)/b4jXr);if(D0ol4<0.0){D0ol4=0.0;}if(D0ol4>CCBY_){D0ol4=CCBY_;}XLg6q
+=fabs((JRB5m+xek11+ZaFFa+_HBf3+T5yjB+OLj8b)*D0ol4+0.5*DAwhk*VNvlY(D0ol4)-b4jXr*
qn3x8(D0ol4)/6.0)+fabs(0.5*(DAwhk-b4jXr*D0ol4)*VNvlY(CCBY_-D0ol4)-b4jXr*qn3x8(
CCBY_-D0ol4)/6.0);}
inQml=fabs(xek11)+fabs(ZaFFa)+fabs(_HBf3)+fabs(T5yjB)+fabs(OLj8b)+fabs(CnAKY);


AF2wW=Kaclx+EvE6x+FvoS1+LOUhW+Qogm3+CCBY_;jfI7z=Ixvmb+zlPa_+g7nwu+mpXRu+xHUUb+
_qvll;gvv1d=xek11+ZaFFa+_HBf3+T5yjB+OLj8b+CnAKY;if(AF2wW!=0.0){fjgkf=
SynchronizationTime-Z4lSz-AF2wW;if(fabs(fjgkf)>(kED7o*(SynchronizationTime-Z4lSz
)+PZBAL)){*DBRdu=true;}}if(jfI7z!=0.0){hxN2g=EgDTv-TfLg_-jfI7z;if(fabs(hxN2g)>(
fabs(Cmmsw*XLg6q)+oUhjD)){*DBRdu=true;}}if(gvv1d!=0.0){jE9zt=EI730-JRB5m-gvv1d;
if(fabs(jE9zt)>(c_4Sr*fabs((fabs(EI730)>fabs(inQml))?(EI730):(inQml)))+Uq_ZW){*
DBRdu=true;}}



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


if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),0.0,-JRB5m,-TfLg_,(
Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*(-b4jXr)),0.0,-JRB5m,(Z4lSz));
_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,-b4jXr,0.0,(Z4lSz));}else{_2E29->fCxBi[
_2E29->djbCD].jGQqQ((b4jXr/6.0),0.0,JRB5m,TfLg_,(Z4lSz));_2E29->diBqY[_2E29->
djbCD].jGQqQ(0.0,(0.5*b4jXr),0.0,JRB5m,(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ
(0.0,0.0,b4jXr,0.0,(Z4lSz));}_2E29->FfhZi[_2E29->djbCD]=(Z4lSz)+LOUhW;_2E29->
djbCD++;
Z4lSz+=(LOUhW);TfLg_+=mpXRu;JRB5m+=T5yjB;jEwtH=DAwhk;


if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0,(0.5*(-DAwhk)),-JRB5m,-TfLg_,(
Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,-DAwhk,-JRB5m,(Z4lSz));_2E29->
NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,-DAwhk,(Z4lSz));}else{_2E29->fCxBi[_2E29->
djbCD].jGQqQ(0.0,(0.5*DAwhk),JRB5m,TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].
jGQqQ(0.0,0.0,DAwhk,JRB5m,(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,
DAwhk,(Z4lSz));}_2E29->FfhZi[_2E29->djbCD]=(Z4lSz)+Qogm3;_2E29->djbCD++;
Z4lSz+=(Qogm3);TfLg_+=xHUUb;JRB5m+=OLj8b;jEwtH=(0x3db+4147-0x140e);


if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ((b4jXr/6.0),(0.5*(-DAwhk)),-JRB5m,-
TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*b4jXr),-DAwhk,-JRB5m,(
Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,b4jXr,-DAwhk,(Z4lSz));}else{
_2E29->fCxBi[_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),(0.5*DAwhk),JRB5m,TfLg_,(Z4lSz))
;_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*(-b4jXr)),DAwhk,JRB5m,(Z4lSz));_2E29
->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,-b4jXr,DAwhk,(Z4lSz));}_2E29->FfhZi[_2E29->
djbCD]=(Z4lSz)+CCBY_;_2E29->djbCD++;
Z4lSz+=(CCBY_);TfLg_+=_qvll;JRB5m+=CnAKY;jEwtH=0.0;
if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0,0.0,-JRB5m,-TfLg_,(Z4lSz));_2E29
->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,-JRB5m,(Z4lSz));_2E29->NYWGt[_2E29->
djbCD].jGQqQ(0.0,0.0,0.0,0.0,(Z4lSz));}else{_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0
,0.0,JRB5m,TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,JRB5m,(
Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,0.0,(Z4lSz));}_2E29->FfhZi[
_2E29->djbCD]=(Z4lSz)+tlPiC;_2E29->djbCD++;return;}

void qPN_6::PJn2j(const double&rTEdP,const double&SynchronizationTime,const 
double&iPzPj,const double&ldeCA,const double&AQzqu,const double&HcINC,const 
double&k6sf4,const double&DAwhk,const double&b4jXr,XkwFr*_2E29,const bool&U7RiH,
bool*DBRdu){double ifRIn=0.0,TfLg_=iPzPj,EgDTv=ldeCA,JRB5m=AQzqu,EI730=HcINC,
jEwtH=k6sf4,Z4lSz=rTEdP,wmBRB=0.0,RIZdZ=0.0,Kaclx=0.0,EvE6x=0.0,FvoS1=0.0,LOUhW=
0.0,Qogm3=0.0,xek11=0.0,ZaFFa=0.0,_HBf3=0.0,T5yjB=0.0,OLj8b=0.0,Ixvmb=0.0,zlPa_=
0.0,g7nwu=0.0,mpXRu=0.0,xHUUb=0.0,hxN2g=0.0,jE9zt=0.0,fjgkf=0.0,jfI7z=0.0,gvv1d=
0.0,AF2wW=0.0,Yoeil=0.0,rDoKh=0.0,XLg6q=0.0,inQml=0.0,D0ol4=0.0;

if(jEwtH>DAwhk){jEwtH=DAwhk;}Yoeil=SynchronizationTime-Z4lSz;ifRIn=Q8oIQ(&H1iwP,
&EQBeY,TfLg_,EgDTv,JRB5m,EI730,jEwtH,DAwhk,b4jXr,Yoeil);


wmBRB=ifRIn;if(wmBRB<jEwtH){wmBRB=jEwtH;}
Kaclx=(wmBRB-jEwtH)/b4jXr;xek11=jEwtH*Kaclx+0.5*b4jXr*VNvlY(Kaclx);Ixvmb=JRB5m*
Kaclx+0.5*jEwtH*VNvlY(Kaclx)+b4jXr*qn3x8(Kaclx)/6.0;
EvE6x=wmBRB/b4jXr;ZaFFa=0.5*VNvlY(wmBRB)/b4jXr;zlPa_=(JRB5m+xek11)*EvE6x+0.5*
wmBRB*VNvlY(EvE6x)-b4jXr*qn3x8(EvE6x)/6.0;

_HBf3=0.0;

RIZdZ=pbQOc((EI730-JRB5m-xek11-ZaFFa-_HBf3)*b4jXr);
LOUhW=RIZdZ/b4jXr;;T5yjB=0.5*VNvlY(RIZdZ)/b4jXr;mpXRu=(JRB5m+xek11+ZaFFa+_HBf3)*
LOUhW+b4jXr*qn3x8(LOUhW)/6.0;
Qogm3=LOUhW;OLj8b=T5yjB;xHUUb=(JRB5m+xek11+ZaFFa+_HBf3+T5yjB)*Qogm3+0.5*(RIZdZ)*
VNvlY(Qogm3)-b4jXr*qn3x8(Qogm3)/6.0;
g7nwu=EgDTv-TfLg_-Ixvmb-zlPa_-mpXRu-xHUUb;if((fabs(JRB5m+xek11+ZaFFa)<Nm0g6)||(
fabs(g7nwu)<Nm0g6)){FvoS1=SynchronizationTime-Z4lSz-Kaclx-EvE6x-LOUhW-Qogm3;}
else{rDoKh=(JRB5m+xek11+ZaFFa);if(rDoKh==0.0){FvoS1=SynchronizationTime-Z4lSz-
Kaclx-EvE6x-LOUhW-Qogm3;}else{FvoS1=g7nwu/rDoKh;}}if(FvoS1<0.0){FvoS1=0.0;}


if(m85bi(JRB5m)==m85bi(JRB5m+xek11)){XLg6q+=fabs(Ixvmb);}else{D0ol4=-jEwtH/b4jXr
+pbQOc(VNvlY(jEwtH/b4jXr)-2.0*JRB5m/b4jXr);if(D0ol4<0.0){D0ol4=0.0;}if(D0ol4>
Kaclx){D0ol4=Kaclx;}XLg6q+=fabs(JRB5m*D0ol4+0.5*jEwtH*VNvlY(D0ol4)+b4jXr*qn3x8(
D0ol4)/6.0)+fabs(0.5*(jEwtH-b4jXr*D0ol4)*VNvlY(Kaclx-D0ol4)+b4jXr*qn3x8(Kaclx-
D0ol4)/6.0);}
if(m85bi(JRB5m+xek11)==m85bi(JRB5m+xek11+ZaFFa)){XLg6q+=fabs(zlPa_);}else{D0ol4=
wmBRB/b4jXr-pbQOc(VNvlY(wmBRB/b4jXr)+2.0*(JRB5m+xek11)/b4jXr);if(D0ol4<0.0){
D0ol4=0.0;}if(D0ol4>EvE6x){D0ol4=EvE6x;}XLg6q+=fabs((JRB5m+xek11)*D0ol4+0.5*
wmBRB*VNvlY(D0ol4)-b4jXr*qn3x8(D0ol4)/6.0)+fabs(0.5*(wmBRB-b4jXr*D0ol4)*VNvlY(
EvE6x-D0ol4)-b4jXr*qn3x8(EvE6x-D0ol4)/6.0);}
XLg6q+=fabs(g7nwu);
if(m85bi(JRB5m+xek11+ZaFFa+_HBf3)==m85bi(JRB5m+xek11+ZaFFa+_HBf3+T5yjB)){XLg6q+=
fabs(mpXRu);}else{D0ol4=pbQOc(-2.0*(JRB5m+xek11+ZaFFa+_HBf3)/b4jXr);if(D0ol4>
LOUhW){D0ol4=LOUhW;}XLg6q+=fabs((JRB5m+xek11+ZaFFa+_HBf3)*D0ol4+b4jXr*qn3x8(
D0ol4)/6.0)+fabs(0.5*(b4jXr*D0ol4)*VNvlY(LOUhW-D0ol4)+b4jXr*qn3x8(LOUhW-D0ol4)/
6.0);}
if(m85bi(JRB5m+xek11+ZaFFa+_HBf3+T5yjB)==m85bi(EI730)){XLg6q+=fabs(xHUUb);}else{
D0ol4=RIZdZ/b4jXr-pbQOc(VNvlY(RIZdZ/b4jXr)+2.0*(JRB5m+xek11+ZaFFa+_HBf3+T5yjB)/
b4jXr);if(D0ol4<0.0){D0ol4=0.0;}if(D0ol4>Qogm3){D0ol4=Qogm3;}XLg6q+=fabs((JRB5m+
xek11+ZaFFa+_HBf3+T5yjB)*D0ol4+0.5*RIZdZ*VNvlY(D0ol4)-b4jXr*qn3x8(D0ol4)/6.0)+
fabs(0.5*(RIZdZ-b4jXr*D0ol4)*VNvlY(Qogm3-D0ol4)-b4jXr*qn3x8(Qogm3-D0ol4)/6.0);}
inQml=fabs(xek11)+fabs(ZaFFa)+fabs(_HBf3)+fabs(T5yjB)+fabs(OLj8b);


AF2wW=Kaclx+EvE6x+FvoS1+LOUhW+Qogm3;jfI7z=Ixvmb+zlPa_+g7nwu+mpXRu+xHUUb;gvv1d=
xek11+ZaFFa+_HBf3+T5yjB+OLj8b;if(AF2wW!=0.0){fjgkf=SynchronizationTime-Z4lSz-
AF2wW;if(fabs(fjgkf)>(kED7o*(SynchronizationTime-Z4lSz)+PZBAL)){*DBRdu=true;}}if
(jfI7z!=0.0){hxN2g=EgDTv-TfLg_-jfI7z;if(fabs(hxN2g)>(fabs(Cmmsw*XLg6q)+oUhjD)){*
DBRdu=true;}}if(gvv1d!=0.0){jE9zt=EI730-JRB5m-gvv1d;if(fabs(jE9zt)>(c_4Sr*fabs((
fabs(EI730)>fabs(inQml))?(EI730):(inQml)))+Uq_ZW){*DBRdu=true;}}



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


if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),0.0,-JRB5m,-TfLg_,(
Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*(-b4jXr)),0.0,-JRB5m,(Z4lSz));
_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,-b4jXr,0.0,(Z4lSz));}else{_2E29->fCxBi[
_2E29->djbCD].jGQqQ((b4jXr/6.0),0.0,JRB5m,TfLg_,(Z4lSz));_2E29->diBqY[_2E29->
djbCD].jGQqQ(0.0,(0.5*b4jXr),0.0,JRB5m,(Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ
(0.0,0.0,b4jXr,0.0,(Z4lSz));}_2E29->FfhZi[_2E29->djbCD]=(Z4lSz)+LOUhW;_2E29->
djbCD++;
Z4lSz+=(LOUhW);TfLg_+=mpXRu;JRB5m+=T5yjB;jEwtH=RIZdZ;


if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ((b4jXr/6.0),(0.5*(-RIZdZ)),-JRB5m,-
TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*b4jXr),-RIZdZ,-JRB5m,(
Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,b4jXr,-RIZdZ,(Z4lSz));}else{
_2E29->fCxBi[_2E29->djbCD].jGQqQ(((-b4jXr)/6.0),(0.5*RIZdZ),JRB5m,TfLg_,(Z4lSz))
;_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,(0.5*(-b4jXr)),RIZdZ,JRB5m,(Z4lSz));_2E29
->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,-b4jXr,RIZdZ,(Z4lSz));}_2E29->FfhZi[_2E29->
djbCD]=(Z4lSz)+Qogm3;_2E29->djbCD++;
Z4lSz+=(Qogm3);TfLg_+=xHUUb;JRB5m+=OLj8b;jEwtH=0.0;
if(U7RiH){_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0,0.0,-JRB5m,-TfLg_,(Z4lSz));_2E29
->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,-JRB5m,(Z4lSz));_2E29->NYWGt[_2E29->
djbCD].jGQqQ(0.0,0.0,0.0,0.0,(Z4lSz));}else{_2E29->fCxBi[_2E29->djbCD].jGQqQ(0.0
,0.0,JRB5m,TfLg_,(Z4lSz));_2E29->diBqY[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,JRB5m,(
Z4lSz));_2E29->NYWGt[_2E29->djbCD].jGQqQ(0.0,0.0,0.0,0.0,(Z4lSz));}_2E29->FfhZi[
_2E29->djbCD]=(Z4lSz)+tlPiC;_2E29->djbCD++;return;}
