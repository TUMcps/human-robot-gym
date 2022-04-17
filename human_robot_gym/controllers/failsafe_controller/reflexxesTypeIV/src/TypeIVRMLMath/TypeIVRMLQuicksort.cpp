































#include <TypeIVRMLQuicksort.h>


void qPN_6::TVFwP(const int&XTwYX,const int&PB_y2,double*wKcTj){int Yr1Mr,OppWT;
double y1dWP,fQ5fq;y1dWP=wKcTj[(XTwYX+PB_y2)/(0x136b+4319-0x2448)];Yr1Mr=XTwYX;
OppWT=PB_y2;
while(Yr1Mr<=OppWT){while(wKcTj[Yr1Mr]<y1dWP){Yr1Mr=Yr1Mr+(0x169a+146-0x172b);}
while(wKcTj[OppWT]>y1dWP){OppWT=OppWT-(0x1d97+858-0x20f0);}if(Yr1Mr<=OppWT){
fQ5fq=wKcTj[Yr1Mr];wKcTj[Yr1Mr]=wKcTj[OppWT];wKcTj[OppWT]=fQ5fq;Yr1Mr=Yr1Mr+
(0xfc5+3378-0x1cf6);OppWT=OppWT-(0xeab+6233-0x2703);}}
if(XTwYX<OppWT){TVFwP(XTwYX,OppWT,wKcTj);}if(Yr1Mr<PB_y2){TVFwP(Yr1Mr,PB_y2,
wKcTj);}}
