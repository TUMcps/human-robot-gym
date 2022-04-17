

































#include <TypeIVRMLMovingAverageFilter.h>
#include <TypeIVRMLDefinitions.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>


qPN_6::Dt6QZ::Dt6QZ(const double&nsANK,const double&LNHYw,const double&rrN84,
const double&L1kki){unsigned int i=(0x18c+5529-0x1725);this->CycleTime=nsANK;
this->oUqUq=LNHYw;if(rrN84<=LNHYw){this->jsUUr=rrN84;}else{this->jsUUr=LNHYw;}
this->Hw3Lj=(unsigned int)ceil(0.5*this->jsUUr/nsANK);this->vxwAt=(unsigned int)
ceil(0.5*LNHYw/nsANK);this->W5C39=(0x88b+416-0xa2b);this->_sHGJ=tlPiC;this->
qJR5J[(0x24a+4194-0x12ac)]=L1kki;this->qJR5J[(0xd00+4810-0x1fc9)]=0.0;this->
sQI8U=L1kki;this->Wd7b9=L1kki;if(this->Hw3Lj<=(0x70b+7262-0x2368)){this->E3N_R=
this->W5C39;this->Hw3Lj=(0x10d9+2935-0x1c4f);}else{this->E3N_R=this->Hw3Lj-
(0x12c6+2790-0x1dab);}this->eva45=new double[this->vxwAt];for(i=
(0xd59+3932-0x1cb5);i<this->vxwAt;i++){this->eva45[i]=L1kki;}}

qPN_6::Dt6QZ::Dt6QZ(const Dt6QZ&dDGa8){this->eva45=new double[this->vxwAt];*this
=dDGa8;}

qPN_6::Dt6QZ::~Dt6QZ(void){delete[]this->eva45;this->eva45=NULL;}

qPN_6::Dt6QZ&qPN_6::Dt6QZ::operator=(const Dt6QZ&dDGa8){this->CycleTime=dDGa8.
CycleTime;this->oUqUq=dDGa8.oUqUq;this->jsUUr=dDGa8.jsUUr;this->Hw3Lj=dDGa8.
Hw3Lj;this->vxwAt=dDGa8.vxwAt;this->W5C39=dDGa8.W5C39;this->_sHGJ=dDGa8._sHGJ;
this->qJR5J[(0x95c+2030-0x114a)]=dDGa8.qJR5J[(0xadb+5661-0x20f8)];this->qJR5J[
(0x309+6736-0x1d58)]=dDGa8.qJR5J[(0x26a3+28-0x26be)];this->sQI8U=dDGa8.sQI8U;
this->Wd7b9=dDGa8.Wd7b9;memcpy((void*)(this->eva45),(void*)(dDGa8.eva45),(this->
vxwAt*sizeof(double)));return(*this);}

int qPN_6::Dt6QZ::D3bQa(const double&qkU9O,double*rur2n,bool*ec83v){double xqzJ0
=0.0;

if((rur2n==NULL)||(ec83v==NULL)){return(qPN_6::Dt6QZ::KNcsy);}if((qkU9O<0.0)||(
qkU9O>ZPeZQ)){return(qPN_6::Dt6QZ::FkSYt);}

if(!ImCzp(this->sQI8U,qkU9O)){this->_sHGJ=0.0;this->qJR5J[(0xd67+4863-0x2066)]=
this->Wd7b9;this->sQI8U=qkU9O;if(this->jsUUr*0.5<wjjRw){this->qJR5J[
(0xae9+2433-0x1469)]=tlPiC;}else{this->qJR5J[(0x864+5024-0x1c03)]=(this->sQI8U-
this->Wd7b9)/(this->jsUUr*0.5);}}

this->_sHGJ+=this->CycleTime;if(this->_sHGJ<this->oUqUq){xqzJ0=this->qJR5J[
(0x44+679-0x2eb)]+this->qJR5J[(0x3f5+4316-0x14d0)]*_sHGJ;if((xqzJ0>this->sQI8U)
&&(this->qJR5J[(0x29+1355-0x573)]>0.0)){xqzJ0=this->sQI8U;}if((xqzJ0<this->sQI8U
)&&(this->qJR5J[(0xa6+286-0x1c3)]<0.0)){xqzJ0=this->sQI8U;}
*rur2n=this->KOGZv(xqzJ0);if(this->_sHGJ<this->jsUUr){*ec83v=true;}else{*ec83v=
false;}}else{*rur2n=this->sQI8U;*ec83v=false;}this->Wd7b9=*rur2n;return(qPN_6::
Dt6QZ::Ud2tx);}

double qPN_6::Dt6QZ::KOGZv(const double&qkU9O){unsigned int i=(0x19a3+76-0x19ef)
;double aAPWs=0.0;this->W5C39=(this->W5C39+(0x117+4851-0x1409))%this->vxwAt;this
->E3N_R=(this->E3N_R+(0x2481+389-0x2605))%this->vxwAt;this->eva45[this->E3N_R]=
qkU9O;for(i=(0xdb2+2746-0x186c);i<this->Hw3Lj;i++){aAPWs+=this->eva45[(this->
W5C39+i)%this->vxwAt];}return(aAPWs/((double)(this->Hw3Lj)));}

int qPN_6::Dt6QZ::Ht9Cv(const double&_IGXH,const double&kNQ3O){unsigned int i=
(0x124a+2631-0x1c91);if((kNQ3O<0.0)||(kNQ3O>this->oUqUq)){return(qPN_6::Dt6QZ::
fZyrl);}if((_IGXH<0.0)||(_IGXH>ZPeZQ)){return(qPN_6::Dt6QZ::FkSYt);}for(i=
(0x1ae4+214-0x1bba);i<this->vxwAt;i++){this->eva45[i]=_IGXH;}this->_sHGJ=tlPiC;
this->sQI8U=_IGXH;this->Wd7b9=_IGXH;this->qJR5J[(0x2c5+8323-0x2348)]=_IGXH;this
->qJR5J[(0x16ed+1249-0x1bcd)]=0.0;this->jsUUr=kNQ3O;this->Hw3Lj=(unsigned int)
ceil(0.5*kNQ3O/this->CycleTime);if((int)this->E3N_R-(int)this->Hw3Lj<
(0x60f+6799-0x209e)){this->W5C39=this->E3N_R-this->Hw3Lj+this->vxwAt;}else{this
->W5C39=this->E3N_R-this->Hw3Lj;}if(this->Hw3Lj<=(0x1bb1+1635-0x2213)){this->
Hw3Lj=(0xc72+1889-0x13d2);}return(qPN_6::Dt6QZ::Ud2tx);}
