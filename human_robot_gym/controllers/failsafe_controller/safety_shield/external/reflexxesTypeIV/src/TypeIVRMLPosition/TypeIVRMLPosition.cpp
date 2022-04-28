






























#include <TypeIVRMLPosition.h>
#include <TypeIVRMLStep1Profiles.h>
#include <TypeIVRMLDefinitions.h>
#include <TypeIVRMLMath.h>
#include <TypeIVRMLDecisionTree1A.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>
#include <RMLVector.h>
#include <ReflexxesAPI.h>
#include <RMLPositionFlags.h>
#include <TypeIVRMLThreadControl.h>
#ifdef bzQVt
#include <pthread.h>
#endif
using namespace qPN_6;

TypeIVRMLPosition::TypeIVRMLPosition(const unsigned int&ggmbr,const double&nsANK
,const unsigned int&m4tf3,const double&cnY6I){
#ifdef bzQVt	
int Meqss=(0x5bd+186-0x677);unsigned int i=(0xa91+1553-0x10a2);this->VR_Fr=false
;pthread_mutex_init(&(this->rdm6I),NULL);pthread_cond_init(&(this->waCo8),NULL);
#endif
this->F2ivR=false;this->kfikd=false;this->kj2Tc=false;this->xheaR=false;this->
gHefb=false;this->sg_1Y=false;this->ReturnValue=ReflexxesAPI::RML_ERROR;this->
NumberOfDOFs=ggmbr;this->c5Mvm=(0x779+5825-0x1e3a);this->zAQo1=qPN_6::cm_3M;this
->CycleTime=nsANK;this->SynchronizationTime=0.0;this->zzKJk=0.0;this->Ghz_l=
QmAvH;this->z2xUW=QmAvH;this->MaxTimeForOverrideFilter=cnY6I;this->IVKuG=
TypeIVRMLPosition::kgwCF;this->GwQsU=new RMLBoolVector(this->NumberOfDOFs);this
->wNCv9=new RMLBoolVector(this->NumberOfDOFs);this->Cc2Sm=new RMLBoolVector(this
->NumberOfDOFs);this->qfrNo=new RMLVector<FOwyh>(this->NumberOfDOFs);this->bbZEf
=new RMLVector<FOwyh>(this->NumberOfDOFs);this->LiWKZ=new RMLDoubleVector(this->
NumberOfDOFs);this->Q_iq8=new RMLDoubleVector(this->NumberOfDOFs);this->Iwaq1=
new RMLDoubleVector(this->NumberOfDOFs);this->F5Xo9=new RMLDoubleVector(this->
NumberOfDOFs);this->ecwWM=new RMLDoubleVector(this->NumberOfDOFs);this->HxNxN=
new RMLDoubleVector(this->NumberOfDOFs);this->Xmk8y=new RMLDoubleVector(this->
NumberOfDOFs);this->adP3s=new RMLDoubleVector(this->NumberOfDOFs);this->dhSEu=
new RMLDoubleVector(this->NumberOfDOFs);this->VmKWV=new RMLDoubleVector(this->
NumberOfDOFs);this->BNvJP=new RMLDoubleVector(this->NumberOfDOFs);this->o04Be=
new RMLDoubleVector(this->NumberOfDOFs);this->wdDbF=new RMLDoubleVector(this->
NumberOfDOFs);this->TH6RH=new RMLDoubleVector(this->NumberOfDOFs);this->CrzAq=
new RMLDoubleVector(this->NumberOfDOFs);this->jIU8s=new RMLDoubleVector(this->
NumberOfDOFs);this->bqLBP=new RMLDoubleVector(this->NumberOfDOFs);this->v580I=
new RMLDoubleVector((0x690+5567-0x1c4d)*this->NumberOfDOFs);this->gE5PU=new 
RMLDoubleVector(this->NumberOfDOFs);this->NkYww=new RMLDoubleVector(this->
NumberOfDOFs);this->xObJM=new RMLPositionInputParameters(this->NumberOfDOFs);
this->_DBry=new RMLPositionInputParameters(this->NumberOfDOFs);this->gSTLu=new 
RMLPositionOutputParameters(this->NumberOfDOFs);this->fCNQO=new 
RMLVelocityInputParameters(this->NumberOfDOFs);this->uqgIZ=new 
RMLVelocityOutputParameters(this->NumberOfDOFs);this->Polynomials=new XkwFr[this
->NumberOfDOFs];this->E24br=new Dt6QZ(this->CycleTime,this->
MaxTimeForOverrideFilter,qIT2N,1.0);this->RMLVelocityObject=new 
TypeIVRMLVelocity(this->NumberOfDOFs,this->CycleTime,false,this->
MaxTimeForOverrideFilter);memset(this->Polynomials,(0x1655+2757-0x211a),this->
NumberOfDOFs*sizeof(XkwFr));this->NkYww->Set(0.0);this->XOquL=RMLPositionFlags()
;this->onYRt=RMLPositionFlags();



#ifdef bzQVt	
this->EuH1E=(0x590+614-0x7f6);this->NumberOfOwnThreads=m4tf3;this->Kv9sX=
(0x1669+2103-0x1ea0);pthread_attr_t fKak5;struct I_cS5 CLr0b;this->EyjZT=new 
BF1yT(this->NumberOfOwnThreads+(0x33+3984-0xfc2),this->NumberOfDOFs);if(this->
NumberOfOwnThreads>(0x248+6318-0x1af6)){this->LXv3E=new pthread_t[this->
NumberOfOwnThreads];pthread_getschedparam(pthread_self(),&Meqss,&CLr0b);
pthread_attr_init(&fKak5);pthread_attr_setschedpolicy(&fKak5,Meqss);
pthread_attr_setinheritsched(&fKak5,PTHREAD_EXPLICIT_SCHED);
pthread_attr_setschedparam(&fKak5,&CLr0b);for(i=(0x1388+830-0x16c6);i<this->
NumberOfOwnThreads;i++){

this->EuH1E=i+(0x1268+3417-0x1fc0);pthread_create(&((this->LXv3E)[i]),&fKak5,&
TypeIVRMLPosition::wfj20,this);
pthread_mutex_lock(&(this->rdm6I));while(!this->VR_Fr){pthread_cond_wait(&(this
->waCo8),&(this->rdm6I));}this->VR_Fr=false;pthread_mutex_unlock(&(this->rdm6I))
;}this->EyjZT->dO4rW();}else{this->LXv3E=NULL;}
#else
this->NumberOfOwnThreads=(0xad+4550-0x1273);this->EyjZT=new BF1yT(this->
NumberOfOwnThreads+(0x513+7761-0x2363),this->NumberOfDOFs);
#endif
}

TypeIVRMLPosition::~TypeIVRMLPosition(void){
#ifdef bzQVt	
if(this->NumberOfOwnThreads>(0xeff+4900-0x2223)){unsigned int i=
(0x400+3081-0x1009);this->EyjZT->s1Izq();for(i=(0x1ccf+689-0x1f80);i<this->
NumberOfOwnThreads;i++){pthread_join(((this->LXv3E)[i]),NULL);}delete[](
pthread_t*)(this->LXv3E);}this->LXv3E=NULL;
#endif
delete this->E24br;delete this->EyjZT;delete this->xObJM;delete this->_DBry;
delete this->gSTLu;delete this->RMLVelocityObject;delete this->GwQsU;delete this
->wNCv9;delete this->Cc2Sm;delete this->qfrNo;delete this->bbZEf;delete this->
LiWKZ;delete this->Q_iq8;delete this->Iwaq1;delete this->F5Xo9;delete this->
ecwWM;delete this->HxNxN;delete this->dhSEu;delete this->Xmk8y;delete this->
adP3s;delete this->VmKWV;delete this->BNvJP;delete this->o04Be;delete this->
wdDbF;delete this->TH6RH;delete this->CrzAq;delete this->jIU8s;delete this->
bqLBP;delete this->v580I;delete this->gE5PU;delete this->NkYww;delete this->
fCNQO;delete this->uqgIZ;delete[](XkwFr*)this->Polynomials;this->EyjZT=NULL;this
->E24br=NULL;this->xObJM=NULL;this->_DBry=NULL;this->gSTLu=NULL;this->
RMLVelocityObject=NULL;this->GwQsU=NULL;this->wNCv9=NULL;this->Cc2Sm=NULL;this->
qfrNo=NULL;this->bbZEf=NULL;this->LiWKZ=NULL;this->Q_iq8=NULL;this->Iwaq1=NULL;
this->F5Xo9=NULL;this->ecwWM=NULL;this->HxNxN=NULL;this->Xmk8y=NULL;this->adP3s=
NULL;this->dhSEu=NULL;this->VmKWV=NULL;this->BNvJP=NULL;this->o04Be=NULL;this->
wdDbF=NULL;this->TH6RH=NULL;this->CrzAq=NULL;this->jIU8s=NULL;this->bqLBP=NULL;
this->v580I=NULL;this->gE5PU=NULL;this->NkYww=NULL;this->fCNQO=NULL;this->uqgIZ=
NULL;this->Polynomials=NULL;}

int TypeIVRMLPosition::Yq2Ls(const RMLPositionInputParameters&zqVEb,
RMLPositionOutputParameters*SDTKM,const RMLPositionFlags&Jdf48){bool ETvnM=false
,MDmqk=false,HIbPA=false;unsigned int i=(0x14fa+2345-0x1e23);if((SDTKM==NULL)||(
&zqVEb==NULL)||(&Jdf48==NULL)){this->ReturnValue=ReflexxesAPI::
RML_ERROR_NULL_POINTER;if(SDTKM!=NULL){SDTKM->ResultValue=this->ReturnValue;}
return(this->ReturnValue);}if((this->NumberOfDOFs!=zqVEb.GetNumberOfDOFs())||(
this->NumberOfDOFs!=SDTKM->GetNumberOfDOFs())){this->Q48EP(zqVEb,this->gSTLu,
Jdf48);*SDTKM=*(this->gSTLu);this->ReturnValue=ReflexxesAPI::
RML_ERROR_NUMBER_OF_DOFS;SDTKM->ResultValue=this->ReturnValue;return(this->
ReturnValue);}
if((zqVEb.OverrideValue>ZPeZQ)||(zqVEb.OverrideValue<0.0)){this->Q48EP(zqVEb,
this->gSTLu,Jdf48);*SDTKM=*(this->gSTLu);this->ReturnValue=ReflexxesAPI::
RML_ERROR_OVERRIDE_OUT_OF_RANGE;SDTKM->ResultValue=this->ReturnValue;return(this
->ReturnValue);}*(this->_DBry)=zqVEb;this->XOquL=Jdf48;this->xheaR=Jdf48.
EnableTheCalculationOfTheExtremumMotionStates;if(((this->ReturnValue==
ReflexxesAPI::RML_FINAL_STATE_REACHED)&&(Jdf48.
BehaviorAfterFinalStateOfMotionIsReached==RMLPositionFlags::RECOMPUTE_TRAJECTORY
))||(Jdf48!=this->onYRt)){MDmqk=true;}else{MDmqk=false;}if(!MDmqk){if(*(this->
_DBry->SelectionVector)!=*(this->xObJM->SelectionVector)){MDmqk=true;}else{for(i
=(0x167c+2051-0x1e7f);i<this->NumberOfDOFs;i++){if((this->_DBry->SelectionVector
->VecData)[i]){if(!(ImCzp((this->_DBry->CurrentVelocityVector->VecData)[i],(this
->gSTLu->NewVelocityVector->VecData)[i])&&ImCzp((this->_DBry->
CurrentAccelerationVector->VecData)[i],(this->gSTLu->NewAccelerationVector->
VecData)[i])&&ImCzp((this->_DBry->MaxJerkVector->VecData)[i],(this->xObJM->
MaxJerkVector->VecData)[i])&&ImCzp((this->_DBry->MaxAccelerationVector->VecData)
[i],(this->xObJM->MaxAccelerationVector->VecData)[i])&&ImCzp((this->_DBry->
MaxVelocityVector->VecData)[i],(this->xObJM->MaxVelocityVector->VecData)[i])&&
ImCzp((this->_DBry->TargetVelocityVector->VecData)[i],(this->xObJM->
TargetVelocityVector->VecData)[i])&&ImCzp(((this->_DBry->TargetPositionVector->
VecData)[i]-(this->_DBry->CurrentPositionVector->VecData)[i]),((this->xObJM->
TargetPositionVector->VecData)[i]-(this->gSTLu->NewPositionVector->VecData)[i]))
)){MDmqk=true;break;}}}}}if((MDmqk)||((this->ReturnValue!=ReflexxesAPI::
RML_WORKING)&&(this->ReturnValue!=ReflexxesAPI::RML_ERROR_POSITIONAL_LIMITS)&&(
this->ReturnValue!=ReflexxesAPI::RML_FINAL_STATE_REACHED))){

MDmqk=true;this->SynchronizationTime=0.0;this->sg_1Y=false;for(i=
(0x51f+2625-0xf60);i<this->NumberOfDOFs;i++){if((this->_DBry->SelectionVector->
VecData)[i]){if((fabs((this->_DBry->TargetVelocityVector->VecData)[i])>(this->
_DBry->MaxVelocityVector->VecData)[i])||((this->_DBry->MaxVelocityVector->
VecData)[i]<=0.0)||((this->_DBry->MaxAccelerationVector->VecData)[i]<=0.0)||((
this->_DBry->MaxJerkVector->VecData)[i]<=0.0)){this->Q48EP(zqVEb,this->gSTLu,
Jdf48);*SDTKM=*(this->gSTLu);this->ReturnValue=ReflexxesAPI::
RML_ERROR_INVALID_INPUT_VALUES;SDTKM->ResultValue=this->ReturnValue;return(this
->ReturnValue);}}}}else{MDmqk=false;}


this->E24br->D3bQa(zqVEb.OverrideValue,&(this->Ghz_l),&(this->gSTLu->
OverrideFilterIsActive));*(this->xObJM)=zqVEb;this->onYRt=Jdf48;if(MDmqk){this->
zzKJk=this->CycleTime*this->Ghz_l;this->Cc2Sm->Set(false);this->ql7rg();this->
ePMhj();this->TTtbr();this->Kv9sX=(0x2367+626-0x25d9);for(i=(0x257+3396-0xf9b);i
<this->NumberOfDOFs;i++){if(zqVEb.SelectionVector->VecData[i]){this->Kv9sX++;}}
this->gHefb=(this->Kv9sX==(0x1d7a+1493-0x234e));*(this->LiWKZ)=*(this->_DBry->
TargetPositionVector);this->hXYgE();this->F2ivR=((Jdf48.SynchronizationBehavior
==RMLFlags::ONLY_PHASE_SYNCHRONIZATION)||(Jdf48.SynchronizationBehavior==
RMLFlags::PHASE_SYNCHRONIZATION_IF_POSSIBLE));this->kj2Tc=(Jdf48.
SynchronizationBehavior==RMLFlags::NO_SYNCHRONIZATION);







ETvnM=RRkLb();if(ETvnM){this->Q48EP(zqVEb,this->gSTLu,Jdf48);*SDTKM=*(this->
gSTLu);if(zqVEb.CheckForValidity()){
this->ReturnValue=ReflexxesAPI::RML_ERROR_EXECUTION_TIME_CALCULATION;}else{this
->ReturnValue=ReflexxesAPI::RML_ERROR_INVALID_INPUT_VALUES;}SDTKM->ResultValue=
this->ReturnValue;return(this->ReturnValue);}if((Jdf48.SynchronizationBehavior==
RMLFlags::ONLY_PHASE_SYNCHRONIZATION)&&(!(this->F2ivR))){this->Q48EP(zqVEb,this
->gSTLu,Jdf48);*SDTKM=*(this->gSTLu);if(zqVEb.CheckForValidity()){this->
ReturnValue=ReflexxesAPI::RML_ERROR_NO_PHASE_SYNCHRONIZATION;}else{this->
ReturnValue=ReflexxesAPI::RML_ERROR_INVALID_INPUT_VALUES;}SDTKM->ResultValue=
this->ReturnValue;return(this->ReturnValue);}if(this->SynchronizationTime>DQDa3)
{this->Q48EP(zqVEb,this->gSTLu,Jdf48);*SDTKM=*(this->gSTLu);if(zqVEb.
CheckForValidity()){this->ReturnValue=ReflexxesAPI::
RML_ERROR_EXECUTION_TIME_TOO_BIG;}else{this->ReturnValue=ReflexxesAPI::
RML_ERROR_INVALID_INPUT_VALUES;}SDTKM->ResultValue=this->ReturnValue;return(this
->ReturnValue);}for(i=(0x81a+6048-0x1fba);i<this->NumberOfDOFs;i++){(this->
Polynomials)[i].djbCD=(0x608+4687-0x1857);}if((Jdf48.SynchronizationBehavior!=
RMLFlags::NO_SYNCHRONIZATION)&&(zqVEb.MinimumSynchronizationTime>this->
SynchronizationTime)){for(i=(0x1281+5073-0x2652);i<(0x547+7407-0x2234)*this->
NumberOfDOFs;i++){if((this->v580I->VecData)[i]>zqVEb.MinimumSynchronizationTime)
{break;}}this->SynchronizationTime=zqVEb.MinimumSynchronizationTime;
while((ZSTS8(this->SynchronizationTime,*(this->F5Xo9),*(this->ecwWM)))&&(i<
(0x5c4+5363-0x1ab5)*this->NumberOfDOFs)){this->SynchronizationTime=(this->v580I
->VecData)[i];i++;}}ETvnM=kKWV_();if(ETvnM){this->Q48EP(zqVEb,this->gSTLu,Jdf48)
;*SDTKM=*(this->gSTLu);if(zqVEb.CheckForValidity()){
this->ReturnValue=ReflexxesAPI::RML_ERROR_SYNCHRONIZATION;}else{this->
ReturnValue=ReflexxesAPI::RML_ERROR_INVALID_INPUT_VALUES;}SDTKM->ResultValue=
this->ReturnValue;return(this->ReturnValue);}}else{this->zzKJk+=this->CycleTime*
this->Ghz_l;this->ePMhj();}this->gSTLu->CurrentOverrideValue=this->Ghz_l;this->
z2xUW=this->Ghz_l;if(qPN_6::UifCr(*(this->wNCv9))==(0xbc+6662-0x1ac2)){this->
SynchronizationTime=0.0;if(Jdf48.SynchronizationBehavior==RMLFlags::
ONLY_TIME_SYNCHRONIZATION){this->F2ivR=false;}else{this->F2ivR=true;}}
this->ReturnValue=_XPNu(this->zzKJk,this->Ghz_l,this->gSTLu);this->gSTLu->
ANewCalculationWasPerformed=MDmqk;this->gSTLu->TrajectoryIsPhaseSynchronized=
this->F2ivR;this->gSTLu->DOFWithTheGreatestExecutionTime=this->c5Mvm;if(this->
kj2Tc){this->gSTLu->SynchronizationTime=0.0;for(i=(0xd18+3267-0x19db);i<this->
NumberOfDOFs;i++){if(this->_DBry->SelectionVector->VecData[i]){if(this->Ghz_l>
wjjRw){this->gSTLu->ExecutionTimes->VecData[i]=(this->Q_iq8->VecData[i]-this->
zzKJk+this->CycleTime)/this->Ghz_l;}else{this->gSTLu->ExecutionTimes->VecData[i]
=tlPiC;}if(this->gSTLu->ExecutionTimes->VecData[i]<0.0){this->gSTLu->
ExecutionTimes->VecData[i]=0.0;}}else{this->gSTLu->ExecutionTimes->VecData[i]=
0.0;}}}else{if(this->Ghz_l>wjjRw){this->gSTLu->SynchronizationTime=(this->
SynchronizationTime-this->zzKJk+this->CycleTime)/this->Ghz_l;}else{this->gSTLu->
SynchronizationTime=tlPiC;}if(this->gSTLu->SynchronizationTime<0.0){this->gSTLu
->SynchronizationTime=0.0;}for(i=(0x1010+2614-0x1a46);i<this->NumberOfDOFs;i++){
if(this->_DBry->SelectionVector->VecData[i]){this->gSTLu->ExecutionTimes->
VecData[i]=this->gSTLu->SynchronizationTime;}else{this->gSTLu->ExecutionTimes->
VecData[i]=0.0;}}}if((this->xheaR)||(this->XOquL.PositionalLimitsBehavior!=
RMLFlags::POSITIONAL_LIMITS_IGNORE)){this->RQuk0(this->zzKJk-this->CycleTime,
this->Ghz_l,this->gSTLu);}if(!this->xheaR){this->chwJj(this->gSTLu);}this->e3pE4
(this->gSTLu);if(this->XOquL.PositionalLimitsBehavior!=RMLFlags::
POSITIONAL_LIMITS_IGNORE){for(i=(0x1704+1394-0x1c76);i<this->NumberOfDOFs;i++){
if(this->_DBry->SelectionVector->VecData[i]){if((this->gSTLu->
MinPosExtremaPositionVectorOnly->VecData[i]<=this->_DBry->MinPositionVector->
VecData[i])||(this->gSTLu->MaxPosExtremaPositionVectorOnly->VecData[i]>=this->
_DBry->MaxPositionVector->VecData[i])){this->sg_1Y=true;this->ReturnValue=
ReflexxesAPI::RML_ERROR_POSITIONAL_LIMITS;break;}}}}for(i=(0x689+388-0x80d);i<
this->NumberOfDOFs;i++){if((this->wNCv9->VecData)[i]){(this->gSTLu->
NewPositionVector->VecData)[i]=(this->_DBry->TargetPositionVector->VecData)[i]-(
(this->LiWKZ->VecData)[i]-(this->gSTLu->NewPositionVector->VecData)[i]);}}if((
this->ReturnValue==ReflexxesAPI::RML_FINAL_STATE_REACHED)&&(MDmqk)){this->gSTLu
->SynchronizationTime=this->Q_iq8->VecData[this->c5Mvm]/this->Ghz_l;}if(this->
XOquL.PositionalLimitsBehavior==RMLFlags::POSITIONAL_LIMITS_ACTIVELY_PREVENT){
HIbPA=this->q9K7w(zqVEb,this->gSTLu);if(HIbPA){this->ReturnValue=ReflexxesAPI::
RML_ERROR_POSITIONAL_LIMITS;}}*SDTKM=*(this->gSTLu);SDTKM->ResultValue=this->
ReturnValue;return(this->ReturnValue);}

int TypeIVRMLPosition::Xvpsd(const double&PxS7M,RMLPositionOutputParameters*
SDTKM){unsigned int i=(0x1261+2784-0x1d41);int kfyzJ=ReflexxesAPI::RML_ERROR;
double uOuSq=PxS7M+this->zzKJk-this->CycleTime;if((this->ReturnValue!=
ReflexxesAPI::RML_WORKING)&&(this->ReturnValue!=ReflexxesAPI::
RML_FINAL_STATE_REACHED)){SDTKM->ResultValue=this->ReturnValue;return(this->
ReturnValue);}if((PxS7M<0.0)||(uOuSq>DQDa3)){SDTKM->ResultValue=ReflexxesAPI::
RML_ERROR_USER_TIME_OUT_OF_RANGE;return(ReflexxesAPI::
RML_ERROR_USER_TIME_OUT_OF_RANGE);}if(SDTKM==NULL){return(ReflexxesAPI::
RML_ERROR_NULL_POINTER);}if(SDTKM->NumberOfDOFs!=this->NumberOfDOFs){SDTKM->
ResultValue=ReflexxesAPI::RML_ERROR_NUMBER_OF_DOFS;return(ReflexxesAPI::
RML_ERROR_NUMBER_OF_DOFS);}SDTKM->ANewCalculationWasPerformed=false;kfyzJ=_XPNu(
uOuSq,1.0,SDTKM);SDTKM->TrajectoryIsPhaseSynchronized=this->F2ivR;SDTKM->
DOFWithTheGreatestExecutionTime=this->c5Mvm;if(this->kj2Tc){SDTKM->
SynchronizationTime=0.0;for(i=(0x101a+3608-0x1e32);i<this->NumberOfDOFs;i++){if(
this->_DBry->SelectionVector->VecData[i]){SDTKM->ExecutionTimes->VecData[i]=(
this->Q_iq8->VecData)[i]-Xu3x0-PxS7M;if(SDTKM->ExecutionTimes->VecData[i]<0.0){
SDTKM->ExecutionTimes->VecData[i]=0.0;}}else{SDTKM->ExecutionTimes->VecData[i]=
0.0;}}}else{SDTKM->SynchronizationTime=this->SynchronizationTime-PxS7M;for(i=
(0x13d7+2123-0x1c22);i<this->NumberOfDOFs;i++){if(this->_DBry->SelectionVector->
VecData[i]){SDTKM->ExecutionTimes->VecData[i]=this->SynchronizationTime-PxS7M;if
(SDTKM->ExecutionTimes->VecData[i]<0.0){SDTKM->ExecutionTimes->VecData[i]=0.0;}}
else{SDTKM->ExecutionTimes->VecData[i]=0.0;}}}if((this->xheaR)||(this->XOquL.
PositionalLimitsBehavior!=RMLFlags::POSITIONAL_LIMITS_IGNORE)){this->RQuk0(this
->zzKJk-this->CycleTime,1.0,SDTKM);}if(!this->xheaR){this->chwJj(SDTKM);}for(i=
(0x1256+4407-0x238d);i<this->NumberOfDOFs;i++){if((this->wNCv9->VecData)[i]){(
SDTKM->NewPositionVector->VecData)[i]=(this->_DBry->TargetPositionVector->
VecData)[i]-((this->LiWKZ->VecData)[i]-(SDTKM->NewPositionVector->VecData)[i]);}
}this->e3pE4(SDTKM);SDTKM->ResultValue=kfyzJ;return(kfyzJ);}

void TypeIVRMLPosition::hXYgE(void){unsigned int i=(0x12cd+2315-0x1bd8);for(i=
(0x1de4+445-0x1fa1);i<this->NumberOfDOFs;i++){if((this->_DBry->SelectionVector->
VecData)[i]){if(!(((this->_DBry->CurrentPositionVector->VecData)[i]==(this->
_DBry->TargetPositionVector->VecData)[i])&&((this->_DBry->CurrentVelocityVector
->VecData)[i]==(this->_DBry->TargetVelocityVector->VecData)[i])&&((this->_DBry->
TargetVelocityVector->VecData)[i]!=0.0)&&((this->_DBry->
CurrentAccelerationVector->VecData)[i]==0.0))){return;}}}for(i=
(0x16f9+2097-0x1f2a);i<this->NumberOfDOFs;i++){if((this->_DBry->SelectionVector
->VecData)[i]){if((this->_DBry->CurrentPositionVector->VecData)[i]!=0.0){(this->
_DBry->CurrentPositionVector->VecData)[i]*=1.0+B4Gkb((this->_DBry->
CurrentVelocityVector->VecData)[i])*OrsGt;}else{(this->_DBry->
CurrentPositionVector->VecData)[i]+=B4Gkb((this->_DBry->CurrentVelocityVector->
VecData)[i])*VfltD;}}}return;}

bool TypeIVRMLPosition::q9K7w(const RMLPositionInputParameters&zqVEb,
RMLPositionOutputParameters*SDTKM){bool zMYri=false;unsigned int i=
(0x7e3+949-0xb98),ip1HJ=(0x243+279-0x35a);int j=(0x13ca+3509-0x217f);double 
Fe3GR=0.0;*(this->fCNQO->SelectionVector)=*(zqVEb.SelectionVector);*(this->fCNQO
->CurrentPositionVector)=*(SDTKM->NewPositionVector);*(this->fCNQO->
CurrentVelocityVector)=*(SDTKM->NewVelocityVector);*(this->fCNQO->
CurrentAccelerationVector)=*(SDTKM->NewAccelerationVector);*(this->fCNQO->
MaxAccelerationVector)=*(zqVEb.MaxAccelerationVector);*(this->fCNQO->
MaxJerkVector)=*(zqVEb.MaxJerkVector);*(this->fCNQO->TargetVelocityVector)=*(
this->NkYww);this->fCNQO->OverrideValue=1.0;this->YROav.PositionalLimitsBehavior
=RMLFlags::POSITIONAL_LIMITS_IGNORE;this->YROav.SynchronizationBehavior=RMLFlags
::NO_SYNCHRONIZATION;this->YROav.EnableTheCalculationOfTheExtremumMotionStates=
true;this->RMLVelocityObject->Yq2Ls(*(this->fCNQO),this->uqgIZ,this->YROav);for(
i=(0x20a1+1067-0x24cc);i<this->NumberOfDOFs;i++){if(this->fCNQO->SelectionVector
->VecData[i]){if((this->uqgIZ->PositionValuesAtTargetVelocity->VecData[i]<=zqVEb
.MinPositionVector->VecData[i]-EqzBo)||(this->uqgIZ->
PositionValuesAtTargetVelocity->VecData[i]>=zqVEb.MaxPositionVector->VecData[i]+
EqzBo)){this->Cc2Sm->VecData[i]=true;}}}if(qPN_6::UifCr(*(this->Cc2Sm))>
(0x17a0+2542-0x218e)){zMYri=true;}if(zMYri){*(this->fCNQO->CurrentPositionVector
)=*(zqVEb.CurrentPositionVector);*(this->fCNQO->CurrentVelocityVector)=*(zqVEb.
CurrentVelocityVector);*(this->fCNQO->CurrentAccelerationVector)=*(zqVEb.
CurrentAccelerationVector);*(this->fCNQO->SelectionVector)=*(this->Cc2Sm);this->
RMLVelocityObject->Yq2Ls(*(this->fCNQO),this->uqgIZ,this->YROav);for(i=
(0x41f+1903-0xb8e);i<this->NumberOfDOFs;i++){if(this->Cc2Sm->VecData[i]){SDTKM->
NewPositionVector->VecData[i]=this->uqgIZ->NewPositionVector->VecData[i];SDTKM->
NewVelocityVector->VecData[i]=this->uqgIZ->NewVelocityVector->VecData[i];SDTKM->
NewAccelerationVector->VecData[i]=this->uqgIZ->NewAccelerationVector->VecData[i]
;SDTKM->MinExtremaTimesVector->VecData[i]=this->uqgIZ->MinExtremaTimesVector->
VecData[i];SDTKM->MaxExtremaTimesVector->VecData[i]=this->uqgIZ->
MaxExtremaTimesVector->VecData[i];SDTKM->ExecutionTimes->VecData[i]=this->uqgIZ
->ExecutionTimes->VecData[i];SDTKM->MinPosExtremaPositionVectorOnly->VecData[i]=
this->uqgIZ->MinPosExtremaPositionVectorOnly->VecData[i];SDTKM->
MaxPosExtremaPositionVectorOnly->VecData[i]=this->uqgIZ->
MaxPosExtremaPositionVectorOnly->VecData[i];this->uqgIZ->
GetMotionStateAtMinPosForOneDOF(i,(SDTKM->MinPosExtremaPositionVectorArray)[i],(
SDTKM->MinPosExtremaVelocityVectorArray)[i],(SDTKM->
MinPosExtremaAccelerationVectorArray)[i]);this->uqgIZ->
GetMotionStateAtMaxPosForOneDOF(i,(SDTKM->MaxPosExtremaPositionVectorArray)[i],(
SDTKM->MaxPosExtremaVelocityVectorArray)[i],(SDTKM->
MaxPosExtremaAccelerationVectorArray)[i]);SDTKM->Polynomials->
NumberOfCurrentlyValidSegments[i]=this->uqgIZ->Polynomials->
NumberOfCurrentlyValidSegments[i];for(j=(0x139f+1593-0x19d8);j<SDTKM->
Polynomials->NumberOfCurrentlyValidSegments[i];j++){SDTKM->Polynomials->
Coefficients[i][j]=this->uqgIZ->Polynomials->Coefficients[i][j];}}if((SDTKM->
ExecutionTimes->VecData[i]>Fe3GR)&&(zqVEb.SelectionVector->VecData[i])){Fe3GR=
SDTKM->ExecutionTimes->VecData[i];ip1HJ=i;}}SDTKM->
DOFWithTheGreatestExecutionTime=i;SDTKM->TrajectoryIsPhaseSynchronized=false;
SDTKM->SynchronizationTime=Fe3GR;SDTKM->ANewCalculationWasPerformed=true;}return
(zMYri);}

void TypeIVRMLPosition::TTtbr(void){unsigned int i=(0x15e1+2660-0x2045);double 
aj_ej=0.0,tVawk=0.0;if(this->z2xUW>MH_kC){aj_ej=1.0/this->z2xUW;tVawk=1.0/VNvlY(
this->z2xUW);for(i=(0x1a3d+1452-0x1fe9);i<this->NumberOfDOFs;i++){if((this->
_DBry->SelectionVector->VecData)[i]){this->_DBry->CurrentVelocityVector->VecData
[i]*=aj_ej;this->_DBry->CurrentAccelerationVector->VecData[i]*=tVawk;}}}else{for
(i=(0x3d0+5850-0x1aaa);i<this->NumberOfDOFs;i++){if((this->_DBry->
SelectionVector->VecData)[i]){this->_DBry->CurrentVelocityVector->VecData[i]=0.0
;this->_DBry->CurrentAccelerationVector->VecData[i]=0.0;}}}return;}

int TypeIVRMLPosition::SetupOverrideFilter(const double&_IGXH,const double&kNQ3O
){if((_IGXH<0.0)||(_IGXH>ZPeZQ)||(kNQ3O<0.0)||(kNQ3O>this->
MaxTimeForOverrideFilter)){return(ReflexxesAPI::RML_ERROR_OVERRIDE_OUT_OF_RANGE)
;}this->E24br->Ht9Cv(_IGXH,kNQ3O);this->Ghz_l=_IGXH;this->z2xUW=_IGXH;return(
ReflexxesAPI::RML_NO_ERROR);}
