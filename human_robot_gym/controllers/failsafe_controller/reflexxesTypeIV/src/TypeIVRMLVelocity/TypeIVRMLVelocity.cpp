






























#include <TypeIVRMLVelocity.h>
#include <TypeIVRMLDefinitions.h>
#include <TypeIVRMLMath.h>
#include <TypeIVRMLStep1Decisions.h>
#include <RMLVelocityInputParameters.h>
#include <RMLVelocityOutputParameters.h>
#include <RMLVector.h>
#include <ReflexxesAPI.h>
#include <RMLVelocityFlags.h>
#include <TypeIVRMLVelocityTools.h>
#include <stdlib.h>
using namespace qPN_6;

TypeIVRMLVelocity::TypeIVRMLVelocity(const unsigned int&ggmbr,const double&nsANK
,const bool&kjOCK,const double&cnY6I){this->F2ivR=false;this->xheaR=false;this->
kj2Tc=false;this->ReturnValue=ReflexxesAPI::RML_ERROR;this->YDfwn=
(0x15d1+2181-0x1e56);this->NumberOfDOFs=ggmbr;this->CycleTime=nsANK;this->zzKJk=
0.0;this->SynchronizationTime=0.0;this->Ghz_l=QmAvH;this->z2xUW=QmAvH;this->
MaxTimeForOverrideFilter=cnY6I;this->Ge1KL=new RMLBoolVector(this->NumberOfDOFs)
;this->qeRBJ=new RMLBoolVector(this->NumberOfDOFs);this->Cc2Sm=new RMLBoolVector
(this->NumberOfDOFs);this->ATtzi=new RMLIntVector(this->NumberOfDOFs);this->
ExecutionTimes=new RMLDoubleVector(this->NumberOfDOFs);this->HxNxN=new 
RMLDoubleVector(this->NumberOfDOFs);this->BNvJP=new RMLDoubleVector(this->
NumberOfDOFs);this->VmKWV=new RMLDoubleVector(this->NumberOfDOFs);this->o04Be=
new RMLDoubleVector(this->NumberOfDOFs);this->CrzAq=new RMLDoubleVector(this->
NumberOfDOFs);this->TH6RH=new RMLDoubleVector(this->NumberOfDOFs);this->Pa6wf=
new RMLDoubleVector(this->NumberOfDOFs);this->NkYww=new RMLDoubleVector(this->
NumberOfDOFs);this->xObJM=new RMLVelocityInputParameters(this->NumberOfDOFs);
this->_DBry=new RMLVelocityInputParameters(this->NumberOfDOFs);this->KZs7z=new 
RMLVelocityInputParameters(this->NumberOfDOFs);this->gSTLu=new 
RMLVelocityOutputParameters(this->NumberOfDOFs);this->_fSN4=new 
RMLVelocityOutputParameters(this->NumberOfDOFs);this->E24br=new Dt6QZ(this->
CycleTime,this->MaxTimeForOverrideFilter,qIT2N,1.0);if(kjOCK){this->z1jCu=NULL;
this->X0n_9=false;}else{this->z1jCu=new TypeIVRMLVelocity(this->NumberOfDOFs,
this->CycleTime,true,this->MaxTimeForOverrideFilter);this->X0n_9=true;}this->
Polynomials=new XkwFr[this->NumberOfDOFs];}

TypeIVRMLVelocity::~TypeIVRMLVelocity(void){delete this->Ge1KL;delete this->
qeRBJ;delete this->Cc2Sm;delete this->ATtzi;delete this->ExecutionTimes;delete 
this->HxNxN;delete this->BNvJP;delete this->VmKWV;delete this->o04Be;delete this
->CrzAq;delete this->TH6RH;delete this->Pa6wf;delete this->NkYww;delete this->
xObJM;delete this->_DBry;delete this->KZs7z;delete this->gSTLu;delete this->
_fSN4;if(X0n_9){delete this->z1jCu;}delete this->E24br;delete[]this->Polynomials
;this->Ge1KL=NULL;this->qeRBJ=NULL;this->Cc2Sm=NULL;this->ATtzi=NULL;this->
ExecutionTimes=NULL;this->HxNxN=NULL;this->BNvJP=NULL;this->VmKWV=NULL;this->
o04Be=NULL;this->CrzAq=NULL;this->TH6RH=NULL;this->Pa6wf=NULL;this->NkYww=NULL;
this->xObJM=NULL;this->_DBry=NULL;this->KZs7z=NULL;this->gSTLu=NULL;this->_fSN4=
NULL;this->z1jCu=NULL;this->E24br=NULL;this->Polynomials=NULL;}

int TypeIVRMLVelocity::Yq2Ls(const RMLVelocityInputParameters&zqVEb,
RMLVelocityOutputParameters*SDTKM,const RMLVelocityFlags&Jdf48){bool pxDmO=false
,MDmqk=false,HIbPA=false;unsigned int i=(0x1017+1239-0x14ee);if((SDTKM==NULL)||(
&zqVEb==NULL)||(&Jdf48==NULL)){this->ReturnValue=ReflexxesAPI::
RML_ERROR_NULL_POINTER;if(SDTKM!=NULL){SDTKM->ResultValue=this->ReturnValue;}
return(this->ReturnValue);}if((this->NumberOfDOFs!=zqVEb.GetNumberOfDOFs())||(
this->NumberOfDOFs!=SDTKM->GetNumberOfDOFs())){this->ReturnValue=ReflexxesAPI::
RML_ERROR_NUMBER_OF_DOFS;SDTKM->ResultValue=this->ReturnValue;return(this->
ReturnValue);}
if((zqVEb.OverrideValue>ZPeZQ)||(zqVEb.OverrideValue<0.0)){this->Q48EP(*(this->
_DBry),this->gSTLu);*SDTKM=*(this->gSTLu);this->ReturnValue=ReflexxesAPI::
RML_ERROR_OVERRIDE_OUT_OF_RANGE;SDTKM->ResultValue=this->ReturnValue;return(this
->ReturnValue);}this->xheaR=Jdf48.EnableTheCalculationOfTheExtremumMotionStates;
*(this->_DBry)=zqVEb;if(Jdf48!=this->onYRt){MDmqk=true;}
if(!MDmqk){if(*(this->_DBry->SelectionVector)!=*(this->xObJM->SelectionVector)){
MDmqk=true;}else{for(i=(0x1e16+47-0x1e45);i<this->NumberOfDOFs;i++){if((this->
_DBry->SelectionVector->VecData)[i]){if(!(ImCzp((this->_DBry->
CurrentVelocityVector->VecData)[i],(this->gSTLu->NewVelocityVector->VecData)[i])
&&ImCzp((this->_DBry->CurrentAccelerationVector->VecData)[i],(this->gSTLu->
NewAccelerationVector->VecData)[i])&&ImCzp((this->_DBry->MaxJerkVector->VecData)
[i],(this->xObJM->MaxJerkVector->VecData)[i])&&ImCzp((this->_DBry->
MaxAccelerationVector->VecData)[i],(this->xObJM->MaxAccelerationVector->VecData)
[i])&&ImCzp((this->_DBry->TargetVelocityVector->VecData)[i],(this->xObJM->
TargetVelocityVector->VecData)[i])&&ImCzp((this->_DBry->CurrentPositionVector->
VecData)[i],(this->gSTLu->NewPositionVector->VecData)[i]))){MDmqk=true;break;}}}
}}

this->E24br->D3bQa(zqVEb.OverrideValue,&(this->Ghz_l),&(this->gSTLu->
OverrideFilterIsActive));if((MDmqk)||((this->ReturnValue!=ReflexxesAPI::
RML_WORKING)&&(this->ReturnValue!=ReflexxesAPI::RML_ERROR_POSITIONAL_LIMITS)&&(
this->ReturnValue!=ReflexxesAPI::RML_FINAL_STATE_REACHED))){this->zzKJk=this->
CycleTime*this->Ghz_l;MDmqk=true;

this->SynchronizationTime=0.0;}else{this->zzKJk+=this->CycleTime*this->Ghz_l;}*(
this->xObJM)=zqVEb;this->onYRt=Jdf48;if(MDmqk){this->Cc2Sm->Set(false);this->
TTtbr();this->F2ivR=(Jdf48.SynchronizationBehavior==RMLFlags::
ONLY_PHASE_SYNCHRONIZATION)||(Jdf48.SynchronizationBehavior==RMLFlags::
PHASE_SYNCHRONIZATION_IF_POSSIBLE);for(i=(0x15ba+121-0x1633);i<this->
NumberOfDOFs;i++){if((this->_DBry->SelectionVector->VecData)[i]){if(((this->
_DBry->MaxAccelerationVector->VecData)[i]<=0.0)||((this->_DBry->MaxJerkVector->
VecData)[i]<=0.0)){pxDmO=true;}}}if(pxDmO){this->Q48EP(*(this->_DBry),this->
gSTLu);*SDTKM=*(this->gSTLu);this->ReturnValue=ReflexxesAPI::
RML_ERROR_INVALID_INPUT_VALUES;SDTKM->ResultValue=this->ReturnValue;return(this
->ReturnValue);}this->kj2Tc=(Jdf48.SynchronizationBehavior==RMLFlags::
NO_SYNCHRONIZATION);this->F2ivR=((Jdf48.SynchronizationBehavior==RMLFlags::
ONLY_PHASE_SYNCHRONIZATION)||(Jdf48.SynchronizationBehavior==RMLFlags::
PHASE_SYNCHRONIZATION_IF_POSSIBLE));this->kJlKk();this->SynchronizationTime=0.0;
for(i=(0xaf4+4587-0x1cdf);i<this->NumberOfDOFs;i++){if(this->_DBry->
SelectionVector->VecData[i]){if((this->ExecutionTimes->VecData)[i]>this->
SynchronizationTime){this->SynchronizationTime=(this->ExecutionTimes->VecData)[i
];this->YDfwn=i;}}}if((Jdf48.SynchronizationBehavior!=RMLFlags::
NO_SYNCHRONIZATION)&&(zqVEb.MinimumSynchronizationTime>this->SynchronizationTime
)){this->SynchronizationTime=zqVEb.MinimumSynchronizationTime;}if(this->F2ivR){
this->lzEiV();}if((!this->F2ivR)&&(Jdf48.SynchronizationBehavior==RMLFlags::
ONLY_PHASE_SYNCHRONIZATION)){this->Q48EP(*(this->_DBry),this->gSTLu);*SDTKM=*(
this->gSTLu);if(zqVEb.CheckForValidity()){this->ReturnValue=ReflexxesAPI::
RML_ERROR_NO_PHASE_SYNCHRONIZATION;}else{this->ReturnValue=ReflexxesAPI::
RML_ERROR_INVALID_INPUT_VALUES;}SDTKM->ResultValue=this->ReturnValue;return(this
->ReturnValue);}if((Jdf48.SynchronizationBehavior==RMLFlags::
ONLY_TIME_SYNCHRONIZATION)||((Jdf48.SynchronizationBehavior==RMLFlags::
PHASE_SYNCHRONIZATION_IF_POSSIBLE)&&(this->F2ivR==false))){this->VfImR();}else{
this->GuUm1();}}this->gSTLu->CurrentOverrideValue=this->Ghz_l;this->z2xUW=this->
Ghz_l;this->gSTLu->ANewCalculationWasPerformed=MDmqk;this->ReturnValue=this->
Y07KE(this->zzKJk,this->Ghz_l,this->gSTLu);this->gSTLu->
TrajectoryIsPhaseSynchronized=this->F2ivR;this->gSTLu->
DOFWithTheGreatestExecutionTime=this->YDfwn;if(this->kj2Tc){this->gSTLu->
SynchronizationTime=0.0;for(i=(0x1b9+9248-0x25d9);i<this->NumberOfDOFs;i++){if(
this->_DBry->SelectionVector->VecData[i]){if(this->Ghz_l>wjjRw){this->gSTLu->
ExecutionTimes->VecData[i]=((this->ExecutionTimes->VecData)[i]-this->zzKJk+this
->CycleTime)/this->Ghz_l;}else{this->gSTLu->ExecutionTimes->VecData[i]=tlPiC;}if
(this->gSTLu->ExecutionTimes->VecData[i]<0.0){this->gSTLu->ExecutionTimes->
VecData[i]=0.0;}}else{this->gSTLu->ExecutionTimes->VecData[i]=0.0;}}}else{if(
this->Ghz_l>wjjRw){this->gSTLu->SynchronizationTime=(this->SynchronizationTime-
this->zzKJk+this->CycleTime)/this->Ghz_l;}else{this->gSTLu->SynchronizationTime=
tlPiC;}if(this->gSTLu->SynchronizationTime<0.0){this->gSTLu->SynchronizationTime
=0.0;}for(i=(0x19ed+3047-0x25d4);i<this->NumberOfDOFs;i++){if(this->_DBry->
SelectionVector->VecData[i]){this->gSTLu->ExecutionTimes->VecData[i]=this->gSTLu
->SynchronizationTime;}else{this->gSTLu->ExecutionTimes->VecData[i]=0.0;}}}if((
this->xheaR)||(Jdf48.PositionalLimitsBehavior!=RMLFlags::
POSITIONAL_LIMITS_IGNORE)){this->RQuk0(this->zzKJk-this->CycleTime,this->Ghz_l,
this->gSTLu);}if(!this->xheaR){this->chwJj(this->gSTLu);}if(Jdf48.
PositionalLimitsBehavior!=RMLFlags::POSITIONAL_LIMITS_IGNORE){for(i=
(0x1479+4424-0x25c1);i<this->NumberOfDOFs;i++){if(this->_DBry->SelectionVector->
VecData[i]){if((this->gSTLu->MinPosExtremaPositionVectorOnly->VecData[i]<=this->
_DBry->MinPositionVector->VecData[i])||(this->gSTLu->
MaxPosExtremaPositionVectorOnly->VecData[i]>=this->_DBry->MaxPositionVector->
VecData[i])){this->ReturnValue=ReflexxesAPI::RML_ERROR_POSITIONAL_LIMITS;break;}
}}}if(Jdf48.PositionalLimitsBehavior==RMLFlags::
POSITIONAL_LIMITS_ACTIVELY_PREVENT){HIbPA=this->q9K7w(zqVEb,this->gSTLu);if(
HIbPA){this->ReturnValue=ReflexxesAPI::RML_ERROR_POSITIONAL_LIMITS;}}*SDTKM=*(
this->gSTLu);SDTKM->ResultValue=this->ReturnValue;return(this->ReturnValue);}

int TypeIVRMLVelocity::Xvpsd(const double&PxS7M,RMLVelocityOutputParameters*
SDTKM){unsigned int i=(0x1661+2572-0x206d);int kfyzJ=ReflexxesAPI::
RML_FINAL_STATE_REACHED;double uOuSq=PxS7M+this->zzKJk-this->CycleTime;if((this
->ReturnValue!=ReflexxesAPI::RML_WORKING)&&(this->ReturnValue!=ReflexxesAPI::
RML_FINAL_STATE_REACHED)){SDTKM->ResultValue=this->ReturnValue;return(this->
ReturnValue);}if((PxS7M<0.0)||(uOuSq>DQDa3)){SDTKM->ResultValue=ReflexxesAPI::
RML_ERROR_USER_TIME_OUT_OF_RANGE;return(ReflexxesAPI::
RML_ERROR_USER_TIME_OUT_OF_RANGE);}if(SDTKM==NULL){return(ReflexxesAPI::
RML_ERROR_NULL_POINTER);}if(SDTKM->NumberOfDOFs!=this->NumberOfDOFs){SDTKM->
ResultValue=ReflexxesAPI::RML_ERROR_NUMBER_OF_DOFS;return(ReflexxesAPI::
RML_ERROR_NUMBER_OF_DOFS);}SDTKM->ANewCalculationWasPerformed=false;kfyzJ=this->
Y07KE(uOuSq,1.0,SDTKM);SDTKM->TrajectoryIsPhaseSynchronized=this->F2ivR;SDTKM->
DOFWithTheGreatestExecutionTime=this->YDfwn;if(this->kj2Tc){SDTKM->
SynchronizationTime=0.0;for(i=(0x2c3+7033-0x1e3c);i<this->NumberOfDOFs;i++){if(
this->_DBry->SelectionVector->VecData[i]){SDTKM->ExecutionTimes->VecData[i]=(
this->ExecutionTimes->VecData)[i]-this->zzKJk+this->CycleTime-uOuSq;if(SDTKM->
ExecutionTimes->VecData[i]<0.0){SDTKM->ExecutionTimes->VecData[i]=0.0;}}else{
SDTKM->ExecutionTimes->VecData[i]=0.0;}}}else{SDTKM->SynchronizationTime=this->
SynchronizationTime-uOuSq;for(i=(0x10f1+164-0x1195);i<this->NumberOfDOFs;i++){if
(this->_DBry->SelectionVector->VecData[i]){SDTKM->ExecutionTimes->VecData[i]=
this->SynchronizationTime-uOuSq;if(SDTKM->ExecutionTimes->VecData[i]<0.0){SDTKM
->ExecutionTimes->VecData[i]=0.0;}}else{SDTKM->ExecutionTimes->VecData[i]=0.0;}}
}if((this->xheaR)||(this->onYRt.PositionalLimitsBehavior!=RMLFlags::
POSITIONAL_LIMITS_IGNORE)){this->RQuk0(this->zzKJk-this->CycleTime,1.0,SDTKM);}
if(!this->xheaR){this->chwJj(SDTKM);}SDTKM->ResultValue=kfyzJ;return(kfyzJ);}

int TypeIVRMLVelocity::SetupOverrideFilter(const double&_IGXH,const double&kNQ3O
){if((_IGXH<0.0)||(_IGXH>ZPeZQ)||(kNQ3O<0.0)||(kNQ3O>this->
MaxTimeForOverrideFilter)){return(ReflexxesAPI::RML_ERROR_OVERRIDE_OUT_OF_RANGE)
;}this->E24br->Ht9Cv(_IGXH,kNQ3O);this->Ghz_l=_IGXH;this->z2xUW=_IGXH;return(
ReflexxesAPI::RML_NO_ERROR);}

void TypeIVRMLVelocity::TTtbr(void){unsigned int i=(0x20a8+696-0x2360);double 
aj_ej=0.0,tVawk=0.0;if(this->z2xUW>MH_kC){aj_ej=1.0/this->z2xUW;tVawk=1.0/VNvlY(
this->z2xUW);for(i=(0xe2+3381-0xe17);i<this->NumberOfDOFs;i++){if((this->_DBry->
SelectionVector->VecData)[i]){this->_DBry->CurrentVelocityVector->VecData[i]*=
aj_ej;this->_DBry->CurrentAccelerationVector->VecData[i]*=tVawk;}}}else{for(i=
(0xdec+423-0xf93);i<this->NumberOfDOFs;i++){if((this->_DBry->SelectionVector->
VecData)[i]){this->_DBry->CurrentVelocityVector->VecData[i]=0.0;this->_DBry->
CurrentAccelerationVector->VecData[i]=0.0;}}}return;}

bool TypeIVRMLVelocity::q9K7w(const RMLVelocityInputParameters&zqVEb,
RMLVelocityOutputParameters*SDTKM){bool zMYri=false;unsigned int i=
(0xfb4+3366-0x1cda),ip1HJ=(0x180+1127-0x5e7);int j=(0x373+3661-0x11c0);double 
Fe3GR=0.0;RMLVelocityFlags Jdf48;*(this->KZs7z->SelectionVector)=*(zqVEb.
SelectionVector);*(this->KZs7z->CurrentPositionVector)=*(SDTKM->
NewPositionVector);*(this->KZs7z->CurrentVelocityVector)=*(SDTKM->
NewVelocityVector);*(this->KZs7z->CurrentAccelerationVector)=*(SDTKM->
NewAccelerationVector);*(this->KZs7z->MaxAccelerationVector)=*(zqVEb.
MaxAccelerationVector);*(this->KZs7z->MaxJerkVector)=*(zqVEb.MaxJerkVector);*(
this->KZs7z->TargetVelocityVector)=*(this->NkYww);Jdf48.PositionalLimitsBehavior
=RMLFlags::POSITIONAL_LIMITS_IGNORE;Jdf48.SynchronizationBehavior=RMLFlags::
NO_SYNCHRONIZATION;Jdf48.EnableTheCalculationOfTheExtremumMotionStates=true;this
->KZs7z->OverrideValue=1.0;this->z1jCu->Yq2Ls(*(this->KZs7z),this->_fSN4,Jdf48);
for(i=(0x1551+2324-0x1e65);i<this->NumberOfDOFs;i++){if(zqVEb.SelectionVector->
VecData[i]){if((this->_fSN4->PositionValuesAtTargetVelocity->VecData[i]<=zqVEb.
MinPositionVector->VecData[i]-EqzBo)||(this->_fSN4->
PositionValuesAtTargetVelocity->VecData[i]>=zqVEb.MaxPositionVector->VecData[i]+
EqzBo)){this->Cc2Sm->VecData[i]=true;}}}if(qPN_6::UifCr(*(this->Cc2Sm))>
(0x7d7+5049-0x1b90)){zMYri=true;}if(zMYri){*(this->KZs7z->CurrentPositionVector)
=*(zqVEb.CurrentPositionVector);*(this->KZs7z->CurrentVelocityVector)=*(zqVEb.
CurrentVelocityVector);*(this->KZs7z->CurrentAccelerationVector)=*(zqVEb.
CurrentAccelerationVector);*(this->KZs7z->SelectionVector)=*(this->Cc2Sm);this->
z1jCu->Yq2Ls(*(this->KZs7z),this->_fSN4,Jdf48);for(i=(0x7ec+7968-0x270c);i<this
->NumberOfDOFs;i++){if(this->Cc2Sm->VecData[i]){SDTKM->NewPositionVector->
VecData[i]=this->_fSN4->NewPositionVector->VecData[i];SDTKM->NewVelocityVector->
VecData[i]=this->_fSN4->NewVelocityVector->VecData[i];SDTKM->
NewAccelerationVector->VecData[i]=this->_fSN4->NewAccelerationVector->VecData[i]
;SDTKM->MinExtremaTimesVector->VecData[i]=this->_fSN4->MinExtremaTimesVector->
VecData[i];SDTKM->MaxExtremaTimesVector->VecData[i]=this->_fSN4->
MaxExtremaTimesVector->VecData[i];SDTKM->ExecutionTimes->VecData[i]=this->_fSN4
->ExecutionTimes->VecData[i];SDTKM->MinPosExtremaPositionVectorOnly->VecData[i]=
this->_fSN4->MinPosExtremaPositionVectorOnly->VecData[i];SDTKM->
MaxPosExtremaPositionVectorOnly->VecData[i]=this->_fSN4->
MaxPosExtremaPositionVectorOnly->VecData[i];this->_fSN4->
GetMotionStateAtMinPosForOneDOF(i,(SDTKM->MinPosExtremaPositionVectorArray)[i],(
SDTKM->MinPosExtremaVelocityVectorArray)[i],(SDTKM->
MinPosExtremaAccelerationVectorArray)[i]);this->_fSN4->
GetMotionStateAtMaxPosForOneDOF(i,(SDTKM->MaxPosExtremaPositionVectorArray)[i],(
SDTKM->MaxPosExtremaVelocityVectorArray)[i],(SDTKM->
MaxPosExtremaAccelerationVectorArray)[i]);SDTKM->Polynomials->
NumberOfCurrentlyValidSegments[i]=this->_fSN4->Polynomials->
NumberOfCurrentlyValidSegments[i];for(j=(0x1b14+2105-0x234d);j<SDTKM->
Polynomials->NumberOfCurrentlyValidSegments[i];j++){SDTKM->Polynomials->
Coefficients[i][j]=this->_fSN4->Polynomials->Coefficients[i][j];}if((SDTKM->
ExecutionTimes->VecData[i]>Fe3GR)&&(zqVEb.SelectionVector->VecData[i])){Fe3GR=
SDTKM->ExecutionTimes->VecData[i];ip1HJ=i;}}}SDTKM->
DOFWithTheGreatestExecutionTime=i;SDTKM->TrajectoryIsPhaseSynchronized=false;
SDTKM->SynchronizationTime=Fe3GR;SDTKM->ANewCalculationWasPerformed=true;}return
(zMYri);}
