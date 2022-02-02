






























#include <TypeIVRMLPosition.h>
#include <TypeIVRMLDecisionTree1A.h>
#include <TypeIVRMLDecisionTree1B1.h>
#include <TypeIVRMLDecisionTree1B2.h>
#include <TypeIVRMLDecisionTree1B3.h>
#include <TypeIVRMLDecisionTree1C.h>
#include <TypeIVRMLDecisionTree2.h>
#include <TypeIVRMLThreadControl.h>
#include <TypeIVRMLMath.h>
#ifdef bzQVt
#include <pthread.h>
#endif
using namespace qPN_6;
#ifdef bzQVt


void*TypeIVRMLPosition::wfj20(void*pHWYU){unsigned int bFhjT=(0xb60+3467-0x18eb)
,KkDzo=BF1yT::YmyrZ,cSYBJ=(0x64f+5934-0x1d7d);TypeIVRMLPosition*i75kT=(
TypeIVRMLPosition*)pHWYU;cSYBJ=i75kT->EuH1E;
pthread_mutex_lock(&(i75kT->rdm6I));i75kT->VR_Fr=true;pthread_mutex_unlock(&(
i75kT->rdm6I));pthread_cond_signal(&(i75kT->waCo8));for(;;){i75kT->EyjZT->g7v8e(
cSYBJ);if(i75kT->EyjZT->DDps5()){break;}i75kT->EyjZT->EQKPL(cSYBJ,&bFhjT,&KkDzo)
;if(KkDzo==BF1yT::nWKRz){vN0kt(i75kT,bFhjT);continue;}if(KkDzo==BF1yT::dPx0Q){
fazSR(i75kT,bFhjT);continue;}if(KkDzo==BF1yT::qJ6kf){GahbM(i75kT,bFhjT);continue
;}}pthread_exit(NULL);return((void*)NULL);
}
#endif


void TypeIVRMLPosition::vN0kt(TypeIVRMLPosition*i75kT,unsigned int&bFhjT){(i75kT
->GwQsU->VecData)[bFhjT]=DKHgG((i75kT->_DBry->CurrentPositionVector->VecData)[
bFhjT],(i75kT->_DBry->CurrentVelocityVector->VecData)[bFhjT],(i75kT->_DBry->
CurrentAccelerationVector->VecData)[bFhjT],(i75kT->_DBry->MaxJerkVector->VecData
)[bFhjT],(i75kT->_DBry->MaxAccelerationVector->VecData)[bFhjT],(i75kT->_DBry->
MaxVelocityVector->VecData)[bFhjT],(i75kT->_DBry->TargetPositionVector->VecData)
[bFhjT],(i75kT->_DBry->TargetVelocityVector->VecData)[bFhjT],i75kT->XOquL.
BehaviorIfInitialStateBreachesConstraints,&((i75kT->qfrNo->VecData)[bFhjT]),&((
i75kT->Q_iq8->VecData)[bFhjT]),&((i75kT->Iwaq1->VecData)[bFhjT]));}

void TypeIVRMLPosition::fazSR(TypeIVRMLPosition*i75kT,unsigned int&bFhjT){R3MMz 
MUzZK;(i75kT->F5Xo9->VecData)[bFhjT]=tlPiC;(i75kT->ecwWM->VecData)[bFhjT]=tlPiC;
MUzZK=ehZic((i75kT->_DBry->CurrentPositionVector->VecData)[bFhjT],(i75kT->_DBry
->CurrentVelocityVector->VecData)[bFhjT],(i75kT->_DBry->
CurrentAccelerationVector->VecData)[bFhjT],(i75kT->_DBry->MaxJerkVector->VecData
)[bFhjT],(i75kT->_DBry->MaxAccelerationVector->VecData)[bFhjT],(i75kT->_DBry->
MaxVelocityVector->VecData)[bFhjT],(i75kT->_DBry->TargetPositionVector->VecData)
[bFhjT],(i75kT->_DBry->TargetVelocityVector->VecData)[bFhjT],i75kT->XOquL.
BehaviorIfInitialStateBreachesConstraints);if(MUzZK==CXiRU){(i75kT->GwQsU->
VecData)[bFhjT]=CcqKD((i75kT->_DBry->CurrentPositionVector->VecData)[bFhjT],(
i75kT->_DBry->CurrentVelocityVector->VecData)[bFhjT],(i75kT->_DBry->
CurrentAccelerationVector->VecData)[bFhjT],(i75kT->_DBry->MaxJerkVector->VecData
)[bFhjT],(i75kT->_DBry->MaxAccelerationVector->VecData)[bFhjT],(i75kT->_DBry->
MaxVelocityVector->VecData)[bFhjT],(i75kT->_DBry->TargetPositionVector->VecData)
[bFhjT],(i75kT->_DBry->TargetVelocityVector->VecData)[bFhjT],(i75kT->Q_iq8->
VecData)[bFhjT],i75kT->XOquL.BehaviorIfInitialStateBreachesConstraints,&((i75kT
->bbZEf->VecData)[bFhjT]),&((i75kT->F5Xo9->VecData)[bFhjT]));}if(MUzZK==GCT0Z){(
i75kT->GwQsU->VecData)[bFhjT]=v746A((i75kT->_DBry->CurrentPositionVector->
VecData)[bFhjT],(i75kT->_DBry->CurrentVelocityVector->VecData)[bFhjT],(i75kT->
_DBry->CurrentAccelerationVector->VecData)[bFhjT],(i75kT->_DBry->MaxJerkVector->
VecData)[bFhjT],(i75kT->_DBry->MaxAccelerationVector->VecData)[bFhjT],(i75kT->
_DBry->MaxVelocityVector->VecData)[bFhjT],(i75kT->_DBry->TargetPositionVector->
VecData)[bFhjT],(i75kT->_DBry->TargetVelocityVector->VecData)[bFhjT],(i75kT->
qfrNo->VecData)[bFhjT],i75kT->XOquL.BehaviorIfInitialStateBreachesConstraints,&(
(i75kT->bbZEf->VecData)[bFhjT]),&((i75kT->F5Xo9->VecData)[bFhjT]),&((i75kT->
ecwWM->VecData)[bFhjT]));}
if(!((i75kT->GwQsU->VecData)[bFhjT])&&((i75kT->F5Xo9->VecData)[bFhjT]!=tlPiC)){
if((i75kT->ecwWM->VecData)[bFhjT]==tlPiC){(i75kT->GwQsU->VecData)[bFhjT]=xlKsn((
i75kT->_DBry->CurrentPositionVector->VecData)[bFhjT],(i75kT->_DBry->
CurrentVelocityVector->VecData)[bFhjT],(i75kT->_DBry->CurrentAccelerationVector
->VecData)[bFhjT],(i75kT->_DBry->MaxJerkVector->VecData)[bFhjT],(i75kT->_DBry->
MaxAccelerationVector->VecData)[bFhjT],(i75kT->_DBry->MaxVelocityVector->VecData
)[bFhjT],(i75kT->_DBry->TargetPositionVector->VecData)[bFhjT],(i75kT->_DBry->
TargetVelocityVector->VecData)[bFhjT],(i75kT->qfrNo->VecData)[bFhjT],(i75kT->
bbZEf->VecData)[bFhjT],i75kT->XOquL.BehaviorIfInitialStateBreachesConstraints,&(
(i75kT->ecwWM->VecData)[bFhjT]));}if(!(i75kT->GwQsU->VecData)[bFhjT]){(i75kT->
ecwWM->VecData)[bFhjT]+=Xu3x0;(i75kT->F5Xo9->VecData)[bFhjT]-=Xu3x0;}}else{(
i75kT->ecwWM->VecData)[bFhjT]=tlPiC;}}

void TypeIVRMLPosition::GahbM(TypeIVRMLPosition*i75kT,unsigned int&bFhjT){(i75kT
->GwQsU->VecData)[bFhjT]=s3TWL((i75kT->_DBry->CurrentPositionVector->VecData)[
bFhjT],(i75kT->_DBry->CurrentVelocityVector->VecData)[bFhjT],(i75kT->_DBry->
CurrentAccelerationVector->VecData)[bFhjT],(i75kT->_DBry->MaxJerkVector->VecData
)[bFhjT],(i75kT->_DBry->MaxAccelerationVector->VecData)[bFhjT],(i75kT->_DBry->
MaxVelocityVector->VecData)[bFhjT],(i75kT->_DBry->TargetPositionVector->VecData)
[bFhjT],(i75kT->_DBry->TargetVelocityVector->VecData)[bFhjT],i75kT->
SynchronizationTime,i75kT->XOquL.BehaviorIfInitialStateBreachesConstraints,&((
i75kT->Polynomials)[bFhjT]));}
