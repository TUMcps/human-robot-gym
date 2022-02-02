





























#include <TypeIVRMLPosition.h>
#include <TypeIVRMLMath.h>
#include <TypeIVRMLDefinitions.h>
#include <TypeIVRMLThreadControl.h>
#include <TypeIVRMLDecisionTree2.h>
#include <TypeIVRMLStep2WithoutSynchronization.h>
#include <RMLPositionInputParameters.h>
#include <ReflexxesAPI.h>
#ifdef bzQVt
#include <pthread.h>
#endif
using namespace qPN_6;

bool TypeIVRMLPosition::kKWV_(void){unsigned int bFhjT=(0x1b73+2142-0x23d1),i=
(0x113b+1499-0x1716);if((this->F2ivR)&&(!this->gHefb)){

this->ZeFtG();}else{if((this->kj2Tc)||(this->gHefb)){for(i=(0x445+4814-0x1713);i
<this->NumberOfDOFs;i++){if((this->wNCv9->VecData)[i]){_oHSd((this->_DBry->
CurrentPositionVector->VecData)[i],(this->_DBry->CurrentVelocityVector->VecData)
[i],(this->_DBry->CurrentAccelerationVector->VecData)[i],(this->_DBry->
MaxJerkVector->VecData)[i],(this->_DBry->MaxAccelerationVector->VecData)[i],(
this->_DBry->MaxVelocityVector->VecData)[i],(this->_DBry->TargetPositionVector->
VecData)[i],(this->_DBry->TargetVelocityVector->VecData)[i],(this->Iwaq1->
VecData)[i],(this->qfrNo->VecData)[i],this->XOquL.
BehaviorIfInitialStateBreachesConstraints,&((this->Polynomials)[i]));}}}else{
this->EyjZT->NAzWd(this->wNCv9->VecData,BF1yT::qJ6kf);


while(this->EyjZT->I19V0(&bFhjT)){GahbM(this,bFhjT);}
this->EyjZT->IHXy5();}}for(i=(0xb2f+6143-0x232e);i<this->NumberOfDOFs;i++){if((
this->wNCv9->VecData)[i]){if((this->GwQsU->VecData)[i]){return(TypeIVRMLPosition
::T3CdJ);}}}return(TypeIVRMLPosition::omT9P);}
