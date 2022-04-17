






























#include <TypeIVRMLPosition.h>
#include <RMLVelocityInputParameters.h>
#include <RMLVelocityOutputParameters.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>
#include <RMLVector.h>
#include <RMLVelocityFlags.h>
#include <TypeIVRMLVelocity.h>


void TypeIVRMLPosition::Q48EP(const RMLPositionInputParameters&zqVEb,
RMLPositionOutputParameters*SDTKM,const RMLPositionFlags&XcMhX){unsigned int i=
(0x1b66+2482-0x2518);*(this->fCNQO->SelectionVector)=*(zqVEb.SelectionVector);*(
this->fCNQO->CurrentPositionVector)=*(zqVEb.CurrentPositionVector);*(this->fCNQO
->CurrentVelocityVector)=*(zqVEb.CurrentVelocityVector);*(this->fCNQO->
CurrentAccelerationVector)=*(zqVEb.CurrentAccelerationVector);*(this->fCNQO->
MaxAccelerationVector)=*(zqVEb.MaxAccelerationVector);*(this->fCNQO->
MaxJerkVector)=*(zqVEb.MaxJerkVector);if(XcMhX.
KeepCurrentVelocityInCaseOfFallbackStrategy){*(this->fCNQO->TargetVelocityVector
)=*(zqVEb.CurrentVelocityVector);}else{*(this->fCNQO->TargetVelocityVector)=*(
zqVEb.AlternativeTargetVelocityVector);}if(XcMhX.SynchronizationBehavior==
RMLFlags::ONLY_PHASE_SYNCHRONIZATION){this->YROav.SynchronizationBehavior=
RMLFlags::ONLY_PHASE_SYNCHRONIZATION;}else{this->YROav.SynchronizationBehavior=
RMLFlags::NO_SYNCHRONIZATION;}this->fCNQO->OverrideValue=1.0;this->
RMLVelocityObject->Yq2Ls(*(this->fCNQO),this->uqgIZ,this->YROav);*(SDTKM->
NewPositionVector)=*(this->uqgIZ->NewPositionVector);*(SDTKM->NewVelocityVector)
=*(this->uqgIZ->NewVelocityVector);*(SDTKM->NewAccelerationVector)=*(this->uqgIZ
->NewAccelerationVector);SDTKM->SynchronizationTime=this->uqgIZ->
GetGreatestExecutionTime();SDTKM->TrajectoryIsPhaseSynchronized=false;SDTKM->
ANewCalculationWasPerformed=true;*(SDTKM->MinPosExtremaPositionVectorOnly)=*(
this->uqgIZ->MinPosExtremaPositionVectorOnly);*(SDTKM->
MaxPosExtremaPositionVectorOnly)=*(this->uqgIZ->MaxPosExtremaPositionVectorOnly)
;*(SDTKM->MinExtremaTimesVector)=*(this->uqgIZ->MinExtremaTimesVector);*(SDTKM->
MaxExtremaTimesVector)=*(this->uqgIZ->MaxExtremaTimesVector);for(i=
(0x463+5920-0x1b83);i<this->NumberOfDOFs;i++){*((SDTKM->
MinPosExtremaPositionVectorArray)[i])=*((this->uqgIZ->
MinPosExtremaPositionVectorArray)[i]);*((SDTKM->MinPosExtremaVelocityVectorArray
)[i])=*((this->uqgIZ->MinPosExtremaVelocityVectorArray)[i]);*((SDTKM->
MinPosExtremaAccelerationVectorArray)[i])=*((this->uqgIZ->
MinPosExtremaAccelerationVectorArray)[i]);*((SDTKM->
MaxPosExtremaPositionVectorArray)[i])=*((this->uqgIZ->
MaxPosExtremaPositionVectorArray)[i]);*((SDTKM->MaxPosExtremaVelocityVectorArray
)[i])=*((this->uqgIZ->MaxPosExtremaVelocityVectorArray)[i]);*((SDTKM->
MaxPosExtremaAccelerationVectorArray)[i])=*((this->uqgIZ->
MaxPosExtremaAccelerationVectorArray)[i]);}}
