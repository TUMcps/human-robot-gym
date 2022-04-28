






























#include <TypeIVRMLVelocity.h>
#include <TypeIVRMLDefinitions.h>
#include <RMLVelocityInputParameters.h>
#include <RMLVelocityOutputParameters.h>
using namespace qPN_6;

void TypeIVRMLVelocity::Q48EP(const RMLVelocityInputParameters&zqVEb,
RMLVelocityOutputParameters*SDTKM){unsigned int i=(0x6e1+2390-0x1037);for(i=
(0x1036+1808-0x1746);i<this->NumberOfDOFs;i++){if(zqVEb.SelectionVector->VecData
[i]){(SDTKM->NewPositionVector->VecData)[i]=zqVEb.CurrentPositionVector->VecData
[i]+this->CycleTime*(zqVEb.CurrentVelocityVector->VecData)[i]+0.5*VNvlY(this->
CycleTime)*(zqVEb.CurrentAccelerationVector->VecData)[i];(SDTKM->
NewVelocityVector->VecData)[i]=zqVEb.CurrentVelocityVector->VecData[i]+this->
CycleTime*(zqVEb.CurrentAccelerationVector->VecData)[i];(SDTKM->
NewAccelerationVector->VecData)[i]=zqVEb.CurrentAccelerationVector->VecData[i];}
else{(SDTKM->NewPositionVector->VecData)[i]=zqVEb.CurrentPositionVector->VecData
[i];(SDTKM->NewVelocityVector->VecData)[i]=zqVEb.CurrentVelocityVector->VecData[
i];(SDTKM->NewAccelerationVector->VecData)[i]=zqVEb.CurrentAccelerationVector->
VecData[i];}SDTKM->ExecutionTimes->VecData[i]=0.0;SDTKM->
PositionValuesAtTargetVelocity->VecData[i]=zqVEb.CurrentPositionVector->VecData[
i];}this->chwJj(SDTKM);SDTKM->TrajectoryIsPhaseSynchronized=false;SDTKM->
SynchronizationTime=0.0;SDTKM->DOFWithTheGreatestExecutionTime=
(0xb0+8264-0x20f8);return;}
