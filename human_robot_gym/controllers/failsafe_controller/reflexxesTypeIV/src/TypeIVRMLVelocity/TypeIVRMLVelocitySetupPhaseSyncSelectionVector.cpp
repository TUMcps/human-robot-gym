






























#include <TypeIVRMLVelocity.h>
#include <TypeIVRMLDefinitions.h>
#include <TypeIVRMLMath.h>
#include <RMLVelocityInputParameters.h>
#include <RMLVector.h>
using namespace qPN_6;

void TypeIVRMLVelocity::OOJL1(void){unsigned int i=(0x2b1+6033-0x1a42);*(this->
qeRBJ)=*(this->_DBry->SelectionVector);for(i=(0x1457+86-0x14ad);i<this->
NumberOfDOFs;i++){if(((this->_DBry->SelectionVector->VecData)[i])&&((this->
ExecutionTimes->VecData)[i]<=this->CycleTime)&&(OI259(0.0,(this->_DBry->
CurrentAccelerationVector->VecData)[i],this->CycleTime*(this->_DBry->
MaxJerkVector->VecData)[i]))&&(OI259(0.0,(this->_DBry->CurrentVelocityVector->
VecData)[i],0.5*VNvlY(this->CycleTime)*(this->_DBry->MaxJerkVector->VecData)[i])
)&&((this->_DBry->TargetVelocityVector->VecData)[i]==0.0)){(this->qeRBJ->VecData
)[i]=false;(this->_DBry->CurrentVelocityVector->VecData)[i]=0.0;(this->_DBry->
CurrentAccelerationVector->VecData)[i]=0.0;}}}
