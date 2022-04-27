






























#include <TypeIVRMLPosition.h>
#include <TypeIVRMLDefinitions.h>
#include <TypeIVRMLMath.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>
#include <RMLVector.h>
using namespace qPN_6;

void TypeIVRMLPosition::dwhIQ(void){unsigned int i=(0x1049+3774-0x1f07);*(this->
wNCv9)=*(this->_DBry->SelectionVector);for(i=(0x6fa+8050-0x266c);i<this->
NumberOfDOFs;i++){if((this->_DBry->SelectionVector->VecData)[i]){if(((this->
_DBry->TargetVelocityVector->VecData)[i]==0.0)&&((this->Q_iq8->VecData)[i]<=this
->CycleTime)){(this->wNCv9->VecData)[i]=false;





(this->_DBry->CurrentPositionVector->VecData)[i]=(this->_DBry->
TargetPositionVector->VecData)[i]/(this->gE5PU->VecData)[i];(this->_DBry->
CurrentVelocityVector->VecData)[i]=0.0;(this->_DBry->CurrentAccelerationVector->
VecData)[i]=0.0;(this->gE5PU->VecData)[i]=1.0;}}}}
