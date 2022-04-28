






























#include <TypeIVRMLPosition.h>
#include <TypeIVRMLDefinitions.h>


void TypeIVRMLPosition::ql7rg(void){unsigned int i=(0x6f3+8192-0x26f3);this->
kfikd=false;for(i=(0x1+5655-0x1618);i<this->NumberOfDOFs;i++){if((this->_DBry->
SelectionVector->VecData)[i]){if((this->_DBry->MaxJerkVector->VecData)[i]<t_T7f)
{(this->gE5PU->VecData)[i]=t_T7f/(this->_DBry->MaxJerkVector->VecData)[i];this->
kfikd=true;continue;}else{if((this->_DBry->MaxAccelerationVector->VecData)[i]<
t_T7f){(this->gE5PU->VecData)[i]=t_T7f/(this->_DBry->MaxAccelerationVector->
VecData)[i];this->kfikd=true;continue;}else{if((this->_DBry->MaxVelocityVector->
VecData)[i]<t_T7f){(this->gE5PU->VecData)[i]=t_T7f/(this->_DBry->
MaxVelocityVector->VecData)[i];this->kfikd=true;continue;}else{if((this->_DBry->
MaxVelocityVector->VecData)[i]>Kde65){(this->gE5PU->VecData)[i]=Kde65/(this->
_DBry->MaxVelocityVector->VecData)[i];this->kfikd=true;}else{if((this->_DBry->
MaxAccelerationVector->VecData)[i]>Kde65){(this->gE5PU->VecData)[i]=Kde65/(this
->_DBry->MaxAccelerationVector->VecData)[i];this->kfikd=true;}else{if((this->
_DBry->MaxJerkVector->VecData)[i]>Kde65){(this->gE5PU->VecData)[i]=Kde65/(this->
_DBry->MaxJerkVector->VecData)[i];this->kfikd=true;}else{(this->gE5PU->VecData)[
i]=1.0;continue;}}}}}}if(((this->_DBry->MaxJerkVector->VecData)[i]*(this->gE5PU
->VecData)[i])<t_T7f){(this->gE5PU->VecData)[i]*=t_T7f/((this->_DBry->
MaxJerkVector->VecData)[i]*(this->gE5PU->VecData)[i]);continue;}if(((this->_DBry
->MaxAccelerationVector->VecData)[i]*(this->gE5PU->VecData)[i])<t_T7f){(this->
gE5PU->VecData)[i]*=t_T7f/((this->_DBry->MaxAccelerationVector->VecData)[i]*(
this->gE5PU->VecData)[i]);continue;}if(((this->_DBry->MaxVelocityVector->VecData
)[i]*(this->gE5PU->VecData)[i])<t_T7f){(this->gE5PU->VecData)[i]*=t_T7f/((this->
_DBry->MaxVelocityVector->VecData)[i]*(this->gE5PU->VecData)[i]);continue;}}else
{(this->gE5PU->VecData)[i]=1.0;}}}

void TypeIVRMLPosition::ePMhj(void){unsigned int i=(0x114+7494-0x1e5a);if(this->
kfikd){for(i=(0xb85+6209-0x23c6);i<this->NumberOfDOFs;i++){if((this->_DBry->
SelectionVector->VecData)[i]){(this->_DBry->CurrentPositionVector->VecData)[i]*=
(this->gE5PU->VecData)[i];(this->_DBry->CurrentVelocityVector->VecData)[i]*=(
this->gE5PU->VecData)[i];(this->_DBry->CurrentAccelerationVector->VecData)[i]*=(
this->gE5PU->VecData)[i];(this->_DBry->MaxVelocityVector->VecData)[i]*=(this->
gE5PU->VecData)[i];(this->_DBry->MaxAccelerationVector->VecData)[i]*=(this->
gE5PU->VecData)[i];(this->_DBry->MaxJerkVector->VecData)[i]*=(this->gE5PU->
VecData)[i];(this->_DBry->TargetPositionVector->VecData)[i]*=(this->gE5PU->
VecData)[i];(this->_DBry->TargetVelocityVector->VecData)[i]*=(this->gE5PU->
VecData)[i];}}}}

void TypeIVRMLPosition::e3pE4(RMLPositionOutputParameters*tg6GG)const{unsigned 
int i=(0x1c+8419-0x20ff),j=(0x92d+637-0xbaa);if(this->kfikd){for(i=
(0x19fb+266-0x1b05);i<this->NumberOfDOFs;i++){if((this->wNCv9->VecData)[i]){(
tg6GG->NewPositionVector->VecData)[i]/=(this->gE5PU->VecData)[i];(tg6GG->
NewVelocityVector->VecData)[i]/=(this->gE5PU->VecData)[i];(tg6GG->
NewAccelerationVector->VecData)[i]/=(this->gE5PU->VecData)[i];(tg6GG->
MinPosExtremaPositionVectorOnly->VecData)[i]/=(this->gE5PU->VecData)[i];(tg6GG->
MaxPosExtremaPositionVectorOnly->VecData)[i]/=(this->gE5PU->VecData)[i];for(j=
(0x15c+346-0x2b6);j<this->NumberOfDOFs;j++){(((tg6GG->
MinPosExtremaPositionVectorArray)[i])->VecData)[j]/=(this->gE5PU->VecData)[i];((
(tg6GG->MinPosExtremaVelocityVectorArray)[i])->VecData)[j]/=(this->gE5PU->
VecData)[i];(((tg6GG->MinPosExtremaAccelerationVectorArray)[i])->VecData)[j]/=(
this->gE5PU->VecData)[i];(((tg6GG->MaxPosExtremaPositionVectorArray)[i])->
VecData)[j]/=(this->gE5PU->VecData)[i];(((tg6GG->
MaxPosExtremaVelocityVectorArray)[i])->VecData)[j]/=(this->gE5PU->VecData)[i];((
(tg6GG->MaxPosExtremaAccelerationVectorArray)[i])->VecData)[j]/=(this->gE5PU->
VecData)[i];}for(j=(0x1903+89-0x195c);(int)j<tg6GG->Polynomials->
NumberOfCurrentlyValidSegments[i];j++){tg6GG->Polynomials->Coefficients[i][j].
AccelerationPolynomialCoefficients[(0xd82+2957-0x190f)]/=(this->gE5PU->VecData)[
i];tg6GG->Polynomials->Coefficients[i][j].AccelerationPolynomialCoefficients[
(0x383+3043-0xf65)]/=(this->gE5PU->VecData)[i];tg6GG->Polynomials->Coefficients[
i][j].VelocityPolynomialCoefficients[(0x5d+1353-0x5a6)]/=(this->gE5PU->VecData)[
i];tg6GG->Polynomials->Coefficients[i][j].VelocityPolynomialCoefficients[
(0x1ecf+280-0x1fe6)]/=(this->gE5PU->VecData)[i];tg6GG->Polynomials->Coefficients
[i][j].VelocityPolynomialCoefficients[(0xabb+4673-0x1cfa)]/=(this->gE5PU->
VecData)[i];tg6GG->Polynomials->Coefficients[i][j].
PositionPolynomialCoefficients[(0xf+8395-0x20da)]/=(this->gE5PU->VecData)[i];
tg6GG->Polynomials->Coefficients[i][j].PositionPolynomialCoefficients[
(0x16c5+1907-0x1e37)]/=(this->gE5PU->VecData)[i];tg6GG->Polynomials->
Coefficients[i][j].PositionPolynomialCoefficients[(0x388+8632-0x253e)]/=(this->
gE5PU->VecData)[i];tg6GG->Polynomials->Coefficients[i][j].
PositionPolynomialCoefficients[(0x1399+990-0x1774)]/=(this->gE5PU->VecData)[i];}
}}}}
