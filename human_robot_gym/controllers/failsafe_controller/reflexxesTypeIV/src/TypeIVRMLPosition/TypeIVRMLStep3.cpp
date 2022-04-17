






























#include <TypeIVRMLPosition.h>
#include <TypeIVRMLMath.h>
#include <TypeIVRMLDefinitions.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>
#include <ReflexxesAPI.h>


int TypeIVRMLPosition::_XPNu(const double&PxS7M,const double&OverrideValue,
RMLPositionOutputParameters*tg6GG)const{unsigned int i=(0x12df+4125-0x22fc);int 
mbgAC=ReflexxesAPI::RML_FINAL_STATE_REACHED,j=(0x112+6438-0x1a38),kleeB=
(0x964+32-0x984);for(i=(0xc4c+781-0xf59);i<this->NumberOfDOFs;i++){if((this->
wNCv9->VecData)[i]){j=(0xa74+5094-0x1e5a);while((PxS7M>(this->Polynomials)[i].
FfhZi[j])&&(j<fKhZN-(0xf31+5776-0x25c0))){j++;}(tg6GG->NewPositionVector->
VecData)[i]=(this->Polynomials)[i].fCxBi[j].zixaV(PxS7M);(tg6GG->
NewVelocityVector->VecData)[i]=(this->Polynomials)[i].diBqY[j].zixaV(PxS7M)*
OverrideValue;(tg6GG->NewAccelerationVector->VecData)[i]=(this->Polynomials)[i].
NYWGt[j].zixaV(PxS7M)*VNvlY(OverrideValue);if(j<((this->Polynomials)[i].djbCD)-
(0x885+5201-0x1cd5)){mbgAC=ReflexxesAPI::RML_WORKING;}tg6GG->Polynomials->
NumberOfCurrentlyValidSegments[i]=(this->Polynomials)[i].djbCD-j;for(kleeB=
(0x16fa+1772-0x1de6);kleeB<tg6GG->Polynomials->NumberOfCurrentlyValidSegments[i]
;kleeB++){tg6GG->Polynomials->Coefficients[i][kleeB].Time_ValidUntil=(this->
Polynomials)[i].FfhZi[j+kleeB]-(this->zzKJk-this->CycleTime);tg6GG->Polynomials
->Coefficients[i][kleeB].AccelerationPolynomialCoefficients[(0x1801+2888-0x2349)
]=(this->Polynomials)[i].NYWGt[j+kleeB].ChgZj-(this->Polynomials)[i].NYWGt[j+
kleeB].Aw4RZ*(this->Polynomials)[i].NYWGt[j+kleeB].Rwvfx;tg6GG->Polynomials->
Coefficients[i][kleeB].AccelerationPolynomialCoefficients[(0xfe8+5820-0x26a3)]=(
this->Polynomials)[i].NYWGt[j+kleeB].Aw4RZ;tg6GG->Polynomials->Coefficients[i][
kleeB].VelocityPolynomialCoefficients[(0x4d5+7064-0x206d)]=(this->Polynomials)[i
].diBqY[j+kleeB].ChgZj+(this->Polynomials)[i].diBqY[j+kleeB].Rwvfx*((this->
Polynomials)[i].diBqY[j+kleeB].ct8hz*(this->Polynomials)[i].diBqY[j+kleeB].Rwvfx
-(this->Polynomials)[i].diBqY[j+kleeB].Aw4RZ);tg6GG->Polynomials->Coefficients[i
][kleeB].VelocityPolynomialCoefficients[(0x446+3352-0x115d)]=(this->Polynomials)
[i].diBqY[j+kleeB].Aw4RZ-2.0*(this->Polynomials)[i].diBqY[j+kleeB].ct8hz*(this->
Polynomials)[i].diBqY[j+kleeB].Rwvfx;tg6GG->Polynomials->Coefficients[i][kleeB].
VelocityPolynomialCoefficients[(0x1ca+8869-0x246d)]=(this->Polynomials)[i].diBqY
[j+kleeB].ct8hz;tg6GG->Polynomials->Coefficients[i][kleeB].
PositionPolynomialCoefficients[(0x898+6829-0x2345)]=(this->Polynomials)[i].fCxBi
[j+kleeB].ChgZj+(this->Polynomials)[i].fCxBi[j+kleeB].Rwvfx*((this->Polynomials)
[i].fCxBi[j+kleeB].Rwvfx*((this->Polynomials)[i].fCxBi[j+kleeB].ct8hz-(this->
Polynomials)[i].fCxBi[j+kleeB].Rwvfx*(this->Polynomials)[i].fCxBi[j+kleeB].CY0Ek
)-(this->Polynomials)[i].fCxBi[j+kleeB].Aw4RZ);tg6GG->Polynomials->Coefficients[
i][kleeB].PositionPolynomialCoefficients[(0x193+5082-0x156c)]=(this->Polynomials
)[i].fCxBi[j+kleeB].Aw4RZ+(this->Polynomials)[i].fCxBi[j+kleeB].Rwvfx*(3.0*(this
->Polynomials)[i].fCxBi[j+kleeB].CY0Ek*(this->Polynomials)[i].fCxBi[j+kleeB].
Rwvfx-2.0*(this->Polynomials)[i].fCxBi[j+kleeB].ct8hz);tg6GG->Polynomials->
Coefficients[i][kleeB].PositionPolynomialCoefficients[(0x71c+650-0x9a4)]=(this->
Polynomials)[i].fCxBi[j+kleeB].ct8hz-3.0*(this->Polynomials)[i].fCxBi[j+kleeB].
CY0Ek*(this->Polynomials)[i].fCxBi[j+kleeB].Rwvfx;tg6GG->Polynomials->
Coefficients[i][kleeB].PositionPolynomialCoefficients[(0x194+3690-0xffb)]=(this
->Polynomials)[i].fCxBi[j+kleeB].CY0Ek;}}else{(tg6GG->NewPositionVector->VecData
)[i]=(this->_DBry->CurrentPositionVector->VecData)[i];(tg6GG->NewVelocityVector
->VecData)[i]=(this->_DBry->CurrentVelocityVector->VecData)[i];(tg6GG->
NewAccelerationVector->VecData)[i]=(this->_DBry->CurrentAccelerationVector->
VecData)[i];tg6GG->Polynomials->NumberOfCurrentlyValidSegments[i]=
(0x12bd+3700-0x2131);}}return(mbgAC);}
