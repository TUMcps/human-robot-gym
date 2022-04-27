






























#include <TypeIVRMLVelocity.h>
#include <TypeIVRMLDefinitions.h>
#include <TypeIVRMLMath.h>
#include <RMLVelocityInputParameters.h>
#include <RMLVelocityOutputParameters.h>
#include <RMLVector.h>
#include <ReflexxesAPI.h>
using namespace qPN_6;

void TypeIVRMLVelocity::RQuk0(const double&PxS7M,const double&OverrideValue,
RMLVelocityOutputParameters*tg6GG)const{unsigned int i=(0xea+7305-0x1d73),pAVmT=
(0xcf7+4429-0x1e44),kleeB=(0x14af+1171-0x1942),VrkW5=(0x334+6353-0x1c05);int j=
(0x8eb+4299-0x19b6);double wAE1H=0.0,MhrfZ=0.0,TYWyR=0.0,FWU1y=0.0,VAVJM=0.0;for
(i=(0x1344+4245-0x23d9);i<this->NumberOfDOFs;i++){if((this->_DBry->
SelectionVector->VecData)[i]){
(tg6GG->MinPosExtremaPositionVectorOnly->VecData)[i]=(tg6GG->NewPositionVector->
VecData)[i];(tg6GG->MaxPosExtremaPositionVectorOnly->VecData)[i]=(tg6GG->
NewPositionVector->VecData)[i];(tg6GG->MaxExtremaTimesVector->VecData)[i]=PxS7M;
(tg6GG->MinExtremaTimesVector->VecData)[i]=PxS7M;for(j=(0x820+2351-0x114f);j<((
this->Polynomials)[i].djbCD-(0x497+600-0x6ee));j++){if((this->Polynomials)[i].
FfhZi[j]>PxS7M){if(m85bi((j==(0xcff+89-0xd58))?((this->Polynomials)[i].diBqY[j].
zixaV(0.0)):((this->Polynomials)[i].diBqY[j].zixaV((this->Polynomials)[i].FfhZi[
j-(0x1624+4205-0x2690)])))!=m85bi((this->Polynomials)[i].diBqY[j].zixaV((this->
Polynomials)[i].FfhZi[j]))){(this->Polynomials)[i].diBqY[j].at3zX(&pAVmT,&MhrfZ,
&TYWyR,&FWU1y);if(pAVmT==(0xf04+4934-0x2248)){if((((j==(0x10e1+4337-0x21d2))?(-
GW7kZ):((this->Polynomials)[i].FfhZi[j-(0x1e98+462-0x2065)]-GW7kZ))<=MhrfZ)&&(((
this->Polynomials)[i].FfhZi[j]+GW7kZ)>=MhrfZ)&&(MhrfZ>PxS7M)){wAE1H=(this->
Polynomials)[i].fCxBi[j].zixaV(MhrfZ);VAVJM=MhrfZ;}else{if((((j==
(0x16af+1056-0x1acf))?(-GW7kZ):((this->Polynomials)[i].FfhZi[j-
(0x245+8336-0x22d4)]-GW7kZ))<=TYWyR)&&(((this->Polynomials)[i].FfhZi[j]+GW7kZ)>=
TYWyR)&&(TYWyR>PxS7M)){wAE1H=(this->Polynomials)[i].fCxBi[j].zixaV(TYWyR);VAVJM=
TYWyR;}else{continue;}}}else{if((pAVmT==(0x1+5170-0x1432))&&(MhrfZ>PxS7M)){wAE1H
=(this->Polynomials)[i].fCxBi[j].zixaV(MhrfZ);VAVJM=MhrfZ;}else{continue;}}if(
wAE1H>(tg6GG->MaxPosExtremaPositionVectorOnly->VecData)[i]){(tg6GG->
MaxPosExtremaPositionVectorOnly->VecData)[i]=wAE1H;(tg6GG->MaxExtremaTimesVector
->VecData)[i]=VAVJM;}if(wAE1H<(tg6GG->MinPosExtremaPositionVectorOnly->VecData)[
i]){(tg6GG->MinPosExtremaPositionVectorOnly->VecData)[i]=wAE1H;(tg6GG->
MinExtremaTimesVector->VecData)[i]=VAVJM;}}}}wAE1H=(this->Polynomials)[i].fCxBi[
((this->Polynomials)[i].djbCD-(0xa86+6418-0x2397))].zixaV((this->Polynomials)[i]
.FfhZi[((this->Polynomials)[i].djbCD-(0xc90+2068-0x14a2))]);VAVJM=(this->
Polynomials)[i].FfhZi[((this->Polynomials)[i].djbCD-(0x13e0+2520-0x1db6))];if((
this->Polynomials)[i].FfhZi[(this->Polynomials)[i].djbCD-(0xa18+1601-0x1058)]>
PxS7M){if((wAE1H<(tg6GG->MinPosExtremaPositionVectorOnly->VecData)[i])&&(PxS7M<=
(this->Polynomials)[i].FfhZi[((this->Polynomials)[i].djbCD-(0x2f3+3632-0x1121))]
)){(tg6GG->MinPosExtremaPositionVectorOnly->VecData)[i]=wAE1H;(tg6GG->
MinExtremaTimesVector->VecData)[i]=VAVJM;}if((wAE1H>(tg6GG->
MaxPosExtremaPositionVectorOnly->VecData)[i])&&(PxS7M<=(this->Polynomials)[i].
FfhZi[((this->Polynomials)[i].djbCD-(0xbcd+4013-0x1b78))])){(tg6GG->
MaxPosExtremaPositionVectorOnly->VecData)[i]=wAE1H;(tg6GG->MaxExtremaTimesVector
->VecData)[i]=VAVJM;}}for(kleeB=(0xaf3+3509-0x18a8);kleeB<this->NumberOfDOFs;
kleeB++){if((this->_DBry->SelectionVector->VecData)[kleeB]){for(VrkW5=
(0x15bd+1176-0x1a55);VrkW5<fKhZN;VrkW5++){if((this->Polynomials)[kleeB].FfhZi[
VrkW5]>=(tg6GG->MinExtremaTimesVector->VecData)[i]){break;}}(((tg6GG->
MinPosExtremaPositionVectorArray)[i])->VecData)[kleeB]=(this->Polynomials)[kleeB
].fCxBi[VrkW5].zixaV((tg6GG->MinExtremaTimesVector->VecData)[i]);(((tg6GG->
MinPosExtremaVelocityVectorArray)[i])->VecData)[kleeB]=(this->Polynomials)[kleeB
].diBqY[VrkW5].zixaV((tg6GG->MinExtremaTimesVector->VecData)[i])*OverrideValue;(
((tg6GG->MinPosExtremaAccelerationVectorArray)[i])->VecData)[kleeB]=(this->
Polynomials)[kleeB].NYWGt[VrkW5].zixaV((tg6GG->MinExtremaTimesVector->VecData)[i
])*VNvlY(OverrideValue);for(VrkW5=(0x5a4+5136-0x19b4);VrkW5<fKhZN;VrkW5++){if((
this->Polynomials)[kleeB].FfhZi[VrkW5]>=(tg6GG->MaxExtremaTimesVector->VecData)[
i]){break;}}(((tg6GG->MaxPosExtremaPositionVectorArray)[i])->VecData)[kleeB]=(
this->Polynomials)[kleeB].fCxBi[VrkW5].zixaV((tg6GG->MaxExtremaTimesVector->
VecData)[i]);(((tg6GG->MaxPosExtremaVelocityVectorArray)[i])->VecData)[kleeB]=(
this->Polynomials)[kleeB].diBqY[VrkW5].zixaV((tg6GG->MaxExtremaTimesVector->
VecData)[i])*OverrideValue;(((tg6GG->MaxPosExtremaAccelerationVectorArray)[i])->
VecData)[kleeB]=(this->Polynomials)[kleeB].NYWGt[VrkW5].zixaV((tg6GG->
MaxExtremaTimesVector->VecData)[i])*VNvlY(OverrideValue);}else{(((tg6GG->
MinPosExtremaPositionVectorArray)[i])->VecData)[kleeB]=(this->_DBry->
CurrentPositionVector->VecData)[kleeB];(((tg6GG->
MinPosExtremaVelocityVectorArray)[i])->VecData)[kleeB]=(this->_DBry->
CurrentVelocityVector->VecData)[kleeB];(((tg6GG->
MinPosExtremaAccelerationVectorArray)[i])->VecData)[kleeB]=(this->_DBry->
CurrentAccelerationVector->VecData)[kleeB];(((tg6GG->
MaxPosExtremaPositionVectorArray)[i])->VecData)[kleeB]=(this->_DBry->
CurrentPositionVector->VecData)[kleeB];(((tg6GG->
MaxPosExtremaVelocityVectorArray)[i])->VecData)[kleeB]=(this->_DBry->
CurrentVelocityVector->VecData)[kleeB];(((tg6GG->
MaxPosExtremaAccelerationVectorArray)[i])->VecData)[kleeB]=(this->_DBry->
CurrentAccelerationVector->VecData)[kleeB];}}if(OverrideValue>wjjRw){tg6GG->
MaxExtremaTimesVector->VecData[i]=(tg6GG->MaxExtremaTimesVector->VecData[i]-
PxS7M)/OverrideValue;}else{tg6GG->MaxExtremaTimesVector->VecData[i]=tlPiC;}if(
tg6GG->MaxExtremaTimesVector->VecData[i]<0.0){tg6GG->MaxExtremaTimesVector->
VecData[i]=0.0;}if(OverrideValue>wjjRw){tg6GG->MinExtremaTimesVector->VecData[i]
=(tg6GG->MinExtremaTimesVector->VecData[i]-PxS7M)/OverrideValue;}else{tg6GG->
MinExtremaTimesVector->VecData[i]=tlPiC;}if(tg6GG->MinExtremaTimesVector->
VecData[i]<0.0){tg6GG->MinExtremaTimesVector->VecData[i]=0.0;}}else{(tg6GG->
MinPosExtremaPositionVectorOnly->VecData)[i]=(this->_DBry->CurrentPositionVector
->VecData)[i];(tg6GG->MaxPosExtremaPositionVectorOnly->VecData)[i]=(this->_DBry
->CurrentPositionVector->VecData)[i];(tg6GG->MinExtremaTimesVector->VecData)[i]=
0.0;(tg6GG->MaxExtremaTimesVector->VecData)[i]=0.0;for(kleeB=
(0x1484+3769-0x233d);kleeB<this->NumberOfDOFs;kleeB++){(((tg6GG->
MinPosExtremaPositionVectorArray)[i])->VecData)[kleeB]=(this->_DBry->
CurrentPositionVector->VecData)[kleeB];(((tg6GG->
MinPosExtremaVelocityVectorArray)[i])->VecData)[kleeB]=(this->_DBry->
CurrentVelocityVector->VecData)[kleeB];(((tg6GG->
MinPosExtremaAccelerationVectorArray)[i])->VecData)[kleeB]=(this->_DBry->
CurrentAccelerationVector->VecData)[kleeB];(((tg6GG->
MaxPosExtremaPositionVectorArray)[i])->VecData)[kleeB]=(this->_DBry->
CurrentPositionVector->VecData)[kleeB];(((tg6GG->
MaxPosExtremaVelocityVectorArray)[i])->VecData)[kleeB]=(this->_DBry->
CurrentVelocityVector->VecData)[kleeB];(((tg6GG->
MaxPosExtremaAccelerationVectorArray)[i])->VecData)[kleeB]=(this->_DBry->
CurrentAccelerationVector->VecData)[kleeB];}}}return;}

void TypeIVRMLVelocity::chwJj(RMLVelocityOutputParameters*tg6GG)const{unsigned 
int i=(0xed7+2176-0x1757),kleeB=(0x2293+510-0x2491);for(i=(0x834+191-0x8f3);i<
this->NumberOfDOFs;i++){for(kleeB=(0xd90+4409-0x1ec9);kleeB<this->NumberOfDOFs;
kleeB++){(((tg6GG->MinPosExtremaPositionVectorArray)[i])->VecData)[kleeB]=0.0;((
(tg6GG->MinPosExtremaVelocityVectorArray)[i])->VecData)[kleeB]=0.0;(((tg6GG->
MinPosExtremaAccelerationVectorArray)[i])->VecData)[kleeB]=0.0;(((tg6GG->
MaxPosExtremaPositionVectorArray)[i])->VecData)[kleeB]=0.0;(((tg6GG->
MaxPosExtremaVelocityVectorArray)[i])->VecData)[kleeB]=0.0;(((tg6GG->
MaxPosExtremaAccelerationVectorArray)[i])->VecData)[kleeB]=0.0;}(tg6GG->
MinPosExtremaPositionVectorOnly->VecData)[i]=0.0;(tg6GG->
MaxPosExtremaPositionVectorOnly->VecData)[i]=0.0;(tg6GG->MinExtremaTimesVector->
VecData)[i]=0.0;(tg6GG->MaxExtremaTimesVector->VecData)[i]=0.0;}}
