






























#include <TypeIVRMLPosition.h>
#include <TypeIVRMLMath.h>
#include <TypeIVRMLPolynomial.h>
#include <RMLPositionOutputParameters.h>
#include <RMLPositionInputParameters.h>
using namespace qPN_6;

void TypeIVRMLPosition::RQuk0(const double&PxS7M,const double&OverrideValue,
RMLPositionOutputParameters*tg6GG)const{unsigned int i=(0x130b+1234-0x17dd),
pAVmT=(0x27d+2-0x27f),kleeB=(0xd1f+4930-0x2061),VrkW5=(0x155c+3754-0x2406);int j
=(0xd8f+1578-0x13b9);double wAE1H=0.0,MhrfZ=0.0,TYWyR=0.0,FWU1y=0.0,VAVJM=0.0;
tg6GG->TrajectoryExceedsTargetPosition=false;for(i=(0x132f+4862-0x262d);i<this->
NumberOfDOFs;i++){if((this->wNCv9->VecData)[i]){
(tg6GG->MinPosExtremaPositionVectorOnly->VecData)[i]=(tg6GG->NewPositionVector->
VecData)[i];(tg6GG->MaxPosExtremaPositionVectorOnly->VecData)[i]=(tg6GG->
NewPositionVector->VecData)[i];(tg6GG->MaxExtremaTimesVector->VecData)[i]=PxS7M;
(tg6GG->MinExtremaTimesVector->VecData)[i]=PxS7M;
for(j=(0xda7+1477-0x136c);j<((this->Polynomials)[i].djbCD-(0x1848+2166-0x20bd));
j++){if((this->Polynomials)[i].FfhZi[j]>PxS7M){if(m85bi((j==(0x1ad+4545-0x136e))
?((this->Polynomials)[i].diBqY[j].zixaV(0.0)):((this->Polynomials)[i].diBqY[j].
zixaV((this->Polynomials)[i].FfhZi[j-(0x14b6+4039-0x247c)])))!=m85bi((this->
Polynomials)[i].diBqY[j].zixaV((this->Polynomials)[i].FfhZi[j]))){(this->
Polynomials)[i].diBqY[j].at3zX(&pAVmT,&MhrfZ,&TYWyR,&FWU1y);if(pAVmT==
(0x1926+3248-0x25d4)){if((((j==(0x11e7+204-0x12b3))?(-GW7kZ):((this->Polynomials
)[i].FfhZi[j-(0x173f+429-0x18eb)]-GW7kZ))<=MhrfZ)&&(((this->Polynomials)[i].
FfhZi[j]+GW7kZ)>=MhrfZ)&&(MhrfZ>PxS7M)){wAE1H=(this->Polynomials)[i].fCxBi[j].
zixaV(MhrfZ);VAVJM=MhrfZ;}else{if((((j==(0x1310+1252-0x17f4))?(-GW7kZ):((this->
Polynomials)[i].FfhZi[j-(0x1858+3311-0x2546)]-GW7kZ))<=TYWyR)&&(((this->
Polynomials)[i].FfhZi[j]+GW7kZ)>=TYWyR)&&(TYWyR>PxS7M)){wAE1H=(this->Polynomials
)[i].fCxBi[j].zixaV(TYWyR);VAVJM=TYWyR;}else{continue;}}}else{if((pAVmT==
(0x809+6478-0x2156))&&(MhrfZ>PxS7M)){wAE1H=(this->Polynomials)[i].fCxBi[j].zixaV
(MhrfZ);VAVJM=MhrfZ;}else{continue;}}if((wAE1H>(tg6GG->
MaxPosExtremaPositionVectorOnly->VecData)[i])&&(VAVJM>PxS7M)){(tg6GG->
MaxPosExtremaPositionVectorOnly->VecData)[i]=wAE1H;(tg6GG->MaxExtremaTimesVector
->VecData)[i]=VAVJM;}if((wAE1H<(tg6GG->MinPosExtremaPositionVectorOnly->VecData)
[i])&&(VAVJM>PxS7M)){(tg6GG->MinPosExtremaPositionVectorOnly->VecData)[i]=wAE1H;
(tg6GG->MinExtremaTimesVector->VecData)[i]=VAVJM;}}}}
if((this->Polynomials)[i].FfhZi[(this->Polynomials)[i].djbCD-
(0x14ff+2585-0x1f16)]>PxS7M){if(((this->_DBry->TargetPositionVector->VecData)[i]
>(tg6GG->MaxPosExtremaPositionVectorOnly->VecData)[i])&&(PxS7M<=(this->
Polynomials)[i].FfhZi[(this->Polynomials)[i].djbCD-(0x10c1+127-0x113e)])){(tg6GG
->MaxPosExtremaPositionVectorOnly->VecData)[i]=(this->_DBry->
TargetPositionVector->VecData)[i];if(this->kj2Tc){tg6GG->MaxExtremaTimesVector->
VecData[i]=this->Q_iq8->VecData[i];}else{tg6GG->MaxExtremaTimesVector->VecData[i
]=this->SynchronizationTime;}}if(((this->_DBry->TargetPositionVector->VecData)[i
]<(tg6GG->MinPosExtremaPositionVectorOnly->VecData)[i])&&(PxS7M<=(this->
Polynomials)[i].FfhZi[(this->Polynomials)[i].djbCD-(0x602+5849-0x1cd9)])){(tg6GG
->MinPosExtremaPositionVectorOnly->VecData)[i]=(this->_DBry->
TargetPositionVector->VecData)[i];if(this->kj2Tc){tg6GG->MinExtremaTimesVector->
VecData[i]=this->Q_iq8->VecData[i];}else{tg6GG->MinExtremaTimesVector->VecData[i
]=this->SynchronizationTime;}}}for(kleeB=(0x8d7+3272-0x159f);kleeB<this->
NumberOfDOFs;kleeB++){if((this->wNCv9->VecData)[kleeB]){for(VrkW5=
(0x9b8+4166-0x19fe);VrkW5<fKhZN;VrkW5++){if((this->Polynomials)[kleeB].FfhZi[
VrkW5]>=(tg6GG->MinExtremaTimesVector->VecData)[i]){break;}}(((tg6GG->
MinPosExtremaPositionVectorArray)[i])->VecData)[kleeB]=(this->Polynomials)[kleeB
].fCxBi[VrkW5].zixaV((tg6GG->MinExtremaTimesVector->VecData)[i]);(((tg6GG->
MinPosExtremaVelocityVectorArray)[i])->VecData)[kleeB]=(this->Polynomials)[kleeB
].diBqY[VrkW5].zixaV((tg6GG->MinExtremaTimesVector->VecData)[i])*OverrideValue;(
((tg6GG->MinPosExtremaAccelerationVectorArray)[i])->VecData)[kleeB]=(this->
Polynomials)[kleeB].NYWGt[VrkW5].zixaV((tg6GG->MinExtremaTimesVector->VecData)[i
])*VNvlY(OverrideValue);(((tg6GG->MinPosExtremaPositionVectorArray)[i])->VecData
)[kleeB]=(this->_DBry->TargetPositionVector->VecData)[kleeB]-((this->LiWKZ->
VecData)[kleeB]-(((tg6GG->MinPosExtremaPositionVectorArray)[i])->VecData)[kleeB]
);for(VrkW5=(0xc28+1664-0x12a8);VrkW5<fKhZN;VrkW5++){if((this->Polynomials)[
kleeB].FfhZi[VrkW5]>=(tg6GG->MaxExtremaTimesVector->VecData)[i]){break;}}(((
tg6GG->MaxPosExtremaPositionVectorArray)[i])->VecData)[kleeB]=(this->Polynomials
)[kleeB].fCxBi[VrkW5].zixaV((tg6GG->MaxExtremaTimesVector->VecData)[i]);(((tg6GG
->MaxPosExtremaVelocityVectorArray)[i])->VecData)[kleeB]=(this->Polynomials)[
kleeB].diBqY[VrkW5].zixaV((tg6GG->MaxExtremaTimesVector->VecData)[i])*
OverrideValue;(((tg6GG->MaxPosExtremaAccelerationVectorArray)[i])->VecData)[
kleeB]=(this->Polynomials)[kleeB].NYWGt[VrkW5].zixaV((tg6GG->
MaxExtremaTimesVector->VecData)[i])*VNvlY(OverrideValue);

(((tg6GG->MaxPosExtremaPositionVectorArray)[i])->VecData)[kleeB]=(this->_DBry->
TargetPositionVector->VecData)[kleeB]-((this->LiWKZ->VecData)[kleeB]-(((tg6GG->
MaxPosExtremaPositionVectorArray)[i])->VecData)[kleeB]);}else{(((tg6GG->
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
VecData[i]<0.0){tg6GG->MinExtremaTimesVector->VecData[i]=0.0;}if(((this->_DBry->
CurrentPositionVector->VecData[i]<this->_DBry->TargetPositionVector->VecData[i])
&&((tg6GG->MaxPosExtremaPositionVectorOnly->VecData)[i]>this->_DBry->
TargetPositionVector->VecData[i]+tFiU3))||((this->_DBry->CurrentPositionVector->
VecData[i]>this->_DBry->TargetPositionVector->VecData[i])&&((tg6GG->
MinPosExtremaPositionVectorOnly->VecData)[i]<this->_DBry->TargetPositionVector->
VecData[i]-tFiU3))){tg6GG->TrajectoryExceedsTargetPosition=true;}}else{
(tg6GG->MinPosExtremaPositionVectorOnly->VecData)[i]=(this->_DBry->
CurrentPositionVector->VecData)[i];(tg6GG->MaxPosExtremaPositionVectorOnly->
VecData)[i]=(this->_DBry->CurrentPositionVector->VecData)[i];(tg6GG->
MinExtremaTimesVector->VecData)[i]=0.0;(tg6GG->MaxExtremaTimesVector->VecData)[i
]=0.0;for(kleeB=(0xee2+1873-0x1633);kleeB<this->NumberOfDOFs;kleeB++){(((tg6GG->
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

void TypeIVRMLPosition::chwJj(RMLPositionOutputParameters*tg6GG)const{unsigned 
int i=(0x1db0+2288-0x26a0),kleeB=(0x2303+123-0x237e);for(i=(0x1168+4564-0x233c);
i<this->NumberOfDOFs;i++){for(kleeB=(0x388+7782-0x21ee);kleeB<this->NumberOfDOFs
;kleeB++){(((tg6GG->MinPosExtremaPositionVectorArray)[i])->VecData)[kleeB]=0.0;(
((tg6GG->MinPosExtremaVelocityVectorArray)[i])->VecData)[kleeB]=0.0;(((tg6GG->
MinPosExtremaAccelerationVectorArray)[i])->VecData)[kleeB]=0.0;(((tg6GG->
MaxPosExtremaPositionVectorArray)[i])->VecData)[kleeB]=0.0;(((tg6GG->
MaxPosExtremaVelocityVectorArray)[i])->VecData)[kleeB]=0.0;(((tg6GG->
MaxPosExtremaAccelerationVectorArray)[i])->VecData)[kleeB]=0.0;}(tg6GG->
MinPosExtremaPositionVectorOnly->VecData)[i]=0.0;(tg6GG->
MaxPosExtremaPositionVectorOnly->VecData)[i]=0.0;(tg6GG->MinExtremaTimesVector->
VecData)[i]=0.0;(tg6GG->MaxExtremaTimesVector->VecData)[i]=0.0;}return;}
