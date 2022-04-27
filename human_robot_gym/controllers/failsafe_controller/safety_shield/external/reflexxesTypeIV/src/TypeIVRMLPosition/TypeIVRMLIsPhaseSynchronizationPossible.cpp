






























#include <TypeIVRMLPosition.h>
#include <TypeIVRMLMath.h>
#include <TypeIVRMLDefinitions.h>
#include <TypeIVRMLStep1IntermediateTimeProfiles.h>
#include <TypeIVRMLStep1Profiles.h>
#include <TypeIVRMLStep1Decisions.h>
#include <TypeIVRMLDecisionTree1A.h>
#include <RMLPositionInputParameters.h>
using namespace qPN_6;

bool TypeIVRMLPosition::tHCRc(RMLDoubleVector*fezGm){bool pQgx4=true,GrftJ=false
;unsigned int i=(0x2ca+363-0x435);double auow6=0.0,Lv0S8=0.0,Kbi0d=0.0,oJgU5=0.0
,azCC3=0.0;for(i=(0x4b7+2343-0xdde);i<this->NumberOfDOFs;i++){if((this->wNCv9->
VecData)[i]){(this->dhSEu->VecData)[i]=(this->_DBry->TargetPositionVector->
VecData)[i]-(this->_DBry->CurrentPositionVector->VecData)[i];(this->VmKWV->
VecData)[i]=(this->_DBry->CurrentVelocityVector->VecData)[i];(this->BNvJP->
VecData)[i]=(this->_DBry->CurrentAccelerationVector->VecData)[i];(this->o04Be->
VecData)[i]=(this->_DBry->TargetVelocityVector->VecData)[i];Lv0S8+=VNvlY((this->
dhSEu->VecData)[i]);auow6+=VNvlY((this->BNvJP->VecData)[i]);Kbi0d+=VNvlY((this->
VmKWV->VecData)[i]);oJgU5+=VNvlY((this->o04Be->VecData)[i]);}else{(this->dhSEu->
VecData)[i]=0.0;(this->VmKWV->VecData)[i]=0.0;(this->BNvJP->VecData)[i]=0.0;(
this->o04Be->VecData)[i]=0.0;}}Lv0S8=pbQOc(Lv0S8);auow6=pbQOc(auow6);Kbi0d=pbQOc
(Kbi0d);oJgU5=pbQOc(oJgU5);if((Lv0S8!=T68ug)&&(Lv0S8!=0.0)){for(i=
(0x701+6563-0x20a4);i<this->NumberOfDOFs;i++){if((this->wNCv9->VecData)[i]){(
this->dhSEu->VecData)[i]/=Lv0S8;}}}else{for(i=(0xa20+5890-0x2122);i<this->
NumberOfDOFs;i++){if((this->wNCv9->VecData)[i]){(this->dhSEu->VecData)[i]=0.0;}}
}if((auow6!=T68ug)&&(auow6!=0.0)){for(i=(0x48+1222-0x50e);i<this->NumberOfDOFs;i
++){if((this->wNCv9->VecData)[i]){(this->BNvJP->VecData)[i]/=auow6;}}}else{for(i
=(0x209+877-0x576);i<this->NumberOfDOFs;i++){if((this->wNCv9->VecData)[i]){(this
->BNvJP->VecData)[i]=0.0;}}}if((Kbi0d!=T68ug)&&(Kbi0d!=0.0)){for(i=
(0x1a6f+2609-0x24a0);i<this->NumberOfDOFs;i++){if((this->wNCv9->VecData)[i]){(
this->VmKWV->VecData)[i]/=Kbi0d;}}}else{for(i=(0x747+4268-0x17f3);i<this->
NumberOfDOFs;i++){if((this->wNCv9->VecData)[i]){(this->VmKWV->VecData)[i]=0.0;}}
}if((oJgU5!=T68ug)&&(oJgU5!=0.0)){for(i=(0x1b8+3532-0xf84);i<this->NumberOfDOFs;
i++){if((this->wNCv9->VecData)[i]){(this->o04Be->VecData)[i]/=oJgU5;}}}else{for(
i=(0x585+2188-0xe11);i<this->NumberOfDOFs;i++){if((this->wNCv9->VecData)[i]){(
this->o04Be->VecData)[i]=0.0;}}}
azCC3=hNosZ;if((Lv0S8>=auow6)&&(Lv0S8>=Kbi0d)&&(Lv0S8>=oJgU5)&&(Lv0S8>=hNosZ)){*
(this->bqLBP)=*(this->dhSEu);azCC3=Lv0S8;this->IVKuG=TypeIVRMLPosition::HtF6w;}
if((auow6>=Lv0S8)&&(auow6>=Kbi0d)&&(auow6>=oJgU5)&&(auow6>=hNosZ)){*(this->bqLBP
)=*(this->BNvJP);azCC3=auow6;this->IVKuG=TypeIVRMLPosition::npWTr;}if((Kbi0d>=
Lv0S8)&&(Kbi0d>=auow6)&&(Kbi0d>=oJgU5)&&(Kbi0d>=hNosZ)){*(this->bqLBP)=*(this->
VmKWV);azCC3=Kbi0d;this->IVKuG=TypeIVRMLPosition::UR0Du;}if((oJgU5>=Lv0S8)&&(
oJgU5>=Kbi0d)&&(oJgU5>=auow6)&&(oJgU5>=hNosZ)){*(this->bqLBP)=*(this->o04Be);
azCC3=oJgU5;this->IVKuG=TypeIVRMLPosition::eC32m;}if(azCC3>hNosZ){
GrftJ=true;for(i=(0x2ea+100-0x34e);i<this->NumberOfDOFs;i++){if((this->wNCv9->
VecData)[i]){if((m85bi((this->dhSEu->VecData)[i])==m85bi((this->bqLBP->VecData)[
i]))&&(fabs((this->dhSEu->VecData)[i])>hNosZ)){GrftJ=false;break;}}}if(GrftJ){
for(i=(0x622+474-0x7fc);i<this->NumberOfDOFs;i++){if((this->wNCv9->VecData)[i]){
(this->dhSEu->VecData)[i]=-(this->dhSEu->VecData)[i];}}}GrftJ=true;for(i=
(0xca3+2557-0x16a0);i<this->NumberOfDOFs;i++){if((this->wNCv9->VecData)[i]){if((
m85bi((this->BNvJP->VecData)[i])==m85bi((this->bqLBP->VecData)[i]))&&(fabs((this
->BNvJP->VecData)[i])>hNosZ)){GrftJ=false;break;}}}if(GrftJ){for(i=
(0x2a+8775-0x2271);i<this->NumberOfDOFs;i++){if((this->wNCv9->VecData)[i]){(this
->BNvJP->VecData)[i]=-(this->BNvJP->VecData)[i];}}}GrftJ=true;for(i=
(0x73+2602-0xa9d);i<this->NumberOfDOFs;i++){if((this->wNCv9->VecData)[i]){if((
m85bi((this->VmKWV->VecData)[i])==m85bi((this->bqLBP->VecData)[i]))&&(fabs((this
->VmKWV->VecData)[i])>hNosZ)){GrftJ=false;break;}}}if(GrftJ){for(i=
(0x914+3265-0x15d5);i<this->NumberOfDOFs;i++){if((this->wNCv9->VecData)[i]){(
this->VmKWV->VecData)[i]=-(this->VmKWV->VecData)[i];}}}GrftJ=true;for(i=
(0x600+3914-0x154a);i<this->NumberOfDOFs;i++){if((this->wNCv9->VecData)[i]){if((
m85bi((this->o04Be->VecData)[i])==m85bi((this->bqLBP->VecData)[i]))&&(fabs((this
->o04Be->VecData)[i])>hNosZ)){GrftJ=false;break;}}}if(GrftJ){for(i=
(0x71f+1550-0xd2d);i<this->NumberOfDOFs;i++){if((this->wNCv9->VecData)[i]){(this
->o04Be->VecData)[i]=-(this->o04Be->VecData)[i];}}}
for(i=(0xee7+1609-0x1530);i<this->NumberOfDOFs;i++){if((this->wNCv9->VecData)[i]
){if(((fabs((this->bqLBP->VecData)[i]-(this->dhSEu->VecData)[i])>(EFIrb*fabs((
this->bqLBP->VecData)[i])))&&(Lv0S8>=hNosZ))||((fabs((this->bqLBP->VecData)[i]-(
this->BNvJP->VecData)[i])>(EFIrb*fabs((this->bqLBP->VecData)[i])))&&(auow6>=
hNosZ))||((fabs((this->bqLBP->VecData)[i]-(this->VmKWV->VecData)[i])>(EFIrb*fabs
((this->bqLBP->VecData)[i])))&&(Kbi0d>=hNosZ))||((fabs((this->bqLBP->VecData)[i]
-(this->o04Be->VecData)[i])>(EFIrb*fabs((this->bqLBP->VecData)[i])))&&(oJgU5>=
hNosZ))){pQgx4=false;break;}}}}else{pQgx4=false;}if(pQgx4){*fezGm=*(this->bqLBP)
;}else{fezGm->Set(0.0);}if(qPN_6::UifCr(*(this->wNCv9))==(0x8e6+5631-0x1ee5)){
fezGm->Set(0.0);pQgx4=true;}return(pQgx4);}
