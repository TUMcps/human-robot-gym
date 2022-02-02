






























#include <TypeIVRMLVelocity.h>
#include <TypeIVRMLDefinitions.h>
#include <TypeIVRMLMath.h>
#include <RMLVelocityInputParameters.h>
#include <RMLVelocityOutputParameters.h>
#include <RMLVector.h>
#include <ReflexxesAPI.h>
#include <RMLVelocityFlags.h>
using namespace qPN_6;

bool TypeIVRMLVelocity::tHCRc(void){bool pQgx4=true,GrftJ=false;unsigned int i=
(0x169f+3821-0x258c);double auow6=0.0,Kbi0d=0.0,oJgU5=0.0,azCC3=0.0;for(i=
(0x1d63+2422-0x26d9);i<this->NumberOfDOFs;i++){if((this->qeRBJ->VecData)[i]){(
this->VmKWV->VecData)[i]=(this->_DBry->CurrentVelocityVector->VecData)[i];(this
->BNvJP->VecData)[i]=(this->_DBry->CurrentAccelerationVector->VecData)[i];(this
->o04Be->VecData)[i]=(this->_DBry->TargetVelocityVector->VecData)[i];auow6+=
VNvlY((this->BNvJP->VecData)[i]);Kbi0d+=VNvlY((this->VmKWV->VecData)[i]);oJgU5+=
VNvlY((this->o04Be->VecData)[i]);}else{(this->VmKWV->VecData)[i]=0.0;(this->
BNvJP->VecData)[i]=0.0;(this->o04Be->VecData)[i]=0.0;}}auow6=pbQOc(auow6);Kbi0d=
pbQOc(Kbi0d);oJgU5=pbQOc(oJgU5);if((auow6!=T68ug)&&(auow6!=0.0)){for(i=
(0xd94+1834-0x14be);i<this->NumberOfDOFs;i++){if((this->qeRBJ->VecData)[i]){(
this->BNvJP->VecData)[i]/=auow6;}}}else{for(i=(0x9d+4147-0x10d0);i<this->
NumberOfDOFs;i++){if((this->qeRBJ->VecData)[i]){(this->BNvJP->VecData)[i]=0.0;}}
}if((Kbi0d!=T68ug)&&(Kbi0d!=0.0)){for(i=(0x454+1413-0x9d9);i<this->NumberOfDOFs;
i++){if((this->qeRBJ->VecData)[i]){(this->VmKWV->VecData)[i]/=Kbi0d;}}}else{for(
i=(0xb82+6164-0x2396);i<this->NumberOfDOFs;i++){if((this->qeRBJ->VecData)[i]){(
this->VmKWV->VecData)[i]=0.0;}}}if((oJgU5!=T68ug)&&(oJgU5!=0.0)){for(i=
(0x474+6224-0x1cc4);i<this->NumberOfDOFs;i++){if((this->qeRBJ->VecData)[i]){(
this->o04Be->VecData)[i]/=oJgU5;}}}else{for(i=(0x2469+127-0x24e8);i<this->
NumberOfDOFs;i++){if((this->qeRBJ->VecData)[i]){(this->o04Be->VecData)[i]=0.0;}}
}
azCC3=hNosZ;if((auow6>=Kbi0d)&&(auow6>=oJgU5)&&(auow6>=hNosZ)){*(this->HxNxN)=*(
this->BNvJP);azCC3=auow6;}if((Kbi0d>=auow6)&&(Kbi0d>=oJgU5)&&(Kbi0d>=hNosZ)){*(
this->HxNxN)=*(this->VmKWV);azCC3=Kbi0d;}if((oJgU5>=auow6)&&(oJgU5>=Kbi0d)&&(
oJgU5>=hNosZ)){*(this->HxNxN)=*(this->o04Be);azCC3=oJgU5;}if(azCC3>hNosZ){
GrftJ=true;for(i=(0x1c7+1603-0x80a);i<this->NumberOfDOFs;i++){if((this->qeRBJ->
VecData)[i]){if((m85bi((this->BNvJP->VecData)[i])==m85bi((this->HxNxN->VecData)[
i]))&&(fabs((this->BNvJP->VecData)[i])>hNosZ)){GrftJ=false;break;}}}if(GrftJ){
for(i=(0x17c8+2755-0x228b);i<this->NumberOfDOFs;i++){if((this->qeRBJ->VecData)[i
]){(this->BNvJP->VecData)[i]=-(this->BNvJP->VecData)[i];}}}GrftJ=true;for(i=
(0x11ff+4857-0x24f8);i<this->NumberOfDOFs;i++){if((this->qeRBJ->VecData)[i]){if(
(m85bi((this->VmKWV->VecData)[i])==m85bi((this->HxNxN->VecData)[i]))&&(fabs((
this->VmKWV->VecData)[i])>hNosZ)){GrftJ=false;break;}}}if(GrftJ){for(i=
(0x86d+1427-0xe00);i<this->NumberOfDOFs;i++){if((this->qeRBJ->VecData)[i]){(this
->VmKWV->VecData)[i]=-(this->VmKWV->VecData)[i];}}}GrftJ=true;for(i=
(0x161+2714-0xbfb);i<this->NumberOfDOFs;i++){if((this->qeRBJ->VecData)[i]){if((
m85bi((this->o04Be->VecData)[i])==m85bi((this->HxNxN->VecData)[i]))&&(fabs((this
->o04Be->VecData)[i])>hNosZ)){GrftJ=false;break;}}}if(GrftJ){for(i=
(0xe8b+1487-0x145a);i<this->NumberOfDOFs;i++){if((this->qeRBJ->VecData)[i]){(
this->o04Be->VecData)[i]=-(this->o04Be->VecData)[i];}}}
for(i=(0xeb7+452-0x107b);i<this->NumberOfDOFs;i++){if((this->qeRBJ->VecData)[i])
{if(((fabs((this->HxNxN->VecData)[i]-(this->BNvJP->VecData)[i])>hNosZ)&&(fabs((
this->BNvJP->VecData)[i])>hNosZ))||((fabs((this->HxNxN->VecData)[i]-(this->VmKWV
->VecData)[i])>hNosZ)&&(fabs((this->VmKWV->VecData)[i])>hNosZ))||((fabs((this->
HxNxN->VecData)[i]-(this->o04Be->VecData)[i])>hNosZ)&&(fabs((this->o04Be->
VecData)[i])>hNosZ))){pQgx4=false;break;}}}}else{pQgx4=false;}if(!pQgx4){this->
HxNxN->Set(0.0);}return(pQgx4);}
