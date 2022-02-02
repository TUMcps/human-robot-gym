






























#include <TypeIVRMLPosition.h>
#include <TypeIVRMLMath.h>
#include <TypeIVRMLDefinitions.h>
#include <TypeIVRMLThreadControl.h>
#include <TypeIVRMLStep1IntermediateTimeProfiles.h>
#include <TypeIVRMLStep1Profiles.h>
#include <TypeIVRMLStep1RootFunctions.h>
#include <TypeIVRMLQuicksort.h>
#include <TypeIVRMLStep1Decisions.h>
#include <TypeIVRMLDecisionTree1A.h>
#include <TypeIVRMLDecisionTree1B1.h>
#include <TypeIVRMLDecisionTree1B2.h>
#include <TypeIVRMLDecisionTree1B3.h>
#include <TypeIVRMLDecisionTree1C.h>
#include <TypeIVRMLABK.h>
#include <RMLPositionInputParameters.h>
#include <ReflexxesAPI.h>
#ifdef bzQVt
#include <pthread.h>
#endif
using namespace qPN_6;

bool TypeIVRMLPosition::RRkLb(void){bool X2crk=true;double eKiXQ=0.0,Xu4v3=0.0,
aV1io=0.0,MVFEr=0.0,aHBg1=0.0;unsigned int i=(0x575+3151-0x11c4),bFhjT=
(0xbbf+6927-0x26ce),rizeD=(0x1348+4160-0x2388),nLYIv=(0x638+8125-0x25f5);this->
EyjZT->NAzWd(this->_DBry->SelectionVector->VecData,BF1yT::nWKRz);


while(this->EyjZT->I19V0(&bFhjT)){vN0kt(this,bFhjT);}
this->EyjZT->IHXy5();for(i=(0x285+1236-0x759);i<this->NumberOfDOFs;i++){if((this
->_DBry->SelectionVector->VecData)[i]){if((this->GwQsU->VecData)[i]){return(
TypeIVRMLPosition::T3CdJ);}if((this->Q_iq8->VecData)[i]>eKiXQ){eKiXQ=(this->
Q_iq8->VecData)[i];this->c5Mvm=i;}}}
this->dwhIQ();
if((this->kj2Tc)||(qPN_6::UifCr(*(this->wNCv9))==(0xfec+4887-0x2303))){this->
SynchronizationTime=eKiXQ;return(TypeIVRMLPosition::omT9P);}

if(this->F2ivR){
this->F2ivR=tHCRc(this->HxNxN);}if((this->F2ivR)&&(fabs((this->HxNxN->VecData)[
this->c5Mvm])>hNosZ)){Xu4v3=(this->_DBry->MaxJerkVector->VecData)[this->c5Mvm]/
fabs((this->HxNxN->VecData)[this->c5Mvm]);aV1io=(this->_DBry->
MaxAccelerationVector->VecData)[this->c5Mvm]/fabs((this->HxNxN->VecData)[this->
c5Mvm]);MVFEr=(this->_DBry->MaxVelocityVector->VecData)[this->c5Mvm]/fabs((this
->HxNxN->VecData)[this->c5Mvm]);for(i=(0x722+2603-0x114d);i<this->NumberOfDOFs;i
++){if((this->wNCv9->VecData)[i]){(this->jIU8s->VecData)[i]=0.0;(this->CrzAq->
VecData)[i]=fabs(Xu4v3*(this->HxNxN->VecData)[i]);(this->TH6RH->VecData)[i]=fabs
(aV1io*(this->HxNxN->VecData)[i]);(this->wdDbF->VecData)[i]=fabs(MVFEr*(this->
HxNxN->VecData)[i]);if(((this->CrzAq->VecData)[i]>((this->_DBry->MaxJerkVector->
VecData)[i]*(1.0+Fnof1)+hNosZ))||((this->TH6RH->VecData)[i]>((this->_DBry->
MaxAccelerationVector->VecData)[i]*(1.0+Fnof1)+hNosZ))||((this->wdDbF->VecData)[
i]>((this->_DBry->MaxVelocityVector->VecData)[i]*(1.0+Fnof1)+hNosZ))){this->
F2ivR=false;break;}}}}else{this->F2ivR=false;}if(this->F2ivR){*(this->Xmk8y)=*(
this->_DBry->CurrentPositionVector);*(this->VmKWV)=*(this->_DBry->
CurrentVelocityVector);*(this->BNvJP)=*(this->_DBry->CurrentAccelerationVector);
*(this->adP3s)=*(this->_DBry->TargetPositionVector);*(this->o04Be)=*(this->_DBry
->TargetVelocityVector);
for(i=(0x159f+3189-0x2214);i<this->NumberOfDOFs;i++){if((this->wNCv9->VecData)[i
]){if((m85bi((this->adP3s->VecData)[i]-(this->Xmk8y->VecData)[i])!=m85bi((this->
adP3s->VecData)[this->c5Mvm]-(this->Xmk8y->VecData)[this->c5Mvm]))){aepf8(&((
this->Xmk8y->VecData)[i]),&((this->VmKWV->VecData)[i]),&((this->BNvJP->VecData)[
i]),&((this->adP3s->VecData)[i]),&((this->o04Be->VecData)[i]));}}}
for(i=(0x473+1836-0xb9f);i<this->NumberOfDOFs;i++){if((this->wNCv9->VecData)[i])
{K7iaM(&((this->jIU8s->VecData)[i]),&((this->Xmk8y->VecData)[i]),&((this->VmKWV
->VecData)[i]),&((this->BNvJP->VecData)[i]),(this->CrzAq->VecData)[i],(this->
TH6RH->VecData)[i],(this->wdDbF->VecData)[i],this->XOquL.
BehaviorIfInitialStateBreachesConstraints,&((this->adP3s->VecData)[i]),&((this->
o04Be->VecData)[i]));}}switch((this->qfrNo->VecData)[this->c5Mvm]){case Gy8r_:
case IVmDp:for(i=(0x29a+7245-0x1ee7);i<this->NumberOfDOFs;i++){if((this->wNCv9->
VecData)[i]){if((this->BNvJP->VecData)[i]<0.0){aepf8(&((this->Xmk8y->VecData)[i]
),&((this->VmKWV->VecData)[i]),&((this->BNvJP->VecData)[i]),&((this->adP3s->
VecData)[i]),&((this->o04Be->VecData)[i]));}if((this->qfrNo->VecData)[this->
c5Mvm]==IVmDp){
n9iwH(&((this->jIU8s->VecData)[i]),&((this->Xmk8y->VecData)[i]),&((this->VmKWV->
VecData)[i]),&((this->BNvJP->VecData)[i]),(this->CrzAq->VecData)[i]);aepf8(&((
this->Xmk8y->VecData)[i]),&((this->VmKWV->VecData)[i]),&((this->BNvJP->VecData)[
i]),&((this->adP3s->VecData)[i]),&((this->o04Be->VecData)[i]));}
if(!r9BxJ(&gPu6H,&RDM53,&F0eNd,(this->BNvJP->VecData)[i],(this->TH6RH->VecData)[
i],(this->VmKWV->VecData)[i],(this->wdDbF->VecData)[i],(this->o04Be->VecData)[i]
,(this->Xmk8y->VecData)[i],(this->adP3s->VecData)[i],(this->CrzAq->VecData)[i]))
{this->F2ivR=false;break;}}}break;case CWXWH:case KELeG:for(i=
(0xc46+4437-0x1d9b);i<this->NumberOfDOFs;i++){if((this->wNCv9->VecData)[i]){if((
this->BNvJP->VecData)[i]<0.0){aepf8(&((this->Xmk8y->VecData)[i]),&((this->VmKWV
->VecData)[i]),&((this->BNvJP->VecData)[i]),&((this->adP3s->VecData)[i]),&((this
->o04Be->VecData)[i]));}if((this->qfrNo->VecData)[this->c5Mvm]==KELeG){
n9iwH(&((this->jIU8s->VecData)[i]),&((this->Xmk8y->VecData)[i]),&((this->VmKWV->
VecData)[i]),&((this->BNvJP->VecData)[i]),(this->CrzAq->VecData)[i]);aepf8(&((
this->Xmk8y->VecData)[i]),&((this->VmKWV->VecData)[i]),&((this->BNvJP->VecData)[
i]),&((this->adP3s->VecData)[i]),&((this->o04Be->VecData)[i]));}
if(!r9BxJ(&eOy_R,&p9gAB,&X79jJ,(this->BNvJP->VecData)[i],(this->TH6RH->VecData)[
i],(this->VmKWV->VecData)[i],(this->wdDbF->VecData)[i],(this->o04Be->VecData)[i]
,(this->Xmk8y->VecData)[i],(this->adP3s->VecData)[i],(this->CrzAq->VecData)[i]))
{this->F2ivR=false;break;}}}break;case Tuwfw:case VieEo:for(i=(0x22+7012-0x1b86)
;i<this->NumberOfDOFs;i++){if((this->wNCv9->VecData)[i]){if((this->BNvJP->
VecData)[i]<0.0){aepf8(&((this->Xmk8y->VecData)[i]),&((this->VmKWV->VecData)[i])
,&((this->BNvJP->VecData)[i]),&((this->adP3s->VecData)[i]),&((this->o04Be->
VecData)[i]));}if((this->qfrNo->VecData)[this->c5Mvm]==VieEo){
n9iwH(&((this->jIU8s->VecData)[i]),&((this->Xmk8y->VecData)[i]),&((this->VmKWV->
VecData)[i]),&((this->BNvJP->VecData)[i]),(this->CrzAq->VecData)[i]);aepf8(&((
this->Xmk8y->VecData)[i]),&((this->VmKWV->VecData)[i]),&((this->BNvJP->VecData)[
i]),&((this->adP3s->VecData)[i]),&((this->o04Be->VecData)[i]));}
if(!r9BxJ(&p4iJ9,&KsLO9,&S5k0a,(this->BNvJP->VecData)[i],(this->TH6RH->VecData)[
i],(this->VmKWV->VecData)[i],(this->wdDbF->VecData)[i],(this->o04Be->VecData)[i]
,(this->Xmk8y->VecData)[i],(this->adP3s->VecData)[i],(this->CrzAq->VecData)[i]))
{this->F2ivR=false;break;}}}break;case cvsfI:case LVHAx:for(i=
(0x9e8+6420-0x22fc);i<this->NumberOfDOFs;i++){if((this->wNCv9->VecData)[i]){if((
this->BNvJP->VecData)[i]<0.0){aepf8(&((this->Xmk8y->VecData)[i]),&((this->VmKWV
->VecData)[i]),&((this->BNvJP->VecData)[i]),&((this->adP3s->VecData)[i]),&((this
->o04Be->VecData)[i]));}if((this->qfrNo->VecData)[this->c5Mvm]==LVHAx){
n9iwH(&((this->jIU8s->VecData)[i]),&((this->Xmk8y->VecData)[i]),&((this->VmKWV->
VecData)[i]),&((this->BNvJP->VecData)[i]),(this->CrzAq->VecData)[i]);aepf8(&((
this->Xmk8y->VecData)[i]),&((this->VmKWV->VecData)[i]),&((this->BNvJP->VecData)[
i]),&((this->adP3s->VecData)[i]),&((this->o04Be->VecData)[i]));}
if(!r9BxJ(&Ryy_6,&tlbcQ,&QkEpP,(this->BNvJP->VecData)[i],(this->TH6RH->VecData)[
i],(this->VmKWV->VecData)[i],(this->wdDbF->VecData)[i],(this->o04Be->VecData)[i]
,(this->Xmk8y->VecData)[i],(this->adP3s->VecData)[i],(this->CrzAq->VecData)[i]))
{this->F2ivR=false;break;}}}break;case YvMlr:case Ehd4a:for(i=
(0x3cb+6993-0x1f1c);i<this->NumberOfDOFs;i++){if((this->wNCv9->VecData)[i]){if((
this->BNvJP->VecData)[i]<0.0){aepf8(&((this->Xmk8y->VecData)[i]),&((this->VmKWV
->VecData)[i]),&((this->BNvJP->VecData)[i]),&((this->adP3s->VecData)[i]),&((this
->o04Be->VecData)[i]));}if((this->qfrNo->VecData)[this->c5Mvm]==Ehd4a){
n9iwH(&((this->jIU8s->VecData)[i]),&((this->Xmk8y->VecData)[i]),&((this->VmKWV->
VecData)[i]),&((this->BNvJP->VecData)[i]),(this->CrzAq->VecData)[i]);aepf8(&((
this->Xmk8y->VecData)[i]),&((this->VmKWV->VecData)[i]),&((this->BNvJP->VecData)[
i]),&((this->adP3s->VecData)[i]),&((this->o04Be->VecData)[i]));}
if(!sD7wJ((this->BNvJP->VecData)[i],(this->TH6RH->VecData)[i],(this->VmKWV->
VecData)[i],(this->wdDbF->VecData)[i],(this->CrzAq->VecData)[i],false)){this->
F2ivR=false;break;}else{
if(qC4La((this->BNvJP->VecData)[i],(this->TH6RH->VecData)[i],(this->VmKWV->
VecData)[i],(this->wdDbF->VecData)[i],(this->o04Be->VecData)[i],(this->CrzAq->
VecData)[i],true)){this->F2ivR=false;break;}else{
if(!cRSW1((this->BNvJP->VecData)[i],(this->TH6RH->VecData)[i],(this->VmKWV->
VecData)[i],(this->wdDbF->VecData)[i],(this->o04Be->VecData)[i],(this->Xmk8y->
VecData)[i],(this->adP3s->VecData)[i],(this->CrzAq->VecData)[i],false)){this->
F2ivR=false;break;}}}}}break;case JSPnP:case yu32u:for(i=(0x279+1705-0x922);i<
this->NumberOfDOFs;i++){if((this->wNCv9->VecData)[i]){if((this->BNvJP->VecData)[
i]<0.0){aepf8(&((this->Xmk8y->VecData)[i]),&((this->VmKWV->VecData)[i]),&((this
->BNvJP->VecData)[i]),&((this->adP3s->VecData)[i]),&((this->o04Be->VecData)[i]))
;}if((this->qfrNo->VecData)[this->c5Mvm]==yu32u){
n9iwH(&((this->jIU8s->VecData)[i]),&((this->Xmk8y->VecData)[i]),&((this->VmKWV->
VecData)[i]),&((this->BNvJP->VecData)[i]),(this->CrzAq->VecData)[i]);aepf8(&((
this->Xmk8y->VecData)[i]),&((this->VmKWV->VecData)[i]),&((this->BNvJP->VecData)[
i]),&((this->adP3s->VecData)[i]),&((this->o04Be->VecData)[i]));}
if(!sD7wJ((this->BNvJP->VecData)[i],(this->TH6RH->VecData)[i],(this->VmKWV->
VecData)[i],(this->wdDbF->VecData)[i],(this->CrzAq->VecData)[i],false)){this->
F2ivR=false;break;}else{
if(!qC4La((this->BNvJP->VecData)[i],(this->TH6RH->VecData)[i],(this->VmKWV->
VecData)[i],(this->wdDbF->VecData)[i],(this->o04Be->VecData)[i],(this->CrzAq->
VecData)[i],false)){this->F2ivR=false;break;}else{
if(!sg7EJ((this->BNvJP->VecData)[i],(this->TH6RH->VecData)[i],(this->VmKWV->
VecData)[i],(this->wdDbF->VecData)[i],(this->o04Be->VecData)[i],(this->Xmk8y->
VecData)[i],(this->adP3s->VecData)[i],(this->CrzAq->VecData)[i],false)){this->
F2ivR=false;break;}}}}}break;case ihEds:case TFVwj:for(i=(0xb60+3619-0x1983);i<
this->NumberOfDOFs;i++){if((this->wNCv9->VecData)[i]){if((this->BNvJP->VecData)[
i]<0.0){aepf8(&((this->Xmk8y->VecData)[i]),&((this->VmKWV->VecData)[i]),&((this
->BNvJP->VecData)[i]),&((this->adP3s->VecData)[i]),&((this->o04Be->VecData)[i]))
;}if((this->qfrNo->VecData)[this->c5Mvm]==TFVwj){
n9iwH(&((this->jIU8s->VecData)[i]),&((this->Xmk8y->VecData)[i]),&((this->VmKWV->
VecData)[i]),&((this->BNvJP->VecData)[i]),(this->CrzAq->VecData)[i]);aepf8(&((
this->Xmk8y->VecData)[i]),&((this->VmKWV->VecData)[i]),&((this->BNvJP->VecData)[
i]),&((this->adP3s->VecData)[i]),&((this->o04Be->VecData)[i]));}
if(sD7wJ((this->BNvJP->VecData)[i],(this->TH6RH->VecData)[i],(this->VmKWV->
VecData)[i],(this->wdDbF->VecData)[i],(this->CrzAq->VecData)[i],true)){this->
F2ivR=false;break;}else{
if(YqOu_((this->BNvJP->VecData)[i],(this->TH6RH->VecData)[i],(this->VmKWV->
VecData)[i],(this->wdDbF->VecData)[i],(this->o04Be->VecData)[i],(this->CrzAq->
VecData)[i],true)){this->F2ivR=false;break;}else{
if(!TN2L4((this->BNvJP->VecData)[i],(this->TH6RH->VecData)[i],(this->VmKWV->
VecData)[i],(this->wdDbF->VecData)[i],(this->o04Be->VecData)[i],(this->Xmk8y->
VecData)[i],(this->adP3s->VecData)[i],(this->CrzAq->VecData)[i],false)){this->
F2ivR=false;break;}}}}}break;case aBNZo:case SG192:for(i=(0xd81+3409-0x1ad2);i<
this->NumberOfDOFs;i++){if((this->wNCv9->VecData)[i]){if((this->BNvJP->VecData)[
i]<0.0){aepf8(&((this->Xmk8y->VecData)[i]),&((this->VmKWV->VecData)[i]),&((this
->BNvJP->VecData)[i]),&((this->adP3s->VecData)[i]),&((this->o04Be->VecData)[i]))
;}if((this->qfrNo->VecData)[this->c5Mvm]==SG192){
n9iwH(&((this->jIU8s->VecData)[i]),&((this->Xmk8y->VecData)[i]),&((this->VmKWV->
VecData)[i]),&((this->BNvJP->VecData)[i]),(this->CrzAq->VecData)[i]);aepf8(&((
this->Xmk8y->VecData)[i]),&((this->VmKWV->VecData)[i]),&((this->BNvJP->VecData)[
i]),&((this->adP3s->VecData)[i]),&((this->o04Be->VecData)[i]));}
if(sD7wJ((this->BNvJP->VecData)[i],(this->TH6RH->VecData)[i],(this->VmKWV->
VecData)[i],(this->wdDbF->VecData)[i],(this->CrzAq->VecData)[i],true)){this->
F2ivR=false;break;}else{
if(!YqOu_((this->BNvJP->VecData)[i],(this->TH6RH->VecData)[i],(this->VmKWV->
VecData)[i],(this->wdDbF->VecData)[i],(this->o04Be->VecData)[i],(this->CrzAq->
VecData)[i],false)){this->F2ivR=false;break;}else{
if(!Yngfp((this->BNvJP->VecData)[i],(this->VmKWV->VecData)[i],(this->wdDbF->
VecData)[i],(this->o04Be->VecData)[i],(this->Xmk8y->VecData)[i],(this->adP3s->
VecData)[i],(this->CrzAq->VecData)[i],false)){this->F2ivR=false;break;}}}}}break
;case YItCk:for(i=(0x173b+527-0x194a);i<this->NumberOfDOFs;i++){if((this->wNCv9
->VecData)[i]){if((this->BNvJP->VecData)[i]<0.0){aepf8(&((this->Xmk8y->VecData)[
i]),&((this->VmKWV->VecData)[i]),&((this->BNvJP->VecData)[i]),&((this->adP3s->
VecData)[i]),&((this->o04Be->VecData)[i]));}
if(!r9BxJ(&PmRIf,&HcX34,&JI0hH,(this->BNvJP->VecData)[i],(this->TH6RH->VecData)[
i],(this->VmKWV->VecData)[i],(this->wdDbF->VecData)[i],(this->o04Be->VecData)[i]
,(this->Xmk8y->VecData)[i],(this->adP3s->VecData)[i],(this->CrzAq->VecData)[i]))
{this->F2ivR=false;break;}}}break;case n1iUG:for(i=(0x1075+5421-0x25a2);i<this->
NumberOfDOFs;i++){if((this->wNCv9->VecData)[i]){if((this->BNvJP->VecData)[i]<0.0
){aepf8(&((this->Xmk8y->VecData)[i]),&((this->VmKWV->VecData)[i]),&((this->BNvJP
->VecData)[i]),&((this->adP3s->VecData)[i]),&((this->o04Be->VecData)[i]));}
if(!r9BxJ(&l2be0,&sebzR,&lNzRf,(this->BNvJP->VecData)[i],(this->TH6RH->VecData)[
i],(this->VmKWV->VecData)[i],(this->wdDbF->VecData)[i],(this->o04Be->VecData)[i]
,(this->Xmk8y->VecData)[i],(this->adP3s->VecData)[i],(this->CrzAq->VecData)[i]))
{this->F2ivR=false;break;}}}break;default:this->F2ivR=false;break;}}if(this->
F2ivR){this->zAQo1=(unsigned int)(this->qfrNo->VecData)[this->c5Mvm];




switch(this->zAQo1){case Gy8r_:case IVmDp:for(i=(0x1af8+5-0x1afd);i<this->
NumberOfDOFs;i++){if((this->wNCv9->VecData)[i]){CVC_x(&((this->jIU8s->VecData)[i
]),(this->Xmk8y->VecData)[i],(this->adP3s->VecData)[i],(this->VmKWV->VecData)[i]
,(this->wdDbF->VecData)[i],(this->o04Be->VecData)[i],(this->BNvJP->VecData)[i],(
this->TH6RH->VecData)[i],(this->CrzAq->VecData)[i],&X2crk);}}break;case CWXWH:
case KELeG:for(i=(0x1c14+2007-0x23eb);i<this->NumberOfDOFs;i++){if((this->wNCv9
->VecData)[i]){HUFN1(&((this->jIU8s->VecData)[i]),(this->Xmk8y->VecData)[i],(
this->adP3s->VecData)[i],(this->VmKWV->VecData)[i],(this->wdDbF->VecData)[i],(
this->o04Be->VecData)[i],(this->BNvJP->VecData)[i],(this->TH6RH->VecData)[i],(
this->CrzAq->VecData)[i],&X2crk);}}break;case Tuwfw:case VieEo:for(i=
(0x136c+3428-0x20d0);i<this->NumberOfDOFs;i++){if((this->wNCv9->VecData)[i]){
RrpMk(&((this->jIU8s->VecData)[i]),(this->Xmk8y->VecData)[i],(this->adP3s->
VecData)[i],(this->VmKWV->VecData)[i],(this->wdDbF->VecData)[i],(this->o04Be->
VecData)[i],(this->BNvJP->VecData)[i],(this->TH6RH->VecData)[i],(this->CrzAq->
VecData)[i],&X2crk);}}break;case cvsfI:case LVHAx:for(i=(0x4b0+1095-0x8f7);i<
this->NumberOfDOFs;i++){if((this->wNCv9->VecData)[i]){p9TIk(&((this->jIU8s->
VecData)[i]),(this->Xmk8y->VecData)[i],(this->adP3s->VecData)[i],(this->VmKWV->
VecData)[i],(this->wdDbF->VecData)[i],(this->o04Be->VecData)[i],(this->BNvJP->
VecData)[i],(this->TH6RH->VecData)[i],(this->CrzAq->VecData)[i],&X2crk);}}break;
case YvMlr:case Ehd4a:for(i=(0x190+509-0x38d);i<this->NumberOfDOFs;i++){if((this
->wNCv9->VecData)[i]){nkINu(&((this->jIU8s->VecData)[i]),&((this->Xmk8y->VecData
)[i]),&((this->VmKWV->VecData)[i]),&((this->BNvJP->VecData)[i]),(this->wdDbF->
VecData)[i],(this->TH6RH->VecData)[i],(this->CrzAq->VecData)[i]);kvFss(&((this->
jIU8s->VecData)[i]),(this->Xmk8y->VecData)[i],(this->adP3s->VecData)[i],(this->
VmKWV->VecData)[i],(this->o04Be->VecData)[i],(this->BNvJP->VecData)[i],(this->
TH6RH->VecData)[i],(this->CrzAq->VecData)[i],&X2crk);}}break;case JSPnP:case 
yu32u:for(i=(0x378+6524-0x1cf4);i<this->NumberOfDOFs;i++){if((this->wNCv9->
VecData)[i]){nkINu(&((this->jIU8s->VecData)[i]),&((this->Xmk8y->VecData)[i]),&((
this->VmKWV->VecData)[i]),&((this->BNvJP->VecData)[i]),(this->wdDbF->VecData)[i]
,(this->TH6RH->VecData)[i],(this->CrzAq->VecData)[i]);dY7Ow(&((this->jIU8s->
VecData)[i]),(this->Xmk8y->VecData)[i],(this->adP3s->VecData)[i],(this->VmKWV->
VecData)[i],(this->o04Be->VecData)[i],(this->BNvJP->VecData)[i],(this->CrzAq->
VecData)[i],&X2crk);}}break;case ihEds:case TFVwj:for(i=(0x1e55+1241-0x232e);i<
this->NumberOfDOFs;i++){if((this->wNCv9->VecData)[i]){QBBJJ(&((this->jIU8s->
VecData)[i]),&((this->Xmk8y->VecData)[i]),&((this->VmKWV->VecData)[i]),&((this->
BNvJP->VecData)[i]),(this->wdDbF->VecData)[i],(this->CrzAq->VecData)[i]);kvFss(&
((this->jIU8s->VecData)[i]),(this->Xmk8y->VecData)[i],(this->adP3s->VecData)[i],
(this->VmKWV->VecData)[i],(this->o04Be->VecData)[i],(this->BNvJP->VecData)[i],(
this->TH6RH->VecData)[i],(this->CrzAq->VecData)[i],&X2crk);}}break;case aBNZo:
case SG192:for(i=(0x38d+1014-0x783);i<this->NumberOfDOFs;i++){if((this->wNCv9->
VecData)[i]){QBBJJ(&((this->jIU8s->VecData)[i]),&((this->Xmk8y->VecData)[i]),&((
this->VmKWV->VecData)[i]),&((this->BNvJP->VecData)[i]),(this->wdDbF->VecData)[i]
,(this->CrzAq->VecData)[i]);dY7Ow(&((this->jIU8s->VecData)[i]),(this->Xmk8y->
VecData)[i],(this->adP3s->VecData)[i],(this->VmKWV->VecData)[i],(this->o04Be->
VecData)[i],(this->BNvJP->VecData)[i],(this->CrzAq->VecData)[i],&X2crk);}}break;
case YItCk:for(i=(0x191c+2193-0x21ad);i<this->NumberOfDOFs;i++){if((this->wNCv9
->VecData)[i]){LmOWJ(&((this->jIU8s->VecData)[i]),(this->Xmk8y->VecData)[i],(
this->adP3s->VecData)[i],(this->VmKWV->VecData)[i],(this->wdDbF->VecData)[i],(
this->o04Be->VecData)[i],(this->BNvJP->VecData)[i],(this->TH6RH->VecData)[i],(
this->CrzAq->VecData)[i],&X2crk);}}break;case n1iUG:for(i=(0xb75+1917-0x12f2);i<
this->NumberOfDOFs;i++){if((this->wNCv9->VecData)[i]){rkg31(&((this->jIU8s->
VecData)[i]),(this->Xmk8y->VecData)[i],(this->adP3s->VecData)[i],(this->VmKWV->
VecData)[i],(this->wdDbF->VecData)[i],(this->o04Be->VecData)[i],(this->BNvJP->
VecData)[i],(this->TH6RH->VecData)[i],(this->CrzAq->VecData)[i],&X2crk);}}break;
default:this->F2ivR=false;break;}aHBg1=0.0;nLYIv=(0x9f0+5260-0x1e7c);for(i=
(0x1403+2296-0x1cfb);i<this->NumberOfDOFs;i++){if((this->wNCv9->VecData)[i]){
aHBg1+=(this->jIU8s->VecData)[i];nLYIv++;}}if(nLYIv==(0x103f+2712-0x1ad7)){
return(TypeIVRMLPosition::omT9P);}aHBg1/=((double)nLYIv);for(i=(0xd8b+426-0xf35)
;i<this->NumberOfDOFs;i++){if((this->wNCv9->VecData)[i]){if(fabs((this->jIU8s->
VecData)[i]-aHBg1)>(hNosZ+Fnof1*aHBg1)){this->F2ivR=false;break;}}}}if(this->
F2ivR){this->SynchronizationTime=this->Q_iq8->VecData[this->c5Mvm];return(
TypeIVRMLPosition::omT9P);}

this->EyjZT->NAzWd(this->wNCv9->VecData,BF1yT::dPx0Q);


while(this->EyjZT->I19V0(&bFhjT)){fazSR(this,bFhjT);}
this->EyjZT->IHXy5();for(i=(0x1bd+3307-0xea8);i<this->NumberOfDOFs;i++){if((this
->wNCv9->VecData)[i]){if((this->GwQsU->VecData)[i]){return(TypeIVRMLPosition::
T3CdJ);}(this->Q_iq8->VecData)[i]+=Xu3x0;if((this->F5Xo9->VecData)[i]<(this->
Q_iq8->VecData)[i]){(this->F5Xo9->VecData)[i]=(this->Q_iq8->VecData)[i];}if((
this->ecwWM->VecData)[i]<(this->F5Xo9->VecData)[i]){(this->ecwWM->VecData)[i]=(
this->F5Xo9->VecData)[i]=((this->F5Xo9->VecData)[i]+(this->ecwWM->VecData)[i])*
0.5;(this->F5Xo9->VecData)[i]-=Xu3x0;(this->ecwWM->VecData)[i]+=Xu3x0;if((this->
F5Xo9->VecData)[i]<(this->Q_iq8->VecData)[i]){(this->Q_iq8->VecData)[i]=(this->
ecwWM->VecData)[i];(this->F5Xo9->VecData)[i]=tlPiC;(this->ecwWM->VecData)[i]=
tlPiC;}}if((this->ecwWM->VecData)[i]<(this->Q_iq8->VecData)[i]){(this->F5Xo9->
VecData)[i]=tlPiC;(this->ecwWM->VecData)[i]=tlPiC;}}}


for(i=(0x802+5456-0x1d52);i<this->NumberOfDOFs;i++){if(!(this->wNCv9->VecData)[i
]){(this->F5Xo9->VecData)[i]=tlPiC;(this->ecwWM->VecData)[i]=tlPiC;}(this->v580I
->VecData)[i]=(this->F5Xo9->VecData)[i];(this->v580I->VecData)[i+this->
NumberOfDOFs]=(this->ecwWM->VecData)[i];}


TVFwP((0x682+3141-0x12c7),((0x1e6f+31-0x1e8c)*this->NumberOfDOFs-
(0x6a3+4485-0x1827)),&((this->v580I->VecData)[(0x1321+1242-0x17fb)]));
for(rizeD=(0x487+3365-0x11ac);rizeD<(0xe77+2498-0x1837)*this->NumberOfDOFs;rizeD
++){if((this->v580I->VecData)[rizeD]>eKiXQ){break;}}this->SynchronizationTime=
eKiXQ;
while((ZSTS8(this->SynchronizationTime,*(this->F5Xo9),*(this->ecwWM)))&&(rizeD<
(0x4b+4897-0x136a)*this->NumberOfDOFs)){this->SynchronizationTime=(this->v580I->
VecData)[rizeD];rizeD++;}return(TypeIVRMLPosition::omT9P);}

bool TypeIVRMLPosition::ZSTS8(const double&HlaZx,const RMLDoubleVector&izcH4,
const RMLDoubleVector&yeziv)const{unsigned int i;for(i=(0x7cd+1869-0xf1a);i<this
->NumberOfDOFs;i++){if((this->wNCv9->VecData)[i]){if(((izcH4.VecData)[i]<HlaZx)
&&(HlaZx<(yeziv.VecData)[i])){return(true);}}}return(false);}
