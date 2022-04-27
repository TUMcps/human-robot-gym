





























#include <TypeIVRMLPosition.h>
#include <TypeIVRMLMath.h>
using namespace qPN_6;

void TypeIVRMLPosition::ZeFtG(void){unsigned int i=(0x366+8734-0x2584);int j=
(0x20fd+631-0x2374);double sypVN=0.0,G2Eq1=0.0,EPDb1=0.0,OFocS=0.0,AvRzx=0.0,
s5cUU=0.0,iSBAT=0.0,V3XaR=0.0,mTSER=0.0,begFk=0.0,yyOCu=0.0,UyfRs=0.0,Rwvfx=0.0,
EJYuc=0.0,gMHCB=0.0,bxFy8=0.0,vjkPz=0.0,m8dP2=0.0;
GahbM(this,this->c5Mvm);if(!((this->GwQsU->VecData)[this->c5Mvm])){



this->SynchronizationTime=(((this->Polynomials)[this->c5Mvm]).FfhZi)[((this->
Polynomials)[this->c5Mvm]).djbCD-(0xd38+6012-0x24b2)];
for(i=(0x6cd+2283-0xfb8);i<this->NumberOfDOFs;i++){if((this->wNCv9->VecData)[i]
&&(i!=this->c5Mvm)){EJYuc=(this->HxNxN->VecData)[i]/(this->HxNxN->VecData)[this
->c5Mvm];for(j=(0x1dc6+2339-0x26e9);j<((this->Polynomials)[this->c5Mvm]).djbCD;j
++){((this->Polynomials)[this->c5Mvm]).fCxBi[j].hYaB2(&OFocS,&EPDb1,&G2Eq1,&
sypVN,&Rwvfx);((this->Polynomials)[this->c5Mvm]).diBqY[j].hYaB2(&V3XaR,&iSBAT,&
s5cUU,&AvRzx,&Rwvfx);((this->Polynomials)[this->c5Mvm]).NYWGt[j].hYaB2(&UyfRs,&
yyOCu,&begFk,&mTSER,&Rwvfx);OFocS*=EJYuc;EPDb1*=EJYuc;G2Eq1*=EJYuc;sypVN=((this
->_DBry->CurrentPositionVector->VecData)[i]+(sypVN-(this->_DBry->
CurrentPositionVector->VecData)[this->c5Mvm])*EJYuc);iSBAT*=EJYuc;s5cUU*=EJYuc;
AvRzx*=EJYuc;begFk*=EJYuc;mTSER*=EJYuc;((this->Polynomials)[i]).fCxBi[j].jGQqQ(
OFocS,EPDb1,G2Eq1,sypVN,Rwvfx);((this->Polynomials)[i]).diBqY[j].jGQqQ(V3XaR,
iSBAT,s5cUU,AvRzx,Rwvfx);((this->Polynomials)[i]).NYWGt[j].jGQqQ(UyfRs,yyOCu,
begFk,mTSER,Rwvfx);((this->Polynomials)[i]).FfhZi[j]=((this->Polynomials)[this->
c5Mvm]).FfhZi[j];}((this->Polynomials)[i]).djbCD=((this->Polynomials)[this->
c5Mvm]).djbCD;


if(this->SynchronizationTime>this->CycleTime){bxFy8=(this->_DBry->
CurrentAccelerationVector->VecData)[i]-((this->Polynomials)[i]).NYWGt[
(0xd18+4641-0x1f39)].zixaV(0.0);for(j=(0x24d8+253-0x25d5);j<((this->Polynomials)
[i]).djbCD;j++){((this->Polynomials)[i]).fCxBi[j].hYaB2(&OFocS,&EPDb1,&G2Eq1,&
sypVN,&Rwvfx);((this->Polynomials)[i]).diBqY[j].hYaB2(&V3XaR,&iSBAT,&s5cUU,&
AvRzx,&Rwvfx);((this->Polynomials)[i]).NYWGt[j].hYaB2(&UyfRs,&yyOCu,&begFk,&
mTSER,&Rwvfx);begFk-=bxFy8/this->SynchronizationTime;mTSER+=bxFy8+Rwvfx*bxFy8/
this->SynchronizationTime;s5cUU=mTSER;EPDb1=0.5*mTSER;((this->Polynomials)[i]).
fCxBi[j].jGQqQ(OFocS,EPDb1,G2Eq1,sypVN,Rwvfx);((this->Polynomials)[i]).diBqY[j].
jGQqQ(V3XaR,iSBAT,s5cUU,AvRzx,Rwvfx);((this->Polynomials)[i]).NYWGt[j].jGQqQ(
UyfRs,yyOCu,begFk,mTSER,Rwvfx);}gMHCB=(this->_DBry->CurrentVelocityVector->
VecData)[i]-((this->Polynomials)[i]).diBqY[(0x1552+239-0x1641)].zixaV(0.0);m8dP2
=(this->_DBry->TargetVelocityVector->VecData)[i]-((this->Polynomials)[i]).diBqY[
((this->Polynomials)[i]).djbCD-(0x128+8793-0x2380)].zixaV(this->
SynchronizationTime);for(j=(0x331+1415-0x8b8);j<((this->Polynomials)[i]).djbCD;j
++){((this->Polynomials)[i]).fCxBi[j].hYaB2(&OFocS,&EPDb1,&G2Eq1,&sypVN,&Rwvfx);
((this->Polynomials)[i]).diBqY[j].hYaB2(&V3XaR,&iSBAT,&s5cUU,&AvRzx,&Rwvfx);
s5cUU+=(m8dP2-gMHCB)/this->SynchronizationTime;AvRzx+=gMHCB-Rwvfx*(m8dP2-gMHCB)/
this->SynchronizationTime;G2Eq1=AvRzx;((this->Polynomials)[i]).fCxBi[j].jGQqQ(
OFocS,EPDb1,G2Eq1,sypVN,Rwvfx);((this->Polynomials)[i]).diBqY[j].jGQqQ(V3XaR,
iSBAT,s5cUU,AvRzx,Rwvfx);}vjkPz=(this->_DBry->TargetPositionVector->VecData)[i]-
((this->Polynomials)[i]).fCxBi[((this->Polynomials)[i]).djbCD-
(0x189+6702-0x1bb6)].zixaV(this->SynchronizationTime);for(j=(0x5dd+1755-0xcb8);j
<((this->Polynomials)[i]).djbCD;j++){((this->Polynomials)[i]).fCxBi[j].hYaB2(&
OFocS,&EPDb1,&G2Eq1,&sypVN,&Rwvfx);G2Eq1+=vjkPz/this->SynchronizationTime;sypVN
-=Rwvfx*vjkPz/this->SynchronizationTime;((this->Polynomials)[i]).fCxBi[j].jGQqQ(
OFocS,EPDb1,G2Eq1,sypVN,Rwvfx);}}
}}}}
