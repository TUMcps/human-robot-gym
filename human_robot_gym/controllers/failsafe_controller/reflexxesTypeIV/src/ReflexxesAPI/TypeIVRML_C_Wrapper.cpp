



















#include <TypeIVRML_C_Wrapper.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>
#include <ReflexxesAPI.h>
#include <RMLPositionFlags.h>



bool MEktg=false;
unsigned int ndofs=(0xd01+6236-0x255d);
ReflexxesAPI*zyNEr=NULL;
RMLPositionInputParameters*IP=NULL;
RMLPositionOutputParameters*OP=NULL;
RMLPositionFlags j2lNR;
RMLVelocityInputParameters*cBL2i=NULL;
RMLVelocityOutputParameters*o27lP=NULL;
RMLVelocityFlags g1bnB;

int TypeIVRMLCreate(const unsigned int NumberOfDOFs,const double nsANK)
{
    if((MEktg)||(nsANK<=0.0))
    {
        return(-(0xab5+6376-0x239c));
    }

    zyNEr=new ReflexxesAPI(NumberOfDOFs,nsANK);
    IP=new RMLPositionInputParameters(NumberOfDOFs);
    OP=new RMLPositionOutputParameters(NumberOfDOFs);
    cBL2i=new RMLVelocityInputParameters(NumberOfDOFs);
    o27lP=new RMLVelocityOutputParameters(NumberOfDOFs);
    MEktg=true;
    ndofs=NumberOfDOFs;
    return((0x1c51+326-0x1d97));}

int TypeIVRMLPosition(  const double*iPzPj,
                        const double*AQzqu,
                        const double*k6sf4,
                        const double*hxixi,
                        const double*DAwhk,
                        const double*b4jXr,
                        const double*ldeCA,
                        const double*HcINC,
                        const int*SelectionVector,
                        double*zpjvt,
                        double*JMglI,
                        double* Emjyn)
{
unsigned int i=0;
int ResultValue=0;
if(!MEktg)
{
    return(-1);
}
for(i=0;i<ndofs;i++)
{
    IP->CurrentPositionVector->VecData[i]=iPzPj[i];
    IP->CurrentVelocityVector->VecData[i]=AQzqu[i];
    IP->CurrentAccelerationVector->VecData[i]=k6sf4[i];
    IP->MaxVelocityVector->VecData[i]=hxixi[i];
    IP->MaxAccelerationVector->VecData[i]=DAwhk[i];
    IP->MaxJerkVector->VecData[i]=b4jXr[i];
    IP->TargetPositionVector->VecData[i]=ldeCA[i];
    IP->TargetVelocityVector->VecData[i]=HcINC[i];
    IP->SelectionVector->VecData[i]=(SelectionVector[i]>0);
}
    ResultValue=zyNEr->RMLPosition(*IP,OP,j2lNR);

for(i=0;i<ndofs;i++)
{
    zpjvt[i]=OP->NewPositionVector->VecData[i];
    JMglI[i]=OP->NewVelocityVector->VecData[i];
    Emjyn[i]=OP->NewAccelerationVector->VecData[i];
}
return(ResultValue);}

int TypeIVRMLVelocity(const double*iPzPj,const double*AQzqu,const double*k6sf4,
const double*DAwhk,const double*b4jXr,const double*HcINC,const int*
SelectionVector,double*zpjvt,double*JMglI,double*Emjyn){unsigned int i=
(0x5cb+784-0x8db);int ResultValue=(0x13ba+1202-0x186c);if(!MEktg){return(-
(0x2d+5261-0x14b9));}for(i=(0x1ac4+300-0x1bf0);i<ndofs;i++){cBL2i->
CurrentPositionVector->VecData[i]=iPzPj[i];cBL2i->CurrentVelocityVector->VecData
[i]=AQzqu[i];cBL2i->CurrentAccelerationVector->VecData[i]=k6sf4[i];cBL2i->
MaxAccelerationVector->VecData[i]=DAwhk[i];cBL2i->MaxJerkVector->VecData[i]=
b4jXr[i];cBL2i->TargetVelocityVector->VecData[i]=HcINC[i];cBL2i->SelectionVector
->VecData[i]=(SelectionVector[i]>(0x0+4830-0x12de));}ResultValue=zyNEr->
RMLVelocity(*cBL2i,o27lP,g1bnB);for(i=(0x1c0d+2302-0x250b);i<ndofs;i++){zpjvt[i]
=o27lP->NewPositionVector->VecData[i];JMglI[i]=o27lP->NewVelocityVector->VecData
[i];Emjyn[i]=o27lP->NewAccelerationVector->VecData[i];}return(ResultValue);}

int TypeIVRMLDestroy(void){if(!MEktg){return(-(0x618+5265-0x1aa8));}delete zyNEr
;delete IP;delete OP;delete cBL2i;delete o27lP;ndofs=(0x12ad+977-0x167e);
MEktg=false;return((0x893+5157-0x1cb8));}
