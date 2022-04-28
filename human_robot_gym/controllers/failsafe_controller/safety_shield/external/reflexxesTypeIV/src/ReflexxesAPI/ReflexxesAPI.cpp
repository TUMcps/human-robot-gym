
#include <ReflexxesAPI.h>
#include <TypeIVRMLPosition.h>
#include <TypeIVRMLVelocity.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>
#include <RMLPositionFlags.h>
#include <RMLVelocityInputParameters.h>
#include <RMLVelocityOutputParameters.h>
#include <RMLVelocityFlags.h>
#include <stdlib.h>


ReflexxesAPI::ReflexxesAPI(const unsigned int&ggmbr,const double&nsANK,const 
unsigned int&m4tf3,const double&cnY6I){this->NumberOfDOFs=ggmbr;this->
NumberOfOwnThreads=m4tf3;this->CycleTime=nsANK;this->MaxTimeForOverrideFilter=
cnY6I;this->RMLPositionObject=(void*)new TypeIVRMLPosition(ggmbr,nsANK,m4tf3,
cnY6I);this->RMLVelocityObject=(void*)new TypeIVRMLVelocity(ggmbr,nsANK,false,
cnY6I);}

ReflexxesAPI::~ReflexxesAPI(void){delete(TypeIVRMLVelocity*)this->
RMLVelocityObject;delete(TypeIVRMLPosition*)this->RMLPositionObject;this->
RMLVelocityObject=NULL;this->RMLPositionObject=NULL;}

int ReflexxesAPI::RMLPosition(const RMLPositionInputParameters&zqVEb,
RMLPositionOutputParameters*SDTKM,const RMLPositionFlags&Jdf48){return(((
TypeIVRMLPosition*)(this->RMLPositionObject))->Yq2Ls(zqVEb,SDTKM,Jdf48));}

int ReflexxesAPI::RMLPositionAtAGivenSampleTime(const double&PxS7M,
RMLPositionOutputParameters*SDTKM){return(((TypeIVRMLPosition*)(this->
RMLPositionObject))->Xvpsd(PxS7M,SDTKM));}

int ReflexxesAPI::RMLVelocity(const RMLVelocityInputParameters&zqVEb,
RMLVelocityOutputParameters*SDTKM,const RMLVelocityFlags&Jdf48){return(((
TypeIVRMLVelocity*)(this->RMLVelocityObject))->Yq2Ls(zqVEb,SDTKM,Jdf48));}

int ReflexxesAPI::RMLVelocityAtAGivenSampleTime(const double&PxS7M,
RMLVelocityOutputParameters*SDTKM){return(((TypeIVRMLVelocity*)(this->
RMLVelocityObject))->Xvpsd(PxS7M,SDTKM));}

int ReflexxesAPI::SetupOverrideFilter(const double&_IGXH,const double&kNQ3O){int
 vwG9h=RML_ERROR,iHHP3=RML_ERROR;vwG9h=((TypeIVRMLPosition*)(this->
RMLPositionObject))->SetupOverrideFilter(_IGXH,kNQ3O);iHHP3=((TypeIVRMLVelocity*
)(this->RMLVelocityObject))->SetupOverrideFilter(_IGXH,kNQ3O);if((vwG9h!=
RML_NO_ERROR)||(iHHP3!=RML_NO_ERROR)){return(ReflexxesAPI::
RML_ERROR_OVERRIDE_OUT_OF_RANGE);}return(ReflexxesAPI::RML_NO_ERROR);}
