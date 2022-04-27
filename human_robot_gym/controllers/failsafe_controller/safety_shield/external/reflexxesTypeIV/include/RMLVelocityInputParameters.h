//  ---------------------- Doxygen info ----------------------
//! \file RMLVelocityInputParameters.h
//!
//! \brief
//! Header file for the class RMLVelocityInputParameters
//!
//! \details
//! The class RMLVelocityInputParameters is derived from the class
//! RMLInputParameters and constitutes a part of the interface for the
//! velocity-based Online Trajectory Generation algorithm.
//!
//! \sa RMLInputParameters
//! \sa RMLVelocityOutputParameters
//! \sa RMLPositionInputParameters
//! \n
//! \n
//! \n
//! Reflexxes GmbH\n
//! Sandknoell 7\n
//! D-24805 Hamdorf\n
//! GERMANY\n
//! \n
//! http://www.reflexxes.com\n
//!
//! \date October 2013
//! 
//! \version 1.3.2
//!
//!	\author Torsten Kroeger, <info@reflexxes.com>
//!	
//!
//! \note Copyright (C) 2013 Reflexxes GmbH.
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------


#ifndef __RMLVelocityInputParameters__
#define __RMLVelocityInputParameters__


#include <RMLInputParameters.h>
#include <math.h>


//  ---------------------- Doxygen info ----------------------
//! \class RMLVelocityInputParameters
//! 
//! \brief
//! Class for the input parameters of the velocity-based Online
//! Trajectory Generation algorithm
//!
//! \details
//! The class RMLVelocityInputParameters is derived from the class
//! RMLInputParameters and constitutes a part of the interface for the
//! velocity-based Online Trajectory Generation algorithm.\n\n
//! 
//! A detailed description can be found at \ref page_InputValues.
//! 
//! \sa ReflexxesAPI
//! \sa RMLInputParameters
//! \sa RMLPositionInputParameters
//! \sa RMLVelocityOutputParameters
//! \sa \ref page_InputValues
//  ----------------------------------------------------------
class RMLVelocityInputParameters : public RMLInputParameters
{
public:


//  ---------------------- Doxygen info ----------------------
//! \fn RMLVelocityInputParameters(const unsigned int DegreesOfFreedom)
//! 
//! \brief
//! Constructor of class RMLVelocityInputParameters
//! 
//! \warning
//! The constructor is \b not real-time capable as heap memory has to be
//! allocated.
//! 
//! \param DegreesOfFreedom
//! Specifies the number of degrees of freedom
//  ----------------------------------------------------------
    RMLVelocityInputParameters(const unsigned int DegreesOfFreedom) : RMLInputParameters(DegreesOfFreedom)
    {
    }


//  ---------------------- Doxygen info ----------------------
//! \fn RMLVelocityInputParameters(const RMLVelocityInputParameters& IP)
//! 
//! \brief
//! Copy constructor of class RMLVelocityInputParameters
//! 
//! \warning
//! The constructor is \b not real-time capable as heap memory has to be
//! allocated.
//! 
//! \param IP
//! Object to be copied
//  ----------------------------------------------------------
    RMLVelocityInputParameters(const RMLVelocityInputParameters &IP) : RMLInputParameters(IP)
    {
    }


//  ---------------------- Doxygen info ----------------------
//! \fn ~RMLVelocityInputParameters(void)
//! 
//! \brief
//! Destructor of class RMLVelocityInputParameters
//  ----------------------------------------------------------
    ~RMLVelocityInputParameters(void)
    {
    }
    
    
//  ---------------------- Doxygen info ----------------------
//! \fn RMLVelocityInputParameters &operator = (const RMLVelocityInputParameters &IP)
//! 
//! \brief
//! Copy operator
//! 
//! \param IP
//! RMLVelocityInputParameters object to be copied
//!
//! \return
//! The duplicated object
//  ----------------------------------------------------------	
    inline RMLVelocityInputParameters &operator = (const RMLVelocityInputParameters &IP)
    {
        RMLInputParameters::operator=(IP);

        return(*this);
    }
        


// #############################################################################	
    

//  ---------------------- Doxygen info ----------------------
//! \fn bool CheckForValidity(unsigned int *DegreeOfFreedom = NULL, int *ErrorCode = NULL) const
//! 
//! \brief
//! Checks the input parameters for validity
//! 
//! \details
//! \copydetails RMLPositionInputParameters::CheckForValidity()
//  ----------------------------------------------------------
    bool CheckForValidity(		unsigned int	*DegreeOfFreedom	= NULL
							,	int				*ErrorCode			= NULL) const
    {
        unsigned int		i							=	0;
    
        double				MinimumOrderOfMagnitude		=	0.0
                        ,	MaximumOrderOfMagnitude		=	0.0;
                        
        for (i = 0; i < this->NumberOfDOFs; i++)
        {
            if ((this->SelectionVector->VecData)[i])
            {
				if (this->MaxAccelerationVector->VecData[i]	<=	0.0)
				{
					if ((ErrorCode != NULL) && (DegreeOfFreedom != NULL))
					{
						*DegreeOfFreedom	=	i;
						*ErrorCode			=	RMLInputParameters::IP_MAX_ACCELERATION;
					}
					return(false);
				}
				if (this->MaxJerkVector->VecData[i]	<=	0.0)
				{
					if ((ErrorCode != NULL) && (DegreeOfFreedom != NULL))
					{
						*DegreeOfFreedom	=	i;
						*ErrorCode			=	RMLInputParameters::IP_MAX_JERK;
					}
					return(false);
				}
                if (	((this->MaxAccelerationVector->VecData)[i]	>=	(this->MaxJerkVector->VecData)					[i]	)
                    &&	((this->MaxAccelerationVector->VecData)[i]	>=	fabs((this->TargetVelocityVector->VecData)		[i]))
                    &&	((this->MaxAccelerationVector->VecData)[i]	>=	fabs((this->CurrentPositionVector->VecData)		[i]))
                    &&	((this->MaxAccelerationVector->VecData)[i]	>=	fabs((this->CurrentVelocityVector->VecData)		[i]))
                    &&	((this->MaxAccelerationVector->VecData)[i]	>=	fabs((this->CurrentAccelerationVector->VecData)	[i])))				
                {
                    MaximumOrderOfMagnitude	=	(this->MaxAccelerationVector->VecData)[i];
                }
                else
                {
                    if (	((this->MaxJerkVector->VecData)[i]	>=	fabs((this->TargetVelocityVector->VecData)		[i]))
                        &&	((this->MaxJerkVector->VecData)[i]	>=	fabs((this->CurrentPositionVector->VecData)		[i]))
                        &&	((this->MaxJerkVector->VecData)[i]	>=	fabs((this->CurrentVelocityVector->VecData)		[i]))
                        &&	((this->MaxJerkVector->VecData)[i]	>=	fabs((this->CurrentAccelerationVector->VecData)	[i])))				
                    {
                        MaximumOrderOfMagnitude	=	(this->MaxJerkVector->VecData)[i];
                    }
                    else
                    {
                        if (	(fabs((this->TargetVelocityVector->VecData)[i])	>=	fabs((this->CurrentPositionVector->VecData)		[i]))
                            &&	(fabs((this->TargetVelocityVector->VecData)[i])	>=	fabs((this->CurrentVelocityVector->VecData)		[i]))
                            &&	(fabs((this->TargetVelocityVector->VecData)[i])	>=	fabs((this->CurrentAccelerationVector->VecData)	[i])))
                        {
                            MaximumOrderOfMagnitude	=	fabs((this->TargetVelocityVector->VecData)[i]);
                        }
                        else
                        {
                            if (	(fabs((this->CurrentPositionVector->VecData)[i])	>=	fabs((this->CurrentVelocityVector->VecData)		[i]))
                                &&	(fabs((this->CurrentPositionVector->VecData)[i])	>=	fabs((this->CurrentAccelerationVector->VecData)	[i])))				
                            {
                                MaximumOrderOfMagnitude	=	fabs((this->CurrentPositionVector->VecData)[i]);
                            }
                            else
                            {
                                MaximumOrderOfMagnitude	=	fabs((this->CurrentAccelerationVector->VecData)[i]);
                            }
                        }
                    }		
                }				

                if ((this->MaxAccelerationVector->VecData)[i]	<=	(this->MaxJerkVector->VecData)[i])
                {
                    MinimumOrderOfMagnitude	=	(this->MaxAccelerationVector->VecData)[i];
                }
                else
                {
                    MinimumOrderOfMagnitude	=	(this->MaxJerkVector->VecData)[i];
                }										
                
                // The value of MinimumOrderOfMagnitude is greater than
                // zero:				
                if (	(MaximumOrderOfMagnitude / MinimumOrderOfMagnitude)
                    >	(double)pow((float)10, (int)(RMLVelocityInputParameters::MAXIMUM_MAGNITUDE_RANGE)))
                {
					if ((ErrorCode != NULL) && (DegreeOfFreedom != NULL))
					{
						*DegreeOfFreedom	=	i;
						*ErrorCode			=	RMLInputParameters::IP_ORDER_OF_MAGNITUDE;
					}        
					return(false);
                }
            }
        }
		if (this->MinimumSynchronizationTime > 1e10)
        {
			if (ErrorCode != NULL)
			{
				*ErrorCode = RMLInputParameters::IP_MINIMUM_SYNC_TIME;
			}        
			return(false);
        }
        
		if (	(this->OverrideValue < 0.0)
			||	(this->OverrideValue > 10.0)	)
		{
			if (ErrorCode != NULL)
			{
				*ErrorCode = RMLInputParameters::IP_OVERRIDE_VALUE;
			}        
			return(false);
		}
        
		if (ErrorCode != NULL)
		{
			*ErrorCode = RMLInputParameters::IP_NO_ERROR;
		}
        return(true);		
    }
    
    
//  ---------------------- Doxygen info ----------------------
//! \fn void Echo(FILE* FileHandler = stdout) const
//! 
//! \brief
//! \copybrief RMLInputParameters::Echo()
//! 
//! \details
//! \copydetails RMLInputParameters::Echo()
//  ----------------------------------------------------------
    void Echo(FILE* FileHandler = stdout) const
    {
        RMLInputParameters::Echo(FileHandler);
        return;
    }	

protected:


    enum
    {
//  ---------------------- Doxygen info ----------------------
//! \brief
//! Specifies the maximum allowed range for the orders of magnitude of
//! the input values.
//  ----------------------------------------------------------	
        MAXIMUM_MAGNITUDE_RANGE	=	10
    };



};// class RMLVelocityInputParameters



#endif


