//  ---------------------- Doxygen info ----------------------
//! \file RMLPositionOutputParameters.h
//!
//! \brief
//! Header file for the class RMLPositionOutputParameters
//!
//! \details
//! The class RMLPositionOutputParameters is derived from the class
//! RMLOutputParameters and constitutes a part of the interface for the
//! position-based Online Trajectory Generation algorithm.
//!
//! \sa RMLInputParameters
//! \sa RMLPositionInputParameters
//! \sa RMLVelocityOutputParameters
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


#ifndef __RMLPositionOutputParameters__
#define __RMLPositionOutputParameters__


#include <RMLOutputParameters.h>


//  ---------------------- Doxygen info ----------------------
//! \class RMLPositionOutputParameters
//! 
//! \brief
//! Class for the output parameters of the position-based Online
//! Trajectory Generation algorithm
//!
//! \details
//! The class RMLPositionOutputParameters is derived from the class
//! RMLOutputParameters and constitutes a part of the interface for the
//! position-based Online Trajectory Generation algorithm.
//! 
//! \sa ReflexxesAPI
//! \sa RMLOutputParameters
//! \sa RMLVelocityOutputParameters
//! \sa RMLPositionInputParameters
//  ----------------------------------------------------------
class RMLPositionOutputParameters : public RMLOutputParameters
{

public:

//  ---------------------- Doxygen info ----------------------
//! \fn RMLPositionOutputParameters(const unsigned int DegreesOfFreedom)
//! 
//! \brief
//! Constructor of class RMLPositionOutputParameters
//! 
//! \warning
//! The constructor is \b not real-time capable as heap memory has to be
//! allocated.
//! 
//! \param DegreesOfFreedom
//! Specifies the number of degrees of freedom
//  ----------------------------------------------------------
    RMLPositionOutputParameters(const unsigned int DegreesOfFreedom) : RMLOutputParameters(DegreesOfFreedom)
    {
		this->TrajectoryExceedsTargetPosition	=	false;
    }	


//  ---------------------- Doxygen info ----------------------
//! \fn RMLPositionOutputParameters(const RMLPositionOutputParameters &OP)
//! 
//! \brief
//! Copy constructor of class RMLPositionOutputParameters
//! 
//! \warning
//! The constructor is \b not real-time capable as heap memory has to be
//! allocated.
//! 
//! \param OP
//! Object to be copied
//  ----------------------------------------------------------
    RMLPositionOutputParameters(const RMLPositionOutputParameters &OP) : RMLOutputParameters(OP)
    {
		this->TrajectoryExceedsTargetPosition	=	OP.TrajectoryExceedsTargetPosition;
    }


//  ---------------------- Doxygen info ----------------------
//! \fn ~RMLPositionOutputParameters(void)
//! 
//! \brief
//! Destructor of class RMLPositionOutputParameters
//  ----------------------------------------------------------
	~RMLPositionOutputParameters(void)
    {
    }


//  ---------------------- Doxygen info ----------------------
//! \fn RMLPositionOutputParameters &operator = (const RMLPositionOutputParameters &OP)
//! 
//! \brief
//! Copy operator
//! 
//! \param OP
//! RMLPositionOutputParameters object to be copied
//!
//! \return
//! The duplicated object
//  ----------------------------------------------------------	
    RMLPositionOutputParameters &operator = (const RMLPositionOutputParameters &OP)
    {
        RMLOutputParameters::operator=(OP);
        
        this->TrajectoryExceedsTargetPosition	=	OP.TrajectoryExceedsTargetPosition;

        return(*this);
    }
    

//  ---------------------- Doxygen info ----------------------
//! \fn void Echo(FILE* FileHandler = stdout) const
//! 
//! \brief
//! \copybrief RMLOutputParameters::Echo()
//! 
//! \details
//! \copydetails RMLOutputParameters::Echo()
//  ----------------------------------------------------------
    void Echo(FILE* FileHandler = stdout) const
    {
        if (FileHandler == NULL)
        {
            return;
        }
        
        RMLOutputParameters::Echo(FileHandler);		
 
        return;
    }
    

//  ---------------------- Doxygen info ----------------------
//! \fn inline bool WillTheTargetPositionBeExceeded(void) const
//! 
//! \brief
//! Indicates, whether the currently computed trajectory will exceed the
//! desired target position in order to reach the desired target state
//! 
//! \details
//! If the currently computed trajectory exceeds the desired target
//! position \f$ \vec{P}_i^{\,trgt} \f$, this flag will be set to true.
//! 
//! \return 
//! The method returns \c true if the trajectory will exceed its target
//! position before reaching its target state and \c false if not.
//!
//! \sa RMLPositionOutputParameters::TrajectoryExceedsTargetPosition
//  ----------------------------------------------------------
    inline bool WillTheTargetPositionBeExceeded(void) const
    {
        return(this->TrajectoryExceedsTargetPosition);
    }
    
   
//  ---------------------- Doxygen info ----------------------
//! \var bool TrajectoryExceedsTargetPosition
//! 
//! \brief
//! Indicates, whether the currently computed trajectory will exceed the
//! desired target position in order to reach the desired target state
//! 
//! \details
//! If the currently computed trajectory exceeds the desired target
//! position \f$ \vec{P}_i^{\,trgt} \f$ before reaching the desired
//! target state of motion \f$ {\bf M}_i^{\,trgt} \f$, this flag will be
//! set to \c true.
//! 
//! This attribute can be accessed directly or by using the method
//! WillTheTargetPositionBeExceeded().
//!
//! \note
//! If the input flag RMLFlags::EnableTheCalculationOfTheExtremumMotionStates
//! is \b not set, this output flag will also not be computed (by default
//! the this input flag is set).
//!
//! \sa \ref page_Code_02_RMLPositionSampleApplication
//  ----------------------------------------------------------
    bool					TrajectoryExceedsTargetPosition;


};// class RMLPositionOutputParameters



#endif


