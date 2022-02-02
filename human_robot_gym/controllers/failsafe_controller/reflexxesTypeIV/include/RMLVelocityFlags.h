//  ---------------------- Doxygen info ----------------------
//! \file RMLVelocityFlags.h
//!
//! \brief
//! Header file for the class RMLVelocityFlags
//!
//! \details 
//! Flags to parameterize the velocity-based Online Trajectory Generation
//! algorithm.
//!
//! \sa RMLFlags.h
//! \sa RMLPositionFlags.h
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


#ifndef __RMLVelocityFlags__
#define __RMLVelocityFlags__


#include <RMLFlags.h>


//  ---------------------- Doxygen info ----------------------
//! \class RMLVelocityFlags
//! 
//! \brief
//! Data structure containing flags to parameterize the execution of the
//! velocity-based Online Trajectory Generation algorithm
//!
//! \sa RMLFlags
//! \sa RMLPositionFlags
//  ----------------------------------------------------------
class RMLVelocityFlags : public RMLFlags
{
public:


//  ---------------------- Doxygen info ----------------------
//! \fn RMLVelocityFlags(void)
//! 
//! \brief
//! Constructor of the class
//!
//! \details
//! Sets the default values:
//!  - Synchronization behavior:  no synchronization (RMLFlags::NO_SYNCHRONIZATION)
//!  - Calculation of extremum motion states enabled
//  ----------------------------------------------------------
    RMLVelocityFlags(void)
    {
        this->SynchronizationBehavior						=	RMLFlags::NO_SYNCHRONIZATION		;
        this->PositionalLimitsBehavior						=	RMLFlags::POSITIONAL_LIMITS_IGNORE	;
        this->EnableTheCalculationOfTheExtremumMotionStates	=	true								;	
    }


//  ---------------------- Doxygen info ----------------------
//! \fn ~RMLVelocityFlags(void)
//! 
//! \brief
//! Destructor of the class RMLVelocityFlags
//  ----------------------------------------------------------	
    ~RMLVelocityFlags(void)
    {
    }
    
//  ---------------------- Doxygen info ----------------------
//! \fn inline bool operator == (const RMLVelocityFlags &Flags) const
//! 
//! \brief
//! Equal operator
//!
//! \return
//!\c true if all attributes of both objects are equal; \c false otherwise
//  ----------------------------------------------------------
    inline bool operator == (const RMLVelocityFlags &Flags) const
    {
        return(RMLFlags::operator==(Flags));
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline bool operator != (const RMLVelocityFlags &Flags) const
//! 
//! \brief
//! Unequal operator
//!
//! \return
//!\c false if all attributes of both objects are equal; \c true otherwise
//  ----------------------------------------------------------
    inline bool operator != (const RMLVelocityFlags &Flags) const
    {
        return(!(*this == Flags));
    }

};// class RMLVelocityFlags



#endif


