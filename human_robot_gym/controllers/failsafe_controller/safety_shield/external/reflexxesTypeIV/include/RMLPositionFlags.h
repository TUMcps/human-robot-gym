//  ---------------------- Doxygen info ----------------------
//! \file RMLPositionFlags.h
//!
//! \brief
//! Header file for the class RMLPositionFlags
//!
//! \details 
//! Flags to parameterize the position-based Online Trajectory Generation
//! algorithm.
//!
//! \sa RMLFlags.h
//! \sa RMLVelocityFlags.h
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


#ifndef __RMLPositionFlags__
#define __RMLPositionFlags__


#include <RMLFlags.h>


//  ---------------------- Doxygen info ----------------------
//! \class RMLPositionFlags
//! 
//! \brief
//! Data structure containing flags to parameterize the execution of the
//! position-based Online Trajectory Generation algorithm
//!
//! \sa RMLFlags
//! \sa RMLVelocityFlags
//  ----------------------------------------------------------
class RMLPositionFlags : public RMLFlags
{
public:


//  ---------------------- Doxygen info ----------------------
//! \fn RMLPositionFlags(void)
//! 
//! \brief
//! Constructor of the class
//!
//! \details
//! Sets the default values:
//!  - Synchronization behavior: phase-synchronization if possible (RMLFlags::PHASE_SYNCHRONIZATION_IF_POSSIBLE)
//!  - Behavior after the target state of motion is reached: keep target velocity (RMLPositionFlags::KEEP_TARGET_VELOCITY)
//!  - Calculation of extremum motion states enabled
//!  - In case the fall back strategy becomes active: velocity to predefined values (default: zero)
//  ----------------------------------------------------------
    RMLPositionFlags(void)
    {
        this->SynchronizationBehavior						=	RMLFlags::PHASE_SYNCHRONIZATION_IF_POSSIBLE	;
        this->BehaviorAfterFinalStateOfMotionIsReached		=	RMLPositionFlags::KEEP_TARGET_VELOCITY		;
		this->BehaviorIfInitialStateBreachesConstraints		=	RMLPositionFlags::GET_INTO_BOUNDARIES_FAST	;
        this->EnableTheCalculationOfTheExtremumMotionStates	=	true										;	
        this->KeepCurrentVelocityInCaseOfFallbackStrategy	=	false										;
		this->PositionalLimitsBehavior						=	RMLFlags::POSITIONAL_LIMITS_IGNORE			;
    }


//  ---------------------- Doxygen info ----------------------
//! \fn ~RMLPositionFlags(void)
//! 
//! \brief
//! Destructor of the class RMLPositionFlags
//  ----------------------------------------------------------
    ~RMLPositionFlags(void)
    {
    }
    
    
//  ---------------------- Doxygen info ----------------------
//! \fn inline bool operator == (const RMLPositionFlags &Flags) const
//! 
//! \brief
//! Equal operator
//!
//! \return
//!\c true if all attributes of both objects are equal; \c false otherwise
//  ----------------------------------------------------------
    inline bool operator == (const RMLPositionFlags &Flags) const
    {
        return(		(RMLFlags::operator==(Flags))
                &&	(this->BehaviorAfterFinalStateOfMotionIsReached
                        ==	Flags.BehaviorAfterFinalStateOfMotionIsReached)
                &&	(this->BehaviorIfInitialStateBreachesConstraints
                        ==	Flags.BehaviorIfInitialStateBreachesConstraints)
                &&	(this->KeepCurrentVelocityInCaseOfFallbackStrategy
                        ==	Flags.KeepCurrentVelocityInCaseOfFallbackStrategy));
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline bool operator != (const RMLPositionFlags &Flags) const
//! 
//! \brief
//! Unequal operator
//!
//! \return
//!\c false if all attributes of both objects are equal; \c true otherwise
//  ----------------------------------------------------------
    inline bool operator != (const RMLPositionFlags &Flags) const
    {
        return(!(*this == Flags));
    }

    
//  ---------------------- Doxygen info ----------------------
//! \enum FinalMotionBehaviorEnum
//! 
//! \brief
//! Enumeration whose values specify the behavior after the final state
//! of motion is reached.
//!
//! \sa BehaviorAfterFinalStateOfMotionIsReached
//  ----------------------------------------------------------
    enum FinalMotionBehaviorEnum
    {
        //! \brief
        //! The desired velocity of the target state of motion will be 
        //! kept at zero acceleration <b>(default)</b>.
        KEEP_TARGET_VELOCITY	=	0	,
        //! \brief
        //! After the final state of motion is reached, a new trajectory
        //! will be computed, such that the desired state of motion will
        //! be reached again (and again, and again, etc.).
        RECOMPUTE_TRAJECTORY	=	1
    };	


	//  ---------------------- Doxygen info ----------------------
	//! \enum BehaviorIfInitialStateBreachesConstraintsEnum
	//! 
	//! \brief
	//! Enumeration whose values specify the behavior if the initial state
	//! of motion is out of bounds
	//!
	//! \details
	//! If the initial velocity and acceleration values are not inside
	//! of the given maximum values, there are two option to bring them
	//! back into their boundaries:
	//! 
	//! <ol>
	//! <li>as soon as possible with non-zero intermediate acceleration
	//!     (this may lead to an overshooting of the output velocity signal)
	//!     or</li>
	//! <li>with an intermediate acceleration of zero so that the motion
	//!     can be continued exactly at the maximum velocity value.</li>
	//! </ol>
	//!
	//! \sa \ref page_OutOfBoundsBehavior
	//! \sa BehaviorAfterFinalStateOfMotionIsReached
	//  ----------------------------------------------------------
	enum BehaviorIfInitialStateBreachesConstraintsEnum
	{
		//! \brief
		//! The system will be brought back into the boundaries as fast
		//! as possible but with a non-zero intermediate acceleration
		//! <b>(default)</b>.
		GET_INTO_BOUNDARIES_FAST					=	0	,
		//! \brief
		//! The system will be brought back into the boundaries with an
		//! intermediate acceleration of zero allowing to continue the motion
		//! exactly at the maximum velocity.
		GET_INTO_BOUNDARIES_AT_ZERO_ACCELERATION	=	1
	};


//  ---------------------- Doxygen info ----------------------
//! \var int BehaviorAfterFinalStateOfMotionIsReached
//! 
//! \brief
//! This flag defines the behavior for the time \em after the final
//! state of motion was reached.
//!
//! \details
//! Two values are possible:
//! 
//!  - RMLPositionFlags::KEEP_TARGET_VELOCITY or
//!  - RMLPositionFlags::RECOMPUTE_TRAJECTORY.
//!
//! The <b>default</b> value is RMLPositionFlags::KEEP_TARGET_VELOCITY.
//! \note
//! If the desired target velocity vector is zero, both values will
//! lead to the same behavior.
//!
//! \sa FinalMotionBehaviorEnum
//! \sa \ref page_FinalStateReachedBehavior
//  ----------------------------------------------------------
    int				BehaviorAfterFinalStateOfMotionIsReached;


//  ---------------------- Doxygen info ----------------------
//! \var int BehaviorIfInitialStateBreachesConstraints
//! 
//! \brief
//! This flag defines the behavior in case the initial constraints 
//! are breached
//!
//! \details
//! Two values are possible:
//! 
//!  - RMLPositionFlags::GET_INTO_BOUNDARIES_FAST or
//!  - RMLPositionFlags::GET_INTO_BOUNDARIES_AT_ZERO_ACCELERATION.
//! 
//! The <b>default</b> value is RMLPositionFlags::GET_INTO_BOUNDARIES_FAST.
//! \note
//! This value will only affect the resulting trajectory if the initial 
//! state of motion is not inside of the kinematic constraints.
//!
//! \sa BehaviorIfInitialStateBreachesConstraintsEnum
//! \sa \ref page_OutOfBoundsBehavior
//  ----------------------------------------------------------
	int				BehaviorIfInitialStateBreachesConstraints;

//  ---------------------- Doxygen info ----------------------
//! \var bool KeepCurrentVelocityInCaseOfFallbackStrategy
//! 
//! \brief
//! If true, RMLPositionInputParameters::AlternativeTargetVelocityVector
//! will be used in TypeIVRMLPosition::FallBackStrategy()
//! 
//! \details
//! In case, the second safety layer becomes activated by calling
//! TypeIVRMLPosition::FallBackStrategy(), this flag can optionally be used
//! to set the current velocity vector \f$ \vec{V}_i \f$ as the
//! alternative target velocity vector for the velocity-based Online 
//! Trajectory Generation algorithm.
//!
//! \note 
//! By default, this flag is set to \c false, and in most cases,
//! is not necessary to change this. Depending on the 
//! application, it may be reasonable to keep the current velocity
//! in case of an error.
//!
//! \sa RMLPositionInputParameters::AlternativeTargetVelocityVector
//! \sa \ref page_ErrorHandling
//  ----------------------------------------------------------
    bool				KeepCurrentVelocityInCaseOfFallbackStrategy;


};// class RMLPositionFlags



#endif


