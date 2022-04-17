//  ---------------------- Doxygen info ----------------------
//! \file RMLFlags.h
//!
//! \brief
//! Header file for the class RMLFlags
//!
//! \details 
//! Flags to parameterize the Online Trajectory Generation algorithms.
//! This basis data structure is inherited to the structures 
//! RMLPositionFlags and RMLVelocityFlags
//!
//! \sa RMLPositionFlags
//! \sa RMLVelocityFlags
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


#ifndef __RMLFlags__
#define __RMLFlags__


//  ---------------------- Doxygen info ----------------------
//! \class RMLFlags
//! 
//! \brief
//! Data structure containing flags to parameterize the execution of the
//! Online Trajectory Generation algorithm
//  ----------------------------------------------------------
class RMLFlags
{

protected:

//  ---------------------- Doxygen info ----------------------
//! \fn RMLFlags(void)
//! 
//! \brief
//! Constructor of the class
//!
//! \note
//! This is only the base class for the classes\n\n
//! <ul>
//!  <li>RMLPositionFlags and</li>
//!  <li>RMLVelocityFlags,\n\n</li>
//! </ul>
//! such that the constructor is declared \c protected.
//  ----------------------------------------------------------
    RMLFlags(void)
    {
    }
    
public:


//  ---------------------- Doxygen info ----------------------
//! \fn ~RMLFlags(void)
//! 
//! \brief
//! Destructor of the class RMLFlags
//  ----------------------------------------------------------
    ~RMLFlags(void)
    {
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline bool operator == (const RMLFlags &Flags) const
//! 
//! \brief
//! Equal operator
//!
//! \return
//!\c true if all attributes of both objects are equal; \c false otherwise
//  ----------------------------------------------------------
    inline bool operator == (const RMLFlags &Flags) const
    {
        return (	(this->SynchronizationBehavior
                        ==	Flags.SynchronizationBehavior)
                &&	(this->EnableTheCalculationOfTheExtremumMotionStates
                        ==	Flags.EnableTheCalculationOfTheExtremumMotionStates)
				&&	(this->PositionalLimitsBehavior
						==	Flags.PositionalLimitsBehavior)	);
    }
    
//  ---------------------- Doxygen info ----------------------
//! \fn inline bool operator != (const RMLFlags &Flags) const
//! 
//! \brief
//! Unequal operator
//!
//! \return
//!\c false if all attributes of both objects are equal; \c true otherwise
//  ----------------------------------------------------------
    inline bool operator != (const RMLFlags &Flags) const
    {
        return(!(*this == Flags));
    }


//  ---------------------- Doxygen info ----------------------
//! \enum SyncBehaviorEnum
//! 
//! \brief
//! Enumeration whose values specify the synchronization method of the
//! Online Trajectory Generation algorithm
//!
//! \sa RMLFlags::SynchronizationBehavior
//! \sa \ref page_SynchronizationBehavior
//! \sa \ref page_PSIfPossible
//  ----------------------------------------------------------
    enum SyncBehaviorEnum
    {
        //! \brief
        //! This is the default value. If it is possible to calculate a
        //! phase-synchronized (i.e., homothetic) trajectory, the algorithm
        //! will generate it. A more detailed description of this flag can
        //!  be found on the page \ref page_PSIfPossible.
        PHASE_SYNCHRONIZATION_IF_POSSIBLE	=	0	,
        //! \brief
        //! Even if it is possible to calculate a phase-synchronized
        //! trajectory, only a time-synchronized trajectory will be
        //! provided. More information can be found on page
        //! \ref page_SynchronizationBehavior.
        ONLY_TIME_SYNCHRONIZATION			=	1	,
        //! \brief
        //! Only phase-synchronized trajectories are allowed. If it is not
        //! possible calculate a phase-synchronized trajectory, an error
        //! will be returned (but feasible, steady, and continuous
        //! output values will still be computed). More information can be
        //! found on page \ref page_SynchronizationBehavior.
        ONLY_PHASE_SYNCHRONIZATION			=	2	,
        //! \brief
        //! No synchronization will be performed, and all selected degrees
        //! of freedom are treated independently. 
        NO_SYNCHRONIZATION					=	3
    };
    
    
//  ---------------------- Doxygen info ----------------------
//! \enum PositionalLimitsEnum
//! 
//! \brief
//! Enumeration whose values specify how positional limits are used
//!
//! \details
//! The trajectory generation algorithms can either prevent 
//!
//!  - breaching positional limits \b or
//!  - constraints for velocity, acceleration, jerk, etc.
//!
//! It is not possible to prevent from breaching both. In order to 
//! guarantee executable motions, the trajectory generation algorithms of
//! the Reflexxes Motion Libraries do not change the constraints for
//! velocity, acceleration, and jerk (i.e.,
//! \f$ \bf B_i = \left(\vec{V}_i^{\,max},\,\vec{A}_i^{\,max},\,\vec{J}_i^{\,max},\right) \f$).
//! If positional limits \f$ \vec{P}_i^{\,max} \f$ and 
//! \f$ \vec{P}_i^{\,min} \f$ will be breached by the currently computed 
//! trajectory, the Reflexxes Motion Libraries provide three different
//! options to specify the behavior.
//!
//!  -# RMLFlags::POSITIONAL_LIMITS_IGNORE: ignore positional limits  <b>(default)</b>
//!  -# RMLFlags::POSITIONAL_LIMITS_ERROR_MSG_ONLY: return an error value but perform no action
//!  -# RMLFlags::POSITIONAL_LIMITS_ACTIVELY_PREVENT: return an error value and actively prevent from breaching the limits
//! 
//! If an error value is returned because the currently computed trajectory
//! will exceed the positional limits, the user application will get
//! informed within one control cycle (typically one millisecond or less).
//! Depending on the application, the user can then implement a desired
//! behavior for such cases.
//!
//! \sa RMLFlags::PositionalLimitsBehavior
//! \sa \ref page_PositionalLimitsBehavior
//! \sa RMLInputParameters::MaxPositionVector
//! \sa RMLInputParameters::MinPositionVector
//! \sa \ref page_Code_09_RMLPositionSampleApplication
//! \sa \ref page_Code_10_RMLPositionSampleApplication
//! \sa \ref page_Code_11_RMLVelocitySampleApplication
//! \sa \ref page_Code_12_RMLVelocitySampleApplication
//  ----------------------------------------------------------
    enum PositionalLimitsEnum
    {
        //! \brief
        //! This is the \b default value. The values of 
        //! RMLInputParameters::MaxPositionVector and
        //! RMLInputParameters::MinPositionVector are ignored when calling
        //! ReflexxesAPI::RMLPosition() or ReflexxesAPI::RMLVelocity().
        POSITIONAL_LIMITS_IGNORE			=	0	,
        //! \brief
        //! If the trajectories will exceed the positional limits of
		//! \f$ \vec{P}_i^{\,max} \f$ or 
		//! \f$ \vec{P}_i^{\,min} \f$, the method 
        //! ReflexxesAPI::RMLPosition() or ReflexxesAPI::RMLVelocity(),
        //! respectively will return an error value,
        //! ReflexxesAPI::RML_ERROR_POSITIONAL_LIMITS.
        POSITIONAL_LIMITS_ERROR_MSG_ONLY	=	1	,
        //! \brief
		//! Same behavior as in the case of
		//! RMLFlags::POSITIONAL_LIMITS_ERROR_MSG_ONLY, and in addition
		//! the exceeding the positional limits will be actively 
		//! prevented. This active prevention will also be performed in
		//! case the desired state of motion has already been reached.
		//! For more information about this, please refer to
		//! the page \ref page_PositionalLimitsBehavior.
        POSITIONAL_LIMITS_ACTIVELY_PREVENT	=	2
    };


//  ---------------------- Doxygen info ----------------------
//! \var unsigned char SynchronizationBehavior
//! 
//! \brief
//! This value specifies the desired synchronization behavior
//! 
//! \details
//! The following four values are possible:
//! 
//! - RMLFlags::PHASE_SYNCHRONIZATION_IF_POSSIBLE <i>(default)</i>
//! - RMLFlags::ONLY_TIME_SYNCHRONIZATION
//! - RMLFlags::ONLY_PHASE_SYNCHRONIZATION
//! - RMLFlags::NO_SYNCHRONIZATION
//!
//! Further details about time- and phase-synchronization can be found 
//! in the section on \ref page_SynchronizationBehavior and at
//! RMLFlags::SyncBehaviorEnum.
//! 
//! \sa RMLPositionFlags
//! \sa RMLVelocityFlags
//! \sa RMLPositionOutputParameters::IsTrajectoryPhaseSynchronized()
//! \sa RMLVelocityOutputParameters::IsTrajectoryPhaseSynchronized()
//! \sa \ref page_SynchronizationBehavior
//! \sa \ref page_PSIfPossible
//  ----------------------------------------------------------	
    unsigned char		SynchronizationBehavior;


//  ---------------------- Doxygen info ----------------------
//! \var unsigned char PositionalLimitsBehavior
//! 
//! \brief
//! This value specifies, how positional limits are applied
//! 
//! \details
//! The following three values are possible:
//! 
//! - RMLFlags::POSITIONAL_LIMITS_IGNORE <i>(default)</i>
//! - RMLFlags::POSITIONAL_LIMITS_ERROR_MSG_ONLY
//! - RMLFlags::POSITIONAL_LIMITS_ACTIVELY_PREVENT
//!
//! Further details can be found on the page
//! \ref page_PositionalLimitsBehavior and at
//! RMLFlags::PositionalLimitsEnum.
//! 
//! \sa RMLFlags::PositionalLimitsEnum
//! \sa \ref page_PositionalLimitsBehavior
//! \sa RMLInputParameters::MaxPositionVector
//! \sa RMLInputParameters::MinPositionVector
//  ----------------------------------------------------------	
    unsigned char		PositionalLimitsBehavior;


//  ---------------------- Doxygen info ----------------------
//! \var bool EnableTheCalculationOfTheExtremumMotionStates
//! 
//! \brief
//! A flag to enable or disable the calculation of the extremum states of motion of the
//! currently calculated trajectory
//! 
//! \details
//! If this flag is set, a call of
//! 
//! - ReflexxesAPI::RMLPosition() or
//! - ReflexxesAPI::RMLVelocity()
//! 
//! will not only calculate the desired trajectory, but also the motion states
//! at the extremum positions. These values will be accessible through the methods
//! of the classes
//! 
//! - RMLPositionOutputParameters and
//! - RMLVelocityOutputParameters
//! 
//! \remark
//! In average, the calculation of the extremum states of motion takes less than five percent
//! the overall computational effort of the OTG algorithm, such that the maximum 
//! execution time of the algorithm will be slightly decreased by disabling this flag.
//! 
//! \note
//! The default value for this flag is \c true.
//!
//! \sa \ref page_OutputValues
//! \sa \ref page_FinalStateReachedBehavior
//! \sa \ref page_PSIfPossible
//! \sa RMLPositionOutputParameters::TrajectoryExceedsTargetPosition
//  ----------------------------------------------------------						
    bool				EnableTheCalculationOfTheExtremumMotionStates;

};// class RMLFlags



#endif


