//  ---------------------- Doxygen info ----------------------
//! \file TypeIVRML_C_Wrapper.h
//!
//! \brief
//! Header file of the C-wrapper for the Type IV Reflexxes Motion
//! Library
//!
//! \details
//! The two relevant functions of the <em>Type IV Reflexxes Motion
//! Library</em> are the
//! the methods
//!
//!  - ReflexxesAPI::RMLPosition() to execute the
//!    <b>position-based</b> Online Trajectory Generation algorithm and\n
//!    
//!  - ReflexxesAPI::RMLVelocity() to execute the <b>velocity-based
//!    </b> Online Trajectory Generation algorithm,
//!
//! both of which calculate a new state of motion from any arbitrary
//! initial state of motion.\n
//! \n
//! Both methods, require the specification of a set of input values
//! RMLPositionInputParameters / RMLVelocityInputParameters
//! and a data structure specifying a certain behavior of the algorithm
//! RMLPositionFlags / RMLVelocityFlags.
//! Based on these input values, the result, that is, the state of motion
//! for the succeeding control cycle, is written to an
//! RMLPositionOutputParameters / RMLVelocityOutputParameters.
//! object. This C-wrapper provides four functions\n\n
//!
//!  - TypeIVRMLCreate()\n\n
//!  - TypeIVRMLPosition()\n\n
//!  - TypeIVRMLVelocity()\n\n
//!  - TypeIVRMLDestroy()\n\n
//!
//! that provide the Reflexxes API for the C programming language.
//!
//! \note
//! The functions TypeIVRMLCreate() and TypeIVRMLDestroy() have to be 
//! called pairwise.
//!
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
//! \version 1.3.2.1
//!
//!	\author Torsten Kroeger, <info@reflexxes.com>
//!	
//!
//! \note Copyright (C) 2013 Reflexxes GmbH.
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------


#ifndef __TypeIVRML_C_Wrapper__
#define __TypeIVRML_C_Wrapper__


#ifdef __cplusplus
extern "C"		// use the C function call standard for all functions  
#endif			// defined within this scope

//  ---------------------- Doxygen info ----------------------
//! \fn int TypeIVRMLCreate(const unsigned int NumberOfDOFs, const double CycleTimeInSeconds)
//!
//! \brief
//! This function allocates memory to run the Reflexxes Online Trajectory
//! Generation algorithms in real-time environments.
//!
//! \details
//! This function of the C-wrapper of the Reflexxes motion libraries
//! calls the constructor of the RelfexxesAPI class that allocates
//! memory for the algorithm. In order to prevent from memory leaks,
//! this function has to be called pairwise with the function
//! TypeIVRMLDestroy(), which is the counter part to TypeIVRMLCreate()
//! and releases the previously allocated heap memory again.\n
//! 
//! Please refer to the documentation of ReflexxesAPI::ReflexxesAPI()
//! for further information.
//!
//! \warning
//! <b>This function does not meet real-time constraints as heap memory is
//! allocated.</b>\n\n
//!
//! \param NumberOfDOFs
//! Integer value of the number of degrees of freedom
//!
//! \param CycleTimeInSeconds
//! Control cycle time (a typical value is 0.001)
//! 
//! \return
//! <ul> 
//! <li>\b 0 if successful</li>
//! <li>\b -1 if TypeIVRMLCreate() has been called before</li>
//! </ul> 
//!
//! \sa TypeIVRMLDestroy()
//  ----------------------------------------------------------	
int		TypeIVRMLCreate	(		const unsigned int	NumberOfDOFs
							,	const double		CycleTimeInSeconds);


#ifdef __cplusplus
extern "C"		// use the C function call standard for all functions  
#endif			// defined within this scope


//  ---------------------- Doxygen info ----------------------
//! \fn int TypeIVRMLPosition(const double* CurrentPosition, const double* CurrentVelocity, const double* CurrentAcceleration, const double* MaxVelocity, const double* MaxAcceleration, const double* MaxJerk, const double* TargetPosition, const double* TargetVelocity, const int* SelectionVector, double* NewPosition, double* NewVelocity, double* NewAcceleration)
//!
//! \brief
//! This function calls the position-based Type IV Online Trajectory
//! Generation algorithm of the Reflexxes Motion Libraries
//!
//! \details
//! This function executes the position-based Type IV Reflexxes Online
//! Trajectory Generation algorithm of the Reflexxes Motion Library.\n
//! \n
//! Based on 
//!
//!  - the current state of motion \f$ {\bf M}_i \f$,
//!     - \f$ \vec{P}_i \f$ \f$\Longrightarrow \f$ \c CurrentPosition
//!     - \f$ \vec{V}_i \f$ \f$\Longrightarrow \f$ \c CurrentVelocity
//!     - \f$ \vec{A}_i \f$ \f$\Longrightarrow \f$ \c CurrentAcceleration,
//!  - the kinematic motion constraints \f$ {\bf B}_i \f$,
//!     - \f$ \vec{V}_i^{\,max} \f$ \f$\Longrightarrow \f$ \c MaxVelocity
//!     - \f$ \vec{A}_i^{\,max} \f$ \f$\Longrightarrow \f$ \c MaxAcceleration
//!     - \f$ \vec{J}_i^{\,max} \f$ \f$\Longrightarrow \f$ \c MaxJerk,
//!  - the desired target state of motion \f$ {\bf M}_i^{\,trgt} \f$,
//!     - \f$ \vec{P}_i^{\,trgt} \f$ \f$\Longrightarrow \f$ \c TargetPosition
//!     - \f$ \vec{V}_i^{\,trgt} \f$ \f$\Longrightarrow \f$ \c TargetVelocity, and
//!  - the selection vector \f$ {\bf S}_i \f$
//!     - \f$ \vec{S}_i \f$ \f$\Longrightarrow \f$ \c SelectionVector
//!
//! a new state of motion \f$ {\bf M}_{i+1} \f$
//!
//!     - \f$ \vec{P}_{i+1} \f$ \f$\Longrightarrow \f$ \c NewPosition
//!     - \f$ \vec{V}_{i+1} \f$ \f$\Longrightarrow \f$ \c NewVelocity
//!     - \f$ \vec{A}_{i+1} \f$ \f$\Longrightarrow \f$ \c NewAcceleration,
//!
//! is calculated, which can be utilized as a set-point value at instant
//! \f$ T_{i+1} \f$. The time difference between \f$ {\bf M}_{i} \f$ and
//! \f$ {\bf M}_{i+1} \f$ is \c CycleTimeInSeconds. For a detailed
//! description, please refer to the corresponding documentation of the
//! \ref page_InputValues "input values" and
//! \ref page_OutputValues "output values" of the Type IV Reflexxes Motion
//! Library.
//!
//! \param CurrentPosition
//! Pointer to an array of \c double values containing the current
//! position/pose vector \f$ \vec{P}_i \f$
//!
//! \param CurrentVelocity
//! Pointer to an array of \c double values containing the current
//! velocity vector \f$ \vec{V}_i \f$
//!
//! \param CurrentAcceleration
//! Pointer to an array of \c double values containing the current
//! acceleration vector \f$ \vec{A}_i \f$
//!
//! \param MaxVelocity
//! Pointer to an array of \c double values containing the maximum
//! velocity vector \f$ \vec{V}_i^{\,max} \f$
//!
//! \param MaxAcceleration
//! Pointer to an array of \c double values containing the maximum
//! acceleration vector \f$ \vec{A}_i^{\,max} \f$
//!
//! \param MaxJerk
//! Pointer to an array of \c double values containing the maximum
//! jerk vector \f$ \vec{J}_i^{\,max} \f$
//!
//! \param TargetPosition
//! Pointer to an array of \c double values containing the target
//! position/pose vector \f$ \vec{P}_i^{\,trgt} \f$.
//!
//! \param TargetVelocity
//! Pointer to an array of \c double values containing the target
//! velocity vector \f$ \vec{V}_i^{\,trgt} \f$.
//!
//! \param SelectionVector
//! Pointer to an array of integer values containing the selection
//! vector \f$ \vec{S}_i \f$. Values of zero and less than zero
//! are interpreted as \c false; values greater than zero are
//! interpreted as \c true.
//!
//! \param NewPosition
//! Pointer to an array of \c double values containing the new
//! position/pose vector \f$ \vec{P}_{i+1} \f$.
//!
//! \param NewVelocity
//! Pointer to an array of \c double values containing the new
//! velocity vector \f$ \vec{V}_{i+1} \f$
//!
//! \param NewAcceleration
//! Pointer to an array of \c double values containing the new
//! acceleration vector \f$ \vec{A}_{i+1} \f$
//! 
//! \return
//! An integer value as specified in RMLResultValue. If the implementation
//! is correct, the function either returns ReflexxesAPI::RML_WORKING or
//! ReflexxesAPI::RML_FINAL_STATE_REACHED. In general, one of the
//! following values can be returned:\n\n
//!
//! <ul>
//! <li><b>0</b>: The Online Trajectory Generation algorithm is working;
//!    the final state of motion has not been reached yet.</li>
//! <li><b>1</b>: The desired final state of motion has been reached.</li>
//! <li><b>-1</b>: If the function TypeIVRMLCreate() has not been called
//!    before, this error values will be returned.</li>
//! <li><b>-100</b>: The applied input values are invalid.</li>
//! <li><b>-101</b>: An error occurred during the first step of
//!    the algorithm (i.e., during the calculation of the synchronization
//!    time).</li>
//! <li><b>-102</b>: An error occurred during the second step of
//!    the algorithm (i.e., during the synchronization of the trajectory).</li>
//! <li><b>-105</b>: To ensure numerical stability, the value of the minimum
//!    trajectory execution time is limited to a maximum value of
//!    \f$ 10^{10} \f$ seconds.</li>
//! </ul>
//! 
//! Please also refer to the corresponding documentation of the
//! in the class TypeIVRMLPosition for more information.
//! 
//! \note
//! A complete description of the Online Trajectory Generation framework
//! may be also found at\n
//! \n
//! <b>T. Kroeger.</b>\n
//! <b>On-line Trajectory Generation in Robotic Systems.</b>\n
//! <b>Springer Tracts in Advanced Robotics, Vol. 58, Springer, January 2010.</b>\n
//! <b><a href="http://www.springer.com/978-3-642-05174-6" target="_blanc" title="You may order your personal copy at www.springer.com.">http://www.springer.com/978-3-642-05174-6</a></b>\n
//!
//! \sa TypeIVRMLCreate()
//! \sa TypeIVRMLVelocity()
//  ----------------------------------------------------------	
int		TypeIVRMLPosition	(		const	double*		CurrentPosition
								,	const	double*		CurrentVelocity
								,	const	double*		CurrentAcceleration
								,	const	double*		MaxVelocity
								,	const	double*		MaxAcceleration
								,	const	double*		MaxJerk
								,	const	double*		TargetPosition
								,	const	double*		TargetVelocity
								,	const	int*		SelectionVector
								,			double*		NewPosition
								,			double*		NewVelocity
								,			double*		NewAcceleration		);


#ifdef __cplusplus
extern "C"		// use the C function call standard for all functions  
#endif			// defined within this scope

//  ---------------------- Doxygen info ----------------------
//! \fn int TypeIVRMLVelocity(const double* CurrentPosition, const double* CurrentVelocity, const double* CurrentAcceleration, const double* MaxAcceleration, const double* MaxJerk, const double* TargetVelocity, const int* SelectionVector, double* NewPosition, double* NewVelocity, double* NewAcceleration)
//!
//! \brief
//! This function calls the velocity-based Type IV Online Trajectory
//! Generation algorithm of the Reflexxes Motion Libraries
//!
//! \details
//! This function executes the velocity-based Type IV Reflexxes Online
//! Trajectory Generation algorithm of the Reflexxes Motion Library.\n
//! \n
//! Based on 
//!
//!  - the current state of motion \f$ {\bf M}_i \f$,
//!     - \f$ \vec{P}_i \f$ \f$\Longrightarrow \f$ \c CurrentPosition
//!     - \f$ \vec{V}_i \f$ \f$\Longrightarrow \f$ \c CurrentVelocity
//!     - \f$ \vec{A}_i \f$ \f$\Longrightarrow \f$ \c CurrentAcceleration,
//!  - the kinematic motion constraints \f$ {\bf B}_i \f$,
//!     - \f$ \vec{A}_i^{\,max} \f$ \f$\Longrightarrow \f$ \c MaxAcceleration
//!     - \f$ \vec{J}_i^{\,max} \f$ \f$\Longrightarrow \f$ \c MaxJerk,
//!  - the desired target state of motion \f$ {\bf M}_i^{\,trgt} \f$,
//!     - \f$ \vec{V}_i^{\,trgt} \f$ \f$\Longrightarrow \f$ \c TargetVelocity, and
//!  - the selection vector \f$ {\bf S}_i \f$
//!     - \f$ \vec{S}_i \f$ \f$\Longrightarrow \f$ \c SelectionVector
//!
//! a new state of motion \f$ {\bf M}_{i+1} \f$
//!
//!     - \f$ \vec{P}_{i+1} \f$ \f$\Longrightarrow \f$ \c NewPosition
//!     - \f$ \vec{V}_{i+1} \f$ \f$\Longrightarrow \f$ \c NewVelocity
//!     - \f$ \vec{A}_{i+1} \f$ \f$\Longrightarrow \f$ \c NewAcceleration,
//!
//! is calculated, which can be utilized as a set-point value at instant
//! \f$ T_{i+1} \f$. The time difference between \f$ {\bf M}_{i} \f$ and
//! \f$ {\bf M}_{i+1} \f$ is \c CycleTimeInSeconds. For a detailed
//! description, please refer to the corresponding documentation of the
//! \ref page_InputValues "input values" and
//! \ref page_OutputValues "output values" of the Type IV Reflexxes Motion
//! Library.
//!
//! \param CurrentPosition
//! Pointer to an array of \c double values containing the current
//! position/pose vector \f$ \vec{P}_i \f$
//!
//! \param CurrentVelocity
//! Pointer to an array of \c double values containing the current
//! velocity vector \f$ \vec{V}_i \f$
//!
//! \param CurrentAcceleration
//! Pointer to an array of \c double values containing the current
//! acceleration vector \f$ \vec{A}_i \f$
//!
//! \param MaxAcceleration
//! Pointer to an array of \c double values containing the maximum
//! acceleration vector \f$ \vec{A}_i^{\,max} \f$
//!
//! \param MaxJerk
//! Pointer to an array of \c double values containing the maximum
//! jerk vector \f$ \vec{J}_i^{\,max} \f$
//!
//! \param TargetVelocity
//! Pointer to an array of \c double values containing the target
//! velocity vector \f$ \vec{V}_i^{\,trgt} \f$.
//!
//! \param SelectionVector
//! Pointer to an array of integer values containing the selection
//! vector \f$ \vec{S}_i \f$. Values of zero and less than zero
//! are interpreted as \c false; values greater than zero are
//! interpreted as \c true.
//!
//! \param NewPosition
//! Pointer to an array of \c double values containing the new
//! position/pose vector \f$ \vec{P}_{i+1} \f$.
//!
//! \param NewVelocity
//! Pointer to an array of \c double values containing the new
//! velocity vector \f$ \vec{V}_{i+1} \f$
//!
//! \param NewAcceleration
//! Pointer to an array of \c double values containing the new
//! acceleration vector \f$ \vec{A}_{i+1} \f$
//! 
//! \return
//! An integer value as specified in RMLResultValue. If the implementation
//! is correct, the function either returns ReflexxesAPI::RML_WORKING or
//! ReflexxesAPI::RML_FINAL_STATE_REACHED. In general, one of the
//! following values can be returned:\n\n
//!
//! <ul>
//! <li><b>0</b>: The Online Trajectory Generation algorithm is working;
//!    the final state of motion has not been reached yet.</li>
//! <li><b>1</b>: The desired final state of motion has been reached.</li>
//! <li><b>-1</b>: If the function TypeIVRMLCreate() has not been called
//!    before, this error values will be returned.</li>
//! <li><b>-100</b>: The applied input values are invalid.</li>
//! <li><b>-105</b>: To ensure numerical stability, the value of the minimum
//!    trajectory execution time is limited to a maximum value of
//!    \f$ 10^{10} \f$ seconds.</li>
//! </ul>
//! 
//! Please also refer to the corresponding documentation of the
//! in the class TypeIVRMLVelocity for more information.
//! 
//! \note
//! A complete description of the Online Trajectory Generation framework
//! may be also found at\n
//! \n
//! <b>T. Kroeger.</b>\n
//! <b>On-line Trajectory Generation in Robotic Systems.</b>\n
//! <b>Springer Tracts in Advanced Robotics, Vol. 58, Springer, January 2010.</b>\n
//! <b><a href="http://www.springer.com/978-3-642-05174-6" target="_blanc" title="You may order your personal copy at www.springer.com.">http://www.springer.com/978-3-642-05174-6</a></b>\n
//!
//! \sa TypeIVRMLCreate()
//! \sa TypeIVRMLPosition()
//  ----------------------------------------------------------	
int		TypeIVRMLVelocity	(		const	double*		CurrentPosition
								,	const	double*		CurrentVelocity
								,	const	double*		CurrentAcceleration
								,	const	double*		MaxAcceleration
								,	const	double*		MaxJerk
								,	const	double*		TargetVelocity
								,	const	int*		SelectionVector								
								,			double*		NewPosition
								,			double*		NewVelocity
								,			double*		NewAcceleration		);


#ifdef __cplusplus
extern "C"		// use the C function call standard for all functions  
#endif			// defined within this scope

//  ---------------------- Doxygen info ----------------------
//! \fn int TypeIVRMLDestroy(void)
//!
//! \brief
//! This function releases memory that was allocated by the function
//! TypeIVRMLCreate().
//!
//! \details
//! This function of the C-wrapper of the Reflexxes motion libraries
//! is the counterpart of the function TypeIVRMLCreate(). In order 
//! to prevent from memory leaks, this function has to be called pairwise 
//! with the function TypeIVRMLCreate().\n
//! 
//! Please refer to the documentation of ReflexxesAPI::~ReflexxesAPI()
//! for further information.
//!
//! \warning
//! <b>This function does not meet real-time constraints as heap memory is
//! released.</b>\n\n
//! 
//! \return
//! <ul> 
//! <li>\b 0 if successful</li>
//! <li>\b -1 if TypeIVRMLCreate() has not been called before</li>
//! </ul> 
//!
//! \sa TypeIVRMLCreate()
//  ----------------------------------------------------------	
int		TypeIVRMLDestroy(void);


#endif
