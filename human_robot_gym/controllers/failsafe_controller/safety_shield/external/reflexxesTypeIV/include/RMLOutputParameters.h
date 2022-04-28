//  ---------------------- Doxygen info ----------------------
//! \file RMLOutputParameters.h
//!
//! \brief
//! Header file for the classes RMLOutputParameters, RMLOutputPolynomials,
//! and RMLPolynomial
//! 
//! \details
//! The class RMLOutputParameters constitutes the basis class for the 
//! actual interface classes RMLPositionOutputParameters and
//! RMLVelocityOutputParameters, which are both derived from this one.
//! Objects of the class RMLOutputPolynomials are used within the class
//! RMLOutputParameters, which consists of objects of the class
//! RMLPolynomial.
//!
//! \sa RMLInputParameters.h
//! \sa RMLVelocityOutputParameters.h
//! \sa RMLPositionOutputParameters.h
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


#ifndef __RMLOutputParameters__
#define __RMLOutputParameters__



#include <RMLVector.h>
#include <string.h>
#include <math.h>


//  ---------------------- Doxygen info ----------------------
//! \class RMLPolynomial
//! 
//! \brief
//! Contains the parameters of a section of polynomials for position,
//! velocity, and acceleration for one single degree of freedom
//!
//! \details
//! The three polynomials
//!
//!  - \f$ p(t)\ =\ p_3t^3\ +\ p_2t^2\ +\ p_1t\ +\ p_0 \f$
//!  - \f$ v(t)\ =\ v_2t^2\ +\ v_1t\ +\ v_0 \f$
//!  - \f$ a(t)\ =\ a_1t\ +\ a_0 \f$
//!
//! describe the trajectory within one section and are valid until the
//! time RMLPolynomial::Time_ValidUntil. An entire trajectory for one degree 
//! of freedom consists of a set of such polynomial segments.
//!
//! \sa RMLOutputPolynomials
//! \sa RMLOutputParameters
//! \sa \ref page_Code_02_RMLPositionSampleApplication
//! \sa \ref page_Code_05_RMLVelocitySampleApplication
//  ----------------------------------------------------------
class RMLPolynomial
{
public:

//  ---------------------- Doxygen info ----------------------
//! \fn RMLPolynomial(void)
//! 
//! \brief
//! Constructor of class RMLPolynomial
//  ----------------------------------------------------------
	RMLPolynomial(void)
	{
		Time_ValidUntil	=	0.0													;
		memset(PositionPolynomialCoefficients,		0x0, 4 * sizeof(double))	;
		memset(VelocityPolynomialCoefficients,		0x0, 3 * sizeof(double))	;
		memset(AccelerationPolynomialCoefficients,	0x0, 2 * sizeof(double))	;
	}


//  ---------------------- Doxygen info ----------------------
//! \fn RMLPolynomial(const RMLPolynomial &P)
//! 
//! \brief
//! Copy constructor of class RMLPolynomial
//! 
//! \param P
//! Object to be copied
//  ----------------------------------------------------------
	RMLPolynomial(const RMLPolynomial &P)
	{
		*this = P;
	}


//  ---------------------- Doxygen info ----------------------
//! \fn ~RMLPolynomial(void)
//! 
//! \brief
//! Destructor of class RMLPolynomial
//  ----------------------------------------------------------
	~RMLPolynomial(void)
	{
	}


//  ---------------------- Doxygen info ----------------------
//! \fn inline RMLPolynomial &operator = (const RMLPolynomial &P)
//! 
//! \brief
//! Copy operator
//! 
//! \param P
//! RMLPolynomial object to be copied
//!
//! \return
//! The duplicated object
//  ----------------------------------------------------------	
	inline RMLPolynomial &operator = (const RMLPolynomial &P)
	{
		memcpy(this, &P, sizeof(RMLPolynomial));
        return(*this);
	}


//  ---------------------- Doxygen info ----------------------
//! \fn void Echo(FILE* FileHandler = stdout) const
//! 
//! \brief
//! Prints all polynomial coefficients to *FileHandler
//! 
//! \param FileHandler
//! File handler for the output
//! 
//! \warning
//! The usage of this method is \b not real-time capable.
//  ----------------------------------------------------------
    void Echo(FILE* FileHandler = stdout) const
    {
        if (FileHandler == NULL)
        {
            return;
        }        
		fprintf(FileHandler,   "Valid until : %.6le seconds\n", this->Time_ValidUntil);
        fprintf(FileHandler,   "Position    : p(t)= %.6lft^3 %c %.6lft^2 %c %.6lft %c %.6lf\n"
					, this->PositionPolynomialCoefficients[3]
					, (this->PositionPolynomialCoefficients[2]>=0)?('+'):('-')
					, fabs(this->PositionPolynomialCoefficients[2])
					, (this->PositionPolynomialCoefficients[1]>=0)?('+'):('-')
					, fabs(this->PositionPolynomialCoefficients[1])
					, (this->PositionPolynomialCoefficients[0]>=0)?('+'):('-')
					, fabs(this->PositionPolynomialCoefficients[0]));
        fprintf(FileHandler,   "Velocity    : v(t)= %.6lft^2 %c %.6lft %c %.6lf\n"
					, this->VelocityPolynomialCoefficients[2]
					, (this->VelocityPolynomialCoefficients[1]>=0)?('+'):('-')
					, fabs(this->VelocityPolynomialCoefficients[1])
					, (this->VelocityPolynomialCoefficients[0]>=0)?('+'):('-')
					, fabs(this->VelocityPolynomialCoefficients[0]));
        fprintf(FileHandler,   "Acceleration: a(t)= %.6lft %c %.6lf\n"
					, this->AccelerationPolynomialCoefficients[1]
					, (this->AccelerationPolynomialCoefficients[0]>=0)?('+'):('-')
					, fabs(this->AccelerationPolynomialCoefficients[0]));
	}


//  ---------------------- Doxygen info ----------------------
//! \var double PositionPolynomialCoefficients[4]
//! 
//! \brief
//! Array containing the coefficients of the position polynomial
//! 
//! \details
//! The four elements of this \c double array contain the coefficients
//! of the polynomial
//!
//!	\f$ p(t)\ =\ p_3t^3\ +\ p_2t^2\ +\ p_1t\ +\ p_0 \f$
//!
//! - <c>PositionPolynomialCoefficients[0]</c> \f$\Longleftrightarrow\ \ \ p_0\f$ 
//! - <c>PositionPolynomialCoefficients[1]</c> \f$\Longleftrightarrow\ \ \ p_1\f$ 
//! - <c>PositionPolynomialCoefficients[2]</c> \f$\Longleftrightarrow\ \ \ p_2\f$ 
//! - <c>PositionPolynomialCoefficients[3]</c> \f$\Longleftrightarrow\ \ \ p_3\f$
//!
//! This polynomial segment is valid until the time instant 
//! RMLPolynomial::Time_ValidUntil.
//  ----------------------------------------------------------
	double					PositionPolynomialCoefficients[4];


//  ---------------------- Doxygen info ----------------------
//! \var double VelocityPolynomialCoefficients[3]
//! 
//! \brief
//! Array containing the coefficients of the velocity polynomial
//! 
//! \details
//! The three elements of this \c double array contain the coefficients
//! of the polynomial
//!
//!	\f$ v(t)\ =\ v_2t^2\ +\ v_1t\ +\ v_0 \f$
//!
//! - <c>VelocityPolynomialCoefficients[0]</c> \f$\Longleftrightarrow\ \ \ v_0\f$ 
//! - <c>VelocityPolynomialCoefficients[1]</c> \f$\Longleftrightarrow\ \ \ v_1\f$ 
//! - <c>VelocityPolynomialCoefficients[2]</c> \f$\Longleftrightarrow\ \ \ v_2\f$ 
//!
//! This polynomial segment is valid until the time instant 
//! RMLPolynomial::Time_ValidUntil.
//  ----------------------------------------------------------
	double					VelocityPolynomialCoefficients[3];


//  ---------------------- Doxygen info ----------------------
//! \var double AccelerationPolynomialCoefficients[2]
//! 
//! \brief
//! Array containing the coefficients of the acceleration polynomial
//! 
//! \details
//! The two elements of this \c double array contain the coefficients
//! of the polynomial
//!
//!	\f$ a(t)\ =\ a_1t\ +\ a_0 \f$
//!
//! - <c>AccelerationPolynomialCoefficients[0]</c> \f$\Longleftrightarrow\ \ \ a_0\f$ 
//! - <c>AccelerationPolynomialCoefficients[1]</c> \f$\Longleftrightarrow\ \ \ a_1\f$
//!
//! This polynomial segment is valid until the time instant 
//! RMLPolynomial::Time_ValidUntil.
//  ----------------------------------------------------------
	double					AccelerationPolynomialCoefficients[2];


//  ---------------------- Doxygen info ----------------------
//! \var double Time_ValidUntil
//! 
//! \brief
//! Time in seconds until this polynomial is valid
//  ----------------------------------------------------------
	double					Time_ValidUntil;

};// class RMLPolynomial


//  ---------------------- Doxygen info ----------------------
//! \class RMLOutputPolynomials
//! 
//! \brief
//! Contains all polynomial coefficients for all selected degrees of
//! freedom
//!
//! \details
//! The class contains a set of piecewise polynomials for position
//! velocity, and acceleration progressions of all selected degrees of
//! freedom. It contains a 2D array of RMLPolynomial objects that 
//! contain the coefficients of the actual polynomials.
//!
//! \sa RMLPolynomial
//! \sa RMLOutputParameters
//! \sa \ref page_Code_02_RMLPositionSampleApplication
//! \sa \ref page_Code_05_RMLVelocitySampleApplication
//  ----------------------------------------------------------
class RMLOutputPolynomials
{
public:

//  ---------------------- Doxygen info ----------------------
//! \fn RMLOutputPolynomials(const unsigned int DegreesOfFreedom)
//! 
//! \brief
//! Constructor of class RMLOutputPolynomials
//! 
//! \warning
//! The constructor is \b not real-time capable as heap memory has to be
//! allocated.
//! 
//! \param DegreesOfFreedom
//! Specifies the number of degrees of freedom
//  ----------------------------------------------------------
	RMLOutputPolynomials(const unsigned int DegreesOfFreedom)
	{
		unsigned int		i					=	0											;

		this->NumberOfDOFs						=	DegreesOfFreedom							;
		this->NumberOfCurrentlyValidSegments	=	new int	[this->NumberOfDOFs]	;
		this->Coefficients						=	new RMLPolynomial*	[this->NumberOfDOFs]	;

		memset(this->NumberOfCurrentlyValidSegments, 0x0, this->NumberOfDOFs * sizeof(unsigned int));

        for (i = 0; i < this->NumberOfDOFs; i++)
        {		
            (this->Coefficients)[i]	=	new RMLPolynomial[14];
        }
	}


//  ---------------------- Doxygen info ----------------------
//! \fn RMLOutputPolynomials(const RMLOutputPolynomials &P)
//! 
//! \brief
//! Copy constructor of class RMLOutputPolynomials
//! 
//! \warning
//! The constructor is \b not real-time capable as heap memory has to be
//! allocated.
//! 
//! \param P
//! Object to be copied
//  ----------------------------------------------------------
	RMLOutputPolynomials(const RMLOutputPolynomials &P)
	{
		unsigned int		i					=	0										;

		this->Coefficients						=	new RMLPolynomial*	[P.NumberOfDOFs]	;
		this->NumberOfCurrentlyValidSegments	=	new int	[P.NumberOfDOFs]	;

        for (i = 0; i < P.NumberOfDOFs; i++)
        {		
            (this->Coefficients)[i]	=	new RMLPolynomial[14];
        }

		*this = P;
	}


//  ---------------------- Doxygen info ----------------------
//! \fn ~RMLOutputPolynomials(void)
//! 
//! \brief
//! Destructor of class RMLOutputPolynomials
//  ----------------------------------------------------------
	~RMLOutputPolynomials(void)
	{
		unsigned int		i			=	0					;

		delete[]		this->NumberOfCurrentlyValidSegments	;

        for (i = 0; i < this->NumberOfDOFs; i++)
        {		
            delete[]	(this->Coefficients)[i]					;
        }
		delete[]		this->Coefficients						;

		this->Coefficients						=	NULL		;
		this->NumberOfCurrentlyValidSegments	=	NULL		;
		this->NumberOfDOFs						=	0			;
	}


//  ---------------------- Doxygen info ----------------------
//! \fn RMLOutputPolynomials &operator = (const RMLOutputPolynomials &P)
//! 
//! \brief
//! Copy operator
//! 
//! \param P
//! RMLOutputPolynomials object to be copied
//!
//! \return
//! The duplicated object
//  ----------------------------------------------------------	
	RMLOutputPolynomials &operator = (const RMLOutputPolynomials &P)
	{
		unsigned int		i					=	0					;
		
		int					j					=	0					;

		this->NumberOfDOFs						=	P.NumberOfDOFs		;		

		for (i = 0; i < this->NumberOfDOFs; i++)
		{
			this->NumberOfCurrentlyValidSegments[i]	=	P.NumberOfCurrentlyValidSegments[i];
			for (j = 0; j < this->NumberOfCurrentlyValidSegments[i]; j++)
			{
				this->Coefficients[i][j]	=	P.Coefficients[i][j];
			}
		}

        return(*this);
	}


//  ---------------------- Doxygen info ----------------------
//! \var unsigned int NumberOfDOFs
//! 
//! \brief
//! The number of degrees of freedom \f$ K \f$
//  ----------------------------------------------------------
	unsigned int			NumberOfDOFs;



//  ---------------------- Doxygen info ----------------------
//! \var int NumberOfDOFs
//! 
//! \brief
//! Pointer to an array of integer numbers with
//! RMLOutputPolynomials::NumberOfDOFs elements that contain the number of
//! valid polynomial segments per degree of freedom \f$ k \f$
//  ----------------------------------------------------------
	int						*NumberOfCurrentlyValidSegments;


//  ---------------------- Doxygen info ----------------------
//! \var RMLPolynomial **Coefficients
//! 
//! \brief
//! Pointer to a 2D array of \c RMLPolynomial objects
//!
//! \details
//! This heap-allocated two-dimensional array contains all coefficients of
//! all polynomial segments for all \f$ K \f$ degrees of freedom. A
//! trajectory for one single degree of freedom may consist of up to 14
//! polynomial segments. The coefficients of a single polynomial can be
//! accessed by <c>RMLPolynomial::Coefficients[k][l]</c>, where \c k is the
//! index of a degree of freedom \f$ k\ \in\ \{0,\,\dots,\,K\} \f$ and
//! \c l is the index of a polynomial segment
//! \f$ l\ \in\ \{0,\,\dots,\,14\} \f$.
//!
//! \sa RMLPolynomial
//  ----------------------------------------------------------
	RMLPolynomial			**Coefficients;

};// class RMLOutputPolynomials


//  ---------------------- Doxygen info ----------------------
//! \class RMLOutputParameters
//! 
//! \brief
//! Class for the output parameters of the Online
//! Trajectory Generation algorithm
//! 
//! \details
//! The class RMLOutputParameters constitutes the basis class for the 
//! actual interface classes RMLPositionOutputParameters and
//! RMLVelocityOutputParameters, which are both derived from this one.
//!
//! \sa ReflexxesAPI
//! \sa RMLPositionOutputParameters
//! \sa RMLVelocityOutputParameters
//! \sa RMLInputParameters
//! \sa \ref page_OutputValues
//  ----------------------------------------------------------
class RMLOutputParameters
{
public:

    
//  ---------------------- Doxygen info ----------------------
//! \enum ReturnValue
//! 
//! \brief
//! Return values for the methods of the class RMLOutputParameters
//  ----------------------------------------------------------	
    enum ReturnValue
    {
        RETURN_SUCCESS	=	0,
        RETURN_ERROR	=	-1
    };


protected:

//  ---------------------- Doxygen info ----------------------
//! \fn RMLOutputParameters(const unsigned int DegreesOfFreedom)
//! 
//! \brief
//! Constructor of class RMLOutputParameters
//! 
//! \warning
//! The constructor is \b not real-time capable as heap memory has to be
//! allocated.
//! 
//! \param DegreesOfFreedom
//! Specifies the number of degrees of freedom
//!
//! \note
//! This is only the base class for the classes\n\n
//! <ul>
//!  <li>RMLPositionOutputParameters and</li>
//!  <li>RMLVelocityOutputParameters,\n\n</li>
//! </ul>
//! such that the constructor is declared \c protected.
//  ----------------------------------------------------------
    RMLOutputParameters(const unsigned int DegreesOfFreedom)
    {
        unsigned int	i							=	0											;
             
        this->TrajectoryIsPhaseSynchronized			=	false										;	       
        this->NumberOfDOFs							=	DegreesOfFreedom							;        
        this->SynchronizationTime					=	0.0											;        
        this->ANewCalculationWasPerformed			=	false										;        
        this->DOFWithTheGreatestExecutionTime		=	0											;
        this->ResultValue							=	-1											;
        this->OverrideFilterIsActive				=	false										;
        this->CurrentOverrideValue					=	1.0											;
    
        this->NewPositionVector						=	new RMLDoubleVector(DegreesOfFreedom)		;
        this->NewVelocityVector						=	new RMLDoubleVector(DegreesOfFreedom)		;
        this->NewAccelerationVector					=	new RMLDoubleVector(DegreesOfFreedom)		;		
        this->MinExtremaTimesVector					=	new RMLDoubleVector(DegreesOfFreedom)		;
        this->MaxExtremaTimesVector					=	new RMLDoubleVector(DegreesOfFreedom)		;
        this->MinPosExtremaPositionVectorOnly		=	new RMLDoubleVector(DegreesOfFreedom)		;
        this->MaxPosExtremaPositionVectorOnly		=	new RMLDoubleVector(DegreesOfFreedom)		;
        this->ExecutionTimes						=	new RMLDoubleVector(DegreesOfFreedom)		;
		this->Polynomials							=	new RMLOutputPolynomials(DegreesOfFreedom)	;
        
        memset(this->NewPositionVector->VecData					,	0x0	,		DegreesOfFreedom * sizeof(double))	;
        memset(this->NewVelocityVector->VecData					,	0x0	,		DegreesOfFreedom * sizeof(double))	;
        memset(this->NewAccelerationVector->VecData				,	0x0	,		DegreesOfFreedom * sizeof(double))	;
        memset(this->MinExtremaTimesVector->VecData				,	0x0	,		DegreesOfFreedom * sizeof(double))	;
        memset(this->MaxExtremaTimesVector->VecData				,	0x0	,		DegreesOfFreedom * sizeof(double))	;
        memset(this->MinPosExtremaPositionVectorOnly->VecData	,	0x0	,		DegreesOfFreedom * sizeof(double))	;
        memset(this->MaxPosExtremaPositionVectorOnly->VecData	,	0x0	,		DegreesOfFreedom * sizeof(double))	;
        memset(this->ExecutionTimes->VecData					,	0x0	,		DegreesOfFreedom * sizeof(double))	;		
        
        this->MinPosExtremaPositionVectorArray		=	new RMLDoubleVector*[DegreesOfFreedom]	;
        this->MinPosExtremaVelocityVectorArray		=	new RMLDoubleVector*[DegreesOfFreedom]	;
        this->MinPosExtremaAccelerationVectorArray	=	new RMLDoubleVector*[DegreesOfFreedom]	;
        this->MaxPosExtremaPositionVectorArray		=	new RMLDoubleVector*[DegreesOfFreedom]	;
        this->MaxPosExtremaVelocityVectorArray		=	new RMLDoubleVector*[DegreesOfFreedom]	;
        this->MaxPosExtremaAccelerationVectorArray	=	new RMLDoubleVector*[DegreesOfFreedom]	;		
        
        for (i = 0; i < DegreesOfFreedom; i++)
        {		
            (this->MinPosExtremaPositionVectorArray)		[i]	=	new RMLDoubleVector(DegreesOfFreedom);
            (this->MinPosExtremaVelocityVectorArray)		[i]	=	new RMLDoubleVector(DegreesOfFreedom);
            (this->MinPosExtremaAccelerationVectorArray)	[i]	=	new RMLDoubleVector(DegreesOfFreedom);
            (this->MaxPosExtremaPositionVectorArray)		[i]	=	new RMLDoubleVector(DegreesOfFreedom);
            (this->MaxPosExtremaVelocityVectorArray)		[i]	=	new RMLDoubleVector(DegreesOfFreedom);
            (this->MaxPosExtremaAccelerationVectorArray)	[i]	=	new RMLDoubleVector(DegreesOfFreedom);
            
            memset(((this->MinPosExtremaPositionVectorArray)			[i])->VecData		,	0x0	,		DegreesOfFreedom * sizeof(double))	;
            memset(((this->MinPosExtremaVelocityVectorArray)			[i])->VecData		,	0x0	,		DegreesOfFreedom * sizeof(double))	;
            memset(((this->MinPosExtremaAccelerationVectorArray)		[i])->VecData		,	0x0	,		DegreesOfFreedom * sizeof(double))	;
            memset(((this->MaxPosExtremaPositionVectorArray)			[i])->VecData		,	0x0	,		DegreesOfFreedom * sizeof(double))	;
            memset(((this->MaxPosExtremaVelocityVectorArray)			[i])->VecData		,	0x0	,		DegreesOfFreedom * sizeof(double))	;
            memset(((this->MaxPosExtremaAccelerationVectorArray)		[i])->VecData		,	0x0	,		DegreesOfFreedom * sizeof(double))	;
        }
    }


//  ---------------------- Doxygen info ----------------------
//! \fn RMLOutputParameters(const RMLOutputParameters &OP)
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
//!
//! \note
//! This is only the base class for the classes\n\n
//! <ul>
//!  <li>RMLPositionOutputParameters and</li>
//!  <li>RMLVelocityOutputParameters,\n\n</li>
//! </ul>
//! such that the constructor is declared \c protected.
//  ----------------------------------------------------------
    RMLOutputParameters(const RMLOutputParameters &OP)
    {
        unsigned int	i							=	0														;
        
        this->NewPositionVector						=	new RMLDoubleVector(OP.NumberOfDOFs)					;
        this->NewVelocityVector						=	new RMLDoubleVector(OP.NumberOfDOFs)					;
        this->NewAccelerationVector					=	new RMLDoubleVector(OP.NumberOfDOFs)					;		
        this->MinExtremaTimesVector					=	new RMLDoubleVector(OP.NumberOfDOFs)					;
        this->MaxExtremaTimesVector					=	new RMLDoubleVector(OP.NumberOfDOFs)					;
        this->MinPosExtremaPositionVectorOnly		=	new RMLDoubleVector(OP.NumberOfDOFs)					;
        this->MaxPosExtremaPositionVectorOnly		=	new RMLDoubleVector(OP.NumberOfDOFs)					;
        this->ExecutionTimes						=	new RMLDoubleVector(OP.NumberOfDOFs)					;
		this->Polynomials							=	new RMLOutputPolynomials(OP.NumberOfDOFs)				;  

        this->MinPosExtremaPositionVectorArray		=	new RMLDoubleVector *[OP.NumberOfDOFs]					;
        this->MinPosExtremaVelocityVectorArray		=	new RMLDoubleVector *[OP.NumberOfDOFs]					;
        this->MinPosExtremaAccelerationVectorArray	=	new RMLDoubleVector *[OP.NumberOfDOFs]					;
        this->MaxPosExtremaPositionVectorArray		=	new RMLDoubleVector *[OP.NumberOfDOFs]					;
        this->MaxPosExtremaVelocityVectorArray		=	new RMLDoubleVector *[OP.NumberOfDOFs]					;
        this->MaxPosExtremaAccelerationVectorArray	=	new RMLDoubleVector *[OP.NumberOfDOFs]					;				

        for (i = 0; i < OP.NumberOfDOFs; i++)
        {		
            (this->MinPosExtremaPositionVectorArray)		[i]	=	new RMLDoubleVector(OP.NumberOfDOFs)		;
            (this->MinPosExtremaVelocityVectorArray)		[i]	=	new RMLDoubleVector(OP.NumberOfDOFs)		;
            (this->MinPosExtremaAccelerationVectorArray)	[i]	=	new RMLDoubleVector(OP.NumberOfDOFs)		;
            (this->MaxPosExtremaPositionVectorArray)		[i]	=	new RMLDoubleVector(OP.NumberOfDOFs)		;
            (this->MaxPosExtremaVelocityVectorArray)		[i]	=	new RMLDoubleVector(OP.NumberOfDOFs)		;
            (this->MaxPosExtremaAccelerationVectorArray)	[i]	=	new RMLDoubleVector(OP.NumberOfDOFs)		;
        }
		       
        *this				=	OP																				;
    }


//  ---------------------- Doxygen info ----------------------
//! \fn void Echo(FILE* FileHandler = stdout) const
//! 
//! \brief
//! Prints the new state of motion of the output parameters to *FileHandler
//! 
//! \param FileHandler
//! File handler for the output
//! 
//! \warning
//! The usage of this method is \b not real-time capable.
//  ----------------------------------------------------------
    void Echo(FILE* FileHandler = stdout) const
    {
        unsigned int		i	=	0;
        
        if (FileHandler == NULL)
        {
            return;
        }
        
        fprintf(FileHandler,   "New position vector        : ");
        for (i = 0; i < this->NumberOfDOFs; i++)
        {
            fprintf(FileHandler, " %.20le ", this->NewPositionVector->VecData[i]);
        }
        fprintf(FileHandler, "\nNew velocity vector        : ");
        for (i = 0; i < this->NumberOfDOFs; i++)
        {
            fprintf(FileHandler, " %.20le ", this->NewVelocityVector->VecData[i]);
        }
        fprintf(FileHandler, "\nNew acceleration vector    : ");
        for (i = 0; i < this->NumberOfDOFs; i++)
        {
            fprintf(FileHandler, " %.20le ", this->NewAccelerationVector->VecData[i]);
        }
        fprintf(FileHandler, "\n");
        return;
    }


public:

//  ---------------------- Doxygen info ----------------------
//! \fn ~RMLOutputParameters(void)
//! 
//! \brief
//! Destructor of class RMLOutputParameters
//  ----------------------------------------------------------
    ~RMLOutputParameters(void)
    {
        unsigned int		i	=	0;
    
        for (i = 0; i < this->NumberOfDOFs; i++)
        {		
            delete ((this->MinPosExtremaPositionVectorArray)		[i])	;
            delete ((this->MinPosExtremaVelocityVectorArray)		[i])	;
            delete ((this->MinPosExtremaAccelerationVectorArray)	[i])	;
            delete ((this->MaxPosExtremaPositionVectorArray)		[i])	;
            delete ((this->MaxPosExtremaVelocityVectorArray)		[i])	;
            delete ((this->MaxPosExtremaAccelerationVectorArray)	[i])	;
        }	
    
        delete 		this->NewPositionVector									;
        delete 		this->NewVelocityVector									;		
        delete 		this->NewAccelerationVector								;
        delete[]	this->MinPosExtremaPositionVectorArray					;
        delete[]	this->MinPosExtremaVelocityVectorArray					;
        delete[]	this->MinPosExtremaAccelerationVectorArray				;
        delete[]	this->MaxPosExtremaPositionVectorArray					;
        delete[]	this->MaxPosExtremaVelocityVectorArray					;
        delete[]	this->MaxPosExtremaAccelerationVectorArray				;		
        delete 		this->MinExtremaTimesVector								;
        delete 		this->MaxExtremaTimesVector								;
        delete 		this->MinPosExtremaPositionVectorOnly					;
        delete 		this->MaxPosExtremaPositionVectorOnly					;
        delete 		this->ExecutionTimes									;
		delete		this->Polynomials										;
        
        this->NewPositionVector						=	NULL				;
        this->NewVelocityVector						=	NULL				;
        this->NewAccelerationVector					=	NULL				;
        this->MinPosExtremaPositionVectorArray		=	NULL				;
        this->MinPosExtremaVelocityVectorArray		=	NULL				;
        this->MinPosExtremaAccelerationVectorArray	=	NULL				;
        this->MaxPosExtremaPositionVectorArray		=	NULL				;
        this->MaxPosExtremaVelocityVectorArray		=	NULL				;
        this->MaxPosExtremaAccelerationVectorArray	=	NULL				;		
        this->MinExtremaTimesVector					=	NULL				;
        this->MaxExtremaTimesVector					=	NULL				;
        this->MinPosExtremaPositionVectorOnly		=	NULL				;
        this->MaxPosExtremaPositionVectorOnly		=	NULL				;
        this->ExecutionTimes						=	NULL				;
		this->Polynomials							=	NULL				;
        
        this->NumberOfDOFs							=	0					;		
    }
    

//  ---------------------- Doxygen info ----------------------
//! \fn RMLOutputParameters &operator = (const RMLOutputParameters &OP)
//! 
//! \brief
//! Copy operator
//! 
//! \param OP
//! RMLOutputParameters object to be copied
//!
//! \return
//! The duplicated object
//  ----------------------------------------------------------	
    RMLOutputParameters &operator = (const RMLOutputParameters &OP)
    {
        unsigned int		i						=	0										;

        this->NumberOfDOFs							=	OP.NumberOfDOFs							;		
        this->TrajectoryIsPhaseSynchronized			=	OP.TrajectoryIsPhaseSynchronized		;
        this->ANewCalculationWasPerformed			=	OP.ANewCalculationWasPerformed			;
        this->ResultValue							=	OP.ResultValue							;
        this->OverrideFilterIsActive				=	OP.OverrideFilterIsActive				;
        this->SynchronizationTime					=	OP.SynchronizationTime					;
        this->DOFWithTheGreatestExecutionTime		=	OP.DOFWithTheGreatestExecutionTime		;
        this->CurrentOverrideValue					=	OP.CurrentOverrideValue					;
        
        *(this->NewPositionVector)					=	*(OP.NewPositionVector)					;
        *(this->NewVelocityVector)					=	*(OP.NewVelocityVector)					;
        *(this->NewAccelerationVector)				=	*(OP.NewAccelerationVector)				;
        *(this->MinExtremaTimesVector)				=	*(OP.MinExtremaTimesVector)				;
        *(this->MaxExtremaTimesVector)				=	*(OP.MaxExtremaTimesVector)				;
        *(this->ExecutionTimes)						=	*(OP.ExecutionTimes)					;
        *(this->MinPosExtremaPositionVectorOnly)	=	*(OP.MinPosExtremaPositionVectorOnly)	;
        *(this->MaxPosExtremaPositionVectorOnly)	=	*(OP.MaxPosExtremaPositionVectorOnly)	;

        for (i = 0; i < this->NumberOfDOFs; i++)
        {
            OP.GetMotionStateAtMinPosForOneDOF(		i
                                                ,	(this->MinPosExtremaPositionVectorArray)[i]
                                                ,	(this->MinPosExtremaVelocityVectorArray)[i]
                                                ,	(this->MinPosExtremaAccelerationVectorArray)[i]);

            OP.GetMotionStateAtMaxPosForOneDOF(		i
                                                ,	(this->MaxPosExtremaPositionVectorArray)[i]
                                                ,	(this->MaxPosExtremaVelocityVectorArray)[i]
                                                ,	(this->MaxPosExtremaAccelerationVectorArray)[i]);											
        }

		*(this->Polynomials)	=	*(OP.Polynomials);

        return(*this);
    }
    

//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetNewPositionVector(RMLDoubleVector *OutputVector) const
//! 
//! \brief
//! Copies the contents of the \c RMLDoubleVector object containing the
//! new position vector \f$ \vec{P}_{i+1} \f$ to the \c RMLDoubleVector
//! object referred to by \c OutputVector
//! 
//! \param OutputVector
//! A pointer to an \c RMLDoubleVector object, to which the data will be 
//! copied
//! 
//! \sa GetNewPositionVector(double *OutputVector, const unsigned int &SizeInBytes) const
//! \sa GetNewPositionVectorElement(double *OutputValue, const unsigned int &Index) const
//  ----------------------------------------------------------
    inline void GetNewPositionVector(RMLDoubleVector *OutputVector) const
    {
        *OutputVector	=	*(this->NewPositionVector);
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetNewPositionVector(double *OutputVector, const unsigned int &SizeInBytes) const
//! 
//! \brief
//! Copies the array of \c double values representing the new
//! position vector \f$ \vec{P}_{i+1} \f$ to the memory pointed to by
//! \c OutputVector
//! 
//! \param OutputVector
//! A pointer to an array of \c double values, to which the data will be 
//! copied
//! 
//! \param SizeInBytes
//! The size of available memory at the location pointed to by 
//! \c OutputVector. To assure safety and to prevent from prohibited writing 
//! into protected memory areas, the user has to specify the amount
//! of available memory in bytes. For a correct operation, the value of
//! \c SizeInBytes has to equal the number of vector elements multiplied
//! by the size of a \c double value.
//! 
//! \sa GetNewPositionVector(RMLDoubleVector *OutputVector) const
//! \sa GetNewPositionVectorElement(double *OutputValue, const unsigned int &Index) const
//  ----------------------------------------------------------
    inline void GetNewPositionVector(		double				*OutputVector
                                        ,	const unsigned int	&SizeInBytes) const
    {
        memcpy(		(void*)OutputVector
                ,	(void*)this->NewPositionVector->VecData
                ,	SizeInBytes	);
    }

    
//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetNewPositionVectorElement(double *OutputValue, const unsigned int &Index) const
//! 
//! \brief
//! Copies one element of the new selection vector 
//! \f$ \vec{P}_{i+1} \f$ to the memory pointed to by \c OutputValue
//! 
//! \param OutputValue
//! A pointer to one \c double value, to which the desired vector element
//! will be copied
//! 
//! \param Index
//! Specifies the desired element of the vector. The element numbering 
//! starts with \em 0 (zero). If this value is greater the number 
//! of vector elements, a value of \em 0.0 will be written to the memory
//! pointed to by \c OutputValue.
//! 
//! \sa GetNewPositionVector(RMLDoubleVector *OutputVector) const
//! \sa GetNewPositionVectorElement(double *OutputValue, const unsigned int &Index) const
//! \sa GetNewPositionVectorElement(const unsigned int &Index) const
//  ----------------------------------------------------------
    inline void GetNewPositionVectorElement(	double				*OutputValue
                                            ,	const unsigned int	&Index) const
    {
        if ( ( Index + 1 ) > ((unsigned int) this->NewPositionVector->GetVecDim() ) )
        {
            *OutputValue	=	0.0;
        }
        else
        {	
            *OutputValue	=	(*this->NewPositionVector)[Index];
        }
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline double GetNewPositionVectorElement(const unsigned int &Index) const
//! 
//! \brief
//! Returns one single element of the new selection vector 
//! \f$ \vec{P}_{i+1} \f$
//! 
//! \param Index
//! Specifies the desired element of the vector. The index of the first 
//! vector element is \em 0 (zero). If the value of \c Index value is 
//! greater the number of vector elements, a value of \em 0.0 will be
//! written to the memory pointed to by \c OutputValue.
//! 
//! \sa GetNewPositionVectorElement(double *OutputValue, const unsigned int &Index) const
//  ----------------------------------------------------------
    inline double GetNewPositionVectorElement(const unsigned int &Index) const
    {
        if ( ( Index + 1 ) > ((unsigned int) this->NewPositionVector->GetVecDim() ) )
        {
            return(0.0);
        }
        else
        {		
            return( (*this->NewPositionVector)[Index] );
        }		
    }
    
    
// #############################################################################	
    

//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetNewVelocityVector(RMLDoubleVector *OutputVector) const
//! 
//! \brief
//! Copies the contents of the \c RMLDoubleVector object containing the
//! new velocity vector \f$ \vec{V}_{i+1} \f$ to the \c RMLDoubleVector
//! object referred to by \c OutputVector
//! 
//! \param OutputVector
//! A pointer to an \c RMLDoubleVector object, to which the data will be 
//! copied
//! 
//! \sa GetNewVelocityVector(double *OutputVector, const unsigned int &SizeInBytes) const
//! \sa GetNewVelocityVectorElement(double *OutputValue, const unsigned int &Index) const
//  ----------------------------------------------------------
    inline void GetNewVelocityVector(RMLDoubleVector *OutputVector) const
    {
        *OutputVector	=	*(this->NewVelocityVector);
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetNewVelocityVector(double *OutputVector, const unsigned int &SizeInBytes) const
//! 
//! \brief
//! Copies the array of \c double values representing the new
//! velocity vector \f$ \vec{V}_{i+1} \f$ to the memory pointed to by
//! \c OutputVector
//! 
//! \param OutputVector
//! A pointer to an array of \c double values, to which the data will be 
//! copied
//! 
//! \param SizeInBytes
//! The size of available memory at the location pointed to by 
//! \c OutputVector. To assure safety and to prevent from prohibited writing 
//! into protected memory areas, the user has to specify the amount
//! of available memory in bytes. For a correct operation, the value of
//! \c SizeInBytes has to equal the number of vector elements multiplied
//! by the size of a \c double value.
//! 
//! \sa GetNewVelocityVector(RMLDoubleVector *OutputVector) const
//! \sa GetNewVelocityVectorElement(double *OutputValue, const unsigned int &Index) const
//  ----------------------------------------------------------
    inline void GetNewVelocityVector(		double				*OutputVector
                                        ,	const unsigned int	&SizeInBytes) const
    {
        memcpy(		(void*)OutputVector
                ,	(void*)this->NewVelocityVector->VecData
                ,	SizeInBytes	);
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetNewVelocityVectorElement(double *OutputValue, const unsigned int &Index) const
//! 
//! \brief
//! Copies one element of the new selection vector 
//! \f$ \vec{V}_{i+1} \f$ to the memory pointed to by \c OutputValue
//! 
//! \param OutputValue
//! A pointer to one \c double value, to which the desired vector element
//! will be copied
//! 
//! \param Index
//! Specifies the desired element of the vector. The element numbering 
//! starts with \em 0 (zero). If this value is greater the number 
//! of vector elements, a value of \em 0.0 will be written to the memory
//! pointed to by \c OutputValue.
//! 
//! \sa GetNewVelocityVector(RMLDoubleVector *OutputVector) const
//! \sa GetNewVelocityVectorElement(double *OutputValue, const unsigned int &Index) const
//! \sa GetNewVelocityVectorElement(const unsigned int &Index) const
//  ----------------------------------------------------------
    inline void GetNewVelocityVectorElement(	double				*OutputValue
                                            ,	const unsigned int	&Index) const
    {
        if ( ( Index + 1 ) > ((unsigned int) this->NewVelocityVector->GetVecDim() ) )
        {
            *OutputValue	=	0.0;
        }
        else
        {	
            *OutputValue	=	(*this->NewVelocityVector)[Index];
        }
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline double GetNewVelocityVectorElement(const unsigned int &Index) const
//! 
//! \brief
//! Returns one single element of the new selection vector 
//! \f$ \vec{V}_{i+1} \f$
//! 
//! \param Index
//! Specifies the desired element of the vector. The index of the first 
//! vector element is \em 0 (zero). If the value of \c Index value is 
//! greater the number of vector elements, a value of \em 0.0 will be
//! written to the memory pointed to by \c OutputValue.
//! 
//! \sa GetNewVelocityVectorElement(double *OutputValue, const unsigned int &Index) const
//  ----------------------------------------------------------
    inline double GetNewVelocityVectorElement(const unsigned int &Index) const
    {
        if ( ( Index + 1 ) > ((unsigned int) this->NewVelocityVector->GetVecDim() ) )
        {
            return(0.0);
        }
        else
        {		
            return( (*this->NewVelocityVector)[Index] );
        }		
    }


// #############################################################################	


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetNewAccelerationVector(RMLDoubleVector *OutputVector) const
//! 
//! \brief
//! Copies the contents of the \c RMLDoubleVector object containing the
//! new acceleration vector \f$ \vec{A}_{i+1} \f$ to the \c RMLDoubleVector
//! object referred to by \c OutputVector
//! 
//! \param OutputVector
//! A pointer to an \c RMLDoubleVector object, to which the data will be 
//! copied
//!
//! \sa GetNewAccelerationVector(double *OutputVector, const unsigned int &SizeInBytes) const
//! \sa GetNewAccelerationVectorElement(double *OutputValue, const unsigned int &Index) const
//  ----------------------------------------------------------
    inline void GetNewAccelerationVector(RMLDoubleVector *OutputVector) const
    {
        *OutputVector	=	*(this->NewAccelerationVector);
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetNewAccelerationVector(double *OutputVector, const unsigned int &SizeInBytes) const
//! 
//! \brief
//! Copies the array of \c double values representing the new
//! acceleration vector \f$ \vec{A}_{i+1} \f$ to the memory pointed to by
//! \c OutputVector
//! 
//! \param OutputVector
//! A pointer to an array of \c double values, to which the data will be 
//! copied
//! 
//! \param SizeInBytes
//! The size of available memory at the location pointed to by 
//! \c OutputVector. To assure safety and to prevent from prohibited writing 
//! into protected memory areas, the user has to specify the amount
//! of available memory in bytes. For a correct operation, the value of
//! \c SizeInBytes has to equal the number of vector elements multiplied
//! by the size of a \c double value.
//! 
//! \sa GetNewAccelerationVector(RMLDoubleVector *OutputVector) const
//! \sa GetNewAccelerationVectorElement(double *OutputValue, const unsigned int &Index) const
//  ----------------------------------------------------------
    inline void GetNewAccelerationVector(		double				*OutputVector
                                            ,	const unsigned int	&SizeInBytes) const
    {
        memcpy(		(void*)OutputVector
                ,	(void*)this->NewAccelerationVector->VecData
                ,	SizeInBytes	);
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetNewAccelerationVectorElement(double *OutputValue, const unsigned int &Index) const
//! 
//! \brief
//! Copies one element of the new selection vector 
//! \f$ \vec{A}_{i+1} \f$ to the memory pointed to by \c OutputValue
//! 
//! \param OutputValue
//! A pointer to one \c double value, to which the desired vector element
//! will be copied
//! 
//! \param Index
//! Specifies the desired element of the vector. The element numbering 
//! starts with \em 0 (zero). If this value is greater the number 
//! of vector elements, a value of \em 0.0 will be written to the memory
//! pointed to by \c OutputValue.
//! 
//! \sa GetNewAccelerationVector(RMLDoubleVector *OutputVector) const
//! \sa GetNewAccelerationVectorElement(double *OutputValue, const unsigned int &Index) const
//! \sa GetNewAccelerationVectorElement(const unsigned int &Index) const
//  ----------------------------------------------------------
    inline void GetNewAccelerationVectorElement(	double				*OutputValue
                                                ,	const unsigned int	&Index) const
    {
        if ( ( Index + 1 ) > ((unsigned int) this->NewAccelerationVector->GetVecDim() ) )
        {
            *OutputValue	=	0.0;
        }
        else
        {	
            *OutputValue	=	(*this->NewAccelerationVector)[Index];
        }
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline double GetNewAccelerationVectorElement(const unsigned int &Index) const
//! 
//! \brief
//! Returns one single element of the new selection vector 
//! \f$ \vec{A}_{i+1} \f$
//! 
//! \param Index
//! Specifies the desired element of the vector. The index of the first 
//! vector element is \em 0 (zero). If the value of \c Index value is 
//! greater the number of vector elements, a value of \em 0.0 will be
//! written to the memory pointed to by \c OutputValue.
//! 
//! \sa GetNewAccelerationVectorElement(double *OutputValue, const unsigned int &Index) const
//  ----------------------------------------------------------
    inline double GetNewAccelerationVectorElement(const unsigned int &Index) const
    {
        if ( ( Index + 1 ) > ((unsigned int) this->NewAccelerationVector->GetVecDim() ) )
        {
            return(0.0);
        }
        else
        {		
            return( (*this->NewAccelerationVector)[Index] );
        }		
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetPositionalExtrema(RMLDoubleVector *MinimumPositionVector, RMLDoubleVector *MaximumPositionVector) const
//! 
//! \brief
//! Copies two \c RMLDoubleVector objects that contain the minimum and 
//! maximum positions, which are reached until the target state of motion
//! is reached.
//!
//! \param MinimumPositionVector
//! A pointer to an \c RMLDoubleVector object, to which the vector of
//! minimum positions will be copied to.
//!
//! \param MaximumPositionVector
//! A pointer to an \c RMLDoubleVector object, to which the vector of
//! maximum positions will be copied to.
//! 
//! \sa GetPositionalExtrema(double *MinimumPositionVector, double *MaximumPositionVector, const unsigned int &SizeInBytes) const
//! \sa RMLVector
//! \sa RMLDoubleVector
//  ----------------------------------------------------------	
    inline void GetPositionalExtrema(		RMLDoubleVector		*MinimumPositionVector
                                        ,	RMLDoubleVector		*MaximumPositionVector) const
    {
        *MinimumPositionVector	=	*(this->MinPosExtremaPositionVectorOnly);
        *MaximumPositionVector	=	*(this->MaxPosExtremaPositionVectorOnly);
    }
            
            
//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetPositionalExtrema(double *MinimumPositionVector, double *MaximumPositionVector, const unsigned int &SizeInBytes) const
//! 
//! \brief
//! Copies two \c RMLDoubleVector objects that contain the minimum and 
//! maximum positions, which are reached until the target state of motion
//! is reached.
//!
//! \param MinimumPositionVector
//! A pointer to a \c double array, to which the elements of the vector of
//! minimum positions will be copied to.
//!
//! \param MaximumPositionVector
//! A pointer to a \c double array, to which the elements of the vector of
//! maximum positions will be copied to.
//! 
//! \param SizeInBytes
//! The size of available memory at the location pointed to by 
//! \c MinimumPositionVector or MaximumPositionVector, respectively.
//! To assure safety and to prevent from prohibited writing 
//! into protected memory areas, the user has to specify the amount
//! of available memory in bytes. For a correct operation, the value of
//! \c SizeInBytes has to equal the number of vector elements multiplied
//! by the size of a \c double value.
//! 
//! \sa GetPositionalExtrema(RMLDoubleVector *MinimumPositionVector, RMLDoubleVector *MaximumPositionVector) const
//  ----------------------------------------------------------	
    inline void GetPositionalExtrema(		double				*MinimumPositionVector
                                        ,	double				*MaximumPositionVector
                                        ,	const unsigned int	&SizeInBytes)  const
    {
        memcpy(		(void*)MinimumPositionVector
                ,	(void*)(this->MinPosExtremaPositionVectorOnly->VecData)
                ,	SizeInBytes														);
                
        memcpy(		(void*)MaximumPositionVector
                ,	(void*)(this->MaxPosExtremaPositionVectorOnly->VecData)
                ,	SizeInBytes														);	
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetTimesAtMinPosition(RMLDoubleVector *ExtremaTimes) const
//! 
//! \brief
//! Copies the contents of a \c RMLDoubleVector object that contains the
//! times (in seconds) at which the minimum positions are reached to the
//! \c RMLDoubleVector object referred to by \c ExtremaTimes
//! 
//! \param ExtremaTimes
//! A pointer to an \c RMLDoubleVector object, to which the data will be 
//! copied
//!
//! \sa GetTimesAtMaxPosition(RMLDoubleVector *ExtremaTimes) const
//! \sa GetPositionalExtrema(RMLDoubleVector *MinimumPositionVector, RMLDoubleVector *MaximumPositionVector) const
//! \sa GetPositionalExtrema(double *MinimumPositionVector, double *MaximumPositionVector, const unsigned int &SizeInBytes) const
//! \sa RMLDoubleVector
//  ----------------------------------------------------------
    inline void GetTimesAtMinPosition(RMLDoubleVector *ExtremaTimes) const
    {
        *ExtremaTimes	=	*(this->MinExtremaTimesVector);
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetTimesAtMaxPosition(RMLDoubleVector *ExtremaTimes) const
//! 
//! \brief
//! Copies the contents of a \c RMLDoubleVector object that contains the
//! times (in seconds) at which the maximum positions are reached to the
//! \c RMLDoubleVector object referred to by \c ExtremaTimes
//! 
//! \param ExtremaTimes
//! A pointer to an \c RMLDoubleVector object, to which the data will be 
//! copied
//!
//! \sa GetTimesAtMinPosition(RMLDoubleVector *ExtremaTimes) const
//! \sa GetPositionalExtrema(RMLDoubleVector *MinimumPositionVector, RMLDoubleVector *MaximumPositionVector) const
//! \sa GetPositionalExtrema(double *MinimumPositionVector, double *MaximumPositionVector, const unsigned int &SizeInBytes) const
//! \sa RMLDoubleVector
//  ----------------------------------------------------------
    inline void GetTimesAtMaxPosition(RMLDoubleVector *ExtremaTimes) const
    {
        *ExtremaTimes	=	*(this->MaxExtremaTimesVector);
    }	


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetTimesAtMinPosition(double*ExtremaTimes, const unsigned int &SizeInBytes) const
//! 
//! \brief
//! Copies the array of \c double values that contain the time values
//! (in seconds) at which the minimum positions are reached to the array
//! of \c double values referred to by \c ExtremaTimes
//! 
//! \param ExtremaTimes
//! A pointer to an array of \c double values, to which the data will be 
//! copied
//! 
//! \param SizeInBytes
//! The size of available memory at the location pointed to by 
//! \c ExtremaTimes. To assure safety and to prevent from prohibited writing 
//! into protected memory areas, the user has to specify the amount
//! of available memory in bytes. For a correct operation, the value of
//! \c SizeInBytes has to equal the number of vector elements multiplied
//! by the size of a \c double value.
//! 
//! \sa GetTimesAtMinPosition(RMLDoubleVector *ExtremaTimes) const
//! \sa GetTimesAtMaxPosition(double*ExtremaTimes, const unsigned int &SizeInBytes) const
//  ----------------------------------------------------------
    inline void GetTimesAtMinPosition(		double				*ExtremaTimes
                                        ,	const unsigned int	&SizeInBytes) const
    {
        memcpy(		(void*)ExtremaTimes
                ,	(void*)(this->MinExtremaTimesVector->VecData)
                ,	SizeInBytes											);
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetTimesAtMaxPosition(double*ExtremaTimes, const unsigned int &SizeInBytes) const
//! 
//! \brief
//! Copies the array of \c double values that contain the time values
//! (in seconds) at which the maximum positions are reached to the array
//! of \c double values referred to by \c ExtremaTimes
//! 
//! \param ExtremaTimes
//! A pointer to an array of \c double values, to which the data will be 
//! copied
//! 
//! \param SizeInBytes
//! The size of available memory at the location pointed to by 
//! \c ExtremaTimes. To assure safety and to prevent from prohibited writing 
//! into protected memory areas, the user has to specify the amount
//! of available memory in bytes. For a correct operation, the value of
//! \c SizeInBytes has to equal the number of vector elements multiplied
//! by the size of a \c double value.
//! 
//! \sa GetTimesAtMaxPosition(RMLDoubleVector *ExtremaTimes) const
//! \sa GetTimesAtMinPosition(double*ExtremaTimes, const unsigned int &SizeInBytes) const
//  ----------------------------------------------------------	
    inline void GetTimesAtMaxPosition(		double				*ExtremaTimes
                                        ,	const unsigned int	&SizeInBytes) const
    {
        memcpy(		(void*)ExtremaTimes
                ,	(void*)(this->MaxExtremaTimesVector->VecData)
                ,	SizeInBytes											);
    }	


//  ---------------------- Doxygen info ----------------------
//! \fn inline int GetMotionStateAtMinPosForOneDOF(const unsigned int &DOF, RMLDoubleVector *PositionVector, RMLDoubleVector *VelocityVector, RMLDoubleVector *AccelerationVector) const
//! 
//! \brief
//! Copies the motion state, at which the position of the degree of freedom
//! with the index \c DOF has reached its minimum, to the three referred
//! \c RMLDoubleVector objects.
//! 
//! \param DOF
//! The index of the degree of freedom, whose minimum position is regarded.
//! The motion state of the time instant, at which the minimum position
//! value of this degree of freedom is reached, will be copied to
//! \c PositionVector, \c VelocityVector, and \c AccelerationVector.
//! 
//! \param PositionVector
//! A pointer to an \c RMLDoubleVector object, to which the position
//! vector at the instant, at which the minimum position for the degree of
//! freedom \c DOF is reached, will be copied.
//! 
//! \param VelocityVector
//! A pointer to an \c RMLDoubleVector object, to which the velocity
//! vector at the instant, at which the minimum position for the degree of
//! freedom \c DOF is reached, will be copied.
//! 
//! \param AccelerationVector
//! A pointer to an \c RMLDoubleVector object, to which the acceleration
//! vector at the instant, at which the minimum position for the degree of
//! freedom \c DOF is reached, will be copied.
//!
//! \return
//!  - RMLOutputParameters::RETURN_ERROR, if the value of \c DOF is greater 
//!    than or equal to the number of degrees of freedom
//!  - RMLOutputParameters::RETURN_SUCCESS, else
//!
//! \sa GetMotionStateAtMaxPosForOneDOF(const unsigned int &DOF, RMLDoubleVector *PositionVector, RMLDoubleVector *VelocityVector, RMLDoubleVector *AccelerationVector) const
//! \sa GetMotionStateAtMinPosForOneDOF(const unsigned int &DOF, double *PositionVector, double *VelocityVector, double *AccelerationVector, const unsigned int &SizeInBytes) const
//! \sa GetTimesAtMinPosition(RMLDoubleVector *ExtremaTimes) const
//! \sa GetPositionalExtrema(RMLDoubleVector *MinimumPositionVector, RMLDoubleVector *MaximumPositionVector) const
//! \sa RMLDoubleVector
//  ----------------------------------------------------------
    inline int GetMotionStateAtMinPosForOneDOF(		const unsigned int	&DOF
                                                ,	RMLDoubleVector		*PositionVector
                                                ,	RMLDoubleVector		*VelocityVector
                                                ,	RMLDoubleVector		*AccelerationVector) const
    {
        if (DOF >= this->NumberOfDOFs)
        {
            return(RMLOutputParameters::RETURN_ERROR);		
        }
    
        *PositionVector		=	*((this->MinPosExtremaPositionVectorArray		)[DOF])	;
        *VelocityVector		=	*((this->MinPosExtremaVelocityVectorArray		)[DOF])	;
        *AccelerationVector	=	*((this->MinPosExtremaAccelerationVectorArray	)[DOF])	;
        
        return(RMLOutputParameters::RETURN_SUCCESS);
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline int GetMotionStateAtMinPosForOneDOF(const unsigned int &DOF, double *PositionVector, double *VelocityVector, double *AccelerationVector, const unsigned int &SizeInBytes) const
//! 
//! \brief
//! Copies the motion state, at which the position of the degree of freedom
//! with the index \c DOF has reached its minimum, to the three referred
//! arrays of \c double values.
//! 
//! \param DOF
//! The index of the degree of freedom, whose minimum position is regarded.
//! The motion state of the time instant, at which the minimum position
//! value of this degree of freedom is reached, will be copied to
//! \c PositionVector, \c VelocityVector, and \c AccelerationVector.
//! 
//! \param PositionVector
//! A pointer to an array of \c double values, to which the position
//! vector at the instant, at which the minimum position for the degree of
//! freedom \c DOF is reached, will be copied.
//! 
//! \param VelocityVector
//! A pointer to an array of \c double values, to which the velocity
//! vector at the instant, at which the minimum position for the degree of
//! freedom \c DOF is reached, will be copied.
//! 
//! \param AccelerationVector
//! A pointer to an array of \c double values, to which the acceleration
//! vector at the instant, at which the minimum position for the degree of
//! freedom \c DOF is reached, will be copied.
//!
//! \param SizeInBytes
//! The size of available memory at the each of the locations pointed to by 
//! \c PositionVector, \c VelocityVector, and \c AccelerationVector.
//! To assure safety and to prevent from prohibited writing 
//! into protected memory areas, the user has to specify the amount
//! of available memory in bytes. For a correct operation, the value of
//! \c SizeInBytes has to equal the number of vector elements multiplied
//! by the size of a \c double value.
//!
//! \return
//!  - RMLOutputParameters::RETURN_ERROR, if the value of \c DOF is greater 
//!    than or equal to the number of degrees of freedom
//!  - RMLOutputParameters::RETURN_SUCCESS, else
//!
//! \sa GetMotionStateAtMinPosForOneDOF(const unsigned int &DOF, RMLDoubleVector *PositionVector, RMLDoubleVector *VelocityVector, RMLDoubleVector *AccelerationVector) const
//! \sa GetMotionStateAtMaxPosForOneDOF(const unsigned int &DOF, double *PositionVector, double *VelocityVector, double *AccelerationVector, const unsigned int &SizeInBytes) const
//! \sa GetTimesAtMinPosition(RMLDoubleVector *ExtremaTimes) const
//! \sa GetPositionalExtrema(RMLDoubleVector *MinimumPositionVector, RMLDoubleVector *MaximumPositionVector) const
//! \sa RMLDoubleVector
//  ----------------------------------------------------------
    inline int GetMotionStateAtMinPosForOneDOF(		const unsigned int	&DOF
                                                ,	double				*PositionVector
                                                ,	double				*VelocityVector
                                                ,	double				*AccelerationVector
                                                ,	const unsigned int	&SizeInBytes) const
    {
        if (DOF >= this->NumberOfDOFs)
        {
            return(RMLOutputParameters::RETURN_ERROR);		
        }
    
        memcpy(		(void*)PositionVector
                ,	(void*)(((this->MinPosExtremaPositionVectorArray)[DOF])->VecData)
                ,	SizeInBytes																);		
                
        memcpy(		(void*)VelocityVector
                ,	(void*)(((this->MinPosExtremaVelocityVectorArray)[DOF])->VecData)
                ,	SizeInBytes																);	
            
        memcpy(		(void*)AccelerationVector
                ,	(void*)(((this->MinPosExtremaAccelerationVectorArray)[DOF])->VecData)
                ,	SizeInBytes																	);
                
        return(RMLOutputParameters::RETURN_SUCCESS);
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline int GetMotionStateAtMaxPosForOneDOF(const unsigned int &DOF, RMLDoubleVector *PositionVector, RMLDoubleVector *VelocityVector, RMLDoubleVector *AccelerationVector) const
//! 
//! \brief
//! Copies the motion state, at which the position of the degree of freedom
//! with the index \c DOF has reached its maximum, to the three referred
//! \c RMLDoubleVector objects.
//! 
//! \param DOF
//! The index of the degree of freedom, whose maximum position is regarded.
//! The motion state of the time instant, at which the maximum position
//! value of this degree of freedom is reached, will be copied to
//! \c PositionVector, \c VelocityVector, and \c AccelerationVector.
//! 
//! \param PositionVector
//! A pointer to an \c RMLDoubleVector object, to which the position
//! vector at the instant, at which the maximum position for the degree of
//! freedom \c DOF is reached, will be copied.
//! 
//! \param VelocityVector
//! A pointer to an \c RMLDoubleVector object, to which the velocity
//! vector at the instant, at which the maximum position for the degree of
//! freedom \c DOF is reached, will be copied.
//! 
//! \param AccelerationVector
//! A pointer to an \c RMLDoubleVector object, to which the acceleration
//! vector at the instant, at which the maximum position for the degree of
//! freedom \c DOF is reached, will be copied.
//!
//! \return
//!  - RMLOutputParameters::RETURN_ERROR, if the value of \c DOF is greater 
//!    than or equal to the number of degrees of freedom
//!  - RMLOutputParameters::RETURN_SUCCESS, else
//!
//! \sa GetMotionStateAtMinPosForOneDOF(const unsigned int &DOF, RMLDoubleVector *PositionVector, RMLDoubleVector *VelocityVector, RMLDoubleVector *AccelerationVector) const
//! \sa GetMotionStateAtMaxPosForOneDOF(const unsigned int &DOF, double *PositionVector, double *VelocityVector, double *AccelerationVector, const unsigned int &SizeInBytes) const
//! \sa GetTimesAtMaxPosition(RMLDoubleVector *ExtremaTimes) const
//! \sa GetPositionalExtrema(RMLDoubleVector *MinimumPositionVector, RMLDoubleVector *MaximumPositionVector) const
//! \sa RMLDoubleVector
//  ----------------------------------------------------------									
    inline int GetMotionStateAtMaxPosForOneDOF(		const unsigned int	&DOF
                                                ,	RMLDoubleVector		*PositionVector
                                                ,	RMLDoubleVector		*VelocityVector
                                                ,	RMLDoubleVector		*AccelerationVector) const
    {
        if (DOF >= this->NumberOfDOFs)
        {
            return(RMLOutputParameters::RETURN_ERROR);		
        }

        *PositionVector		=	*((this->MaxPosExtremaPositionVectorArray		)[DOF])	;
        *VelocityVector		=	*((this->MaxPosExtremaVelocityVectorArray		)[DOF])	;
        *AccelerationVector	=	*((this->MaxPosExtremaAccelerationVectorArray	)[DOF])	;

        return(RMLOutputParameters::RETURN_SUCCESS);												
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline int GetMotionStateAtMaxPosForOneDOF(const unsigned int &DOF, double *PositionVector, double *VelocityVector, double *AccelerationVector, const unsigned int &SizeInBytes) const
//! 
//! \brief
//! Copies the motion state, at which the position of the degree of freedom
//! with the index \c DOF has reached its maximum, to the three referred
//! arrays of \c double values.
//! 
//! \param DOF
//! The index of the degree of freedom, whose maximum position is regarded.
//! The motion state of the time instant, at which the maximum position
//! value of this degree of freedom is reached, will be copied to
//! \c PositionVector, \c VelocityVector, and \c AccelerationVector.
//! 
//! \param PositionVector
//! A pointer to an array of \c double values, to which the position
//! vector at the instant, at which the maximum position for the degree of
//! freedom \c DOF is reached, will be copied.
//! 
//! \param VelocityVector
//! A pointer to an array of \c double values, to which the velocity
//! vector at the instant, at which the maximum position for the degree of
//! freedom \c DOF is reached, will be copied.
//! 
//! \param AccelerationVector
//! A pointer to an array of \c double values, to which the acceleration
//! vector at the instant, at which the maximum position for the degree of
//! freedom \c DOF is reached, will be copied.
//!
//! \param SizeInBytes
//! The size of available memory at the each of the locations pointed to by 
//! \c PositionVector, \c VelocityVector, and \c AccelerationVector.
//! To assure safety and to prevent from prohibited writing 
//! into protected memory areas, the user has to specify the amount
//! of available memory in bytes. For a correct operation, the value of
//! \c SizeInBytes has to equal the number of vector elements multiplied
//! by the size of a \c double value.
//!
//! \return
//!  - RMLOutputParameters::RETURN_ERROR, if the value of \c DOF is greater 
//!    than or equal to the number of degrees of freedom
//!  - RMLOutputParameters::RETURN_SUCCESS, else
//!
//! \sa GetMotionStateAtMaxPosForOneDOF(const unsigned int &DOF, RMLDoubleVector *PositionVector, RMLDoubleVector *VelocityVector, RMLDoubleVector *AccelerationVector) const
//! \sa GetMotionStateAtMinPosForOneDOF(const unsigned int &DOF, double *PositionVector, double *VelocityVector, double *AccelerationVector, const unsigned int &SizeInBytes) const
//! \sa GetTimesAtMaxPosition(RMLDoubleVector *ExtremaTimes) const
//! \sa GetPositionalExtrema(RMLDoubleVector *MinimumPositionVector, RMLDoubleVector *MaximumPositionVector) const
//! \sa RMLDoubleVector
//  ----------------------------------------------------------
    inline int GetMotionStateAtMaxPosForOneDOF(		const unsigned int	&DOF
                                                ,	double				*PositionVector
                                                ,	double				*VelocityVector
                                                ,	double				*AccelerationVector
                                                ,	const unsigned int	&SizeInBytes) const
    {
        if (DOF >= this->NumberOfDOFs)
        {
            return(RMLOutputParameters::RETURN_ERROR);		
        }

        memcpy(		(void*)PositionVector
                ,	(void*)(((this->MaxPosExtremaPositionVectorArray)[DOF])->VecData)
                ,	SizeInBytes																);		

        memcpy(		(void*)VelocityVector
                ,	(void*)(((this->MaxPosExtremaVelocityVectorArray)[DOF])->VecData)
                ,	SizeInBytes																);	

        memcpy(		(void*)AccelerationVector
                ,	(void*)(((this->MaxPosExtremaAccelerationVectorArray)[DOF])->VecData)
                ,	SizeInBytes																	);

        return(RMLOutputParameters::RETURN_SUCCESS);	
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline unsigned int GetNumberOfDOFs(void) const
//! 
//! \brief
//! Returns the number of degrees of freedom
//! 
//! \return
//! The number of degrees of freedom.
//  ----------------------------------------------------------									
    inline unsigned int GetNumberOfDOFs(void) const
    {
        return(this->NumberOfDOFs);
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline bool WasACompleteComputationPerformedDuringTheLastCycle(void) const
//! 
//! \brief
//! Indicates, whether a new computation was performed in the last cycle
//! 
//! \details
//! \copydetails RMLOutputParameters::ANewCalculationWasPerformed
//! 
//! \returns
//! - \c true if a new computation was performed
//! - \c false if the previously calculated trajectory parameters did not
//! change and were used.
//  ----------------------------------------------------------
    inline bool WasACompleteComputationPerformedDuringTheLastCycle(void) const
    {
        return(this->ANewCalculationWasPerformed);
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline bool IsTrajectoryPhaseSynchronized(void) const
//! 
//! \brief
//! Indicates whether the currently calculated trajectory is phase-
//! synchronized or only time-synchronized
//! 
//! \return 
//! The method returns \c true if the trajectory is phase-synchronized
//! and \c false if it is time-synchronized.
//  ----------------------------------------------------------
    inline bool IsTrajectoryPhaseSynchronized(void) const
    {
        return(this->TrajectoryIsPhaseSynchronized);
    }
    

//  ---------------------- Doxygen info ----------------------
//! \fn inline double GetSynchronizationTime(void) const
//! 
//! \brief
//! Returns the synchronization time
//! 
//! \details
//! The position-based Online Trajectory Generation algorithm transfers
//! all selected degrees of freedom into their desired target state of
//! motion, such that all of them reach the target state of motion 
//! \f$_{k}\vec{M}_{i}^{\,trgt} \f$ at the very same time instant, that is,
//! at the minimum possible synchronization time \f$ t_{i}^{,sync} \f$.
//! If this value is used as an output value of the velocity-based 
//! Online Trajectory Generation algorithm, this time determines when
//! all selected degree of freedom will reach the desired target velocity.
//! 
//! \return
//! The value of the synchronization time in seconds
//!
//! \sa RMLOutputParameters::SynchronizationTime
//  ----------------------------------------------------------									
    inline double GetSynchronizationTime(void) const
    {
        return(this->SynchronizationTime);
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline double GetCurrentOverrideValue(void) const
//! 
//! \brief
//! Returns the currently applied override value
//! 
//! \details
//! As the override value may not change abruptly, a filter is used to
//! soothly progress towards the desired override value
//! (cf. RMLInputParameters::OverrideValue). This output value
//! contains the currently applied override value.
//! 
//! \return
//! The currently applied override value
//!
//! \sa RMLOutputParameters::CurrentOverrideValue
//! \sa RMLInputParameters::OverrideValue
//! \sa \ref page_Override
//  ----------------------------------------------------------									
    inline double GetCurrentOverrideValue(void) const
    {
        return(this->CurrentOverrideValue);
    }
    
//  ---------------------- Doxygen info ----------------------
//! \fn inline bool IsTheOverrideFilterActive(void) const
//! 
//! \brief
//! Returns true if the desired override value is not yet applied, because
//! the filter is still active
//! 
//! \details
//! As the override value may not change abruptly, a filter is used to
//! soothly progress towards the desired override value
//! (cf. RMLInputParameters::OverrideValue). This output value
//! is true if the desired override values is not yet applied and the 
//! override filter is still active.
//! 
//! \return
//! - \c true if the override filter is active and the desired override value is not yet applied
//! - \c false if the override filter is not active anymore and the desired override value is already applied
//!
//! \sa RMLOutputParameters::OverrideFilterIsActive
//! \sa RMLInputParameters::OverrideValue
//! \sa \ref page_Override
//  ----------------------------------------------------------									
    inline bool IsTheOverrideFilterActive(void) const
    {
        return(this->OverrideFilterIsActive);
    }    
    

//  ---------------------- Doxygen info ----------------------
//! \fn inline unsigned int GetDOFWithTheGreatestExecutionTime(void) const
//! 
//! \brief
//! Returns the index of the degree of freedom with the greatest trajectory
//! execution time
//! 
//! \sa RMLOutputParameters::GetGreatestExecutionTime()
//  ----------------------------------------------------------
    inline unsigned int GetDOFWithTheGreatestExecutionTime(void) const
    {
        return(this->DOFWithTheGreatestExecutionTime);
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetExecutionTimes(RMLDoubleVector *OutputVector) const
//! 
//! \brief
//! Copies the contents of the \c RMLDoubleVector object containing the
//! execution times for each degree of freedom, at which the 
//! desired target velocity \f$\ _{k}V_{i}^{\,trgt} \f$ is reached, to the
//! \c RMLDoubleVector object referred to by \c OutputVector
//! 
//! \param OutputVector
//! A pointer to an \c RMLDoubleVector object, to which the data will be 
//! copied
//! 
//! \sa GetExecutionTimes(double *OutputVector, const unsigned int &SizeInBytes) const
//! \sa GetExecutionTimesElement(double *OutputValue, const unsigned int &Index) const
//! \sa RMLOutputParameters::ExecutionTimes
//  ----------------------------------------------------------
    inline void GetExecutionTimes(RMLDoubleVector *OutputVector) const
    {
        *OutputVector	=	*(this->ExecutionTimes);
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetExecutionTimes(double *OutputVector, const unsigned int &SizeInBytes) const
//! 
//! \brief
//! Copies the array of \c double values representing the execution times
//! for each degree of freedom, at which the desired target velocity
//! \f$\ _{k}V_{i}^{\,trgt} \f$ is reached, to the memory pointed to by
//! \c OutputVector
//! 
//! \param OutputVector
//! A pointer to an array of \c double values, to which the data will be 
//! copied
//! 
//! \param SizeInBytes
//! The size of available memory at the location pointed to by 
//! \c OutputVector. To assure safety and to prevent from prohibited writing 
//! into protected memory areas, the user has to specify the amount
//! of available memory in bytes. For a correct operation, the value of
//! \c SizeInBytes has to equal the number of vector elements multiplied
//! by the size of a \c double value.
//! 
//! \sa GetExecutionTimes(RMLDoubleVector *OutputVector) const
//! \sa GetExecutionTimesElement(double *OutputValue, const unsigned int &Index) const
//! \sa RMLOutputParameters::ExecutionTimes
//  ----------------------------------------------------------
    inline void GetExecutionTimes(		double				*OutputVector
                                    ,	const unsigned int	&SizeInBytes) const
    {
        memcpy(		(void*)OutputVector
                ,	(void*)this->ExecutionTimes->VecData
                ,	SizeInBytes	);
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetExecutionTimesElement(double *OutputValue, const unsigned int &Index) const
//! 
//! \brief
//! Copies one element of the execution times for each degree of freedom,
//! at which the desired target velocity \f$\ _{k}V_{i}^{\,trgt} \f$
//! is reached, to the memory pointed to by \c OutputValue
//! 
//! \param OutputValue
//! A pointer to one \c double value, to which the desired vector element
//! will be copied
//! 
//! \param Index
//! Specifies the desired element of the vector. The element numbering 
//! starts with \em 0 (zero). If this value is greater the number 
//! of vector elements, a value of \em 0.0 will be written to the memory
//! pointed to by \c OutputValue.
//! 
//! \sa GetExecutionTimes(RMLDoubleVector *OutputVector) const
//! \sa GetExecutionTimesElement(double *OutputValue, const unsigned int &Index) const
//! \sa GetExecutionTimesElement(const unsigned int &Index) const
//! \sa RMLOutputParameters::ExecutionTimes
//  ----------------------------------------------------------
    inline void GetExecutionTimesElement(		double				*OutputValue
                                            ,	const unsigned int	&Index) const
    {
        if ( ( Index + 1 ) > ((unsigned int) this->ExecutionTimes->GetVecDim() ) )
        {
            *OutputValue	=	0.0;
        }
        else
        {	
            *OutputValue	=	(*this->ExecutionTimes)[Index];
        }
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline double GetExecutionTimesElement(const unsigned int &Index) const
//! 
//! \brief
//! Returns one single element of the execution times
//! for each degree of freedom, at which the desired target velocity
//! \f$\ _{k}V_{i}^{\,trgt} \f$ is reached
//! 
//! \param Index
//! Specifies the desired element of the vector. The index of the first 
//! vector element is \em 0 (zero). If the value of \c Index value is 
//! greater the number of vector elements, a value of \em 0.0 will be
//! written to the memory pointed to by \c OutputValue.
//! 
//! \sa GetExecutionTimesElement(double *OutputValue, const unsigned int &Index) const
//! \sa RMLOutputParameters::ExecutionTimes
//  ----------------------------------------------------------
    inline double GetExecutionTimesElement(const unsigned int &Index) const
    {
        if ( ( Index + 1 ) > ((unsigned int) this->ExecutionTimes->GetVecDim() ) )
        {
            return(0.0);
        }
        else
        {		
            return( (*this->ExecutionTimes)[Index] );
        }		
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline double GetGreatestExecutionTime(void) const
//! 
//! \brief
//! Returns the time value in seconds which is required by the degree
//! with the greatest execution to reach its desired target velocity
//! 
//! \sa GetGreatestExecutionTime (void) const 
//  ----------------------------------------------------------	
    inline double GetGreatestExecutionTime(void) const
    {
        return((this->ExecutionTimes->VecData)[this->DOFWithTheGreatestExecutionTime]);
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline char* GetErrorString(void) const
//! 
//! \brief
//! Creates a C-string containing an error message for the last call of
//! ReflexxesAPI::RMLPosition(), ReflexxesAPI::RMLVelocity(),
//! ReflexxesAPI::RMLPositionAtAGivenSampleTime(), or 
//! ReflexxesAPI::RMLVelocityAtAGivenSampleTime()
//! 
//! \details
//! A string is created that contains a description of the return value
//! of 
//!
//! - ReflexxesAPI::RMLPosition(),
//! - ReflexxesAPI::RMLPositionAtAGivenSampleTime(),
//! - ReflexxesAPI::RMLVelocity(), or
//! - ReflexxesAPI::RMLVelocityAtAGivenSampleTime().
//! 
//! Possible return values are are:
//!  - ReflexxesAPI::RML_WORKING: \copydoc ReflexxesAPI::RML_WORKING
//!  - ReflexxesAPI::RML_FINAL_STATE_REACHED: \copydoc ReflexxesAPI::RML_FINAL_STATE_REACHED
//!  - ReflexxesAPI::RML_ERROR_INVALID_INPUT_VALUES: \copydoc ReflexxesAPI::RML_ERROR_INVALID_INPUT_VALUES
//!  - ReflexxesAPI::RML_ERROR_EXECUTION_TIME_CALCULATION: \copydoc ReflexxesAPI::RML_ERROR_EXECUTION_TIME_CALCULATION
//!  - ReflexxesAPI::RML_ERROR_SYNCHRONIZATION: \copydoc ReflexxesAPI::RML_ERROR_SYNCHRONIZATION
//!  - ReflexxesAPI::RML_ERROR_NUMBER_OF_DOFS: \copydoc ReflexxesAPI::RML_ERROR_NUMBER_OF_DOFS
//!  - ReflexxesAPI::RML_ERROR_NO_PHASE_SYNCHRONIZATION: \copydoc ReflexxesAPI::RML_ERROR_NO_PHASE_SYNCHRONIZATION
//!  - ReflexxesAPI::RML_ERROR_NULL_POINTER: \copydoc ReflexxesAPI::RML_ERROR_NULL_POINTER
//!  - ReflexxesAPI::RML_ERROR_EXECUTION_TIME_TOO_BIG: \copydoc ReflexxesAPI::RML_ERROR_EXECUTION_TIME_TOO_BIG
//!  - ReflexxesAPI::RML_ERROR_USER_TIME_OUT_OF_RANGE: \copydoc ReflexxesAPI::RML_ERROR_USER_TIME_OUT_OF_RANGE
//!  - ReflexxesAPI::RML_ERROR_POSITIONAL_LIMITS: \copydoc ReflexxesAPI::RML_ERROR_POSITIONAL_LIMITS
//!  - ReflexxesAPI::RML_ERROR_OVERRIDE_OUT_OF_RANGE: \copydoc ReflexxesAPI::RML_ERROR_OVERRIDE_OUT_OF_RANGE
//!
//!\n\n
//!
//! \return
//! A pointer to a character array containing the error description.
//! 
//! \sa RMLOutputParameters::ResultValue
//! \sa ReflexxesAPI::RMLResultValue
//  ----------------------------------------------------------	
    inline char* GetErrorString(void) const
    {
		if (this == NULL)	// this->ResultValue == -105
		{
			return((char*)"RML_ERROR_NULL_POINTER: One of the pointers to the input objects is NULL.");
		}
		
		switch (this->ResultValue)
		{
			case 0:
				return((char*)"RML_WORKING: The Online Trajectory Generation algorithm is working; the final state of motion has not been reached yet.");
				break;
			case 1:
				return((char*)"RML_FINAL_STATE_REACHED: The desired final state of motion has been reached.");
				break;
			case 2:
				return((char*)"RML_NO_ERROR: No error.");
				break;
			case -100:
				return((char*)"RML_ERROR_INVALID_INPUT_VALUES: The applied input values are invalid.");
				break;
			case -101:
				return((char*)"RML_ERROR_EXECUTION_TIME_CALCULATION: An error occurred during the calculation of the synchronization time.");
				break;
			case -102:
				return((char*)"RML_ERROR_SYNCHRONIZATION: An error occurred during the synchronization of the trajectory.");
				break;
			case -103:
				return((char*)"RML_ERROR_NUMBER_OF_DOFS: The number of degree of freedom of th input parameters, the output parameters, and the Online Trajectory Generation algorithm do not match.");
				break;
			case -104:
				return((char*)"RML_ERROR_NO_PHASE_SYNCHRONIZATION: The input flag RMLFlags::ONLY_PHASE_SYNCHRONIZATION is set, but a phase-synchronized trajectory cannot be executed.");
				break;
			case -106:
				return((char*)"RML_ERROR_EXECUTION_TIME_TOO_BIG: The execution time of the computed trajectory is to big (>1e10 seconds).");
				break;
			case -107:
				return((char*)"RML_ERROR_USER_TIME_OUT_OF_RANGE: The sample time for the previously computed trajectory is out of range.");
				break;
			case -108:
				return((char*)"RML_ERROR_POSITIONAL_LIMITS: The computed trajectory will exceed the positional limits.");
				break;
			case -109:
				return((char*)"RML_ERROR_OVERRIDE_OUT_OF_RANGE: The override value is out of range (0.0-10.0).");
				break;																												
			case -1:
			default:
				return((char*)"RML_ERROR: An unknown error has occurred.");
				break;
		}
	}
	

//  ---------------------- Doxygen info ----------------------
//! \var bool ANewCalculationWasPerformed
//! 
//! \brief
//! Indicates, whether a new computation was performed in the last cycle
//! 
//! \details
//! If the computation of completely new trajectory parameters
//! was performed, this flag will be set to \c true. If the input values 
//! remained constant and the output parameters of the last computation
//! cycle were directly fed back to the input parameters, such that the
//! previously computed trajectory did not change, the flag will be set
//! to \c false.
//! 
//! This attribute can be accessed directly or by using one of the
//! following methods:\n\n
//!  - WasACompleteComputationPerformedDuringTheLastCycle(void) const\n\n
//  ----------------------------------------------------------
    bool					ANewCalculationWasPerformed				;
    
    
//  ---------------------- Doxygen info ----------------------
//! \var bool TrajectoryIsPhaseSynchronized
//! 
//! \brief
//! Boolean flag that indicates whether the current trajectory is
//! phase-synchronized
//! 
//! \details
//! This attribute can be accessed directly or by using one of the method
//! IsTrajectoryPhaseSynchronized().
//  ----------------------------------------------------------	
    bool					TrajectoryIsPhaseSynchronized			;
    
    
//  ---------------------- Doxygen info ----------------------
//! \var bool OverrideFilterIsActive
//! 
//! \brief
//! Boolean flag that indicates whether the desired override value of
//! RMLInputParameters::OverrideValue has been reached
//! 
//! \details
//! This attribute is (\c true) if the value of
//! RMLInputParameters::OverrideValue has been reached, and is (\c false)
//! if the filter is still active.
//!
//! \sa RMLInputParameters::OverrideValue
//! \sa RMLOutputParameters::CurrentOverrideValue
//! \sa \ref page_InputValues
//! \sa \ref page_OutputValues
//! \sa \ref page_Override
//  ----------------------------------------------------------	
    bool					OverrideFilterIsActive					;


//  ---------------------- Doxygen info ----------------------
//! \var unsigned int NumberOfDOFs
//! 
//! \brief
//! The number of degrees of freedom \f$ K \f$
//! 
//! \details
//! This attribute can be accessed directly or by using one of the method
//! RMLOutputParameters::GetNumberOfDOFs().
//  ----------------------------------------------------------
    unsigned int			NumberOfDOFs							;


//  ---------------------- Doxygen info ----------------------
//! \var unsigned int DOFWithTheGreatestExecutionTime
//! 
//! \brief
//! Index of the degree of freedom that requires the greatest execution 
//! time to reach its desired target velocity value
//! 
//! \details
//! <ul>
//!   <li>In case of non-synchronized trajectories, this integer value 
//!       specifies the index of the degree-of-freedom with the greatest
//!       execution time.</li>
//!   <li>In case of time- or phase-synchronized trajectories, this integer
//!       value specifies the degree of freedom that determined the
//!       synchronization time.</li>
//!   <li>If more that one degree of freedom feature the (same) 
//!       execution time, the lowest index will be used.\n\n</li>
//! </ul>
//!
//! This attribute can be accessed directly or by using the method
//! GetDOFWithTheGreatestExecutionTime().
//  ----------------------------------------------------------	
    unsigned int			DOFWithTheGreatestExecutionTime			;


//  ---------------------- Doxygen info ----------------------
//! \var unsigned int ResultValue
//! 
//! \brief
//! The result value of the call of	ReflexxesAPI::RMLPosition() or
//! ReflexxesAPI::RMLVelocity()
//! 
//! \details
//! The return value of the last call of
//!
//! - ReflexxesAPI::RMLPosition(),
//! - ReflexxesAPI::RMLPositionAtAGivenSampleTime(),
//! - ReflexxesAPI::RMLVelocity(), or
//! - ReflexxesAPI::RMLVelocityAtAGivenSampleTime()
//! 
//! is stored in this variable (cf. ReflexxesAPI::RMLResultValue). 
//! The function RMLOutputParameters::GetErrorString() creates
//! an error message based on the return value.
//! 
//! \sa ReflexxesAPI::RMLResultValue
//! \sa RMLOutputParameters::GetErrorString()
//  ----------------------------------------------------------
    int						ResultValue								;    


//  ---------------------- Doxygen info ----------------------
//! \var double SynchronizationTime
//! 
//! \brief
//! The synchronization time \f$ t_{i}^{\,sync} \f$ in seconds
//! 
//! \details
//! If the trajectory is time- or phase-synchronized, this attribute
//! will contain the synchronization time. Otherwise, it is set to zero.
//!
//! This attribute can be accessed directly or by using the method
//! GetSynchronizationTime().
//  ----------------------------------------------------------						
    double					SynchronizationTime						;
   
   
//  ---------------------- Doxygen info ----------------------
//! \var double CurrentOverrideValue
//! 
//! \brief
//! The currently applied override value
//! 
//! \details
//! The applied override value is not allowed to change abruptly, and a
//! filter is applied to guarantee smooth transitions for the applied
//! value. The desired value can be found in 
//! RMLInputParameters::OverrideValue.
//!
//! It can be accessed with the method GetCurrentOverrideValue().
//!
//! \sa RMLInputParameters::OverrideValue
//! \sa RMLOutputParameters::OverrideFilterIsActive
//! \sa RMLOutputParameters::GetCurrentOverrideValue()
//! \sa \ref page_InputValues
//! \sa \ref page_OutputValues
//! \sa \ref page_Override
//  ----------------------------------------------------------						
    double					CurrentOverrideValue					;    


//  ---------------------- Doxygen info ----------------------
//! \var RMLDoubleVector *NewPositionVector
//! 
//! \brief
//! A pointer to the new position vector \f$ \vec{P}_{i+1} \f$
//! 
//! \details
//! This attribute can be accessed directly or by using one of the following methods:\n\n
//!  - GetNewPositionVector(RMLDoubleVector *OutputVector) const\n\n
//!  - GetNewPositionVector(double *OutputVector, const unsigned int &SizeInBytes) const\n\n
//!  - GetNewPositionVectorElement(double *OutputValue, const unsigned int &Index) const\n\n
//!  - GetNewPositionVectorElement(const unsigned int &Index) const\n\n
//  ----------------------------------------------------------
    RMLDoubleVector			*NewPositionVector						;
    
    
//  ---------------------- Doxygen info ----------------------
//! \var RMLDoubleVector *NewVelocityVector
//! 
//! \brief
//! A pointer to the new velocity vector \f$ \vec{V}_{i+1} \f$
//! 
//! \details
//! This attribute can be accessed directly or by using one of the following methods:\n\n
//!  - GetNewVelocityVector(RMLDoubleVector *OutputVector) const\n\n
//!  - GetNewVelocityVector(double *OutputVector, const unsigned int &SizeInBytes) const\n\n
//!  - GetNewVelocityVectorElement(double *OutputValue, const unsigned int &Index) const\n\n
//!  - GetNewVelocityVectorElement(const unsigned int &Index) const\n\n
//  ----------------------------------------------------------	
    RMLDoubleVector			*NewVelocityVector						;
    
    
//  ---------------------- Doxygen info ----------------------
//! \var RMLDoubleVector *NewAccelerationVector
//! 
//! \brief
//! A pointer to the new acceleration vector \f$ \vec{A}_{i+1} \f$
//! 
//! \details
//! This attribute can be accessed directly or by using one of the following methods:\n\n
//!  - GetNewAccelerationVector(RMLDoubleVector *OutputVector) const\n\n 
//!  - GetNewAccelerationVector(double *OutputVector, const unsigned int &SizeInBytes) const\n\n
//!  - GetNewAccelerationVectorElement(double *OutputValue, const unsigned int &Index) const\n\n
//!  - GetNewAccelerationVectorElement(const unsigned int &Index) const\n\n
//  ----------------------------------------------------------	
    RMLDoubleVector			*NewAccelerationVector					;				

    
//  ---------------------- Doxygen info ----------------------
//! \var RMLDoubleVector *MinExtremaTimesVector
//! 
//! \brief
//! A pointer to an \c RMLDoubleVector object that contains
//! the times at which each degree of freedom reaches its minimum
//! position during the execution of the calculated trajectory.
//! 
//! \details
//! This attribute can be accessed directly or by using one of the following methods:\n\n
//!  - GetTimesAtMinPosition(RMLDoubleVector *ExtremaTimes) const\n\n 
//!  - GetTimesAtMinPosition(double *ExtremaTimes, const unsigned int &SizeInBytes) const\n\n
//  ----------------------------------------------------------	
    RMLDoubleVector			*MinExtremaTimesVector					;
    
    
//  ---------------------- Doxygen info ----------------------
//! \var RMLDoubleVector *MaxExtremaTimesVector
//! 
//! \brief
//! A pointer to an \c RMLDoubleVector object that contains
//! the times at which each degree of freedom reaches its maximum
//! position during the execution of the calculated trajectory.
//! 
//! \details
//! This attribute can be accessed directly or by using one of the following methods:\n\n
//!  - GetTimesAtMaxPosition(RMLDoubleVector *ExtremaTimes) const\n\n 
//!  - GetTimesAtMaxPosition(double *ExtremaTimes, const unsigned int &SizeInBytes) const\n\n
//  ----------------------------------------------------------
    RMLDoubleVector			*MaxExtremaTimesVector					;
    
    
//  ---------------------- Doxygen info ----------------------
//! \var RMLDoubleVector *MinPosExtremaPositionVectorOnly
//! 
//! \brief
//! A pointer to an \c RMLDoubleVector object that contains
//! the maximum positions for all degrees of freedom that
//! occur during the execution of the calculated trajectory.
//! 
//! \details
//! This attribute can be accessed directly or by using one of the following methods:\n\n
//!  - GetPositionalExtrema(RMLDoubleVector *MinimumPositionVector, RMLDoubleVector *MaximumPositionVector) const\n\n
//!  - GetPositionalExtrema(double *MinimumPositionVector, double *MaximumPositionVector, const unsigned int &SizeInBytes) const\n\n
//  ----------------------------------------------------------			
    RMLDoubleVector			*MinPosExtremaPositionVectorOnly		;

    
//  ---------------------- Doxygen info ----------------------
//! \var RMLDoubleVector *MaxPosExtremaPositionVectorOnly
//! 
//! \brief
//! A pointer to an \c RMLDoubleVector object that contains
//! the maximum positions for all degrees of freedom that
//! occur during the execution of the calculated trajectory.
//! 
//! \details
//! This attribute can be accessed directly or by using one of the following methods:\n\n
//!  - GetPositionalExtrema(RMLDoubleVector *MinimumPositionVector, RMLDoubleVector *MaximumPositionVector) const\n\n
//!  - GetPositionalExtrema(double *MinimumPositionVector, double *MaximumPositionVector, const unsigned int &SizeInBytes) const\n\n
//  ----------------------------------------------------------
    RMLDoubleVector			*MaxPosExtremaPositionVectorOnly		;
    
    
//  ---------------------- Doxygen info ----------------------
//! \var RMLDoubleVector *ExecutionTimes
//! 
//! \brief
//! A pointer to an \c RMLDoubleVector object that contains
//! the execution times of all selected degrees of freedom in the case
//! non-synchronized trajectories
//!
//! \details
//! <ul>
//! <li>In case of non-synchronized trajectories, this vector contains
//!     the execution times of all selected degrees of freedom.</li>
//! <li>In the case of time- and phase-synchronized trajectories, this
//!     vector contains the synchronization time for all selected 
//!     degree of freedom.</li>
//! <li>The values non-selected degrees of freedom is zero.\n\n</li>
//! </ul>
//!
//! This attribute can be accessed directly or by using one of the following methods:\n\n
//!  - GetExecutionTimes(RMLDoubleVector *OutputVector) const\n\n
//!  - GetExecutionTimes(double *OutputVector, const unsigned int &SizeInBytes) const\n\n
//!  - GetExecutionTimesElement(double *OutputValue, const unsigned int &Index) const\n\n 
//!  - GetExecutionTimesElement(const unsigned int &Index) const\n\n 
//  ----------------------------------------------------------
    RMLDoubleVector			*ExecutionTimes;
        
    
//  ---------------------- Doxygen info ----------------------
//! \var RMLDoubleVector *MinPosExtremaPositionVectorArray
//! 
//! \brief
//! A pointer to an array of pointers to \c RMLDoubleVector 
//! objects. The number of array elements equals the number of
//! degrees of freedom, and each of these \c RMLDOubleVector
//! objects consists of the same number of entries, such that it
//! is a square matrix. A single vector contains the position 
//! vector, that will be achieved when the respective degree of
//! freedom reaches its minimum position.
//! 
//! \details
//! This attribute can be accessed directly or by using one of the following methods:\n\n
//!  - GetMotionStateAtMinPosForOneDOF(const unsigned int &DOF, RMLDoubleVector *PositionVector, RMLDoubleVector *VelocityVector, RMLDoubleVector *AccelerationVector) const\n\n
//!  - GetMotionStateAtMinPosForOneDOF(const unsigned int &DOF, double *PositionVector, double *VelocityVector, double *AccelerationVector, const unsigned int &SizeInBytes) const\n\n
//  ----------------------------------------------------------
    RMLDoubleVector			**MinPosExtremaPositionVectorArray		;		
    
    
//  ---------------------- Doxygen info ----------------------
//! \var RMLDoubleVector *MinPosExtremaVelocityVectorArray
//! 
//! \brief
//! A pointer to an array of pointers to \c RMLDoubleVector 
//! objects. The number of array elements equals the number of
//! degrees of freedom, and each of these \c RMLDOubleVector
//! objects consists of the same number of entries, such that it
//! is a square matrix. A single vector contains the velocity 
//! vector, that will be achieved when the respective degree of
//! freedom reaches its minimum position.
//! 
//! \details
//! This attribute can be accessed directly or by using one of the following methods:\n\n
//!  - GetMotionStateAtMinPosForOneDOF(const unsigned int &DOF, RMLDoubleVector *PositionVector, RMLDoubleVector *VelocityVector, RMLDoubleVector *AccelerationVector) const\n\n
//!  - GetMotionStateAtMinPosForOneDOF(const unsigned int &DOF, double *PositionVector, double *VelocityVector, double *AccelerationVector, const unsigned int &SizeInBytes) const\n\n
//  ----------------------------------------------------------	
    RMLDoubleVector			**MinPosExtremaVelocityVectorArray		;
    

//  ---------------------- Doxygen info ----------------------
//! \var RMLDoubleVector **MinPosExtremaAccelerationVectorArray
//! 
//! \brief
//! A pointer to an array of pointers to \c RMLDoubleVector 
//! objects. The number of array elements equals the number of
//! degrees of freedom, and each of these \c RMLDOubleVector
//! objects consists of the same number of entries, such that it
//! is a square matrix. A single vector contains the position 
//! vector, that will be achieved when the respective degree of
//! freedom reaches its minimum position.
//! 
//! \details
//! This attribute can be accessed directly or by using one of the following methods:\n\n
//!  - GetMotionStateAtMinPosForOneDOF(const unsigned int &DOF, RMLDoubleVector *PositionVector, RMLDoubleVector *VelocityVector, RMLDoubleVector *AccelerationVector) const\n\n
//!  - GetMotionStateAtMinPosForOneDOF(const unsigned int &DOF, double *PositionVector, double *VelocityVector, double *AccelerationVector, const unsigned int &SizeInBytes) const\n\n
//  ----------------------------------------------------------	
    RMLDoubleVector			**MinPosExtremaAccelerationVectorArray	;
    
    
//  ---------------------- Doxygen info ----------------------
//! \var RMLDoubleVector *MaxPosExtremaPositionVectorArray
//! 
//! \brief
//! A pointer to an array of pointers to \c RMLDoubleVector 
//! objects. The number of array elements equals the number of
//! degrees of freedom, and each of these \c RMLDOubleVector
//! objects consists of the same number of entries, such that it
//! is a square matrix. A single vector contains the position 
//! vector, that will be achieved when the respective degree of
//! freedom reaches its maximum position.
//! 
//! \details
//! This attribute can be accessed directly or by using one of the following methods:\n\n
//!  - GetMotionStateAtMaxPosForOneDOF(const unsigned int &DOF, RMLDoubleVector *PositionVector, RMLDoubleVector *VelocityVector, RMLDoubleVector *AccelerationVector) const\n\n
//!  - GetMotionStateAtMaxPosForOneDOF(const unsigned int &DOF, double *PositionVector, double *VelocityVector, double *AccelerationVector, const unsigned int &SizeInBytes) const\n\n
//  ----------------------------------------------------------
    RMLDoubleVector			**MaxPosExtremaPositionVectorArray		;
    

//  ---------------------- Doxygen info ----------------------
//! \var RMLDoubleVector *MaxPosExtremaVelocityVectorArray
//! 
//! \brief
//! A pointer to an array of pointers to \c RMLDoubleVector 
//! objects. The number of array elements equals the number of
//! degrees of freedom, and each of these \c RMLDOubleVector
//! objects consists of the same number of entries, such that it
//! is a square matrix. A single vector contains the velocity
//! vector, that will be achieved when the respective degree of
//! freedom reaches its maximum position.
//! 
//! \details
//! This attribute can be accessed directly or by using one of the following methods:\n\n
//!  - GetMotionStateAtMaxPosForOneDOF(const unsigned int &DOF, RMLDoubleVector *PositionVector, RMLDoubleVector *VelocityVector, RMLDoubleVector *AccelerationVector) const\n\n
//!  - GetMotionStateAtMaxPosForOneDOF(const unsigned int &DOF, double *PositionVector, double *VelocityVector, double *AccelerationVector, const unsigned int &SizeInBytes) const\n\n
//  ----------------------------------------------------------	
    RMLDoubleVector			**MaxPosExtremaVelocityVectorArray		;
    

//  ---------------------- Doxygen info ----------------------
//! \var RMLDoubleVector *MaxPosExtremaAccelerationVectorArray
//! 
//! \brief
//! A pointer to an array of pointers to \c RMLDoubleVector 
//! objects. The number of array elements equals the number of
//! degrees of freedom, and each of these \c RMLDOubleVector
//! objects consists of the same number of entries, such that it
//! is a square matrix. A single vector contains the acceleration
//! vector, that will be achieved when the respective degree of
//! freedom reaches its maximum position.
//! 
//! \details
//! This attribute can be accessed directly or by using one of the following methods:\n\n
//!  - GetMotionStateAtMaxPosForOneDOF(const unsigned int &DOF, RMLDoubleVector *PositionVector, RMLDoubleVector *VelocityVector, RMLDoubleVector *AccelerationVector) const\n\n 
//!  - GetMotionStateAtMaxPosForOneDOF(const unsigned int &DOF, double *PositionVector, double *VelocityVector, double *AccelerationVector, const unsigned int &SizeInBytes) const\n\n
//  ----------------------------------------------------------	
    RMLDoubleVector			**MaxPosExtremaAccelerationVectorArray	;


//  ---------------------- Doxygen info ----------------------
//! \var RMLOutputPolynomials	*Polynomials
//! 
//! \brief
//! Pointer to an RMLOutputPolynomials object that contains the coefficients
//! of all polynomials that represent the current trajectory
//!
//! \details
//! Besides a new state of motion consisting of \c NewPositionVector,
//! \c NewVelocityVector, and \c NewAccelerationVector, the entire
//! trajectory represented by piecewise polynomials for the position
//! velocity, and acceleration progressions are returned by the Reflexxes
//! API methods ReflexxesAPI::RMLPosition() and ReflexxesAPI::RMLVelocity().
//! The coefficients may be used to add time-scaling algorithms or to
//! add custom sample algorithms for the currently computed trajectory.
//!
//! \sa \ref page_Code_02_RMLPositionSampleApplication
//! \sa \ref page_Code_05_RMLVelocitySampleApplication
//  ----------------------------------------------------------	
	RMLOutputPolynomials	*Polynomials							;


};// class RMLOutputParameters

#endif


