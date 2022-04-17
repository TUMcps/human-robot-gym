//  ---------------------- Doxygen info ----------------------
//! \file 14_RMLVelocitySampleApplication.cpp
//!
//! \brief
//! Test application number 14 for the Reflexxes Motion Libraries
//! (override functionality using the target velocity-based algorithm)
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


#include <stdio.h>
#include <stdlib.h>

#include <ReflexxesAPI.h>
#include <RMLPositionFlags.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>


//*************************************************************************
// defines

#define CYCLE_TIME_IN_SECONDS					0.001
#define NUMBER_OF_DOFS							3
#define NUMBER_OF_ADDITIONAL_THREADS			0
#define MAX_OVERRIDE_FILTER_TIME				4.0
#define APPLIED_OVERRIDE_FILTER_TIME			0.75
#define INITIAL_OVERRIDE_VALUE					0.8


//*************************************************************************
// Main function to run the process that contains the test application
// 
// This function contains source code to get started with the Type IV  
// Reflexxes Motion Library. Based on example 04, this example shows the
// use of the override functionality for the target velocity-based 
// online trajectory generation algorithm to scale the velocity of the 
// trajectory. This code snippet directly corresponds to the example
// trajectories shown in the documentation.
//*************************************************************************
int main()
{
    // ********************************************************************
    // Variable declarations and definitions
    
    int							ResultValue					=	0		;
    
    double						Time						=	0.0		;    

    ReflexxesAPI				*RML						=	NULL	;
    
	RMLVelocityInputParameters	*IP							=	NULL	;

	RMLVelocityOutputParameters	*OP							=	NULL	;

	RMLVelocityFlags			Flags									;

    // ********************************************************************
    // Creating all relevant objects of the Type IV Reflexxes Motion Library	
    
    RML	=	new ReflexxesAPI(					NUMBER_OF_DOFS
                                            ,	CYCLE_TIME_IN_SECONDS	
                                            ,	NUMBER_OF_ADDITIONAL_THREADS
                                            ,	MAX_OVERRIDE_FILTER_TIME);
    
	IP	=	new RMLVelocityInputParameters(		NUMBER_OF_DOFS			);

	OP	=	new RMLVelocityOutputParameters(	NUMBER_OF_DOFS			);
    
    // ********************************************************************
    // Set-up a timer with a period of one millisecond
    // (not implemented in this example in order to keep it simple)
    // ********************************************************************
    
	printf("-------------------------------------------------------\n"	);
	printf("Reflexxes Motion Libraries                             \n"	);
	printf("Example: 14_RMLVelocitySampleApplication               \n\n");
	printf("This example demonstrates using the override           \n"	);
	printf("functionality of the Reflexxes Motion Libraries using  \n"	);
	printf("the velocity-based Type IV Online Trajectory Generation\n"	);
	printf("algorithm.                                             \n\n");       
	printf("Copyright (C) 2013 Reflexxes GmbH                      \n"	);
	printf("-------------------------------------------------------\n"	);


	// ********************************************************************
	// Initialize the filter for override values
	//
	// Initial override value: 0.8 ( = 100%)
	// Override filter time  : 1.0 (seconds)
	
	RML->SetupOverrideFilter(		INITIAL_OVERRIDE_VALUE
								,	APPLIED_OVERRIDE_FILTER_TIME	);

    // ********************************************************************
    // Set-up the input parameters
    
   // The following arbitrary input values are taken from Example 4
    
	IP->CurrentPositionVector->VecData		[0]	=	-200.0		;
	IP->CurrentPositionVector->VecData		[1]	=	 100.0		;
	IP->CurrentPositionVector->VecData		[2]	=	-300.0		;

	IP->CurrentVelocityVector->VecData		[0]	=	-150.0		;
	IP->CurrentVelocityVector->VecData		[1]	=	 100.0		;
	IP->CurrentVelocityVector->VecData		[2]	=	  50.0		;

	IP->CurrentAccelerationVector->VecData	[0]	=	 350.0		;
	IP->CurrentAccelerationVector->VecData	[1]	=	-500.0		;
	IP->CurrentAccelerationVector->VecData	[2]	=	   0.0		;	

	IP->MaxAccelerationVector->VecData		[0]	=	 500.0		;
	IP->MaxAccelerationVector->VecData		[1]	=	 500.0		;
	IP->MaxAccelerationVector->VecData		[2]	=	1000.0		;		

	IP->MaxJerkVector->VecData				[0]	=	1000.0		;
	IP->MaxJerkVector->VecData				[1]	=	 700.0		;
	IP->MaxJerkVector->VecData				[2]	=	 500.0		;

	IP->TargetVelocityVector->VecData		[0]	=	 150.0		;
	IP->TargetVelocityVector->VecData		[1]	=	  75.0		;
	IP->TargetVelocityVector->VecData		[2]	=    100.0		;

	IP->SelectionVector->VecData			[0]	=	true		;
	IP->SelectionVector->VecData			[1]	=	true		;
	IP->SelectionVector->VecData			[2]	=	true		;

	// ********************************************************************
	// Set-up the override input value

	IP->OverrideValue							=	INITIAL_OVERRIDE_VALUE;

    // ********************************************************************
    // Starting the control loop
    
    while (ResultValue != ReflexxesAPI::RML_FINAL_STATE_REACHED)
    {
        // ****************************************************************									
        // Wait for the next timer tick
        // (not implemented in this example in order to keep it simple)
        // ****************************************************************	
    
        // Calling the Reflexxes OTG algorithm
        ResultValue	=	RML->RMLVelocity(		*IP
                                            ,	OP
                                            ,	Flags		);
                                   
		if (ResultValue < 0)
		{
			printf("An error occurred (%d).\n", ResultValue	);
			printf("%s\n", OP->GetErrorString());
			break;
		}
        
        // ****************************************************************									
        // Here, the new state of motion, that is
        //
        // - OP->NewPositionVector		
        // - OP->NewVelocityVector		
        // - OP->NewAccelerationVector
        //
        // can be used as input values for lower level controllers. In the 
        // most simple case, a position controller in actuator space is 
        // used, but the computed state can be applied to many other 
        // controllers (e.g., Cartesian impedance controllers, 
        // operational space controllers).
        // ****************************************************************

        // ****************************************************************
        // Feed the output values of the current control cycle back to 
        // input values of the next control cycle
        
        *IP->CurrentPositionVector		=	*OP->NewPositionVector		;
        *IP->CurrentVelocityVector		=	*OP->NewVelocityVector		;
        *IP->CurrentAccelerationVector	=	*OP->NewAccelerationVector	;
        
		Time	+=	CYCLE_TIME_IN_SECONDS;

		// ****************************************************************
		// Change the override value:
		// 120% after 0.50 seconds
		// 100% after 1.50 seconds

		if (Time >= 0.50)
		{			
			IP->OverrideValue	=	1.2;
		}        
		if (Time >= 1.5)
		{
			IP->OverrideValue	=	1.0;
		}
    }

    // ********************************************************************
    // Deleting the objects of the Reflexxes Motion Library end terminating
    // the process
    
    delete	RML			;
    delete	IP			;
    delete	OP			;

    exit(EXIT_SUCCESS)	;
}
