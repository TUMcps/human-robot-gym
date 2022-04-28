//  ---------------------- Doxygen info ----------------------
//! \file 09_RMLPositionSampleApplication.cpp
//!
//! \brief
//! Test application number 9 for the Reflexxes Motion Libraries
//! (testing positional limits, POSITIONAL_LIMITS_ERROR_MSG_ONLY)
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


//*************************************************************************
// Main function to run the process that contains the test application
// 
// This function contains source code to get started with the Type IV  
// Reflexxes Motion Library. Only a minimum amount of functionality is
// contained in this program: a simple trajectory for a
// three-degree-of-freedom system is executed. This code snippet
// directly corresponds to the example trajectories shown in the 
// documentation.
//*************************************************************************
int main()
{
    // ********************************************************************
    // Variable declarations and definitions
    
    int							ResultValue					=	0		;

    ReflexxesAPI				*RML						=	NULL	;
    
    RMLPositionInputParameters	*IP							=	NULL	;
    
    RMLPositionOutputParameters	*OP							=	NULL	;
    
    RMLPositionFlags			Flags									;

    // ********************************************************************
    // Creating all relevant objects of the Type IV Reflexxes Motion Library	
    
    RML	=	new ReflexxesAPI(					NUMBER_OF_DOFS
                                            ,	CYCLE_TIME_IN_SECONDS	);
    
    IP	=	new RMLPositionInputParameters(		NUMBER_OF_DOFS			);
    
    OP	=	new RMLPositionOutputParameters(	NUMBER_OF_DOFS			);
    
    // ********************************************************************
    // Set-up a timer with a period of one millisecond
    // (not implemented in this example in order to keep it simple)
    // ********************************************************************
    
    printf("-------------------------------------------------------\n"	);
    printf("Reflexxes Motion Libraries                             \n"	);
    printf("Example: 09_RMLPositionSampleApplication               \n\n");
    printf("This example demonstrates the most basic use of the    \n"	);
    printf("Reflexxes API (class ReflexxesAPI) using the position- \n"	);
    printf("based Type IV Online Trajectory Generation algorithm.  \n"  );
    printf("In addition the basic functionality of Example 1,  \n"  );
    printf("this is the first of two examples showing the usage of \n"  );
    printf("the positional limits flag. After the first cycle,     \n"  );
    printf("the method ReflexxesAPI::RMLPosition() will return -108\n"  );
    printf("(ReflexxesAPI::RML_ERROR_POSITIONAL_LIMITS) to indicate\n"  );
    printf("that the currently computed trajectory will exceed the \n"  );
	printf("given positional limits before reaching the desired    \n"  );
	printf("target state of motion.                                \n\n");
    printf("Copyright (C) 2013 Reflexxes GmbH                      \n"	);
    printf("-------------------------------------------------------\n"	);

    // ********************************************************************
    // Set-up the input parameters
    
    // In order to test the functionality of positional limits
    // 
    //  - RMLPositionInputParameters::MinPositionVector and
    //  - RMLPositionInputParameters::MaxPositionVector,
    // 
    // the input values are chosen, so that an these limits will be
    // exceeded by the computed trajectory. This examples shows the use
    // of the flag RMLFlags::POSITIONAL_LIMITS_ERROR_MSG_ONLY. Immediately
    // after the first cycle (i.e., after the first call of
    // ReflexxesAPI::RMLPosition()), a value of
    // ReflexxesAPI::RML_ERROR_POSITIONAL_LIMITS will be returned.
    
	IP->CurrentPositionVector->VecData		[0]	=	 100.0		;
	IP->CurrentPositionVector->VecData		[1]	=	   0.0		;
	IP->CurrentPositionVector->VecData		[2]	=	  50.0		;

	IP->CurrentVelocityVector->VecData		[0]	=	 100.0		;
	IP->CurrentVelocityVector->VecData		[1]	=	-220.0		;
	IP->CurrentVelocityVector->VecData		[2]	=	 -50.0		;

	IP->CurrentAccelerationVector->VecData	[0]	=	-150.0		;
	IP->CurrentAccelerationVector->VecData	[1]	=	 250.0		;
	IP->CurrentAccelerationVector->VecData	[2]	=	 -50.0		;	

	IP->MaxVelocityVector->VecData			[0]	=	 300.0		;
	IP->MaxVelocityVector->VecData			[1]	=	 100.0		;
	IP->MaxVelocityVector->VecData			[2]	=	 300.0		;

	IP->MaxAccelerationVector->VecData		[0]	=	 300.0		;
	IP->MaxAccelerationVector->VecData		[1]	=	 200.0		;
	IP->MaxAccelerationVector->VecData		[2]	=	 100.0		;		

	IP->MaxJerkVector->VecData				[0]	=	 400.0		;
	IP->MaxJerkVector->VecData				[1]	=	 300.0		;
	IP->MaxJerkVector->VecData				[2]	=	 200.0		;

	IP->TargetPositionVector->VecData		[0]	=	-600.0		;
	IP->TargetPositionVector->VecData		[1]	=	-200.0		;
	IP->TargetPositionVector->VecData		[2]	=	-350.0		;

	IP->TargetVelocityVector->VecData		[0]	=	 150.0		;
	IP->TargetVelocityVector->VecData		[1]	=	 -50.0		;
	IP->TargetVelocityVector->VecData		[2]	=   -200.0		;

	IP->MaxPositionVector->VecData			[0]	=	 150.0		;
	IP->MaxPositionVector->VecData			[1]	=    200.0		;
	IP->MaxPositionVector->VecData			[2]	=    250.0		;

	IP->MinPositionVector->VecData			[0]	=   -650.0		;
	IP->MinPositionVector->VecData			[1]	=   -500.0		;
	IP->MinPositionVector->VecData			[2]	=   -700.0		;		

    IP->SelectionVector->VecData			[0]	=	true		;
    IP->SelectionVector->VecData			[1]	=	true		;
    IP->SelectionVector->VecData			[2]	=	true		;
    
    // ********************************************************************
    // Setting the input value for RMLFlags::PositionalLimitsBehavior
    // Possible options:
    //
    //  - RMLFlags::POSITIONAL_LIMITS_IGNORE (default)
    //  - RMLFlags::POSITIONAL_LIMITS_ERROR_MSG_ONLY
    //  - RMLFlags::POSITIONAL_LIMITS_ACTIVELY_PREVENT
    
	Flags.PositionalLimitsBehavior	=	RMLFlags::POSITIONAL_LIMITS_ERROR_MSG_ONLY;

    // ********************************************************************
    // Starting the control loop
    
    while (ResultValue != ReflexxesAPI::RML_FINAL_STATE_REACHED)
    {
    
        // ****************************************************************									
        // Wait for the next timer tick
        // (not implemented in this example in order to keep it simple)
        // ****************************************************************	
    
        // Calling the Reflexxes OTG algorithm
        ResultValue	=	RML->RMLPosition(		*IP
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
    }
    
    // ********************************************************************
    // The loop will only be executed once, and the method 
	// ReflexxesAPI::RMLPosition() returns the error value -108 
	// (ReflexxesAPI::RML_ERROR_POSITIONAL_LIMITS) to indicate that the
	// currently executed trajectory will exceed the positional limits.    
    

    // ********************************************************************
    // Deleting the objects of the Reflexxes Motion Library end terminating
    // the process
    
    delete	RML			;
    delete	IP			;
    delete	OP			;

    exit(EXIT_SUCCESS)	;
}
