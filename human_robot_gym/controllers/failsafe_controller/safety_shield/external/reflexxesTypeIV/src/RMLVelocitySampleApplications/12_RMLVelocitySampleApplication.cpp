//  ---------------------- Doxygen info ----------------------
//! \file 12_RMLVelocitySampleApplication.cpp
//!
//! \brief
//! Test application number 12 for the Reflexxes Motion Libraries
//! (testing positional limits, POSITIONAL_LIMITS_ACTIVELY_PREVENT)
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

	unsigned int				CycleCounter				=	0		;

	ReflexxesAPI				*RML						=	NULL	;
    
	RMLVelocityInputParameters	*IP							=	NULL	;

	RMLVelocityOutputParameters	*OP							=	NULL	;

	RMLVelocityFlags			Flags									;

    // ********************************************************************
    // Creating all relevant objects of the Type IV Reflexxes Motion Library	
    
    RML	=	new ReflexxesAPI(					NUMBER_OF_DOFS
                                            ,	CYCLE_TIME_IN_SECONDS	);
    
    IP	=	new RMLVelocityInputParameters(		NUMBER_OF_DOFS			);
    
    OP	=	new RMLVelocityOutputParameters(	NUMBER_OF_DOFS			);
    
    // ********************************************************************
    // Set-up a timer with a period of one millisecond
    // (not implemented in this example in order to keep it simple)
    // ********************************************************************
    
    printf("-------------------------------------------------------\n"	);
    printf("Reflexxes Motion Libraries                             \n"	);
    printf("Example: 12_RMLVelocitySampleApplication               \n\n");
    printf("This example demonstrates the most basic use of the    \n"	);
    printf("Reflexxes API (class ReflexxesAPI) using the velocity- \n"	);
    printf("based Type IV Online Trajectory Generation algorithm.  \n"  );
    printf("Based on Example 11, this example shows the            \n"  );
    printf("functionality of the input flag to handle the behavior \n"  );
    printf("if a trajectory exceeds positional limits. Here,       \n"  );
    printf("RMLFlags::POSITIONAL_LIMITS_ACTIVELY_PREVENT is used.  \n"  );
    printf("After the first cycle, an exceeding of the positional  \n"  );
    printf("is detected. In order to show the active behavior to   \n"  );
    printf("prevent from exceeding these limits, the execution of  \n"  );
    printf("is not aborted.                                        \n\n");
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
    // of the flag RMLFlags::POSITIONAL_LIMITS_ACTIVELY_PREVENT. Immediately
    // after the first cycle (i.e., after the first call of
    // ReflexxesAPI::RMLPosition()), a value of
    // ReflexxesAPI::RML_ERROR_POSITIONAL_LIMITS will be returned, but this
    // error value is ignored in order to show the behavior of the
    // algorithm.
    
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

	IP->MaxPositionVector->VecData			[0]	=	 -175.0		;
	IP->MaxPositionVector->VecData			[1]	=	 150.0		;
	IP->MaxPositionVector->VecData			[2]	=	 -150.0		;

	IP->MinPositionVector->VecData			[0]	=	 -250.0		;
	IP->MinPositionVector->VecData			[1]	=	 25.0		;
	IP->MinPositionVector->VecData			[2]	=	 -325.0		;

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
    
	Flags.PositionalLimitsBehavior	=	RMLFlags::POSITIONAL_LIMITS_ACTIVELY_PREVENT;

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
                                            
		if (	(ResultValue	<	0											)
			&&	(ResultValue	!=	ReflexxesAPI::RML_ERROR_POSITIONAL_LIMITS	)	)
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
        
		// ****************************************************************
		// The following part is only related to this example. Because 
		// exceeding the positional limits is actively prevented, the
		// desired final state of motion can never be reached. The
		// method ReflexxesAPI::RMLPosition() returns with an error that
		// is ignored in this example. In order to prevent from an endless
		// loop, we stop the execution of this loop after 8000 cycles.

		CycleCounter++;

		if (CycleCounter > 8000)
		{
			break;		// to prevent from an endless loop
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
