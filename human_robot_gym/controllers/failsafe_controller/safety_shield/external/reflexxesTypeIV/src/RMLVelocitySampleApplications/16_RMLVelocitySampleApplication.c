//  ---------------------- Doxygen info ----------------------
//! \file 16_RMLVelocitySampleApplication.c
//!
//! \brief
//! Test application number 16 for the Reflexxes Motion Libraries
//! (C-wrapper, target velocity-based algorithm)
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
#include <string.h>

#include <TypeIVRML_C_Wrapper.h>

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
    
    int							ResultValue					=	0		
							,	SelectionVector				[NUMBER_OF_DOFS];
    
    double						CurrentPositionVector		[NUMBER_OF_DOFS]
							,	CurrentVelocityVector		[NUMBER_OF_DOFS]
							,	CurrentAccelerationVector	[NUMBER_OF_DOFS]
							,	MaxAccelerationVector		[NUMBER_OF_DOFS]
							,	MaxJerkVector				[NUMBER_OF_DOFS]
							,	TargetVelocityVector		[NUMBER_OF_DOFS]
							,	NewPositionVector			[NUMBER_OF_DOFS]
							,	NewVelocityVector			[NUMBER_OF_DOFS]	
							,	NewAccelerationVector		[NUMBER_OF_DOFS];
						

    // ********************************************************************
    // Allocate memory for the algorithms of the Type IV Reflexxes Motion
    // Library
    
    TypeIVRMLCreate(	NUMBER_OF_DOFS
					,	CYCLE_TIME_IN_SECONDS	);
    
    // ********************************************************************
    // Set-up a timer with a period of one millisecond
    // (not implemented in this example in order to keep it simple)
    // ********************************************************************
    
    printf("-------------------------------------------------------\n"  );
    printf("Reflexxes Motion Libraries                             \n"  );
    printf("Example: 16_RMLVelocitySampleApplication               \n\n");
    printf("This example demonstrates the most basic use of the    \n"  );
    printf("C-wrapper for the Reflexxes API using the velocity-    \n"  );
    printf("based Type IV Online Trajectory Generation algorithm.  \n"  );
    printf("It is based on Example 4.                              \n\n");
    printf("Copyright (C) 2013 Reflexxes GmbH                      \n"  );
    printf("-------------------------------------------------------\n"  );

    // ********************************************************************
    // Set-up the input parameters
    
    // In this test program, arbitrary values are chosen. If executed on a
    // real robot or mechanical system, the position is read and stored in
    // the CurrentPositionVector array.
    // For the very first motion after starting the controller, velocities
    // and acceleration are commonly set to zero. The desired target state
    // of motion and the motion constraints depend on the robot and the
    // current task/application.
    
    CurrentPositionVector		[0]	=	-200.0		;
    CurrentPositionVector		[1]	=	 100.0		;
    CurrentPositionVector		[2]	=	-300.0		;
										
    CurrentVelocityVector		[0]	=	-150.0		;
    CurrentVelocityVector		[1]	=	 100.0		;
    CurrentVelocityVector		[2]	=	  50.0		;
										
    CurrentAccelerationVector	[0]	=	 350.0		;
    CurrentAccelerationVector	[1]	=	-500.0		;
    CurrentAccelerationVector	[2]	=	   0.0		;	
										
    MaxAccelerationVector		[0]	=	 500.0		;
    MaxAccelerationVector		[1]	=	 500.0		;
    MaxAccelerationVector		[2]	=	1000.0		;		
										
    MaxJerkVector				[0]	=	1000.0		;
    MaxJerkVector				[1]	=	 700.0		;
    MaxJerkVector				[2]	=	 500.0		;
										
    TargetVelocityVector		[0]	=	 150.0		;
    TargetVelocityVector		[1]	=	  75.0		;
    TargetVelocityVector		[2]	=    100.0		;

    SelectionVector				[0]	=	1			;
    SelectionVector				[1]	=	1			;
    SelectionVector				[2]	=	1			;

    // ********************************************************************
    // Starting the control loop
    
    while (ResultValue != 1)
    {
    
        // ****************************************************************									
        // Wait for the next timer tick
        // (not implemented in this example in order to keep it simple)
        // ****************************************************************	
    
        // Calling the Reflexxes OTG algorithm
        ResultValue	=	TypeIVRMLVelocity(		CurrentPositionVector
											,	CurrentVelocityVector
											,	CurrentAccelerationVector
											,	MaxAccelerationVector
											,	MaxJerkVector
											,	TargetVelocityVector
											,	SelectionVector
											,	NewPositionVector
											,	NewVelocityVector
											,	NewAccelerationVector		);
                                            
        if (ResultValue < 0)
        {
            printf("An error occurred (%d).\n", ResultValue	);
            break;
        }
        
        // ****************************************************************									
        // Here, the new state of motion, that is
        //
        // - NewPositionVector		
        // - NewVelocityVector		
        // - NewAccelerationVector
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
        
        memcpy(		CurrentPositionVector
				,	NewPositionVector
				,	NUMBER_OF_DOFS * sizeof(NewPositionVector[0]));
        memcpy(		CurrentVelocityVector
				,	NewVelocityVector
				,	NUMBER_OF_DOFS * sizeof(NewPositionVector[0]));
        memcpy(		CurrentAccelerationVector
				,	NewAccelerationVector
				,	NUMBER_OF_DOFS * sizeof(NewPositionVector[0]));										
    }

    // ********************************************************************
    // Releasing the memory used for the algorithms of the Reflexxes Motion
    // Library end terminating the process
    
	TypeIVRMLDestroy();

    exit(EXIT_SUCCESS)	;
}
