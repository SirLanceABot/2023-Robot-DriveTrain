// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.invoke.MethodHandles;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.SwerveDrive;
import frc.robot.constants.Port;
import frc.robot.controls.DriverController;
import frc.robot.subsystems.Drivetrain;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
    // This string gets the full name of the class, including the package name
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();

    // *** STATIC INITIALIZATION BLOCK ***
    // This block of code is run first when the class is loaded
    static
    {
        System.out.println("Loading: " + fullClassName);
    }
	
	private boolean useFullRobot		= false;
	private boolean useBindings			= false;

	private boolean useDrivetrain   	= false;
    private boolean useDriverController = false;

    public final Drivetrain drivetrain;
    public final DriverController driverController;
    private static final Accelerometer accelerometer = new BuiltInAccelerometer(Accelerometer.Range.k2G);

	
	// private Joystick joystick;
	
	/** 
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 * Use the default modifier so that new objects can only be constructed in the same package.
	 */
	RobotContainer()
	{
		// Create the needed subsystems
		driverController = (useFullRobot || useDriverController) ? new DriverController(0)     : null;
		drivetrain 	= (useFullRobot || useDrivetrain) ? new Drivetrain(Port.DrivetrainSetup.DRIVETRAIN_DATA, accelerometer, driverController) 	                : null;
        
		

		// Configure the trigger bindings
		if(useFullRobot || useBindings)
			configureBindings();
	}

	/**
	 * Use this method to define your trigger->command mappings. Triggers can be created via the
	 * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
	 * predicate, or via the named factories in {@link
	 * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
	 * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
	 * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
	 * joysticks}.
	 */
    private void configureBindings()
    {
        configureDriverBindings();
    }

    //FIXME 
    private void configureDriverBindings()
    {
      if(driverController != null && drivetrain != null)
        {
            //JoystickButton drivetrainA = new JoystickButton(joystick,1);
        Supplier<Double> leftYAxis = () -> { return driverController.getRawAxis(1); };
        Supplier<Double> rightXAxis = () -> {return driverController.getRawAxis(1); };
        drivetrain.setDefaultCommand(new SwerveDrive(drivetrain, leftYAxis, rightXAxis));
        }
  
    }

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand()
	{
		return null;
	}
}
