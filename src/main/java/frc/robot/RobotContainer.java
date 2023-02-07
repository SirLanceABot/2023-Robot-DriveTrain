// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.invoke.MethodHandles;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Autonomous1;
import frc.robot.commands.LockWheels;
import frc.robot.commands.SwerveDrive;
import frc.robot.controls.DriverController;
import frc.robot.controls.Xbox;
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
	
	private boolean useFullRobot		= true;
	private boolean useBindings			= true;

	private boolean useDrivetrain   	= true;
    private boolean useDriverController = true;

    public final Drivetrain drivetrain;
    public final DriverController driverController;
    private static final Accelerometer accelerometer = new BuiltInAccelerometer(Accelerometer.Range.k2G);
	private final WPI_Pigeon2 gyro = new WPI_Pigeon2(Port.Sensor.PIGEON, Port.Motor.CAN_BUS);
	


	
	// private Joystick joystick;
	
	/** 
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 * Use the default modifier so that new objects can only be constructed in the same package.
	 */
	RobotContainer()
	{
		// Create the needed subsystems
		driverController = (useFullRobot || useDriverController) ? new DriverController(0)     : null;
		drivetrain 	= (useFullRobot || useDrivetrain) ? new Drivetrain(Port.DrivetrainSetup.DRIVETRAIN_DATA, accelerometer, gyro) 	 : null;
 		

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
			BooleanSupplier aButton = () -> {return driverController.getRawButton(Xbox.Button.kA); };
			Trigger aButtonTrigger = new Trigger(aButton);
			//aButtonTrigger.onTrue(new LockWheels(drivetrain));
			aButtonTrigger.toggleOnTrue(new LockWheels(drivetrain));
			//JoystickButton drivetrainA = new JoystickButton(joystick,1);
			Supplier<Double> leftYAxis = () -> { return driverController.getRawAxis(Xbox.Axis.kLeftY); };
			Supplier<Double> leftXAxis = () -> { return driverController.getRawAxis(Xbox.Axis.kLeftX); };
			Supplier<Double> rightXAxis = () -> {return driverController.getRawAxis(Xbox.Axis.kRightX); };
			
			drivetrain.setDefaultCommand(new SwerveDrive(drivetrain, leftYAxis, leftXAxis, rightXAxis, true));
        }

    }

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand()
	{
		Command autoCommand = null;
		autoCommand = new Autonomous1(drivetrain);
		return autoCommand;
	}
}
