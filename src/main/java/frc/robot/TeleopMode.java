package frc.robot;

import java.lang.invoke.MethodHandles;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.drivetrain.Drivetrain;

import frc.constants.Constant;

import frc.controls.DriverController;
import frc.controls.DriverController.DriverAxisAction;
import frc.controls.DriverController.DriverButtonAction;
import frc.controls.DriverController.DriverDpadAction;

public class TeleopMode implements ModeTransition
{
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();

    // *** STATIC INITIALIZATION BLOCK ***
    // This block of code is run first when the class is loaded
    static
    {
        System.out.println("Loading: " + fullClassName);
    }

    
    // *** CLASS & INSTANCE VARIABLES ***
    private static final DriverController DRIVER_CONTROLLER = RobotContainer.DRIVER_CONTROLLER;
    private static final Drivetrain DRIVETRAIN = RobotContainer.DRIVETRAIN;

    private static final PowerDistribution PDH = RobotContainer.PDH;
  
    // Testing variable

    private static final double FEET_TO_METERS = 0.3048;

    // Toggle variables
    private static double angleToTurn;
    private static double driveTrainRotation;

    // *** CLASS CONSTRUCTOR ***
    public TeleopMode()
    {
        System.out.println(fullClassName + " : Constructor Started");
        
        System.out.println(fullClassName + ": Constructor Finished");
    }

    /**
     * This method runs one time before the periodic() method.
     */
    public void init()
    {

        if(DRIVER_CONTROLLER != null && DRIVETRAIN != null)
        {
            DRIVER_CONTROLLER.resetRumbleCounter();
            DRIVETRAIN.resetEncoders();
        }
    }

    /**
     * This method runs periodically (every 20ms).
     */
    public void periodic()
    {      
         // tilt of the roboRIO (robot on Charging Station)
        SmartDashboard.putNumber("tilt degrees", DRIVETRAIN.tiltXZ());

        if(DRIVER_CONTROLLER != null)
        {
            DRIVER_CONTROLLER.checkRumbleEvent();

            if(DRIVETRAIN != null)
            {
                // TODO : Add slew rate limiter
                double drivePowerLimit = 0.8;
                double turnPowerLimit = 0.1;
                double xSpeed = DRIVER_CONTROLLER.getAction(DriverAxisAction.kMoveY) * Constant.MAX_DRIVE_SPEED;
                double ySpeed = DRIVER_CONTROLLER.getAction(DriverAxisAction.kMoveX) * Constant.MAX_DRIVE_SPEED;
                double turn = DRIVER_CONTROLLER.getAction(DriverAxisAction.kRotate) * Constant.MAX_ROBOT_TURN_SPEED;

                if (DRIVER_CONTROLLER.getAction(DriverButtonAction.kBoostOrAutoAim))
                {
                    drivePowerLimit = 1.0;
                }

                // Scales down the input power
            
                // drivePowerLimit += DRIVER_CONTROLLER.getAction(DriverAxisAction.kDriverBoost) * (1.0 - drivePowerLimit);

                xSpeed *= drivePowerLimit;
                ySpeed *= drivePowerLimit;
                turn *= turnPowerLimit;
                driveTrainRotation = turn;
                
                if (DRIVER_CONTROLLER.getAction(DriverDpadAction.kLockSwerve))
                {
                    // System.out.println("Locking swerve drive");
                    DRIVETRAIN.lockWheels();
                }
                else
                {
                    DRIVETRAIN.drive(xSpeed, ySpeed, driveTrainRotation, !DRIVER_CONTROLLER.getAction(DriverButtonAction.kRobotOriented));
                }

                // running the drivetrain
                // DRIVETRAIN.moveYAxis(DRIVER_CONTROLLER.getAction(DriverAxisAction.kMoveY));

                // DRIVETRAIN.moveXAxis(DRIVER_CONTROLLER.getAction(DriverAxisAction.kMoveX));

                // DRIVETRAIN.rotate(DRIVER_CONTROLLER.getAction(DriverAxisAction.kRotate));

                // DRIVETRAIN.driveBoost(DRIVER_CONTROLLER.getAction(DriverAxisAction.kDriverBoost));

                if (DRIVER_CONTROLLER.getAction(DriverButtonAction.kResetGyro))
                {
                    DRIVETRAIN.resetGyro();
                }
            }
        }
    }

    /**
     * This method runs one time after the periodic() method.
     */
    public void exit()
    {

    }
}
