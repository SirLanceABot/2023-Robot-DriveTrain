package frc.robot.subsystems;

import java.lang.invoke.MethodHandles;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import frc.robot.Constant;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;


/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends Subsystem4237 
{

    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();

    // *** STATIC INITIALIZATION BLOCK ***
    // This block of code is run first when the class is loaded
    static
    {
        System.out.println("Loading: " + fullClassName);
    }

    /**
     * define all the inputs to be read at once
     * define all the outputs to be written at once
     */
    private class PeriodicIO 
    {
      // INPUTS
      private double xSpeed;
      private double ySpeed;
      private double turn;
      private boolean fieldRelative;
      private ChassisSpeeds chassisSpeeds;
      private SwerveModuleState[] swerveModuleStates;
  
    
      
      //FIXME is this right?
      // OUTOUTS
      // FIXME should we add odometry and kinematics?
    }

    private enum DriveMode
    {
        kDrive, kLockwheels, kStop;
    }

    // *** CLASS & INSTANCE VARIABLES ***
    // private static final Translation2d frontLeftLocation = new Translation2d(Constant.DRIVETRAIN_WHEELBASE_METERS / 2, Constant.DRIVETRAIN_TRACKWIDTH_METERS / 2);
    // private static final Translation2d frontRightLocation = new Translation2d(Constant.DRIVETRAIN_WHEELBASE_METERS / 2, -Constant.DRIVETRAIN_TRACKWIDTH_METERS / 2);
    // private static final Translation2d backLeftLocation = new Translation2d(-Constant.DRIVETRAIN_WHEELBASE_METERS / 2, Constant.DRIVETRAIN_TRACKWIDTH_METERS / 2);
    // private static final Translation2d backRightLocation = new Translation2d(-Constant.DRIVETRAIN_WHEELBASE_METERS / 2, -Constant.DRIVETRAIN_TRACKWIDTH_METERS / 2);

    private final SwerveModule frontLeft;// = new SwerveModule(Port.Module.FRONT_LEFT);
    private final SwerveModule frontRight;// = new SwerveModule(Port.Module.FRONT_RIGHT);
    private final SwerveModule backLeft;// = new SwerveModule(Port.Module.BACK_LEFT);
    private final SwerveModule backRight;// = new SwerveModule(Port.Module.BACK_RIGHT);

    private final WPI_Pigeon2 gyro; //Pigeon2
    private final Accelerometer accelerometer;

    // private static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    //         frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

    

    private final SwerveDriveKinematics kinematics;// = new SwerveDriveKinematics(
            // Port.Module.FRONT_LEFT.moduleLocation, Port.Module.FRONT_RIGHT.moduleLocation, Port.Module.BACK_LEFT.moduleLocation, Port.Module.BACK_RIGHT.moduleLocation);

    private final SwerveDriveOdometry odometry;

    // TODO: Make final by setting to an initial stopped state
    //private SwerveModuleState[] previousSwerveModuleStates = null;
    private DriveMode driveMode = DriveMode.kDrive;
    private boolean resetEncoders = false;

    private PeriodicIO periodicIO;
    
    // *** CLASS CONSTRUCTOR ***
    public Drivetrain(DrivetrainConfig dd, Accelerometer accelerometer, WPI_Pigeon2 gyro)//, DriverController driverController)
    {
        // super();  // call the RobotDriveBase constructor
        // setSafetyEnabled(false);
  
  
        periodicIO = new PeriodicIO(); // all the periodic I/O appear here
            
        this.accelerometer = accelerometer;
        this.gyro = gyro;


        frontLeft = new SwerveModule(dd.frontLeftSwerveModule);
        frontRight = new SwerveModule(dd.frontRightSwerveModule);
        backLeft = new SwerveModule(dd.backLeftSwerveModule);
        backRight = new SwerveModule(dd.backRightSwerveModule);

        // gyro = new WPI_Pigeon2(Port.Sensor.PIGEON, Port.Motor.CAN_BUS);

        kinematics = new SwerveDriveKinematics(
            dd.frontLeftSwerveModule.moduleLocation,
            dd.frontRightSwerveModule.moduleLocation,
            dd.backLeftSwerveModule.moduleLocation,
            dd.backRightSwerveModule.moduleLocation);

        odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d(),
            new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()

        });
        //FIXME is this ok here?
        gyro.reset();
        setGyro(180.0);
        resetOdometry();
        // setSafetyEnabled(true);
    }

    // *** CLASS & INSTANCE METHODS ***
    //FIXME Is this used?
    // public void configOpenLoopRamp(double seconds)
    // {
    //     frontLeft.configOpenLoopRamp(seconds);
    //     frontRight.configOpenLoopRamp(seconds);
    //     backLeft.configOpenLoopRamp(seconds);
    //     backRight.configOpenLoopRamp(seconds);
    // }

    
    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed Speed of the robot in the x direction (forward).
     * @param ySpeed Speed of the robot in the y direction (sideways).
     * @param turn Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the field.
     */
    @SuppressWarnings("ParameterName")
    public void drive(double xSpeed, double ySpeed, double turn, boolean fieldRelative)
    {
        driveMode = DriveMode.kDrive;
        // updateOdometry();
        periodicIO.xSpeed = xSpeed;
        periodicIO.ySpeed = ySpeed;
        periodicIO.turn = turn;
        periodicIO.fieldRelative = fieldRelative;

        //ChassisSpeeds chassisSpeeds;
        //SwerveModuleState[] swerveModuleStates;

        // if(fieldRelative)
        //     chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turn, gyro.getRotation2d());
        // else
        //     chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turn);
        
        // swerveModuleStates = kinematics.toSwerveModuleStates(periodicIO.chassisSpeeds);
        // SwerveDriveKinematics.desaturateWheelSpeeds(periodicIO.swerveModuleStates, Constant.MAX_DRIVE_SPEED);
        // printDesiredStates(swerveModuleStates);
      
        // frontLeft.setDesiredState(periodicIO.swerveModuleStates[0]);
        // frontRight.setDesiredState(periodicIO.swerveModuleStates[1]);
        // backLeft.setDesiredState(periodicIO.swerveModuleStates[2]);
        // backRight.setDesiredState(periodicIO.swerveModuleStates[3]);

        // previousSwerveModuleStates = periodicIO.swerveModuleStates;

        // feedWatchdog();
    }
    
    
    /**
     * Rotate swerve modules to an X shape to hopefully prevent being pushed 
     */
    @SuppressWarnings("ParameterName")
    public void lockWheels()
    {
        driveMode = DriveMode.kLockwheels;
        //updateOdometry();
        
        // SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];
        
        // // TODO: Check that this works
        // swerveModuleStates[0] = new SwerveModuleState(0.0, Rotation2d.fromDegrees(45));
        // swerveModuleStates[1] = new SwerveModuleState(0.0, Rotation2d.fromDegrees(135));
        // swerveModuleStates[2] = new SwerveModuleState(0.0, Rotation2d.fromDegrees(135));
        // swerveModuleStates[3] = new SwerveModuleState(0.0, Rotation2d.fromDegrees(45));

        // SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constant.MAX_DRIVE_SPEED);
        // printDesiredStates(swerveModuleStates);

        // frontLeft.setDesiredState(swerveModuleStates[0]);
        // frontRight.setDesiredState(swerveModuleStates[1]);
        // backLeft.setDesiredState(swerveModuleStates[2]);
        // backRight.setDesiredState(swerveModuleStates[3]);

        //previousSwerveModuleStates = swerveModuleStates;

        //feedWatchdog();
    }



    /** Updates the field relative position of the robot. */
    
    private void updateOdometry()
    {
        //FIXME do we need to put this into periodic methods
        odometry.update(
            gyro.getRotation2d(),
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
            }
        );
        
        System.out.format( "pose: X:%f Y:%f degrees %f\n"
        // a POSE has a TRANSLATION and a ROTATION
        // POSE can return directly the X and Y of the TRANSLATION but not the Degrees, Radians,
        // or trig functions of the ROTATION
        // pose: X:-0.565898 Y:-0.273620 degrees 137.436218
            ,odometry.getPoseMeters().getX()
            ,odometry.getPoseMeters().getY()
            ,odometry.getPoseMeters().getRotation().getDegrees()
        );
        
    }
    


    // public Translation2d getCurrentTranslation()
    // {
    //     return odometry.getPoseMeters().getTranslation();
    // }

    public void resetEncoders()
    {
        resetEncoders = true;
        // frontLeft.resetEncoders();
        // frontRight.resetEncoders();
        // backLeft.resetEncoders();
        // backRight.resetEncoders();
    }

    public void resetOdometry()
    {
        //FIXME
        odometry.resetPosition(new Rotation2d(), /*zero*/
            new SwerveModulePosition[]
                {/*zeros distance, angle*/
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
                },
            new Pose2d(/*zeros facing X*/));

        //FIXME  reset to what? 0, 0, 0? or AprilTags pose? or other?
        //FIXME odometry.resetPosition(new Pose2d(), new Rotation2d(gyro.getYaw()));
    }

    //@Override
    public void stopMotor()
    {
        driveMode = DriveMode.kStop;
        // frontLeft.stopModule();
        // frontRight.stopModule();
        // backLeft.stopModule();
        // backRight.stopModule();
        //feedWatchdog();
    }

    public void resetGyro()
    {
        gyro.reset();
        System.out.println("GYRO RESET");
    }

    public void setGyro(double angle)
    {
        gyro.setYaw(angle);
        System.out.println("GYRO RESET AT " + angle);
    }

    public void printGyro()
    {
        System.out.println("Yaw: " + gyro.getYaw());
    }

    public double getGyro()
    {
        return gyro.getYaw();
    }

    // @Override
    // public String getDescription()
    // {
    //     return "Swerve Drivetrain";
    // }

    /**
     * roboRIO tilt in degrees
     * @return angle degrees
     */
    public double tiltXZ()
    {
        var accelX = accelerometer.getX();
        var accelY = accelerometer.getY();
        var accelZ = accelerometer.getZ();

        // var angleXY = Math.atan2(mPeriodicIO.accelX, mPeriodicIO.accelY);
        var angleXZ = Math.atan2(accelX, accelZ);
        // var angleYZ = Math.atan2(mPeriodicIO.accelY, mPeriodicIO.accelZ);

        return angleXZ*360./(2.*Math.PI);
    }



    @Override
    public void readPeriodicInputs()
    {
        updateOdometry();
        switch (driveMode)
        {
            case kDrive:
        
                if(periodicIO.fieldRelative)
                    periodicIO.chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(periodicIO.xSpeed, periodicIO.ySpeed, periodicIO.turn, gyro.getRotation2d());
                else
                    periodicIO.chassisSpeeds = new ChassisSpeeds(periodicIO.xSpeed, periodicIO.ySpeed, periodicIO.turn);
                
                periodicIO.swerveModuleStates = kinematics.toSwerveModuleStates(periodicIO.chassisSpeeds);

                SwerveDriveKinematics.desaturateWheelSpeeds(periodicIO.swerveModuleStates, Constant.MAX_DRIVE_SPEED);
                break;

            case kLockwheels:

                periodicIO.swerveModuleStates = new SwerveModuleState[4];
            
                
                periodicIO.swerveModuleStates[0] = new SwerveModuleState(0.0, Rotation2d.fromDegrees(45));
                periodicIO.swerveModuleStates[1] = new SwerveModuleState(0.0, Rotation2d.fromDegrees(135));
                periodicIO.swerveModuleStates[2] = new SwerveModuleState(0.0, Rotation2d.fromDegrees(135));
                periodicIO.swerveModuleStates[3] = new SwerveModuleState(0.0, Rotation2d.fromDegrees(45));
                break;

            case kStop:
                //No calculations to do
                break;
        }



        //FIXME is this used at all?
        //previousSwerveModuleStates = periodicIO.swerveModuleStates;

    }


    @Override
    public void writePeriodicOutputs()
    {
        if ( resetEncoders)
        {   //FIXME do we need to add a time delay to reset the encoders?
            frontLeft.stopModule();
            frontRight.stopModule();
            backLeft.stopModule();
            backRight.stopModule();

            frontLeft.resetEncoders();
            frontRight.resetEncoders();
            backLeft.resetEncoders();
            backRight.resetEncoders();
        }
        else if (driveMode == DriveMode.kStop)
        {
            frontLeft.stopModule();
            frontRight.stopModule();
            backLeft.stopModule();
            backRight.stopModule();
        }
        else 
        {
            frontLeft.setDesiredState(periodicIO.swerveModuleStates[0]);
            frontRight.setDesiredState(periodicIO.swerveModuleStates[1]);
            backLeft.setDesiredState(periodicIO.swerveModuleStates[2]);
            backRight.setDesiredState(periodicIO.swerveModuleStates[3]);
        }
        

        feedWatchdog();
    }

    public void feedWatchdog() 
    {
        frontLeft.feed();
        backLeft.feed();
        frontRight.feed();
        backRight.feed();
    }

        /**
     * Drive a "straight" distance in meters
     * 
     * @param startingPosition of the robot
     * @param velocity in meters per second (+ forward, - reverse)
     * @param distanceToDrive in meters
     * @return true when drive is complete
     */
    // public boolean driveStraight(Translation2d startingPosition, double velocity, double distanceToDrive)
    // {
    //     boolean isDone = false;
    //     double distanceDriven = odometry.getPoseMeters().getTranslation().getDistance(startingPosition);
        
    //     updateOdometry();

    //     if(Math.abs(distanceDriven) < Math.abs(distanceToDrive))
    //     {
    //         drive(velocity, 0.0, 0.0, false);
    //     }
    //     else
    //     {
    //         stopMotor();
    //         isDone = true;
    //         // System.out.println("Dist (meters) = " + distanceDriven);
    //     }

    //     return isDone;
    // }

    /**
     * Drive a "vector" distance in meters
     * 
     * @param startingPosition of the robot
     * @param velocity in meters per second (+ forward, - reverse)
     * @param distanceToDriveX in meters
     * @param distanceToDriveY in meters
     * @return true when drive is complete
     */
    // public boolean driveVector(Translation2d startingPosition, double velocity, double distanceToDriveX, double distanceToDriveY)
    // {
    //     boolean isDone = false;

    //     double distanceToDrive = Math.sqrt(distanceToDriveX * distanceToDriveX + distanceToDriveY * distanceToDriveY);

    //     double velocityX = velocity * distanceToDriveX / distanceToDrive;
    //     double velocityY = velocity * distanceToDriveY / distanceToDrive;

    //     double distanceDriven = odometry.getPoseMeters().getTranslation().getDistance(startingPosition);

    //     double distanceToNearestEndpoint = Math.min(distanceDriven, distanceToDrive - distanceDriven);
    //     double maxVelocity = velocity;
    //     double minVelocity = 0.75;

    //     if (distanceToNearestEndpoint < 1.0)
    //     {
    //         velocityX *= distanceToNearestEndpoint / 1.0 * (maxVelocity - minVelocity) + minVelocity;
    //         velocityY *= distanceToNearestEndpoint / 1.0 * (maxVelocity - minVelocity) + minVelocity;
    //         System.out.println("DRIVE SPEED" + distanceToNearestEndpoint / 1.0 * (maxVelocity - minVelocity) + minVelocity);
    //     }

    //     updateOdometry();

    //     if(Math.abs(distanceDriven) < Math.abs(distanceToDrive))
    //     {
    //         drive(velocityX, velocityY, 0.0, true);
    //     }
    //     else
    //     {
    //         stopMotor();
    //         isDone = true;
    //         // System.out.println("Dist (meters) = " + distanceDriven);
    //     }

    //     return isDone;
    // }

    /**
     * Turn to an angle in degrees
     * 
     * @param minAngularVelocity the robot can turn
     * @param maxAngularVelocity the robot can turn
     * @param desiredAngle in degrees to turn to
     * @param angleThreshold in degrees
     * @return true when turn is complete
     */
    // public boolean turnToAngle(double minAngularVelocity, double maxAngularVelocity, double desiredAngle, double angleThreshold)
    // {
    //     boolean isDone = false;
    //     // double currentAngle = odometry.getPoseMeters().getRotation().getDegrees();
    //     double currentAngle = gyro.getYaw();
    //     double angleToTurn = (desiredAngle - currentAngle) % 360;

    //     if (angleToTurn <= -180.0)
    //     {
    //         angleToTurn += 360.0;
    //     }
    //     else if (angleToTurn > 180.0)
    //     {
    //         angleToTurn -= 360.0;
    //     }

    //     System.out.println("ANGLE TO TURN: " + angleToTurn);
        
    //     updateOdometry();

    //     if(!isAtAngle(desiredAngle, angleThreshold))
    //     {
    //         //proportion of how close the speed will be to the max speed from the min speed, so it doesn't exceed the max speed
    //         double turnSpeedProportion = angleToTurn / 30.0;

    //         if (Math.abs(turnSpeedProportion) > 1.0)
    //         {
    //             turnSpeedProportion = Math.signum(turnSpeedProportion);
    //         }

    //         // Use calculateTurnRotation
    //         drive(0.0, 0.0, turnSpeedProportion * (maxAngularVelocity - minAngularVelocity) + minAngularVelocity * Math.signum(angleToTurn), true);
    //     }
    //     else
    //     {
    //         stopMotor();
    //         isDone = true;
    //         // System.out.println("Angle turned (degrees) = ");
    //     }

    //     return isDone;
    //}

    /**
     * Drive a "vector" distance in meters and rotate to desired angle
     * 
     * @param startingPosition of the robot
     * @param velocity in meters per second (+ forward, - reverse)
     * @param distanceToDriveX in meters
     * @param distanceToDriveY in meters
     * @param desiredAngle in degrees in navigatioanl position
     * @return true when drive is complete
     */
    // public boolean driveVectorAndTurnToAngle(Translation2d startingPosition, double velocity, double distanceToDriveX, double distanceToDriveY, double minAngularVelocity, double maxAngularVelocity, double desiredAngle, double angleThreshold)
    // {
    //     boolean isDone = false;

    //     double distanceToDrive = Math.sqrt(distanceToDriveX * distanceToDriveX + distanceToDriveY * distanceToDriveY);

    //     double velocityX = velocity * distanceToDriveX / distanceToDrive;
    //     double velocityY = velocity * distanceToDriveY / distanceToDrive;

    //     double distanceDriven = odometry.getPoseMeters().getTranslation().getDistance(startingPosition);

    //     double distanceToNearestEndpoint = Math.min(distanceDriven, distanceToDrive - distanceDriven);
    //     double maxVelocity = velocity;
    //     double minVelocity = 0.75;

    //     if (distanceToNearestEndpoint < 1.0)
    //     {
    //         velocityX *= distanceToNearestEndpoint / 1.0 * (maxVelocity - minVelocity) + minVelocity;
    //         velocityY *= distanceToNearestEndpoint / 1.0 * (maxVelocity - minVelocity) + minVelocity;
    //         System.out.println("DRIVE SPEED" + distanceToNearestEndpoint / 1.0 * (maxVelocity - minVelocity) + minVelocity);
    //     }

    //     updateOdometry();

    //     if(Math.abs(distanceDriven) < Math.abs(distanceToDrive) && !isAtAngle(desiredAngle, angleThreshold))
    //     {
    //         drive(velocityX, velocityY, calculateTurnRotation(minAngularVelocity, maxAngularVelocity, desiredAngle, angleThreshold), true);
    //     }
    //     else
    //     {
    //         stopMotor();
    //         isDone = true;
    //         // System.out.println("Dist (meters) = " + distanceDriven);
    //     }

    //     return isDone;
    // }

    // public double calculateTurnRotation(double minAngularVelocity, double maxAngularVelocity, double desiredAngle, double angleThreshold)
    // {
    //     // double currentAngle = odometry.getPoseMeters().getRotation().getDegrees();
    //     double currentAngle = gyro.getYaw();
    //     double angleToTurn = (desiredAngle - currentAngle) % 360;
    //     double angularVelocity;

    //     if (angleToTurn <= -180.0)
    //     {
    //         angleToTurn += 360.0;
    //     }
    //     else if (angleToTurn > 180.0)
    //     {
    //         angleToTurn -= 360.0;
    //     }

    //     System.out.println("ANGLE TO TURN: " + angleToTurn);

    //     if(!isAtAngle(desiredAngle, angleThreshold))
    //     {
    //         //proportion of how close the speed will be to the max speed from the min speed, so it doesn't exceed the max speed
    //         double turnSpeedProportion = angleToTurn / 30.0;

    //         if (Math.abs(turnSpeedProportion) > 1.0)
    //         {
    //             turnSpeedProportion = Math.signum(turnSpeedProportion);
    //         }

    //         angularVelocity = turnSpeedProportion * (maxAngularVelocity - minAngularVelocity) + minAngularVelocity * Math.signum(angleToTurn);
    //     }
    //     else
    //     {
    //         angularVelocity = 0.0;
    //         // stopMotor();
    //         // isDone = true;
    //         // System.out.println("Angle turned (degrees) = ");
    //     }

    //     return angularVelocity;
    // }

    // public boolean isAtAngle(double desiredAngle, double angleThreshold)
    // {
    //     double currentAngle = gyro.getYaw();
    //     double angleToTurn = (currentAngle - desiredAngle) % 360.0;

    //     if (angleToTurn <= -180.0)
    //     {
    //         angleToTurn += 360.0;
    //     }
    //     else if (angleToTurn > 180.0)
    //     {
    //         angleToTurn -= 360.0;
    //     }
    //     System.out.println("CURRENT ANGLE: " + currentAngle);
    //     System.out.println("DESIRED ANGLE: " + desiredAngle);

    //     return (Math.abs(angleToTurn) < angleThreshold);
    // }
}