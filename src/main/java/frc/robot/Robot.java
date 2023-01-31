// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.invoke.MethodHandles;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Subsystem4237;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot
{
    // This string gets the full name of the class, including the package name
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();

    // *** STATIC INITIALIZATION BLOCK ***
    // This block of code is run first when the class is loaded
    static
    {
        System.out.println("Loading: " + fullClassName);
    }

    
    // *** CLASS AND INSTANCE VARIABLES ***
    private final RobotContainer robotContainer = new RobotContainer();
    private Command autonomousCommand = null;
    // private TestMode testMode = null;
    

    /** 
     * This class determines the actions of the robot, depending on the mode and state of the robot.
     * Use the default modifier so that new objects can only be constructed in the same package.
     */
    Robot()
    {}


    /**
     * This method runs when the robot first starts up.
     */
    @Override
    public void robotInit()
    {
        System.out.println("Robot Init");
    }


    /**
     * This method runs periodically (20ms) while the robot is powered on.
     */
    @Override
    public void robotPeriodic()
    {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
       
        Subsystem4237.readInputs();
        CommandScheduler.getInstance().run();
        Subsystem4237.writeOutputs();
    }

    /**
     * This method runs one time when the robot enters disabled mode.
     */
    @Override
    public void disabledInit()
    {
        System.out.println("Disabled Mode");
    }

    /**
     * This method runs periodically (20ms) during disabled mode.
     */
    @Override
    public void disabledPeriodic()
    {}

    /**
     * This method runs one time when the robot exits disabled mode.
     */
    @Override
    public void disabledExit()
    {}

    /**
     * This method runs one time when the robot enters autonomous mode.
     */
    @Override
    public void autonomousInit()
    {
        System.out.println("Autonomous Mode");

        autonomousCommand = robotContainer.getAutonomousCommand();

        // Schedule the autonomous command
        if (autonomousCommand != null)
        {
            autonomousCommand.schedule();
        }
    }

    /**
     * This method runs periodically (20ms) during autonomous mode.
     */
    @Override
    public void autonomousPeriodic()
    {}

        /**
     * This method runs one time when the robot exits autonomous mode.
     */
    @Override
    public void autonomousExit()
    {}

    /**
     * This method runs one time when the robot enters teleop mode.
     */
    @Override
    public void teleopInit()
    {
        System.out.println("Teleop Mode");

        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null)
        {
            autonomousCommand.cancel();
            autonomousCommand = null;
        }
    }

    /**
     * This method runs periodically (20ms) during teleop mode.
     */
    @Override
    public void teleopPeriodic()
    {}

    /**
     * This method runs one time when the robot exits teleop mode.
     */
    @Override
    public void teleopExit()
    {}

    /**
     * This method runs one time when the robot enters test mode.
     */
    @Override
    public void testInit()
    {
        System.out.println("Test Mode");

        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();

        // // Create a TestMode object to test one team members code.
        // testMode = new TestMode(robotContainer);

        // testMode.init();
    }

    /**
     * This method runs periodically (20ms) during test mode.
     */
    @Override
    public void testPeriodic()
    {
        // testMode.periodic();
    }

    /**
     * This method runs one time when the robot exits test mode.
     */
    @Override
    public void testExit()
    {
        // testMode.exit();

        // // Set the TestMode object to null so that garbage collection will remove the object.
        // testMode = null;
    }

    /**
     * This method runs one time when the robot enters simulation mode.
     */
    @Override
    public void simulationInit()
    {
        System.out.println("Simulation Mode");
    }

    /**
     * This method runs periodically (20ms) during simulation mode.
     */
    @Override
    public void simulationPeriodic()
    {}
}
