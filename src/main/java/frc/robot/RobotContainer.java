package frc.robot;

import java.io.IOException;
import java.lang.invoke.MethodHandles;
import java.nio.file.Files;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import frc.constants.Port;
import frc.controls.DriverController;
import frc.drivetrain.Drivetrain;
import frc.shuffleboard.AutonomousTab;
import frc.shuffleboard.DriverControllerTab;


public final class RobotContainer 
{
    //FIXME write code for joystickDriveCommand
    // private void configDefaultSubsystemCommands()
    // {
    //     Drivetrain.setDefaultCommand(Drivetrain.joystickDriveCommand() );
    // }

    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();
  

    // *** STATIC INITIALIZATION BLOCK ***
    // This block of code is run first when the class is loaded

    
    static
    {
        System.out.println("Loading: " + fullClassName);
    }

    
    // *** INCLUDED ROBOT OBJECTS ***
    // Switch objects to true that you need to use
    private static boolean useFullRobot               = false;
    
    private static boolean useDrivetrain              = true;
   
    private static boolean useDriverController        = true;
    private static boolean useAutonomousTab           = true;

   
    // *** ROBOT OBJECT DECLARATION ***
    public static final Drivetrain DRIVETRAIN;
    public static final DriverController DRIVER_CONTROLLER;
    public static final DriverControllerTab DRIVER_CONTROLLER_TAB;
    public static final AutonomousTab AUTONOMOUS_TAB;
    public static final PowerDistribution PDH;
    private static final Accelerometer accelerometer = new BuiltInAccelerometer(Accelerometer.Range.k2G);

    // *** ROBOT OBJECT INSTANTIATION ***
    static
    {
        //// start get roboRIO comment
/*
roboRIO dashboard reads:
Programmers' Tub 1

prints from here:
The roboRIO comment is >PRETTY_HOSTNAME="Programmers' Tub 1"
<
*/
        final Path commentPath = Path.of("/etc/machine-info");
        try {  
        // var temp = System.currentTimeMillis() + "," + (Runtime.getRuntime().totalMemory() - Runtime.getRuntime().freeMemory() + "\n");
        // Files.writeString(memoryLog, temp, StandardCharsets.UTF_8, StandardOpenOption.CREATE, StandardOpenOption.APPEND);
        var comment = Files.readString(commentPath);
        System.out.println("The roborRIO comment is >" + comment + "<");
        } catch (IOException e) {
        // Couldn't read the file -- handle it how you want
        System.out.println(e);
        }
        //// end get roboRIO comment

    
        final boolean isCompetitionRobot = true; //competitionRobotFlag.get();
        String robot = "";
        if(isCompetitionRobot)
            robot = "**   Competition Robot  Competition Robot   **";
        else
            robot = "**    Test Robot  Test Robot  Test Robot    **";
        System.out.println("\n\n**********************************************");
        System.out.println(robot);
        System.out.println(robot);
        System.out.println(robot);
        System.out.println("**********************************************\n\n");

       
        // final int CLIMBER_STAGE_ONE_LEADER_PORT = isCompetitionRobot ? Port.Motor.CLIMBER_STAGE_ONE_LEADER : Port.MotorTesting.CLIMBER_STAGE_ONE_LEADER_TEST;
        // final int CLIMBER_STAGE_TWO_LEADER_PORT  = isCompetitionRobot ? Port.Motor.CLIMBER_STAGE_TWO_LEADER : Port.MotorTesting.CLIMBER_STAGE_TWO_LEADER_TEST;

        DRIVETRAIN = useFullRobot || useDrivetrain ? new Drivetrain(Port.DrivetrainSetup.DRIVETRAIN_DATA, accelerometer) : null;
       
      
        DRIVER_CONTROLLER = useFullRobot || useDriverController ? new DriverController(Port.Controller.DRIVER) : null;
        DRIVER_CONTROLLER_TAB = useFullRobot || useDriverController ? new DriverControllerTab() : null;
        AUTONOMOUS_TAB = useFullRobot || useAutonomousTab ? new AutonomousTab() : null;
      
        PDH = new PowerDistribution(Port.Sensor.PDH_CAN_ID, ModuleType.kRev);
    }


    // *** CLASS CONSTRUCTOR ***
    private RobotContainer()
    {
        throw new UnsupportedOperationException("This is a utility class!");
    }


    // *** CLASS & INSTANCE METHODS ***
    public static void runMeFirst()
    {

    }
        
}
