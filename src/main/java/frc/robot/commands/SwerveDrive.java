package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class SwerveDrive extends CommandBase
{
        private final Drivetrain drivetrain;
        private Supplier<Double> speed;
        private Supplier<Double> rotate;


        public SwerveDrive(Drivetrain drivetrain, Supplier<Double> speed, Supplier<Double> rotate)
        {
        this.drivetrain = drivetrain;
        this.speed = speed;
        this.rotate = rotate;
        }

        @Override 
        public void initialize()
        {}

        @Override
        public void execute()
        {
            if(drivetrain != null)
               drivetrain.SwerveDrive(speed.get(), rotate.get());
        }

        @Override
        public void end(boolean interrupted)
        {}

        @Override
        public boolean isFinished()
        {
            if(drivetrain != null)
                return false;
            else
                return true;
        }


}