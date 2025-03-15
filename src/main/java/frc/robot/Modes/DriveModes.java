package frc.robot.Modes;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class DriveModes
{
    private static double speedFactor = 0.15;//Between -1 and 1
    public static double getSpeedFactor()
    {
        return speedFactor;
    }
    public static InstantCommand switchToTurboSpeed()
    {
        return new InstantCommand(() -> speedFactor = 0.5);//Set to 1 if necessary.
    }
    public static InstantCommand switchToRegularSpeed()
    {
        return new InstantCommand(() -> speedFactor = 0.15);
    }
}

