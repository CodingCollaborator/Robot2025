package frc.robot.Modes;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ElevatorModes implements BooleanSupplier
{
    private static String mode = "coral";

    public static String getMode()
    {
        return mode;
    }

    public static InstantCommand switchToAlgaeMode()
    {
        return new InstantCommand(() -> mode = "algae");
    }

    public static InstantCommand switchToCoralMode()
    {
        return new InstantCommand(() -> mode = "coral");
    }
    
    public boolean getAsBoolean()
    {
        return mode.equals("coral");
    }
}