package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.Servo;

public class Climber extends SubsystemBase
{
    private TalonSRX climberMotor;
    private Servo ratchetRalph;
    private static final double PUSH_OUT = -0.03;
    private static final double BRAKE = 0.0;
    private static final double PULL_IN = 0.06;
    public Climber()
    {
        climberMotor = new TalonSRX(14);
        ratchetRalph = new Servo(0);
    }
    public SequentialCommandGroup letOut()//May have to crank in first.
    {
        return new InstantCommand(() -> {
            ratchetRalph.setAngle(90);
        }).andThen(new WaitCommand(1.0)).andThen(() -> {
            climberMotor.set(TalonSRXControlMode.PercentOutput, PUSH_OUT);
        }).andThen(new WaitCommand(1.0)).andThen(() -> {
            climberMotor.set(TalonSRXControlMode.PercentOutput, BRAKE);
        });
    }
    public SequentialCommandGroup pullIn()
    {
        return new InstantCommand(() -> {
            ratchetRalph.setAngle(90);
        }).andThen(new WaitCommand(1.0)).andThen(() -> {
            climberMotor.set(TalonSRXControlMode.PercentOutput, PULL_IN);
        }).andThen(new WaitCommand(1.0)).andThen(() -> {
            climberMotor.set(TalonSRXControlMode.PercentOutput, BRAKE);
        });
    }

}
