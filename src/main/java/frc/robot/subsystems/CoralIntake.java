package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Modes.ElevatorModes;

public class CoralIntake extends SubsystemBase {

    private TalonSRX coralIntake;
    private static final double PUSH_OUT = -1;
    private static final double BRAKE = 0.0;
    private static final double PULL_IN = 0.5;
    private static final double HOLD = 0.20;
    //VERY IMPORTANT, REMEMBER TO SET TO FULL SPEED FOR COMPETITION.

    public CoralIntake() {
        coralIntake = new TalonSRX(15);
    }

    public ConditionalCommand pullCoralIn() 
    {
        InstantCommand onTrue = new InstantCommand(() -> coralIntake.set(TalonSRXControlMode.PercentOutput, PULL_IN));//# Starting At 5% Power
        InstantCommand onFalse = new InstantCommand();
        BooleanSupplier supplier = new ElevatorModes();
        return new ConditionalCommand(onTrue, onFalse, supplier);
    }

    public ConditionalCommand pushCoralOut() {
        
        InstantCommand onTrue = new InstantCommand(() -> coralIntake.set(TalonSRXControlMode.PercentOutput, PUSH_OUT));//# Starting At 5% Power
        InstantCommand onFalse = new InstantCommand();
        BooleanSupplier supplier = new ElevatorModes();
        return new ConditionalCommand(onTrue, onFalse, supplier);
    }

    public ConditionalCommand hold() {
        InstantCommand onTrue = new InstantCommand(() -> coralIntake.set(TalonSRXControlMode.PercentOutput, HOLD));//# Starting At 5% Power
        InstantCommand onFalse = new InstantCommand(() -> coralIntake.set(TalonSRXControlMode.PercentOutput, BRAKE));
        BooleanSupplier supplier = new ElevatorModes();
        return new ConditionalCommand(onTrue, onFalse, supplier);
    }

    public InstantCommand stopIntake() {
        return new InstantCommand(() -> coralIntake.set(TalonSRXControlMode.PercentOutput, BRAKE));//Stop
    }

}