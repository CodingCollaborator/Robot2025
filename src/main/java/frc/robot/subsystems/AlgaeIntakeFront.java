package frc.robot.subsystems;

import java.net.http.HttpResponse.PushPromiseHandler;
import java.util.function.BooleanSupplier;


import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.wrappers.MonitoredPIDController;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.Modes.ElevatorModes;

public class AlgaeIntakeFront
{
    private SparkMax frontAlgaeMotor;
    private SparkMaxConfig frontAlgaeMotorConfig;
    private SparkClosedLoopController closedLoopController;
    private RelativeEncoder encoder;
    private SparkMax backAlgaeMotor;
    private SparkMaxConfig backAlgaeMotorConfig;
    private static final double PUSH_OUT = 0.8;
    private static final double BRAKE = 0.0;
    private static final double PULL_IN = -0.4;
    private static final double HOLD = 0.0;
    public AlgaeIntakeFront()
    {
        frontAlgaeMotor = new SparkMax(28, MotorType.kBrushless);
        frontAlgaeMotorConfig = new SparkMaxConfig();
        encoder = frontAlgaeMotor.getEncoder();
        closedLoopController = frontAlgaeMotor.getClosedLoopController();

        frontAlgaeMotorConfig.inverted(false).idleMode(IdleMode.kBrake);  // Sets direction, as well as what to do when idle.
        frontAlgaeMotorConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);  //  Conversion factors for position and velocity of encoder.  (Set to default)

        frontAlgaeMotor.configure(frontAlgaeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }
    public ConditionalCommand pullAlgaeIn()
    {
        InstantCommand onFalse = new InstantCommand(() -> frontAlgaeMotor.set(PULL_IN));
        InstantCommand onTrue = new InstantCommand();
        BooleanSupplier supplier = new ElevatorModes();
        return new ConditionalCommand(onTrue, onFalse, supplier);
    }
    public ConditionalCommand pushAlgaeOut()
    {
        InstantCommand onFalse = new InstantCommand(() -> frontAlgaeMotor.set(PUSH_OUT));
        InstantCommand onTrue = new InstantCommand();
        BooleanSupplier supplier = new ElevatorModes();
        return new ConditionalCommand(onTrue, onFalse, supplier);
    }
    public ConditionalCommand hold()
    {
        InstantCommand onFalse = new InstantCommand(() -> frontAlgaeMotor.set(HOLD));
        InstantCommand onTrue = new InstantCommand();
        BooleanSupplier supplier = new ElevatorModes();
        return new ConditionalCommand(onTrue, onFalse, supplier);
    }
    public InstantCommand stopIntake()
    {
        return new InstantCommand(() -> frontAlgaeMotor.set(HOLD));
    }
}
