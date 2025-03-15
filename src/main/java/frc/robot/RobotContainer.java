// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj.Encoder;
import autos.AutoRoutines;
import frc.robot.Modes.ElevatorModes;
import frc.robot.Modes.DriveModes;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.VideoCamera;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class RobotContainer {

    private final Telemetry logger = new Telemetry(Constants.MaxSpeed);
    //private static VideoCamera camera;
    private final CommandXboxController joystick0 = new CommandXboxController(0); //#This Joystick Controls The Driving(Will Also Control Elevator)
    private final CommandXboxController joystick1 = new CommandXboxController(1);//#This Joystick Controls The Intake System


    private int alliance; //for Auto

    public RobotContainer(int all) //Is there a way to set this at the drive station.
    {
        CameraServer.startAutomaticCapture();
        configureBindings(all);
    }

    private void configureBindings(int all) {
        this.alliance=all;
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        Constants.drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            Constants.drivetrain.applyRequest(() ->
                Constants.drive.withVelocityX(-joystick0.getLeftY() * Constants.MaxSpeed * DriveModes.getSpeedFactor() * Math.pow(-1, alliance)) // Drive forward with negative Y (forward) #SpeedHalved
                    .withVelocityY(-joystick0.getLeftX() * Constants.MaxSpeed * DriveModes.getSpeedFactor() * Math.pow(-1, alliance)) // Drive left with negative X (left) #SpeedHalved
                    .withRotationalRate(-joystick0.getRightX() * Constants.MaxAngularRate * 0.5) // Drive counterclockwise with negative X (left) #SpeedHalved
            )                                                                              //Change if Possible(1.00)
            //# When Command Is Executed, Request Is Applied Based on What Joystick Buttons Are Being Pressed

            //VERY IMPORTANT, REMEMBER TO SET TO FULL SPEED FOR COMPETITION.

        );
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        //DRIVE SYSTEM CONTROLS
        joystick0.back().and(joystick0.y()).whileTrue(Constants.drivetrain.sysIdDynamic(Direction.kForward));
        joystick0.back().and(joystick0.x()).whileTrue(Constants.drivetrain.sysIdDynamic(Direction.kReverse));
        joystick0.start().and(joystick0.y()).whileTrue(Constants.drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick0.start().and(joystick0.x()).whileTrue(Constants.drivetrain.sysIdQuasistatic(Direction.kReverse));
        joystick0.leftBumper().onTrue(Constants.drivetrain.runOnce(() -> Constants.drivetrain.seedFieldCentric()));
        joystick0.povUp().onTrue(DriveModes.switchToTurboSpeed());
        joystick0.povDown().onTrue(DriveModes.switchToRegularSpeed());
        //Drive System Controls

        //Climber Controls
        //joystick0.a().onTrue(Constants.climber.letOut());
        //joystick0.b().onTrue(Constants.climber.pullIn());
        //Climber Controls

        //CORAL MODE CONTROLS
        joystick1.a().onTrue(Constants.elevator.setCoralHeight(11.0));             //  LevelOne/Reset/!Feeder!
        joystick1.a().onTrue(Constants.flipper.setDesiredCoralAngle(7.85));//FEEDER ANGLE - TUNE

        joystick1.x().onTrue(Constants.elevator.setCoralHeight(50));            //  LevelTwo
        joystick1.x().onTrue(Constants.flipper.setDesiredCoralAngle(-14.5));//LEVEL TWO ANGLE

        joystick1.y().onTrue(Constants.elevator.setCoralHeight(115));        //  LevelThree
        joystick1.y().onTrue(Constants.flipper.setDesiredCoralAngle(-14.5));//SAME AS LAST ANGLE

        joystick1.b().onTrue(Constants.elevator.setCoralHeight(235));           //  LevelFour
        joystick1.b().onTrue(Constants.flipper.setDesiredCoralAngle(-13.4));//LEVEL OUR ANGLE 

        joystick1.rightStick().onTrue(Constants.elevator.setCoralHeight(8));
        joystick1.rightStick().onTrue(Constants.flipper.setDesiredCoralAngle(0));//TROUGH ANGLE - TUNE

        joystick1.leftBumper().whileTrue(Constants.coralIntake.pullCoralIn()).onFalse(Constants.coralIntake.hold());//On true, button pressed, calls method.
        joystick1.rightBumper().whileTrue(Constants.coralIntake.pushCoralOut()).onFalse(Constants.coralIntake.stopIntake());
        //CORAL MODE CONTROLS
       
        //SWITCH MODES
        joystick1.povUp().onTrue(ElevatorModes.switchToAlgaeMode());
        joystick1.povDown().onTrue(ElevatorModes.switchToCoralMode());
        //SWITCH MODES

        //ALGAE MODE CONTROLS
        joystick1.a().onTrue(Constants.elevator.setAlgaeHeight(0));//PROCESSOR HEIGHT - TUNE
        joystick1.a().onTrue(Constants.flipper.setDesiredAlgaeAngle(0));//PROCESSOR ANGLE - TUNE

        joystick1.x().onTrue(Constants.elevator.setAlgaeHeight(20));//  LEVEL TWO HEIGHT
        joystick1.x().onTrue(Constants.flipper.setDesiredAlgaeAngle(-9.5));//LEVEL TWO ANGLE

        joystick1.y().onTrue(Constants.elevator.setAlgaeHeight(64));//  LEVEL THREE HEIGHT
        joystick1.y().onTrue(Constants.flipper.setDesiredAlgaeAngle(-11.5));////LEVEL THREE ANGLE

        joystick1.b().onTrue(Constants.elevator.setAlgaeHeight(220));//BARGE SHOT HEIGHT - TUNE
        joystick1.b().onTrue(Constants.flipper.setDesiredAlgaeAngle(-9.5));//BARGE SHOT ANGLE - TUNE

        joystick1.leftBumper().whileTrue(Constants.frontAlgaeIntake.pullAlgaeIn()).onFalse(Constants.frontAlgaeIntake.hold());//On true, button pressed, calls method.
        joystick1.rightBumper().whileTrue(Constants.frontAlgaeIntake.pushAlgaeOut()).onFalse(Constants.frontAlgaeIntake.stopIntake());
        joystick1.leftBumper().whileTrue(Constants.backAlgaeIntake.pullAlgaeIn()).onFalse(Constants.backAlgaeIntake.hold());//On true, button pressed, calls method.
        joystick1.rightBumper().whileTrue(Constants.backAlgaeIntake.pushAlgaeOut()).onFalse(Constants.backAlgaeIntake.stopIntake());
        //ALGAE MODE CONTROLS


        Constants.drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() 
    {
        return AutoRoutines.Taxi.routine(Constants.drivetrain, alliance);
    }
}