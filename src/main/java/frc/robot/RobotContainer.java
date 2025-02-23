// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Wrist;

public class RobotContainer {
  // Some constants needed for swerve
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond);

  // Slew rate limiters smooth out drivetrain. 
  private SlewRateLimiter xLimiter = new SlewRateLimiter(kMoveSlewRateLimiter);
  private SlewRateLimiter yLimiter = new SlewRateLimiter(kMoveSlewRateLimiter);
  private SlewRateLimiter rotLimiter = new SlewRateLimiter(kRotateSlewRateLimiter);

  // Set up controllers
  private final CommandXboxController DriverController = new CommandXboxController(kDriverController);
  private final CommandXboxController ManipulatorController = new CommandXboxController(kManipulatorController);
  
  // Setting up bindings for necessary control of the swerve drive platform
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  Camera camera = new Camera();
  Elevator elevator = new Elevator();
  Intake intake = new Intake();
  Pivot pivot = new Pivot();
  Pneumatics pneumatics = new Pneumatics();
  Wrist wrist = new Wrist();

  // Swerve Requests
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop

  private final SwerveRequest.RobotCentric driveTrack = new SwerveRequest.RobotCentric()
      .withDeadband(MaxSpeed * 0.02).withRotationalDeadband(MaxAngularRate * 0.02) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  // For Auto
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {

    /*
    NamedCommands.registerCommand("Track Left", drivetrain.runEnd(
      () -> PPHolonomicDriveController.overrideXYFeedback(
        () -> -camera.MoveReefY(kTrackYOffsetLeft),
        () -> -camera.MoveReefX(kTrackXOffsetLeft)),
      () -> PPHolonomicDriveController.clearFeedbackOverrides()));

    NamedCommands.registerCommand("Track Right", drivetrain.runEnd(
      () -> PPHolonomicDriveController.overrideXYFeedback(
        () -> -camera.MoveReefY(kTrackYOffsetRight),
        () -> -camera.MoveReefX(kTrackXOffsetRight)),
      () -> PPHolonomicDriveController.clearFeedbackOverrides()));
    */

    NamedCommands.registerCommand("Track Left", drivetrain.applyRequest(
      () -> driveTrack.withVelocityX(camera.MoveReefY(kTrackYOffsetLeft) + xLimiter.calculate(-MathUtil.applyDeadband(DriverController.getLeftY(), 0.05) * MaxSpeed)) // Drive forward with negative Y (forward)
                      .withVelocityY(camera.MoveReefX(kTrackXOffsetLeft) + yLimiter.calculate(-MathUtil.applyDeadband(DriverController.getLeftX(), 0.05) * MaxSpeed)) // Drive left with negative X (left)
                      .withRotationalRate(-camera.MoveReefRot(kTrackRotOffsetLeft) + rotLimiter.calculate(-MathUtil.applyDeadband(DriverController.getRightX(), .1) * MaxAngularRate))));

    NamedCommands.registerCommand("Track Right", drivetrain.applyRequest(
      () -> driveTrack.withVelocityX(camera.MoveReefY(kTrackYOffsetRight) + xLimiter.calculate(-MathUtil.applyDeadband(DriverController.getLeftY(), 0.05) * MaxSpeed)) // Drive forward with negative Y (forward)
                      .withVelocityY(camera.MoveReefX(kTrackXOffsetRight) + yLimiter.calculate(-MathUtil.applyDeadband(DriverController.getLeftX(), 0.05) * MaxSpeed)) // Drive left with negative X (left)
                      .withRotationalRate(-camera.MoveReefRot(kTrackRotOffsetRight) + rotLimiter.calculate(-MathUtil.applyDeadband(DriverController.getRightX(), .1) * MaxAngularRate))));


    NamedCommands.registerCommand("Run Intake With Beam Break", intake.runEnd(
      () -> intake.RunIntakeWithBeam(),
      () -> intake.Stop()));

    NamedCommands.registerCommand("Run Intake", intake.runEnd(
      () -> intake.RunIntake(),
      () -> intake.Stop()));

    NamedCommands.registerCommand("L2", intake.run(
      () -> intake.Open(false))
      .alongWith(new WaitCommand(0.6)
        .andThen(elevator.run(
        () -> elevator.Set(kElevatorL2))))
      .alongWith(pivot.run(
      () -> pivot.Set(kPivotL2)))
      .alongWith(wrist.run(
      () -> wrist.Set(kWristL2))));

    NamedCommands.registerCommand("L3", intake.run(
      () -> intake.Open(false))
      .alongWith(elevator.run(
      () -> elevator.Set(kElevatorL3)))
      .alongWith(new WaitCommand(0.75)
        .andThen(pivot.run(
        () -> pivot.Set(kPivotL3))))
      .alongWith(wrist.run(
      () -> wrist.Set(kWristL3))));

    NamedCommands.registerCommand("L4", intake.run(
      () -> intake.Open(false))
      .alongWith(elevator.run(
      () -> elevator.Set(kElevatorL4)))
      .alongWith(new WaitCommand(1)
        .andThen(pivot.run(
        () -> pivot.Set(kPivotL4))))
      .alongWith(wrist.run(
      () -> wrist.Set(kWristL4))));

    NamedCommands.registerCommand("Intake", intake.runOnce(
        () -> intake.Open(false))
      .alongWith(new WaitCommand(0.8)
        .andThen(elevator.runOnce(
        () -> elevator.Set(kElevatorIntake))))
      .alongWith(pivot.runOnce(
      () -> pivot.Set(kPivotIntake)))
      .alongWith(new WaitCommand(0.8)
        .andThen(wrist.runOnce(
        () -> wrist.Set(kWristIntake)))));

    // For Auto
    autoChooser = AutoBuilder.buildAutoChooser("test");

    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
  }

  private void configureBindings() {

    // Drivetrain will execute this command periodically
    drivetrain.setDefaultCommand(
      drivetrain.applyRequest(
        () -> drive.withVelocityX(xLimiter.calculate(MathUtil.applyDeadband(-DriverController.getLeftY(), 0.02) * MaxSpeed)) // Drive forward with negative Y (forward)
                   .withVelocityY(yLimiter.calculate(MathUtil.applyDeadband(-DriverController.getLeftX(), 0.02) * MaxSpeed)) // Drive left with negative X (left)
                   .withRotationalRate(rotLimiter.calculate(MathUtil.applyDeadband(-DriverController.getRightX(), 0.02) * MaxAngularRate)) // Drive counterclockwise with negative X (left)
      ));

    // ** Driver Control **

    // Track Left
    DriverController.leftBumper().whileTrue(drivetrain.applyRequest(
      () -> driveTrack.withVelocityX(camera.MoveReefY(kTrackYOffsetLeft) + xLimiter.calculate(-MathUtil.applyDeadband(DriverController.getLeftY(), 0.05) * MaxSpeed)) // Drive forward with negative Y (forward)
                      .withVelocityY(camera.MoveReefX(kTrackXOffsetLeft) + yLimiter.calculate(-MathUtil.applyDeadband(DriverController.getLeftX(), 0.05) * MaxSpeed)) // Drive left with negative X (left)
                      .withRotationalRate(-camera.MoveReefRot(kTrackRotOffsetLeft) + rotLimiter.calculate(-MathUtil.applyDeadband(DriverController.getRightX(), 0.05) * MaxAngularRate))));

    // Track Center
    DriverController.leftTrigger().whileTrue(drivetrain.applyRequest(
      () -> driveTrack.withVelocityX(camera.MoveReefY(kTrackYOffsetCenter) + xLimiter.calculate(-MathUtil.applyDeadband(DriverController.getLeftY(), 0.05) * MaxSpeed)) // Drive forward with negative Y (forward)
                      .withVelocityY(camera.MoveReefX(kTrackXOffsetCenter) + yLimiter.calculate(-MathUtil.applyDeadband(DriverController.getLeftX(), 0.05) * MaxSpeed)) // Drive left with negative X (left)
                      .withRotationalRate(-camera.MoveReefRot(kTrackRotOffsetCenter) + rotLimiter.calculate(-MathUtil.applyDeadband(DriverController.getRightX(), 0.05) * MaxAngularRate))));

    // Track Right
    DriverController.rightBumper().whileTrue(drivetrain.applyRequest(
      () -> driveTrack.withVelocityX(camera.MoveReefY(kTrackYOffsetRight) + xLimiter.calculate(-MathUtil.applyDeadband(DriverController.getLeftY(), 0.05) * MaxSpeed)) // Drive forward with negative Y (forward)
                      .withVelocityY(camera.MoveReefX(kTrackXOffsetRight) + yLimiter.calculate(-MathUtil.applyDeadband(DriverController.getLeftX(), 0.05) * MaxSpeed)) // Drive left with negative X (left)
                      .withRotationalRate(-camera.MoveReefRot(kTrackRotOffsetRight) + rotLimiter.calculate(-MathUtil.applyDeadband(DriverController.getRightX(), 0.05) * MaxAngularRate))));

    // Run Intake
    DriverController.rightTrigger().whileTrue(intake.runEnd(
      () -> intake.RunIntakeWithBeam(),
      () -> intake.Stop()));

    // Eject Algae
    DriverController.y().whileTrue(intake.runEnd(
      () -> intake.EjectAlgae(),
      () -> intake.Stop()));

    // End Climb
    DriverController.b().onTrue(intake.runOnce(
      () -> intake.Open(false))
      .alongWith(elevator.runOnce(
      () -> elevator.Set(kElevatorClimb)))
      .alongWith(pivot.runOnce(
      () -> pivot.Set(kPivotClimbEnd)))
      .alongWith(wrist.runOnce(
      () -> wrist.Set(kWristClimb))));

    // Stop
    DriverController.button(8).whileTrue(drivetrain.applyRequest(
      () -> brake));

    // Reset Field Heading
    DriverController.button(7).onTrue(drivetrain.runOnce(
      () -> drivetrain.seedFieldCentric()));

    // ** Manipulator Control **
    
    // Run Intake
    ManipulatorController.rightTrigger().whileTrue(intake.runEnd(
      () -> intake.RunIntake(),
      () -> intake.Stop()));

    // Reverse Intake
    ManipulatorController.leftTrigger().whileTrue(intake.runEnd(
      () -> intake.RunIntakeReverse(),
      () -> intake.Stop()));

    // L1
    ManipulatorController.rightBumper().onTrue(intake.runOnce(
      () -> intake.Open(false))
      .alongWith(elevator.runOnce(
      () -> elevator.Set(kElevatorL1)))
      .alongWith(pivot.runOnce(
      () -> pivot.Set(kPivotL1)))
      .alongWith(wrist.runOnce(
      () -> wrist.Set(kWristL1))));

    // L2
    ManipulatorController.a().onTrue(intake.runOnce(
      () -> intake.Open(false))
      .alongWith(new WaitCommand(0.6)
        .andThen(elevator.runOnce(
        () -> elevator.Set(kElevatorL2))))
      .alongWith(pivot.runOnce(
      () -> pivot.Set(kPivotL2)))
      .alongWith(wrist.runOnce(
      () -> wrist.Set(kWristL2))));

    // L3
    ManipulatorController.x().onTrue(intake.runOnce(
      () -> intake.Open(false))
      .alongWith(elevator.runOnce(
      () -> elevator.Set(kElevatorL3)))
      .alongWith(new WaitCommand(0.75)
        .andThen(pivot.runOnce(
        () -> pivot.Set(kPivotL3))))
      .alongWith(wrist.runOnce(
      () -> wrist.Set(kWristL3))));

    // L4
    ManipulatorController.y().onTrue(intake.runOnce(
      () -> intake.Open(false))
      .alongWith(elevator.runOnce(
      () -> elevator.Set(kElevatorL4)))
      .alongWith(new WaitCommand(1)
        .andThen(pivot.runOnce(
        () -> pivot.Set(kPivotL4))))
      .alongWith(wrist.runOnce(
      () -> wrist.Set(kWristL4))));

    // Home
    ManipulatorController.button(9).onTrue(intake.runOnce(
      () -> intake.Open(false))
      .alongWith(elevator.runOnce(
      () -> elevator.Set(kElevatorHome)))
      .alongWith(pivot.runOnce(
      () -> pivot.Set(kPivotHome)))
      .alongWith(wrist.runOnce(
      () -> wrist.Set(kWristHome))));

    // Alage Home
    ManipulatorController.button(7).onTrue(intake.runOnce(
      () -> intake.Open(false))
      .alongWith(elevator.runOnce(
      () -> elevator.Set(kElevatorHome)))
      .alongWith(pivot.runOnce(
      () -> pivot.Set(kPivotAlgaeHome)))
      .alongWith(wrist.runOnce(
      () -> wrist.Set(kWristAlgaeHome))));

    // Intake
    ManipulatorController.b().onTrue(intake.runOnce(
      () -> intake.Open(false))
      .alongWith(new WaitCommand(0.8)
        .andThen(elevator.runOnce(
        () -> elevator.Set(kElevatorIntake))))
      .alongWith(pivot.runOnce(
      () -> pivot.Set(kPivotIntake)))
      .alongWith(new WaitCommand(0.8)
        .andThen(wrist.runOnce(
        () -> wrist.Set(kWristIntake)))));

    // Start Climb
    ManipulatorController.leftBumper().onTrue(intake.runOnce(
      () -> intake.Open(false))
      .alongWith(elevator.runOnce(
      () -> elevator.Set(kElevatorClimb)))
      .alongWith(pivot.runOnce(
      () -> pivot.Set(kPivotClimbEnd)))
      .alongWith(wrist.runOnce(
      () -> wrist.Set(kWristClimb))));

    // Barge
    ManipulatorController.povLeft().onTrue(intake.runOnce(
      () -> intake.Open(true))
      .alongWith(elevator.runOnce(
      () -> elevator.Set(kElevatorBarge)))
      .alongWith(pivot.runOnce(
      () -> pivot.Set(kPivotL1)))
      .alongWith(wrist.runOnce(
      () -> wrist.Set(kWristBarge))));

    // Processor
    ManipulatorController.button(10).onTrue(intake.runOnce(
      () -> intake.Open(true))
      .alongWith(elevator.runOnce(
      () -> elevator.Set(kElevatorHome)))
      .alongWith(pivot.runOnce(
      () -> pivot.Set(kPivotL1)))
      .alongWith(wrist.runOnce(
      () -> wrist.Set(kWristProcessor))));
      
    // Ground Algae
    ManipulatorController.povRight().onTrue(intake.runOnce(
      () -> intake.Open(true))
      .alongWith(elevator.runOnce(
      () -> elevator.Set(kElevatorGround)))
      .alongWith(pivot.runOnce(
      () -> pivot.Set(kPivotGround)))
      .alongWith(wrist.runOnce(
      () -> wrist.Set(kWristGround))));

    // Bottom Algae
    ManipulatorController.povDown().onTrue(intake.runOnce(
      () -> intake.Open(true))
      .alongWith(elevator.runOnce(
      () -> elevator.Set(kElevatorBottomAlge)))
      .alongWith(pivot.runOnce(
      () -> pivot.Set(kPivotL2)))
      .alongWith(wrist.runOnce(
      () -> wrist.Set(kWristBottomAlgae))));

    // Top Algae
    ManipulatorController.povUp().onTrue(intake.runOnce(
      () -> intake.Open(true))
      .alongWith(elevator.runOnce(
      () -> elevator.Set(kElevatorTopAlge)))
      .alongWith(pivot.runOnce(
      () -> pivot.Set(kPivotL3)))
      .alongWith(wrist.runOnce(
      () -> wrist.Set(kWristTopAlgae))));

    // Manual Control
    ManipulatorController.button(8).whileTrue(intake.run(
      () -> intake.Open(ManipulatorController.rightBumper().getAsBoolean()))
      .alongWith(elevator.runEnd(
      () -> elevator.ManualMovement(MathUtil.applyDeadband(ManipulatorController.getLeftY(), 0.2), 1, false),
      () -> elevator.Stop())
      .alongWith(pivot.runEnd(
      () -> pivot.ManualMovement(MathUtil.applyDeadband(ManipulatorController.getLeftX(), 0.2), 1, false),
      () -> pivot.Stop())
      .alongWith(wrist.runEnd(
      () -> wrist.ManualMovement(MathUtil.applyDeadband(ManipulatorController.getRightX() * 0.4, 0.2), 3, false),
      () -> wrist.Stop())))));
  }

  public Command getAutonomousCommand() {

    // Return an auto from pathplanner
    return autoChooser.getSelected();
  }
}
