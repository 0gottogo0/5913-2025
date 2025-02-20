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
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
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
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  Arm arm = new Arm();
  Camera camera = new Camera();
  Elevator elevator = new Elevator();
  Intake claw = new Intake();
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

    // For Auto
    autoChooser = AutoBuilder.buildAutoChooser("test");

    NamedCommands.registerCommand("Track Left", drivetrain.applyRequest(
      () -> driveTrack.withVelocityX(camera.MoveReefX(kTrackDistance) + xLimiter.calculate(-MathUtil.applyDeadband(DriverController.getLeftY(), 0.05) * MaxSpeed)) // Drive forward with negative Y (forward)
                      .withVelocityY(camera.MoveReefY(kTrackOffsetLeft) + yLimiter.calculate(-MathUtil.applyDeadband(DriverController.getLeftX(), 0.05) * MaxSpeed)) // Drive left with negative X (left)
                      .withRotationalRate(rotLimiter.calculate(-MathUtil.applyDeadband(DriverController.getRightX(), .1) * MaxAngularRate))));

    NamedCommands.registerCommand("Track Right", drivetrain.runEnd(
      () -> PPHolonomicDriveController.overrideXYFeedback(null, null),
      () -> PPHolonomicDriveController.clearFeedbackOverrides()));
    
    /*
    drivetrain.applyRequest(
      () -> driveTrack.withVelocityX(camera.MoveReefX(kTrackDistance) + xLimiter.calculate(-MathUtil.applyDeadband(DriverController.getLeftY(), 0.05) * MaxSpeed)) // Drive forward with negative Y (forward)
                      .withVelocityY(camera.MoveReefY(kTrackOffsetRight) + yLimiter.calculate(-MathUtil.applyDeadband(DriverController.getLeftX(), 0.05) * MaxSpeed)) // Drive left with negative X (left)
                      .withRotationalRate(rotLimiter.calculate(-MathUtil.applyDeadband(DriverController.getRightX(), .1) * MaxAngularRate))));
    */

    NamedCommands.registerCommand("Intake With Beam Break", claw.runEnd(
      () -> claw.RunIntakeWithBeam(),
      () -> claw.Stop()));

    NamedCommands.registerCommand("Intake Raw", claw.runEnd(
      () -> claw.RunIntake(),
      () -> claw.Stop()));

    NamedCommands.registerCommand("L2", arm.runOnce(
      () -> arm.Set(kArmL2))
      .alongWith(claw.runOnce(
      () -> claw.Open(false)))
      .alongWith(new WaitCommand(0.6)
        .andThen(elevator.runOnce(
        () -> elevator.Set(kElevatorL2))))
      .alongWith(pivot.runOnce(
      () -> pivot.Set(kPivotL2)))
      .alongWith(wrist.runOnce(
      () -> wrist.Set(kWristL2))));

    NamedCommands.registerCommand("L3", arm.runOnce(
      () -> arm.Set(kArmL3))
      .alongWith(claw.runOnce(
      () -> claw.Open(false)))
      .alongWith(elevator.runOnce(
      () -> elevator.Set(kElevatorL3)))
      .alongWith(new WaitCommand(0.75)
        .andThen(pivot.runOnce(
        () -> pivot.Set(kPivotL3))))
      .alongWith(wrist.runOnce(
      () -> wrist.Set(kWristL3))));

    NamedCommands.registerCommand("L4", arm.runOnce(
      () -> arm.Set(kArmL4))
      .alongWith(claw.runOnce(
      () -> claw.Open(false)))
      .alongWith(elevator.runOnce(
      () -> elevator.Set(kElevatorL4)))
      .alongWith(new WaitCommand(1)
        .andThen(pivot.runOnce(
        () -> pivot.Set(kPivotL4))))
      .alongWith(wrist.runOnce(
      () -> wrist.Set(kWristL4))));

    NamedCommands.registerCommand("Intake", new WaitCommand(0.8)
        .andThen(arm.runOnce(
        () -> arm.Set(kArmIntake)))
      .alongWith(new WaitCommand(0.8)
        .andThen(claw.runOnce(
        () -> claw.Open(false))))
      .alongWith(new WaitCommand(0.8)
        .andThen(elevator.runOnce(
        () -> elevator.Set(kElevatorIntake))))
      .alongWith(pivot.runOnce(
      () -> pivot.Set(kPivotIntake)))
      .alongWith(new WaitCommand(0.8)
        .andThen(wrist.runOnce(
        () -> wrist.Set(kWristIntake)))));

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
      () -> driveTrack.withVelocityX(camera.MoveReefX(kTrackDistance) + xLimiter.calculate(-MathUtil.applyDeadband(DriverController.getLeftY(), 0.05) * MaxSpeed)) // Drive forward with negative Y (forward)
                      .withVelocityY(camera.MoveReefY(kTrackOffsetLeft) + yLimiter.calculate(-MathUtil.applyDeadband(DriverController.getLeftX(), 0.05) * MaxSpeed)) // Drive left with negative X (left)
                      .withRotationalRate(rotLimiter.calculate(-MathUtil.applyDeadband(DriverController.getRightX(), .1) * MaxAngularRate))));

    // Track Right
    DriverController.rightBumper().whileTrue(drivetrain.applyRequest(
      () -> driveTrack.withVelocityX(camera.MoveReefX(kTrackDistance) + xLimiter.calculate(-MathUtil.applyDeadband(DriverController.getLeftY(), 0.05) * MaxSpeed)) // Drive forward with negative Y (forward)
                      .withVelocityY(camera.MoveReefY(kTrackOffsetRight) + yLimiter.calculate(-MathUtil.applyDeadband(DriverController.getLeftX(), 0.05) * MaxSpeed)) // Drive left with negative X (left)
                      .withRotationalRate(rotLimiter.calculate(-MathUtil.applyDeadband(DriverController.getRightX(), .1) * MaxAngularRate))));

    // Run Intake
    DriverController.rightTrigger().whileTrue(claw.runEnd(
      () -> claw.RunIntakeWithBeam(),
      () -> claw.Stop()));

    // Reverse Intake
    DriverController.leftTrigger().whileTrue(claw.runEnd(
      () -> claw.RunIntakeReverse(),
      () -> claw.Stop()));

    /*
    // End Climb
    DriverController.b().onTrue(arm.runOnce(
      () -> arm.Set(kArmClimb))
      .alongWith(claw.runOnce(
      () -> claw.Open(false)))
      .alongWith(elevator.runOnce(
      () -> elevator.Set(kElevatorClimb)))
      .alongWith(pivot.runOnce(
      () -> pivot.Set(kPivotClimb)))
      .alongWith(wrist.runOnce(
      () -> wrist.Set(kWristClimb))));
    */

    // Stop
    DriverController.button(8).whileTrue(drivetrain.applyRequest(
      () -> brake));

    // Reset Field Heading
    DriverController.button(7).onTrue(drivetrain.runOnce(
      () -> drivetrain.seedFieldCentric()));

    // ** Manipulator Control **
    
    // Run Intake
    ManipulatorController.rightTrigger().whileTrue(claw.runEnd(
      () -> claw.RunIntakeWithBeam(),
      () -> claw.Stop()));

    // Reverse Intake
    ManipulatorController.leftTrigger().whileTrue(claw.runEnd(
      () -> claw.RunIntakeReverse(),
      () -> claw.Stop()));

    // Eject Algae
    ManipulatorController.povDown().whileTrue(claw.runEnd(
      () -> claw.EjectAlgae(),
      () -> claw.Stop()));

    /*
    // L1
    ManipulatorController.b().onTrue(arm.runOnce(
      () -> arm.Set(kArmL1))
      .alongWith(claw.runOnce(
      () -> claw.Open(false)))
      .alongWith(elevator.runOnce(
      () -> elevator.Set(kElevatorL1)))
      .alongWith(pivot.runOnce(
      () -> pivot.Set(kPivotL1)))
      .alongWith(wrist.runOnce(
      () -> wrist.Set(kWristL1))));
    */

    // L2
    ManipulatorController.a().onTrue(arm.runOnce(
      () -> arm.Set(kArmL2))
      .alongWith(claw.runOnce(
      () -> claw.Open(false)))
      .alongWith(new WaitCommand(0.6)
        .andThen(elevator.runOnce(
        () -> elevator.Set(kElevatorL2))))
      .alongWith(pivot.runOnce(
      () -> pivot.Set(kPivotL2)))
      .alongWith(wrist.runOnce(
      () -> wrist.Set(kWristL2))));

    // L3
    ManipulatorController.x().onTrue(arm.runOnce(
      () -> arm.Set(kArmL3))
      .alongWith(claw.runOnce(
      () -> claw.Open(false)))
      .alongWith(elevator.runOnce(
      () -> elevator.Set(kElevatorL3)))
      .alongWith(new WaitCommand(0.75)
        .andThen(pivot.runOnce(
        () -> pivot.Set(kPivotL3))))
      .alongWith(wrist.runOnce(
      () -> wrist.Set(kWristL3))));

    // L4
    ManipulatorController.y().onTrue(arm.runOnce(
      () -> arm.Set(kArmL4))
      .alongWith(claw.runOnce(
      () -> claw.Open(false)))
      .alongWith(elevator.runOnce(
      () -> elevator.Set(kElevatorL4)))
      .alongWith(new WaitCommand(1)
        .andThen(pivot.runOnce(
        () -> pivot.Set(kPivotL4))))
      .alongWith(wrist.runOnce(
      () -> wrist.Set(kWristL4))));

    // Home
    ManipulatorController.button(10).onTrue(arm.runOnce(
      () -> arm.Set(kArmHome))
      .alongWith(claw.runOnce(
      () -> claw.Open(false)))
      .alongWith(elevator.runOnce(
      () -> elevator.Set(kElevatorHome)))
      .alongWith(pivot.runOnce(
      () -> pivot.Set(kPivotHome)))
      .alongWith(wrist.runOnce(
      () -> wrist.Set(kWristHome))));

    // Intake
    ManipulatorController.b().onTrue(new WaitCommand(0.8)
        .andThen(arm.runOnce(
        () -> arm.Set(kArmIntake)))
      .alongWith(new WaitCommand(0.8)
        .andThen(claw.runOnce(
        () -> claw.Open(false))))
      .alongWith(new WaitCommand(0.8)
        .andThen(elevator.runOnce(
        () -> elevator.Set(kElevatorIntake))))
      .alongWith(pivot.runOnce(
      () -> pivot.Set(kPivotIntake)))
      .alongWith(new WaitCommand(0.8)
        .andThen(wrist.runOnce(
        () -> wrist.Set(kWristIntake)))));

    /*
    // Start Climb
    DriverController.b().onTrue(arm.runOnce(
      () -> arm.Set(kArmClimb))
      .alongWith(claw.runOnce(
      () -> claw.Open(false)))
      .alongWith(elevator.runOnce(
      () -> elevator.Set(kElevatorClimb)))
      .alongWith(pivot.runOnce(
      () -> pivot.Set(kPivotClimbEnd)))
      .alongWith(wrist.runOnce(
      () -> wrist.Set(kWristClimb))));

    // Barge
    ManipulatorController.povUp().onTrue(arm.runOnce(
      () -> arm.Set(kArmBarge))
      .alongWith(claw.runOnce(
      () -> claw.Open(true)))
      .alongWith(elevator.runOnce(
      () -> elevator.Set(kElevatorBarge)))
      .alongWith(pivot.runOnce(
      () -> pivot.Set(kPivotL1)))
      .alongWith(wrist.runOnce(
      () -> wrist.Set(kWristBarge))));

    // Processor
    ManipulatorController.povRight().onTrue(arm.runOnce(
      () -> arm.Set(kArmProcessor))
      .alongWith(claw.runOnce(
        () -> claw.Open(true)))
      .alongWith(elevator.runOnce(
      () -> elevator.Set(kElevatorHome)))
      .alongWith(pivot.runOnce(
      () -> pivot.Set(kPivotL1)))
      .alongWith(wrist.runOnce(
      () -> wrist.Set(kWristProcessor))));
      
    // Bottom Alge
    ManipulatorController.povDown().onTrue(arm.runOnce(
      () -> arm.Set(kArmL2))
      .alongWith(claw.runOnce(
      () -> claw.Open(true)))
      .alongWith(elevator.runOnce(
      () -> elevator.Set(kElevatorBottomAlge)))
      .alongWith(pivot.runOnce(
      () -> pivot.Set(kPivotL2)))
      .alongWith(wrist.runOnce(
      () -> wrist.Set(kWristAlge))));

    // Top Alge
    ManipulatorController.povLeft().onTrue(arm.runOnce(
      () -> arm.Set(kArmL3))
      .alongWith(claw.runOnce(
      () -> claw.Open(true)))
      .alongWith(elevator.runOnce(
      () -> elevator.Set(kElevatorTopAlge)))
      .alongWith(pivot.runOnce(
      () -> pivot.Set(kPivotL3)))
      .alongWith(wrist.runOnce(
      () -> wrist.Set(kWristAlge))));
    */

    // Manual Control
    ManipulatorController.button(8).whileTrue(arm.runEnd(
      () -> arm.ManualMovement(MathUtil.applyDeadband(ManipulatorController.getRightY(), 0.2), 1, false),
      () -> arm.Stop())
      .alongWith(claw.run(
      () -> claw.Open(ManipulatorController.rightBumper().getAsBoolean()))
      .alongWith(elevator.runEnd(
      () -> elevator.ManualMovement(MathUtil.applyDeadband(ManipulatorController.getLeftY(), 0.2), 1, false),
      () -> elevator.Stop())
      .alongWith(pivot.runEnd(
      () -> pivot.ManualMovement(MathUtil.applyDeadband(ManipulatorController.getLeftX(), 0.2), 1, false),
      () -> pivot.Stop())
      .alongWith(wrist.runEnd(
      () -> wrist.ManualMovement(MathUtil.applyDeadband(ManipulatorController.getRightX() * 0.4, 0.2), 1, false),
      () -> wrist.Stop()))))));
  }

  public Command getAutonomousCommand() {

    // Return an auto from pathplanner
    return autoChooser.getSelected();
  }
}
