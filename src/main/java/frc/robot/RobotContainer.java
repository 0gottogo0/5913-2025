// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Wrist;

public class RobotContainer {
  // Some constants needed for swerve
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(.75).in(RadiansPerSecond);

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
  Claw claw = new Claw();
  Elevator elevator = new Elevator();
  Pivot pivot = new Pivot();
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

    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putNumber("Left X", ManipulatorController.getLeftX());
    SmartDashboard.putNumber("Left Y", ManipulatorController.getRightY());
    SmartDashboard.putNumber("Right X", ManipulatorController.getLeftX());
    SmartDashboard.putNumber("Right Y", ManipulatorController.getRightY());

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
                      .withVelocityY(camera.MoveReefY(kTrackOffset) + yLimiter.calculate(-MathUtil.applyDeadband(DriverController.getLeftX(), 0.05) * MaxSpeed)) // Drive left with negative X (left)
                      .withRotationalRate(rotLimiter.calculate(-MathUtil.applyDeadband(DriverController.getRightX(), .1) * MaxAngularRate))));

    // Track Right
    DriverController.rightBumper().whileTrue(drivetrain.applyRequest(
      () -> driveTrack.withVelocityX(camera.MoveReefX(kTrackDistance) + xLimiter.calculate(-MathUtil.applyDeadband(DriverController.getLeftY(), 0.05) * MaxSpeed)) // Drive forward with negative Y (forward)
                      .withVelocityY(camera.MoveReefY(-1 * kTrackOffset) + yLimiter.calculate(-MathUtil.applyDeadband(DriverController.getLeftX(), 0.05) * MaxSpeed)) // Drive left with negative X (left)
                      .withRotationalRate(rotLimiter.calculate(-MathUtil.applyDeadband(DriverController.getRightX(), .1) * MaxAngularRate))));

    // Run Intake
    DriverController.a().whileTrue(claw.runEnd(
      () -> claw.Intake(),
      () -> claw.Stop()));

    // Climb
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

    // Stop
    DriverController.button(8).whileTrue(drivetrain.applyRequest(
      () -> brake));

    // Reset Field Heading
    DriverController.button(7).onTrue(drivetrain.runOnce(
      () -> drivetrain.seedFieldCentric()));

    // ** Manipulator Control **
    
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

    // L2
    ManipulatorController.a().onTrue(arm.runOnce(
      () -> arm.Set(kArmL2))
      .alongWith(claw.runOnce(
      () -> claw.Open(false)))
      .alongWith(elevator.runOnce(
      () -> elevator.Set(kElevatorL2)))
      .alongWith(pivot.runOnce(
      () -> pivot.Set(kPivotL2)))
      .alongWith(wrist.runOnce(
      () -> wrist.Set(kWristL2or3))));
    
    // L3
    ManipulatorController.x().onTrue(arm.runOnce(
      () -> arm.Set(kArmL3))
      .alongWith(claw.runOnce(
      () -> claw.Open(false)))
      .alongWith(elevator.runOnce(
      () -> elevator.Set(kElevatorL3)))
      .alongWith(pivot.runOnce(
      () -> pivot.Set(kPivotL3)))
      .alongWith(wrist.runOnce(
      () -> wrist.Set(kWristL2or3))));
    
    // L4
    ManipulatorController.y().onTrue(arm.runOnce(
      () -> arm.Set(kArmL4))
      .alongWith(claw.runOnce(
      () -> claw.Open(false)))
      .alongWith(elevator.runOnce(
      () -> elevator.Set(kElevatorL4)))
      .alongWith(pivot.runOnce(
      () -> pivot.Set(kPivotL4)))
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
    ManipulatorController.rightBumper().onTrue(arm.runOnce(
      () -> arm.Set(kArmIntake))
      .alongWith(claw.runOnce(
      () -> claw.Open(false)))
      .alongWith(elevator.runOnce(
      () -> elevator.Set(kElevatorIntake)))
      .alongWith(pivot.runOnce(
      () -> pivot.Set(kPivotIntake)))
      .alongWith(wrist.runOnce(
      () -> wrist.Set(kWristIntake))));

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

    // Manual Control
    ManipulatorController.button(8).whileTrue(arm.runEnd(
      () -> arm.ManualMovement(MathUtil.applyDeadband(ManipulatorController.getRightX(), 0.2), 1, false),
      () -> arm.Stop())
      .alongWith(claw.run(
      () -> claw.Open(ManipulatorController.button(9).getAsBoolean()))
      .alongWith(elevator.runEnd(
      () -> elevator.ManualMovement(MathUtil.applyDeadband(ManipulatorController.getLeftY(), 0.2), 1, false),
      () -> elevator.Stop())
      .alongWith(pivot.runEnd(
      () -> pivot.ManualMovement(MathUtil.applyDeadband(ManipulatorController.getLeftX(), 0.2), 1, false),
      () -> pivot.Stop())
      .alongWith(wrist.runEnd(
      () -> wrist.ManualMovement(MathUtil.applyDeadband(ManipulatorController.getRightY() * 0.4, 0.2), 1, false),
      () -> wrist.Stop()))))));
  }

  public Command getAutonomousCommand() {

    // Return an auto from pathplanner
    return autoChooser.getSelected();
  }
}
