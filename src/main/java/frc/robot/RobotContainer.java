// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

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
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(.75).in(RadiansPerSecond);

  private SlewRateLimiter xLimiter = new SlewRateLimiter(Constants.kMoveSlewRateLimiter);
  private SlewRateLimiter yLimiter = new SlewRateLimiter(Constants.kMoveSlewRateLimiter); //limit rate of change of joystick inputs
  private SlewRateLimiter rotLimiter = new SlewRateLimiter(Constants.kRotateSlewRateLimiter); //reduce brownouts

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController DriverController = new CommandXboxController(Constants.kDriverController); // My DriverController
  private final CommandXboxController ManipulatorController = new CommandXboxController(Constants.kManipulatorController);
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain(); // My drivetrain

  Arm arm = new Arm();
  Camera camera = new Camera();
  Claw claw = new Claw();
  Elevator elevator = new Elevator();
  Pivot pivot = new Pivot();
  Wrist wrist = new Wrist();

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop

  private final SwerveRequest.RobotCentric driveTrack = new SwerveRequest.RobotCentric()
      .withDeadband(MaxSpeed * 0.02).withRotationalDeadband(MaxAngularRate * 0.02) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {

    autoChooser = AutoBuilder.buildAutoChooser("test");

    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
      drivetrain.applyRequest(
        () -> drive.withVelocityX(xLimiter.calculate(MathUtil.applyDeadband(-DriverController.getLeftY(), 0.02) * MaxSpeed)) // Drive forward with negative Y (forward)
                   .withVelocityY(yLimiter.calculate(MathUtil.applyDeadband(-DriverController.getLeftX(), 0.02) * MaxSpeed)) // Drive left with negative X (left)
                   .withRotationalRate(rotLimiter.calculate(MathUtil.applyDeadband(-DriverController.getRightX(), 0.02) * MaxAngularRate)) // Drive counterclockwise with negative X (left)
      ));

    // ** Driver Control **

    // Track Left
    DriverController.leftBumper().whileTrue(drivetrain.applyRequest(
      () -> driveTrack.withVelocityX(camera.MoveReefX(Constants.kTrackDistance) + xLimiter.calculate(-MathUtil.applyDeadband(DriverController.getLeftY(), 0.05) * MaxSpeed)) // Drive forward with negative Y (forward)
                      .withVelocityY(camera.MoveReefY(Constants.kTrackOffset) + yLimiter.calculate(-MathUtil.applyDeadband(DriverController.getLeftX(), 0.05) * MaxSpeed)) // Drive left with negative X (left)
                      .withRotationalRate(rotLimiter.calculate(-MathUtil.applyDeadband(DriverController.getRightX(), .1) * MaxAngularRate))));

    // Track Right
    DriverController.rightBumper().whileTrue(drivetrain.applyRequest(
      () -> driveTrack.withVelocityX(camera.MoveReefX(Constants.kTrackDistance) + xLimiter.calculate(-MathUtil.applyDeadband(DriverController.getLeftY(), 0.05) * MaxSpeed)) // Drive forward with negative Y (forward)
                      .withVelocityY(camera.MoveReefY(-1 * Constants.kTrackOffset) + yLimiter.calculate(-MathUtil.applyDeadband(DriverController.getLeftX(), 0.05) * MaxSpeed)) // Drive left with negative X (left)
                      .withRotationalRate(rotLimiter.calculate(-MathUtil.applyDeadband(DriverController.getRightX(), .1) * MaxAngularRate))));

    // Stop
    DriverController.button(8).whileTrue(drivetrain.applyRequest(
      () -> brake));

    // Reset Field Heading
    DriverController.button(7).onTrue(drivetrain.runOnce(
      () -> drivetrain.seedFieldCentric()));

    // ** Manipulator Control **
    
    // L1
    ManipulatorController.a().onTrue(arm.runOnce(
      () -> arm.Set(Constants.kArmL1))
      .alongWith(elevator.runOnce(
      () -> elevator.Set(Constants.kElevatorL1)))
      .alongWith(pivot.runOnce(
      () -> pivot.Set(Constants.kPivotReef)))
      .alongWith(wrist.runOnce(
      () -> wrist.Set(Constants.kWristL1))));

    // L2
    ManipulatorController.x().onTrue(arm.runOnce(
      () -> arm.Set(Constants.kArmL2or3))
      .alongWith(elevator.runOnce(
      () -> elevator.Set(Constants.kElevatorL2)))
      .alongWith(pivot.runOnce(
      () -> pivot.Set(Constants.kPivotReef)))
      .alongWith(wrist.runOnce(
      () -> wrist.Set(Constants.kWristL2or3))));
    
    // L3
    ManipulatorController.y().onTrue(elevator.runOnce(
      () -> arm.Set(Constants.kArmL2or3))
      .alongWith(elevator.runOnce(
      () -> elevator.Set(Constants.kElevatorL3)))
      .alongWith(pivot.runOnce(
      () -> pivot.Set(Constants.kPivotReef)))
      .alongWith(wrist.runOnce(
      () -> wrist.Set(Constants.kWristL2or3))));
    
    // L4
    ManipulatorController.b().onTrue(elevator.runOnce(
      () -> arm.Set(Constants.kArmL4))
      .alongWith(elevator.runOnce(
      () -> elevator.Set(Constants.kElevatorL4)))
      .alongWith(pivot.runOnce(
      () -> pivot.Set(Constants.kPivotReef)))
      .alongWith(wrist.runOnce(
      () -> wrist.Set(Constants.kWristL4))));

    // Climb / Home
    ManipulatorController.povDown().onTrue(elevator.runOnce(
      () -> arm.Set(Constants.kArmHome))
      .alongWith(elevator.runOnce(
      () -> elevator.Set(Constants.kElevatorHome)))
      .alongWith(pivot.runOnce(
      () -> pivot.Set(Constants.kPivotHome)))
      .alongWith(wrist.runOnce(
      () -> wrist.Set(Constants.kWristHome))));
  
    // Intake
    ManipulatorController.povLeft().onTrue(elevator.runOnce(
      () -> arm.Set(Constants.kArmIntake))
      .alongWith(elevator.runOnce(
      () -> elevator.Set(Constants.kElevatorIntake)))
      .alongWith(pivot.runOnce(
      () -> pivot.Set(Constants.kPivotIntake)))
      .alongWith(wrist.runOnce(
      () -> wrist.Set(Constants.kWristIntake))));

    // Barge
    ManipulatorController.povUp().onTrue(elevator.runOnce(
      () -> arm.Set(Constants.kArmBarge))
      .alongWith(elevator.runOnce(
      () -> elevator.Set(Constants.kElevatorBarge)))
      .alongWith(pivot.runOnce(
      () -> pivot.Set(Constants.kPivotReef)))
      .alongWith(wrist.runOnce(
      () -> wrist.Set(Constants.kWristBarge))));

    // Processor
    ManipulatorController.povRight().onTrue(elevator.runOnce(
      () -> arm.Set(Constants.kArmProcessor))
      .alongWith(elevator.runOnce(
      () -> elevator.Set(Constants.kElevatorHome)))
      .alongWith(pivot.runOnce(
      () -> pivot.Set(Constants.kPivotReef)))
      .alongWith(wrist.runOnce(
      () -> wrist.Set(Constants.kWristProcessor))));
    
    // Top Alge
    ManipulatorController.button(9).onTrue(elevator.runOnce(
      () -> arm.Set(Constants.kArmL2or3))
      .alongWith(elevator.runOnce(
      () -> elevator.Set(Constants.kElevatorTopAlge)))
      .alongWith(pivot.runOnce(
      () -> pivot.Set(Constants.kPivotReef)))
      .alongWith(wrist.runOnce(
      () -> wrist.Set(Constants.kWristAlge))));

    // Bottom Alge
    ManipulatorController.button(10).onTrue(elevator.runOnce(
      () -> arm.Set(Constants.kArmL2or3))
      .alongWith(elevator.runOnce(
      () -> elevator.Set(Constants.kElevatorBottomAlge)))
      .alongWith(pivot.runOnce(
      () -> pivot.Set(Constants.kPivotReef)))
      .alongWith(wrist.runOnce(
      () -> wrist.Set(Constants.kWristAlge))));

    // Run Intake
    ManipulatorController.leftTrigger(Constants.kTriggerThreshold).whileTrue(claw.runEnd(
      () -> claw.Intake(false),
      () -> claw.Stop()));

    // Run Outake
    ManipulatorController.rightTrigger(Constants.kTriggerThreshold).whileTrue(claw.runEnd(
      () -> claw.Intake(true),
      () -> claw.Stop()));

    // Open Jaws
    ManipulatorController.leftBumper().onTrue(claw.runOnce(
      () -> claw.Open(false)));

    // Close Jaws
    ManipulatorController.rightBumper().onTrue(claw.runOnce(
      () -> claw.Open(true)));

    // Manual Control
    ManipulatorController.button(8).whileTrue(arm.runEnd(
      () -> arm.ManualMovement(ManipulatorController.getRightX(), 1),
      () -> arm.Stop())
      .alongWith(elevator.runEnd(
      () -> elevator.ManualMovement(ManipulatorController.getLeftY(), 1),
      () -> elevator.Stop())
      .alongWith(pivot.runEnd(
      () -> pivot.ManualMovement(ManipulatorController.getLeftX(), 1),
      () -> pivot.Stop())
      .alongWith(wrist.runEnd(
      () -> wrist.ManualMovement(ManipulatorController.getRightY(), 1),
      () -> wrist.Stop())))));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
