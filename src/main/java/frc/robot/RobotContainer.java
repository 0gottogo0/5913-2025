// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

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
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(.75).in(RadiansPerSecond);

  private SlewRateLimiter xLimiter = new SlewRateLimiter(20.0);
  private SlewRateLimiter yLimiter = new SlewRateLimiter(20.0); //limit rate of change of joystick inputs
  private SlewRateLimiter rotLimiter = new SlewRateLimiter(40.0); //reduce brownouts

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController DriverController = new CommandXboxController(Constants.kDriverController); // My DriverController
  private final CommandXboxController ManipulatorController = new CommandXboxController(Constants.kManipulatorController);
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain(); // My drivetrain

  Claw claw = new Claw();
  Elevator elevator = new Elevator();
  Pivot pivot = new Pivot();

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
    DriverController.button(8).whileTrue(drivetrain.applyRequest(
      () -> brake));

    DriverController.button(7).onTrue(drivetrain.runOnce(
      () -> drivetrain.seedFieldCentric()));

    // ** Manipulator Control **
    ManipulatorController.povRight().onTrue(elevator.runOnce(
      () -> elevator.Set(Constants.kElevatorL4)));

    ManipulatorController.povUp().onTrue(elevator.runOnce(
      () -> elevator.Set(Constants.kElevatorL3)));
    
    ManipulatorController.povLeft().onTrue(elevator.runOnce(
      () -> elevator.Set(Constants.kElevatorL2)));
    
    ManipulatorController.povDown().onTrue(elevator.runOnce(
      () -> elevator.Set(Constants.kElevatorL1)));

    ManipulatorController.leftStick().whileTrue(elevator.run(
      () -> elevator.ManualMovement(ManipulatorController.getLeftY(), 1)));

    ManipulatorController.leftTrigger(Constants.kManipulatorTriggerThreshold).whileTrue(claw.runEnd(
      () -> claw.Intake(false),
      () -> claw.Stop()));

    ManipulatorController.rightTrigger(Constants.kManipulatorTriggerThreshold).whileTrue(claw.runEnd(
      () -> claw.Intake(true),
      () -> claw.Stop()));

    ManipulatorController.leftBumper().onTrue(claw.runOnce(
      () -> claw.Open(false)));

    ManipulatorController.rightBumper().onTrue(claw.runOnce(
      () -> claw.Open(true)));

    ManipulatorController.y().onTrue(pivot.runOnce(
      () -> pivot.Set(Constants.kPivotReef)));

    ManipulatorController.b().onTrue(pivot.runOnce(
      () -> pivot.Set(Constants.kPivotIntake)));
    
    ManipulatorController.a().onTrue(pivot.runOnce(
      () -> pivot.Set(Constants.kPivotClimb)));

    ManipulatorController.rightStick().whileTrue(pivot.run(
      () -> pivot.ManualMovement(ManipulatorController.getRightY(), 1)));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
