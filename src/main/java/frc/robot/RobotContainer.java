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
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Wrist;

public class RobotContainer {
  // Some constants needed for swerve
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(Drivetrain.kRotateMagnitude).in(RadiansPerSecond);

  // Slew rate limiters smooth out drivetrain. 
  private SlewRateLimiter xLimiter = new SlewRateLimiter(Drivetrain.kMoveSlewRateLimiter);
  private SlewRateLimiter yLimiter = new SlewRateLimiter(Drivetrain.kMoveSlewRateLimiter);
  private SlewRateLimiter rotLimiter = new SlewRateLimiter(Drivetrain.kRotateSlewRateLimiter);

  // Set up controllers
  private final CommandXboxController DriverController = new CommandXboxController(Controllers.kDriverController);
  private final CommandXboxController ManipulatorController = new CommandXboxController(Controllers.kManipulatorController);
  
  // Setting up bindings for necessary control of the swerve drive platform
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  Camera camera = new Camera();
  Elevator elevator = new Elevator();
  Intake intake = new Intake();
  Lights lights = new Lights(camera, intake);
  Pivot pivot = new Pivot();
  Pneumatics pneumatics = new Pneumatics();
  Wrist wrist = new Wrist();

  // Swerve Requests
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * Drivetrain.kStickDeadzone).withRotationalDeadband(MaxAngularRate * Drivetrain.kStickDeadzone) // Add a 2% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop

  private final SwerveRequest.RobotCentric driveTrack = new SwerveRequest.RobotCentric()
      .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 5% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  // For Auto
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {

    NamedCommands.registerCommand("Track Left", drivetrain.applyRequest(
      () -> driveTrack.withVelocityX(camera.MoveY(PID.Track.kTrackYOffsetLeft) * MaxSpeed)              // Drive forward with negative Y (forward)
                      .withVelocityY(camera.MoveX(PID.Track.kTrackXOffsetLeft, false) * MaxSpeed) // Drive left with negative X (left)
                      .withRotationalRate(-camera.MoveRot(Degrees.of(0)) * MaxAngularRate))
      .alongWith(camera.runEnd(
      () -> camera.SetLEDOn(),
      () -> camera.SetLEDOff())));

    NamedCommands.registerCommand("Track Right", drivetrain.applyRequest(
      () -> driveTrack.withVelocityX(camera.MoveY(PID.Track.kTrackYOffsetRight) * MaxSpeed)              // Drive forward with negative Y (forward)
                      .withVelocityY(camera.MoveX(PID.Track.kTrackXOffsetRight, false) * MaxSpeed) // Drive left with negative X (left)
                      .withRotationalRate(-camera.MoveRot(Degrees.of(0)) * MaxAngularRate))
      .alongWith(camera.runEnd(
      () -> camera.SetLEDOn(),
      () -> camera.SetLEDOff())));

    NamedCommands.registerCommand("Track Ball", drivetrain.applyRequest(
      () -> driveTrack.withVelocityX(camera.MoveY(PID.Track.kTrackYOffsetCenter) * MaxSpeed)              // Drive forward with negative Y (forward)
                      .withVelocityY(camera.MoveX(PID.Track.kTrackXOffsetCenter, false) * MaxSpeed) // Drive left with negative X (left)
                      .withRotationalRate(-camera.MoveRot(Degrees.of(0)) * MaxAngularRate))
      .alongWith(camera.runEnd(
      () -> camera.SetLEDOn(),
      () -> camera.SetLEDOff())));

    NamedCommands.registerCommand("Track Coral", drivetrain.applyRequest(
    () -> driveTrack.withVelocityX(-camera.MoveY(PID.Track.kTrackYOffsetCoral) * MaxSpeed * 1.5)              // Drive forward with negative Y (forward)
                    .withVelocityY(-camera.MoveX(PID.Track.kTrackXOffsetCoral, true) * MaxSpeed * 1.5)  // Drive left with negative X (left)
                    .withRotationalRate(camera.MoveRot(Degrees.of(0)) * MaxAngularRate * 1.5))
    .alongWith(camera.runEnd(
    () -> camera.SetLEDOn(),
    () -> camera.SetLEDOff())));

    NamedCommands.registerCommand("Run Intake With Beam Break", intake.runEnd(
      () -> intake.RunIntakeWithBeam(),
      () -> intake.Stop()));

    NamedCommands.registerCommand("Run Intake", intake.runEnd(
      () -> intake.RunIntake(),
      () -> intake.Stop()));

    NamedCommands.registerCommand("Go to L4", elevator.run(
      () -> elevator.Set(PID.Elevator.kElevatorL4))
      .alongWith(new WaitCommand(0.6)
        .andThen(pivot.run(
        () -> pivot.Set(PID.Pivot.kPivotL4))))
      .alongWith(wrist.run(
      () -> wrist.Set(PID.Wrist.kWristL4))));

    NamedCommands.registerCommand("Go to L4 Only If Intaked", elevator.run(
      () -> elevator.SetIfTrue(PID.Elevator.kElevatorL4, intake.GetBeamBreak()))
      .alongWith(pivot.runOnce(
        () -> pivot.Set(PID.Pivot.kPivotL4)))
      .alongWith(new WaitCommand(0.55)
      .andThen(wrist.runOnce(
      () -> wrist.SetIfTrue(PID.Wrist.kWristL4, intake.GetBeamBreak())))));

    NamedCommands.registerCommand("Go to L4 No Wait Pivot", intake.run( 
      () -> intake.Open(false))
      .alongWith(elevator.run(
      () -> elevator.Set(PID.Elevator.kElevatorL4)))
      .alongWith(pivot.run(
        () -> pivot.Set(PID.Pivot.kPivotL4)))
      .alongWith(wrist.run(
      () -> wrist.Set(PID.Wrist.kWristL4))));

    NamedCommands.registerCommand("Go to Intake", new WaitCommand(0.3)
    .andThen(elevator.runOnce(
        () -> elevator.Set(PID.Elevator.kElevatorIntake)))
      .alongWith(pivot.runOnce(
      () -> pivot.Set(PID.Pivot.kPivotIntake)))
      .alongWith(new WaitCommand(0.3)
        .andThen(wrist.runOnce(
        () -> wrist.Set(PID.Wrist.kWristIntake)))));

    NamedCommands.registerCommand("Go to Intake Then L4", new WaitCommand(0.3)
    .andThen(elevator.runOnce(
        () -> elevator.Set(PID.Elevator.kElevatorIntake)))
        .andThen(new WaitCommand(.5)
        .andThen(elevator.runOnce(
          () -> elevator.Set(PID.Elevator.kElevatorL4))))
      .alongWith(new WaitCommand(.8)
      .andThen(pivot.runOnce(
      () -> pivot.Set(PID.Pivot.kPivotIntake)))
      .alongWith(new WaitCommand(.8)
        .andThen(wrist.runOnce(
        () -> wrist.Set(PID.Wrist.kWristL4))))));

    NamedCommands.registerCommand("Go to Bottom Algae", intake.runOnce(
      () -> intake.Open(true))
      .alongWith(elevator.runOnce(
      () -> elevator.Set(PID.Elevator.kElevatorBottomAlgae)))
      .alongWith(pivot.runOnce(
      () -> pivot.Set(PID.Pivot.kPivotL2)))
      .alongWith(wrist.runOnce(
      () -> wrist.Set(PID.Wrist.kWristBottomAlgae)))
      );

    NamedCommands.registerCommand("Go to Top Algae", intake.runOnce(
      () -> intake.Open(true))
      .alongWith(elevator.runOnce(
      () -> elevator.Set(PID.Elevator.kElevatorTopAlgae)))
      .alongWith(pivot.runOnce(
      () -> pivot.Set(PID.Pivot.kPivotL3)))
      .alongWith(wrist.runOnce(
      () -> wrist.Set(PID.Wrist.kWristTopAlgae)))
      );

    NamedCommands.registerCommand("Go to Algae Barge", elevator.runOnce(
      () -> elevator.Set(PID.Elevator.kElevatorBarge))
      .alongWith(pivot.runOnce(
      () -> pivot.Set(PID.Pivot.kPivotIntake))
      .alongWith(wrist.runOnce(
      () -> wrist.Set(PID.Wrist.kWristBarge)))));

    NamedCommands.registerCommand("Go to Algae Home", intake.runOnce(
      () -> intake.Open(true))
      .alongWith(new WaitCommand(0.6)
        .andThen(elevator.runOnce(
        () -> elevator.Set(PID.Elevator.kElevatorIntake))))
      .alongWith(pivot.runOnce(
      () -> pivot.Set(PID.Pivot.kPivotIntake)))
      .alongWith(new WaitCommand(0.6)
        .andThen(wrist.runOnce(
        () -> wrist.Set(PID.Wrist.kWristHomeAlgae))))
      );

    NamedCommands.registerCommand("Eject Algae", intake.runEnd(
      () -> intake.EjectAlgae(),
      () -> intake.Stop())
      );

    NamedCommands.registerCommand("Reset Gyro", drivetrain.runOnce(
      () -> drivetrain.ResetDrivePose()));

    // For Auto
    autoChooser = AutoBuilder.buildAutoChooser("None");

    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
  }

  private void configureBindings() {

    // Drivetrain will execute this command periodically
    drivetrain.setDefaultCommand(
      drivetrain.applyRequest(
        () -> drive.withVelocityX(xLimiter.calculate(MathUtil.applyDeadband(Math.pow(-DriverController.getLeftY(), 2), Drivetrain.kStickDeadzone) * MaxSpeed * drivetrain.SlowSwerve(DriverController.x().getAsBoolean()))) // Drive forward with negative Y (forward)
                   .withVelocityY(yLimiter.calculate(MathUtil.applyDeadband(Math.pow(-DriverController.getLeftX(), 2), Drivetrain.kStickDeadzone) * MaxSpeed * drivetrain.SlowSwerve(DriverController.x().getAsBoolean()))) // Drive left with negative X (left)
                   .withRotationalRate(rotLimiter.calculate(MathUtil.applyDeadband(-DriverController.getRightX(), Drivetrain.kStickDeadzone) * MaxAngularRate * drivetrain.SlowSwerve(DriverController.x().getAsBoolean()))) // Drive counterclockwise with negative X (left)
      ));

    // ** Driver Control **

    // Track Left
    DriverController.leftBumper().whileTrue(drivetrain.applyRequest(
      () -> driveTrack.withVelocityX((camera.MoveY(PID.Track.kTrackYOffsetLeft) + xLimiter.calculate(-MathUtil.applyDeadband(DriverController.getLeftY(), Drivetrain.kStickDeadzone))) * MaxSpeed) // Drive forward with negative Y (forward)
                      .withVelocityY((camera.MoveX(PID.Track.kTrackXOffsetLeft, false) + yLimiter.calculate(-MathUtil.applyDeadband(DriverController.getLeftX(), Drivetrain.kStickDeadzone))) * MaxSpeed) // Drive left with negative X (left)
                      .withRotationalRate((-camera.MoveRot(Degrees.of(0)) + rotLimiter.calculate(-MathUtil.applyDeadband(DriverController.getRightX(), Drivetrain.kStickDeadzone))) * MaxAngularRate))
      .alongWith(camera.runEnd(
      () -> camera.SetLEDOn(),
      () -> camera.SetLEDOff())));

    // Track Center
    DriverController.leftTrigger().whileTrue(drivetrain.applyRequest(
      () -> driveTrack.withVelocityX((camera.MoveY(PID.Track.kTrackYOffsetCenter) + xLimiter.calculate(-MathUtil.applyDeadband(DriverController.getLeftY(), Drivetrain.kStickDeadzone))) * MaxSpeed) // Drive forward with negative Y (forward)
                      .withVelocityY((camera.MoveX(PID.Track.kTrackXOffsetCenter, false) + yLimiter.calculate(-MathUtil.applyDeadband(DriverController.getLeftX(), Drivetrain.kStickDeadzone))) * MaxSpeed) // Drive left with negative X (left)
                      .withRotationalRate((-camera.MoveRot(Degrees.of(0)) + rotLimiter.calculate(-MathUtil.applyDeadband(DriverController.getRightX(), Drivetrain.kStickDeadzone))) * MaxAngularRate))
      .alongWith(camera.runEnd(
      () -> camera.SetLEDOn(),
      () -> camera.SetLEDOff())));

    // Track Right
    DriverController.rightBumper().whileTrue(drivetrain.applyRequest(
      () -> driveTrack.withVelocityX((camera.MoveY(PID.Track.kTrackYOffsetRight) + xLimiter.calculate(-MathUtil.applyDeadband(DriverController.getLeftY(), Drivetrain.kStickDeadzone))) * MaxSpeed) // Drive forward with negative Y (forward)
                      .withVelocityY((camera.MoveX(PID.Track.kTrackXOffsetRight, false) + yLimiter.calculate(-MathUtil.applyDeadband(DriverController.getLeftX(), Drivetrain.kStickDeadzone))) * MaxSpeed) // Drive left with negative X (left)
                      .withRotationalRate((-camera.MoveRot(Degrees.of(0)) + rotLimiter.calculate(-MathUtil.applyDeadband(DriverController.getRightX(), Drivetrain.kStickDeadzone))) * MaxAngularRate))
      .alongWith(camera.runEnd(
      () -> camera.SetLEDOn(),
      () -> camera.SetLEDOff())));

    //Track Coral
    DriverController.a().whileTrue(drivetrain.applyRequest(
      () -> driveTrack.withVelocityX((-camera.MoveY(PID.Track.kTrackYOffsetCoral) + xLimiter.calculate(-MathUtil.applyDeadband(DriverController.getLeftY(), Drivetrain.kStickDeadzone))) * MaxSpeed) // Drive forward with negative Y (forward)
                      .withVelocityY((-camera.MoveX(PID.Track.kTrackXOffsetCoral, true) + yLimiter.calculate(-MathUtil.applyDeadband(DriverController.getLeftX(), Drivetrain.kStickDeadzone))) * MaxSpeed) // Drive left with negative X (left)
                      .withRotationalRate((camera.MoveRot(Degrees.of(0)) + rotLimiter.calculate(-MathUtil.applyDeadband(DriverController.getRightX(), Drivetrain.kStickDeadzone))) * MaxAngularRate)
                      )
      .alongWith(camera.runEnd(
      () -> camera.SetLEDOn(),
      () -> camera.SetLEDOff())));

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
      () -> elevator.Set(PID.Elevator.kElevatorClimb)))
      .alongWith(pivot.runOnce(
      () -> pivot.Set(PID.Pivot.kPivotClimbEnd)))
      .alongWith(wrist.runOnce(
      () -> wrist.Set(PID.Wrist.kWristIntake))));

    // Home
    DriverController.povLeft().onTrue(intake.runOnce(
      () -> intake.Open(false))
      .alongWith(elevator.runOnce(
      () -> elevator.Set(PID.Elevator.kElevatorHome)))
      .alongWith(pivot.runOnce(
      () -> pivot.Set(PID.Pivot.kPivotHome)))
      .alongWith(wrist.runOnce(
      () -> wrist.Set(PID.Wrist.kWristHome))));

    // Stop
    DriverController.button(8).whileTrue(drivetrain.applyRequest(
      () -> brake));

    // Reset Field Heading
    DriverController.button(7).onTrue(drivetrain.runOnce(
      () -> drivetrain.seedFieldCentric()));

    // Open
    DriverController.button(10).onTrue(intake.runOnce(
      () -> intake.Open(true)));

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
    // Rebind back buttons pls during offseason thx :)
    ManipulatorController.button(9).onTrue(intake.runOnce(
      () -> intake.CloseNoAlgae())
      .alongWith(elevator.runOnce(
        () -> elevator.Set(PID.Elevator.kElevatorL1)))
      .alongWith(pivot.runOnce(
      () -> pivot.Set(PID.Pivot.kPivotL1)))
      .alongWith(new WaitCommand(0.55)
      .andThen(wrist.runOnce(
      () -> wrist.Set(PID.Wrist.kWristL1)))));

    // Coral Ground
    // Rebind back buttons pls during offseason thx :)
    ManipulatorController.button(10).onTrue(intake.runOnce(
      () -> intake.OpenNoAlgae(true))
      .alongWith(elevator.runOnce(
        () -> elevator.Set(PID.Elevator.kElevatorIntake)))
      .alongWith(pivot.runOnce(
      () -> pivot.Set(PID.Pivot.kPivotGroundCoral)))
      .alongWith(wrist.runOnce(
      () -> wrist.Set(PID.Wrist.kWristGroundCoral))));

    // L2
    ManipulatorController.a().onTrue(intake.runOnce(
      () -> intake.Open(false))
      .alongWith(elevator.runOnce(
        () -> elevator.Set(PID.Elevator.kElevatorL2)))
      .alongWith(pivot.runOnce(
      () -> pivot.Set(PID.Pivot.kPivotL2)))
      .alongWith(new WaitCommand(0.55)
      .andThen(wrist.runOnce(
      () -> wrist.Set(PID.Wrist.kWristL2)))));

    // L3
    ManipulatorController.x().onTrue(intake.runOnce(
      () -> intake.Open(false))
      .alongWith(elevator.runOnce(
      () -> elevator.Set(PID.Elevator.kElevatorL3)))
      .alongWith(pivot.runOnce(
        () -> pivot.Set(PID.Pivot.kPivotL3)))
      .alongWith(new WaitCommand(0.55)
      .andThen(wrist.runOnce(
      () -> wrist.Set(PID.Wrist.kWristL3)))));

    // L4
    ManipulatorController.y().onTrue(intake.runOnce(
      () -> intake.Open(false))
      .alongWith(elevator.runOnce(
      () -> elevator.Set(PID.Elevator.kElevatorL4)))
      .alongWith(pivot.runOnce(
        () -> pivot.Set(PID.Pivot.kPivotL4)))
      .alongWith(new WaitCommand(0.55)
      .andThen(wrist.runOnce(
      () -> wrist.Set(PID.Wrist.kWristL4)))));

    // Intake
    ManipulatorController.b().onTrue(intake.runOnce(
      () -> intake.Open(false))
      .alongWith(new WaitCommand(0.3)
        .andThen(elevator.runOnce(
        () -> elevator.Set(PID.Elevator.kElevatorIntake))))
      .alongWith(pivot.runOnce(
      () -> pivot.Set(PID.Pivot.kPivotIntake)))
      .alongWith(new WaitCommand(0.3)
        .andThen(wrist.runOnce(
        () -> wrist.Set(PID.Wrist.kWristIntake)))));

    // Start Climb
    ManipulatorController.leftBumper().onTrue(intake.runOnce(
      () -> intake.Open(false))
      .alongWith(elevator.runOnce(
      () -> elevator.Set(PID.Elevator.kElevatorClimb)))
      .alongWith(pivot.runOnce(
      () -> pivot.Set(PID.Pivot.kPivotClimb)))
      .alongWith(wrist.runOnce(
      () -> wrist.Set(PID.Wrist.kWristIntake))));

    // Barge
    ManipulatorController.povLeft().onTrue(intake.runOnce(
      () -> intake.Open(true))
      .alongWith(elevator.runOnce(
      () -> elevator.Set(PID.Elevator.kElevatorBarge)))
      .alongWith(pivot.runOnce(
      () -> pivot.Set(PID.Pivot.kPivotIntake)))
      .alongWith(wrist.runOnce(
      () -> wrist.Set(PID.Wrist.kWristBarge))));

    // Processor
    ManipulatorController.povRight().onTrue(intake.runOnce(
      () -> intake.Open(true))
      .alongWith(elevator.runOnce(
      () -> elevator.Set(PID.Elevator.kElevatorProcessor)))
      .alongWith(pivot.runOnce(
      () -> pivot.Set(PID.Pivot.kPivotProcessor)))
      .alongWith(wrist.runOnce(
      () -> wrist.Set(PID.Wrist.kWristProcessor))));

    // Bottom Algae
    ManipulatorController.povDown().onTrue(intake.runOnce(
      () -> intake.Open(true))
      .alongWith(elevator.runOnce(
      () -> elevator.Set(PID.Elevator.kElevatorBottomAlgae)))
      .alongWith(pivot.runOnce(
      () -> pivot.Set(PID.Pivot.kPivotL2)))
      .alongWith(wrist.runOnce(
      () -> wrist.Set(PID.Wrist.kWristBottomAlgae))));

    // Top Algae
    ManipulatorController.povUp().onTrue(intake.runOnce(
      () -> intake.Open(true))
      .alongWith(elevator.runOnce(
      () -> elevator.Set(PID.Elevator.kElevatorTopAlgae)))
      .alongWith(pivot.runOnce(
      () -> pivot.Set(PID.Pivot.kPivotL3)))
      .alongWith(wrist.runOnce(
      () -> wrist.Set(PID.Wrist.kWristTopAlgae))));

    // Home Algae
    ManipulatorController.button(7).onTrue(intake.runOnce(
      () -> intake.Open(true))
      .alongWith(new WaitCommand(0.6)
        .andThen(elevator.runOnce(
        () -> elevator.Set(PID.Elevator.kElevatorIntake))))
      .alongWith(pivot.runOnce(
      () -> pivot.Set(PID.Pivot.kPivotIntake)))
      .alongWith(new WaitCommand(0.6)
        .andThen(wrist.runOnce(
        () -> wrist.Set(PID.Wrist.kWristHomeAlgae)))));

    // Ground Algae
    ManipulatorController.rightBumper().onTrue(intake.runOnce(
      () -> intake.Open(true))
      .alongWith(elevator.runOnce(
      () -> elevator.Set(PID.Elevator.kElevatorGroundAlgae)))
      .alongWith(pivot.runOnce(
      () -> pivot.Set(PID.Pivot.kPivotGroundAlgae)))
      .alongWith(wrist.runOnce(
      () -> wrist.Set(PID.Wrist.kWristGroundAlgae))));

    // Manual Control
    // Janky but works
    ManipulatorController.button(8).whileTrue(elevator.runEnd(
      () -> elevator.ManualMovement(MathUtil.applyDeadband(ManipulatorController.getLeftY(), 0.05), 0.75, false),
      () -> elevator.Stop())
      .alongWith(pivot.runEnd(
      () -> pivot.ManualMovement(MathUtil.applyDeadband(ManipulatorController.getLeftX(), 0.05), 1, false),
      () -> pivot.Stop())
      .alongWith(wrist.runEnd(
      () -> wrist.ManualMovement(MathUtil.applyDeadband(ManipulatorController.getRightX(), 0.05), 1.5, false),
      () -> wrist.Stop()))));
  }

  public Command getAutonomousCommand() {
    // Return an auto from pathplanner
    return autoChooser.getSelected();
  }
}
