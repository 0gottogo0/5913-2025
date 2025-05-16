// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.*;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private final CommandXboxController ManipulatorController = new CommandXboxController(Controllers.MANIPULATOR_CONTROLLER);

  Timer intakeTimer = new Timer();

  public Robot() {
    // Disable CTRE Logging
    SignalLogger.enableAutoLogging(false);

    m_robotContainer = new RobotContainer();

    DataLogManager.start();

    // Pathplanner
    // FollowPathCommand.warmupCommand().schedule();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    // MetaTag2
    if (true) {
      var driveState = m_robotContainer.drivetrain.getState();
      double headingDeg = driveState.Pose.getRotation().getDegrees();
      double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

      LimelightHelpers.SetRobotOrientation(IO.Camera.LIMELIGHT_CORAL, headingDeg, 0, 0, 0, 0, 0);
      LimelightHelpers.SetRobotOrientation(IO.Camera.LIMELIGHT_REEF, headingDeg, 0, 0, 0, 0, 0);
      var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(IO.Camera.LIMELIGHT_REEF);
      
      if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
        m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds);
      }
    }

    // Dont ignore the beambreak if we are intaking
    m_robotContainer.intake.ignoreBeamBreak = m_robotContainer.pivot.GetSetpoint() != PID.Pivot.PIVOT_INTAKE && m_robotContainer.wrist.GetSetpoint() != PID.Wrist.WRIST_INTAKE;

    // Slow the elevator if we are holding algae
    m_robotContainer.elevator.holdAglae = m_robotContainer.intake.holdAlgae == true;

    // Set lights to red if "bad error"
    m_robotContainer.lights.badError = m_robotContainer.pivot.GetAngle().in(Degree) == (360 - IO.Misc.PIVOT_ENCODER_OFFSET) || m_robotContainer.wrist.GetAngle(false).in(Degree) == (360 - IO.Misc.WRIST_ENCODER_OFFSET) || m_robotContainer.wrist.GetAngle(true).in(Degree) < 0;

    SmartDashboard.putNumber("Controller Y", ManipulatorController.getLeftY());
    SmartDashboard.putNumber("Intake Timer", intakeTimer.get());

    if (m_robotContainer.intake.GetBeamBreak() && m_robotContainer.pivot.GetSetpoint() == PID.Pivot.PIVOT_INTAKE) {
      intakeTimer.start();
      if (intakeTimer.get() >= 0.3) {
        intakeTimer.stop();
        intakeTimer.reset();
        m_robotContainer.pivot.Set(PID.Pivot.PIVOT_L4);
      }
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
