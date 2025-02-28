// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.Constants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Camera extends SubsystemBase {
  private PIDController reefXController = new PIDController(kTrackXKP, 0, 0);
  private PIDController reefYController = new PIDController(kTrackYKP, 0, 0);
  private PIDController reefRotController = new PIDController(kTrackRotKP, 0, 0);

  private double reefX;
  private double reefY;
  private double reefRot;

  private double xToTarget = 0;
  private double yToTarget = 0;
  private Angle rotToTarget = Degrees.of(0);
  
  private boolean isTracking = false;

  /** Creates a new Camera. */
  public Camera() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Get the metatag2 results of where our robot is relative to the tag
    double[] results = NetworkTableInstance.getDefault().getTable("limelight-ll").getEntry("botpose_targetspace").getDoubleArray(new double[6]);

    // Get the X, Y, and Rotation from said results
    xToTarget = results[0];
    yToTarget = results[2];
    rotToTarget = Degrees.of(results[4]);

    // Calculate X, Y, and Rotation movements
    reefX = reefXController.calculate(xToTarget);
    reefY = reefYController.calculate(yToTarget);
    reefRot = reefRotController.calculate(rotToTarget.in(Degrees));

    // Debug
    SmartDashboard.putNumber("Reef PID X", reefX);
    SmartDashboard.putNumber("Reef PID Y", reefY);
    SmartDashboard.putNumber("Reef PID Rot", reefRot);
    SmartDashboard.putNumber("Reef ID", LimelightHelpers.getFiducialID(kLimeLightReef));

    SmartDashboard.putNumber("X to Target", xToTarget);
    SmartDashboard.putNumber("Y to Target", yToTarget);
    SmartDashboard.putNumber("Rot to Target", rotToTarget.in(Degrees));
  }

  // Set setpoint and return X movement
  public double MoveReefX(double position) {
    reefXController.setSetpoint(position);

    return -MathUtil.clamp(reefX, -1 * kTrackMoveMax, kTrackMoveMax);
  }

  // Set setpoint and return Y movement
  public double MoveReefY(double position) {
    reefYController.setSetpoint(position);

    return MathUtil.clamp(reefY, -1 * kTrackMoveMax, kTrackMoveMax);
  }

  // Set setpoint and return Rotation movement
  public double MoveReefRot(Angle position) {
    reefRotController.setSetpoint(position.in(Degrees));
    
    return MathUtil.clamp(reefRot, -1 * kTrackRotateMax, kTrackRotateMax);
  } 
  
  // Set leds to off to save on power when we are not tracking
  public void SetLEDOn() {
    LimelightHelpers.setLEDMode_ForceOn(kLimeLightReef);
    isTracking = true;
  }

  // Set leds to on if the venue lights are shit
  public void SetLEDOff() {
    LimelightHelpers.setLEDMode_ForceOff(kLimeLightReef);
    isTracking = false;
  }

  public boolean IsTracking() {
    return isTracking;
  }
}
