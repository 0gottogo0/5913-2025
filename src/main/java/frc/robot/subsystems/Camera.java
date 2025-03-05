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
import frc.robot.Constants.Speeds;
import frc.robot.LimelightHelpers;

public class Camera extends SubsystemBase {
  private PIDController XController = new PIDController(PID.Track.kTrackXKP, 0, 0);
  private PIDController YController = new PIDController(PID.Track.kTrackYKP, 0, 0);
  private PIDController RotController = new PIDController(PID.Track.kTrackRotKP, 0, 0);

  private double moveX;
  private double moveY;
  private double moveRot;

  private double xToTargetReef = 0;
  private double yToTargetReef = 0;
  private Angle rotToTargetReef = Degrees.of(0);

  private double xToTargetCoral = 0;
  private double yToTargetCoral = 0;
  private Angle rotToTargetCoral = Degrees.of(0);
  
  private boolean isTracking = false;

  /** Creates a new Camera. */
  public Camera() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Get the metatag2 results of where our robot is relative to the tag
    double[] resultsReef = NetworkTableInstance.getDefault().getTable(IO.Camera.kLimeLightReef).getEntry("botpose_targetspace").getDoubleArray(new double[6]);
    double[] resultsCoral = NetworkTableInstance.getDefault().getTable(IO.Camera.KLimeLightCoral).getEntry("botpose_targetspace").getDoubleArray(new double[6]);

    // Get the X, Y, and Rotation from said results
    xToTargetReef = resultsReef[0];
    yToTargetReef = resultsReef[2];
    rotToTargetReef = Degrees.of(resultsReef[4]);

    xToTargetCoral = resultsCoral[0];
    yToTargetCoral = resultsCoral[2];
    rotToTargetCoral = Degrees.of(resultsCoral[4]);

    // Calculate X, Y, and Rotation movements
    // If the reef LEDs are on then we are tracking the reef
    // If those LEDs are off then its safe to say we are tracking the coral station
    // We do this because who wants to blind the human player and I'm not doing all that
    if (isTracking) {
      moveX = XController.calculate(xToTargetReef);
      moveY = YController.calculate(yToTargetReef);
      moveRot = RotController.calculate(rotToTargetReef.in(Degrees));
    } else {
      moveX = XController.calculate(xToTargetCoral);
      moveY = YController.calculate(yToTargetCoral);
      moveRot = RotController.calculate(rotToTargetCoral.in(Degrees));
    }

    // Debug
    SmartDashboard.putNumber("PID X", moveX);
    SmartDashboard.putNumber("PID Y", moveY);
    SmartDashboard.putNumber("PID Rot", moveRot);

    SmartDashboard.putNumber("Reef X to Target", xToTargetReef);
    SmartDashboard.putNumber("Reef Y to Target", yToTargetReef);
    SmartDashboard.putNumber("Reef Rot to Target", rotToTargetReef.in(Degrees));

    SmartDashboard.putNumber("Coral X to Target", xToTargetReef);
    SmartDashboard.putNumber("Coral Y to Target", yToTargetReef);
    SmartDashboard.putNumber("Coral Rot to Target", rotToTargetReef.in(Degrees));
  }

  // Set setpoint and return X movement
  public double MoveX(double position) {
    XController.setSetpoint(position);

    return -MathUtil.clamp(moveX, -1 * Speeds.kTrackMoveMax, Speeds.kTrackMoveMax);
  }

  // Set setpoint and return Y movement
  public double MoveY(double position) {
    YController.setSetpoint(position);

    return MathUtil.clamp(moveY, -1 * Speeds.kTrackMoveMax, Speeds.kTrackMoveMax);
  }

  // Set setpoint and return Rotation movement
  public double MoveRot(Angle position) {
    RotController.setSetpoint(position.in(Degrees));
    
    return MathUtil.clamp(moveRot, -1 * Speeds.kTrackRotateMax, Speeds.kTrackRotateMax);
  } 
  
  // Set leds to off to save on power when we are not tracking
  public void SetLEDOn() {
    LimelightHelpers.setLEDMode_ForceOn(IO.Camera.kLimeLightReef);
    isTracking = true;
  }

  // Set leds to on if the venue lights are shit and to indicate tracking
  public void SetLEDOff() {
    LimelightHelpers.setLEDMode_ForceOff(IO.Camera.kLimeLightReef);
    isTracking = false;
  }

  public boolean IsTracking() {
    return isTracking;
  }
}
