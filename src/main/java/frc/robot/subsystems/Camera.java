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
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Speeds;
import frc.robot.LimelightHelpers;

public class Camera extends SubsystemBase {
  private PIDController XReefController = new PIDController(PID.Track.kReefTrackXKP, 0, 0);
  private PIDController YReefController = new PIDController(PID.Track.kReefTrackYKP, 0, 0);
  private PIDController RotReefController = new PIDController(PID.Track.kReefTrackRotKP, 0, 0);

  private PIDController XCoralController = new PIDController(PID.Track.kCoralTrackXKP, 0, 0);
  private PIDController YCoralController = new PIDController(PID.Track.kCoralTrackYKP, 0, 0);
  private PIDController RotCoralController = new PIDController(PID.Track.kCoralTrackRotKP, 0, 0);

  private double moveX;
  private double moveY;
  private double moveRot;

  private double xToTargetReef = 0;
  private double yToTargetReef = 0;
  private Angle rotToTargetReef = Degrees.of(0);

  // Coral Station abreviated to "Coral"
  private double xToTargetCoral = 0;
  private double yToTargetCoral = 0;
  private Angle rotToTargetCoral = Degrees.of(0);
  
  private boolean isTracking = false;
  private boolean isReefTracking = true;

  /** Creates a new Camera. */
  public Camera() {}

  @SuppressWarnings("unused")
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Get the metatag2 results of where our robot is relative to the tag
    double[] resultsReef = NetworkTableInstance.getDefault().getTable(IO.Camera.kLimeLightReef).getEntry("botpose_targetspace").getDoubleArray(new double[6]);
    double[] resultsCoral = NetworkTableInstance.getDefault().getTable(IO.Camera.kLimeLightCoral).getEntry("botpose_targetspace").getDoubleArray(new double[6]);

    var llReefMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(IO.Camera.kLimeLightReef);
    var llCoralMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(IO.Camera.kLimeLightCoral);

    // Get the X, Y, and Rotation from said results
    xToTargetReef = resultsReef[0];
    yToTargetReef = resultsReef[2];
    rotToTargetReef = Degrees.of(resultsReef[4]);

    xToTargetCoral = resultsCoral[0];
    yToTargetCoral = resultsCoral[2];
    rotToTargetCoral = Degrees.of(resultsCoral[4]);

    // We dont need to run though this logic twice but we will
    CalculatePID(llReefMeasurement != null && llReefMeasurement.tagCount > 0 || llCoralMeasurement != null && llCoralMeasurement.tagCount > 0);

    // Debug
    SmartDashboard.putNumber("PID X", moveX);
    SmartDashboard.putNumber("PID Y", moveY);
    SmartDashboard.putNumber("PID Rot", moveRot);
    SmartDashboard.putBoolean("Tracking Tag", llReefMeasurement != null && llReefMeasurement.tagCount > 0);

    SmartDashboard.putNumber("Reef X to Target", xToTargetReef);
    SmartDashboard.putNumber("Reef Y to Target", yToTargetReef);
    SmartDashboard.putNumber("Reef Rot to Target", rotToTargetReef.in(Degrees));

    SmartDashboard.putNumber("Coral X to Target", xToTargetCoral);
    SmartDashboard.putNumber("Coral Y to Target", yToTargetCoral);
    SmartDashboard.putNumber("Coral Rot to Target", rotToTargetCoral.in(Degrees));

    if (llReefMeasurement == null) {
      DataLogManager.log("Reef Camera Lost!");
    }

    if (llCoralMeasurement == null) {
      DataLogManager.log("Coral Camera Lost!");
    }
  }

  public void CalculatePID (boolean calculate) {
    var llReefMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(IO.Camera.kLimeLightReef);
    var llCoralMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(IO.Camera.kLimeLightCoral);

    if (!calculate) {
      moveX = 0;
      moveY = 0;
      moveRot = 0;
      return; // Exit function early if we see no april tags
    }

    if (isReefTracking) {
      if (llReefMeasurement != null && llReefMeasurement.tagCount > 0) {
        moveX = XReefController.calculate(xToTargetReef);
        moveY = YReefController.calculate(yToTargetReef);
        moveRot = RotReefController.calculate(rotToTargetReef.in(Degrees));
      }
    } else {
      // TODO: Track correctly
      if (llCoralMeasurement != null && llCoralMeasurement.tagCount > 0) {
        moveX = XCoralController.calculate(xToTargetCoral) + 0.1; // Drive to the side a bit
        moveY = YCoralController.calculate(yToTargetCoral);
        moveRot = RotCoralController.calculate(rotToTargetCoral.in(Degrees));
      }
    }
  }

  public double MoveX(double position, boolean coral) {
    XReefController.setSetpoint(position);
    isReefTracking = !coral;

    // Move slower in tele so brodie can cope
    if (DriverStation.isTeleop()) {
      return -MathUtil.clamp(moveX, -Speeds.kTrackMoveSlow, Speeds.kTrackMoveSlow);
    }

    return -MathUtil.clamp(moveX, isReefTracking?-Speeds.kTrackMoveMax:-Speeds.kTrackMoveSlow, isReefTracking?Speeds.kTrackMoveMax:Speeds.kTrackMoveSlow);
  }

  public double MoveY(double position) {
    YReefController.setSetpoint(position);

    // Move slower in tele so brodie can cope
    if (DriverStation.isTeleop()) {
      return MathUtil.clamp(moveY, -Speeds.kTrackMoveSlow, Speeds.kTrackMoveSlow);
    }

    return MathUtil.clamp(moveY, isReefTracking?-Speeds.kTrackMoveMax:-Speeds.kTrackMoveSlow, isReefTracking?Speeds.kTrackMoveMax:Speeds.kTrackMoveSlow);
  }

  public double MoveRot(Angle position) {
    RotReefController.setSetpoint(position.in(Degrees));
    return MathUtil.clamp(moveRot, -Speeds.kTrackRotateMax, Speeds.kTrackRotateMax);
  } 
  
  public void SetLEDOn() {
    LimelightHelpers.setLEDMode_ForceOn(isReefTracking?IO.Camera.kLimeLightReef:IO.Camera.kLimeLightCoral);
    isTracking = true;
  }

  public void SetLEDOff() {
    LimelightHelpers.setLEDMode_ForceOff(isReefTracking?IO.Camera.kLimeLightReef:IO.Camera.kLimeLightCoral);
    isTracking = false;
  }

  public boolean IsTracking() {
    return isTracking;
  }
}
