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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Get the metatag2 results of where our robot is relative to the tag
    double[] resultsReef = NetworkTableInstance.getDefault().getTable(IO.Camera.kLimeLightReef).getEntry("botpose_targetspace").getDoubleArray(new double[6]);
    double[] resultsCoral = NetworkTableInstance.getDefault().getTable(IO.Camera.kLimeLightCoral).getEntry("botpose_targetspace").getDoubleArray(new double[6]);

    // Get the X, Y, and Rotation from said results
    xToTargetReef = resultsReef[0];
    yToTargetReef = resultsReef[2];
    rotToTargetReef = Degrees.of(resultsReef[4]);

    xToTargetCoral = resultsCoral[0];
    yToTargetCoral = resultsCoral[2];
    rotToTargetCoral = Degrees.of(resultsCoral[4]);

    // Calculate X, Y, and Rotation movements
    // Also check if we are going to track the reef or coral station
    if (isReefTracking) {
      moveX = XReefController.calculate(xToTargetReef);
      moveY = YReefController.calculate(yToTargetReef);
      moveRot = RotReefController.calculate(rotToTargetReef.in(Degrees));
    } else {
      moveX = XCoralController.calculate(xToTargetCoral);
      moveY = YCoralController.calculate(yToTargetCoral) + 0.05; // Drive into the coral station a bit
      moveRot = RotCoralController.calculate(rotToTargetCoral.in(Degrees));
    }

    // Debug
    SmartDashboard.putNumber("PID X", moveX);
    SmartDashboard.putNumber("PID Y", moveY);
    SmartDashboard.putNumber("PID Rot", moveRot);

    SmartDashboard.putNumber("Reef X to Target", xToTargetReef);
    SmartDashboard.putNumber("Reef Y to Target", yToTargetReef);
    SmartDashboard.putNumber("Reef Rot to Target", rotToTargetReef.in(Degrees));

    SmartDashboard.putNumber("Coral X to Target", xToTargetCoral);
    SmartDashboard.putNumber("Coral Y to Target", yToTargetCoral);
    SmartDashboard.putNumber("Coral Rot to Target", rotToTargetCoral.in(Degrees));
  }

  
  /**
   * Set setpoint and return X movement
   * @param position
   * @param coral false = reef
   * @return pid output
   */
  public double MoveX(double position, boolean coral) {
    XReefController.setSetpoint(position);

    isReefTracking = !coral;
    return -MathUtil.clamp(moveX, -Speeds.kTrackReefMoveMax, Speeds.kTrackReefMoveMax);
  }

  
  /**
   * Set setpoint and return Y movement
   * @param position
   * coral or reef controlled by moveX
   * @return pid output 
   */
  public double MoveY(double position) {
    YReefController.setSetpoint(position);
    return MathUtil.clamp(moveY, -Speeds.kTrackReefMoveMax, Speeds.kTrackReefMoveMax);
  }

  /**
   * Set setpoint and return Rotation movement
   * @param position
   * coral or reef controlled by moveX
   * @return pid output
   */
  public double MoveRot(Angle position) {
    RotReefController.setSetpoint(position.in(Degrees));
    return MathUtil.clamp(moveRot, -Speeds.kTrackReefRotateMax, Speeds.kTrackReefRotateMax);
  } 
  
  // Set leds to off because we aint trackin
  public void SetLEDOn() {
    LimelightHelpers.setLEDMode_ForceOn(isReefTracking?IO.Camera.kLimeLightReef:IO.Camera.kLimeLightCoral);
    isTracking = true;
  }

  // Set leds to on to indicate trackin
  public void SetLEDOff() {
    LimelightHelpers.setLEDMode_ForceOff(isReefTracking?IO.Camera.kLimeLightReef:IO.Camera.kLimeLightCoral);
    isTracking = false;
  }

  public boolean IsTracking() {
    return isTracking;
  }
}
