// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.Constants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.generated.TunerConstants;

public class Camera extends SubsystemBase {
  private PIDController reefXController = new PIDController(kTrackXKP, kTrackXKI, kTrackXKD);
  private PIDController reefYController = new PIDController(kTrackYKP, kTrackYKI, kTrackYKD);
  private PIDController reefRotController = new PIDController(kTrackRotKP, kTrackRotKI, kTrackRotKD);

  private double reefX;
  private double reefY;
  private double reefRot;

  private double tx;
  private double ty;

  private double xToTarget = 0;
  private double yToTarget = 0;
  private Angle rotToTarget = Degrees.of(0);

  /** Creates a new Camera. */
  public Camera() {
    reefXController.setTolerance(kTrackTolerance);
    reefYController.setTolerance(kTrackTolerance);
    reefRotController.setTolerance(kTrackTolerance);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    double[] results = NetworkTableInstance.getDefault().getTable("limelight-ll").getEntry("botpose_targetspace").getDoubleArray(new double[6]);

    xToTarget = results[0];
    yToTarget = results[2];
    rotToTarget = Degrees.of(results[4]);

    reefX = reefXController.calculate(xToTarget);
    reefY = reefYController.calculate(yToTarget);
    reefRot = reefRotController.calculate(rotToTarget.in(Degrees));

    SmartDashboard.putNumber("Reef PID X", reefX);
    SmartDashboard.putNumber("Reef PID Y", reefY);
    SmartDashboard.putNumber("Reef PID Rot", reefRot);
    SmartDashboard.putNumber("Reef ID", LimelightHelpers.getFiducialID(kLimeLightReef));

    SmartDashboard.putNumber("X to Target", xToTarget);
    SmartDashboard.putNumber("Y to Target", yToTarget);
    SmartDashboard.putNumber("Rot to Target", rotToTarget.in(Degrees));
  }

  public double MoveReefX(double position) {
    // double tx = LimelightHelpers.getTX(kLimeLightReef); // Get April Tag X
    
    // LimelightHelpers.setLEDMode_ForceOn(kLimeLightReef);

    // Calculate pid
    // reefX = reefXController.calculate(position, tx);
    // reefX = reefX / 1.6;
    
    reefXController.setSetpoint(position);

    return -MathUtil.clamp(reefX, -0.7, 0.7);
  }

  public double MoveReefY(double position) {
    //double ty = LimelightHelpers.getTY(kLimeLightReef); // Get April Tag Y

    //LimelightHelpers.setLEDMode_ForceOn(kLimeLightReef);

    // Calculate pid

    reefYController.setSetpoint(position);

    return MathUtil.clamp(reefY, -0.7, 0.7);
  }

  public double MoveReefRot(Angle position) {

    reefRotController.setSetpoint(position.in(Degrees));
    
    return reefRot;
  } 
  
  public void SetLEDOff() {
    LimelightHelpers.setLEDMode_ForceOff(kLimeLightReef);
  }

}
