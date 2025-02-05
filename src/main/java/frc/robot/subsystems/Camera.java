// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class Camera extends SubsystemBase {
  public final String ReefLL = Constants.kLimeLightReef;

  private PIDController reefXController = new PIDController(Constants.kTrackKP, 0, Constants.kTrackKD);
  private PIDController reefYController = new PIDController(Constants.kTrackKP, 0, Constants.kTrackKD);

  private double reefX;
  private double reefY;

  /** Creates a new Camera. */
  public Camera() {
    reefXController.setTolerance(Constants.kTrackTolerance);
    reefYController.setTolerance(Constants.kTrackTolerance);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Reef TX", LimelightHelpers.getTX(ReefLL));
    SmartDashboard.putNumber("Reef TY", LimelightHelpers.getTY(ReefLL));
    SmartDashboard.putNumber("Reef PID X", reefX);
    SmartDashboard.putNumber("Reef PID Y", reefY);
    SmartDashboard.putNumber("Reef ID", LimelightHelpers.getFiducialID(ReefLL));
  }

  public double MoveReefX(double position) {
    double tx = LimelightHelpers.getTX(ReefLL); // Get April Tag X

    // Calculate pid
    if(!reefXController.atSetpoint()) {
      reefX = reefXController.calculate(tx, position);
    }

    return reefX;
  }
  
  public double MoveReefY(double position) {
    double ty = LimelightHelpers.getTY(ReefLL); // Get April Tag Y

    // Calculate pid
    if(!reefYController.atSetpoint()) {
      reefY = reefYController.calculate(ty, position);
    }

    return reefY;
  }
}
