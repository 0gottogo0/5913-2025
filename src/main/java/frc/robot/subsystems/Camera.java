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

  private PIDController trackController = new PIDController(Constants.kTrackKP, 0, Constants.kTrackKD);

  private double pidx;
  private double pidy;

  /** Creates a new Camera. */
  public Camera() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Reef TX", LimelightHelpers.getTX(ReefLL));
    SmartDashboard.putNumber("Reef TY", LimelightHelpers.getTY(ReefLL));
    SmartDashboard.putNumber("Reef PID X", pidx);
    SmartDashboard.putNumber("Reef PID Y", pidy);
    SmartDashboard.putNumber("Reef ID", LimelightHelpers.getFiducialID(ReefLL));
  }

  public double MoveReefX(double position) {
    double tx = LimelightHelpers.getTX(ReefLL); // Get April Tag X

    // Calculate pid
    pidx = trackController.calculate(tx, position);
    return pidx;
  }
  
  public double MoveReefY(double position) {
    double ty = LimelightHelpers.getTY(ReefLL); // Get April Tag Y

    // Calculate pid
    pidy = trackController.calculate(ty, position);
    return pidy;
  }
  
  // TODO: Get the april tag id and turn that into a rotation that then is fed through a pid to rotate a robot or somthing
  public double GetID() {
    double id = LimelightHelpers.getFiducialID(ReefLL); // Get April Tag ID
    return id;
  }
}
