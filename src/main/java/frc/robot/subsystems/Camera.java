// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;

public class Camera extends SubsystemBase {

  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain(); // My drivetrain

  public final String ReefLL = Constants.kLimeLightReef;

  private PIDController reefXController = new PIDController(Constants.kTrackKP, 0, Constants.kTrackKD);
  private PIDController reefYController = new PIDController(Constants.kTrackKP, 0, Constants.kTrackKD);
  private PIDController reefRotateController = new PIDController(Constants.kTrackKP, 0, Constants.kTrackKD);

  private double reefX;
  private double reefY;
  private double reefRotate;

  /** Creates a new Camera. */
  public Camera() {}

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
    reefX = reefXController.calculate(tx, position);
    return reefX;
  }
  
  public double MoveReefY(double position) {
    double ty = LimelightHelpers.getTY(ReefLL); // Get April Tag Y

    // Calculate pid
    reefY = reefYController.calculate(ty, position);
    return reefY;
  }
  
  public double RotateReef() {
    double id = LimelightHelpers.getFiducialID(ReefLL); // Get April Tag ID
    double tr = 0;

    // Calculate rotation
    if (id >= 18 && id <= 18 || id >= 7 && id <= 7) {
      tr = 180;
    } else if (id >= 19 && id <= 19 || id >= 6 && id <= 6) {
      tr = 240;
    } else if (id >= 20 && id <= 20 || id >= 11 && id <= 11) {
      tr = 300;
    } else if (id >= 21 && id <= 21 || id >= 10 && id <= 10) {
      tr = 0;
    } else if (id >= 22 && id <= 22 || id >= 9 && id <= 9) {
      tr = 60;
    } else if (id >= 17 && id <= 17 || id >= 8 && id <= 8) {
      tr = 120;
    }

    reefRotate = reefRotateController.calculate(drivetrain.getState().Pose.getRotation().getDegrees(), tr);
    return reefRotate;
  }
}
