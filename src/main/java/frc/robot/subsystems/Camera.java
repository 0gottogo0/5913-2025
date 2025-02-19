// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;

public class Camera extends SubsystemBase {
  public final String ReefLL = kLimeLightReef;

  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  private PIDController reefXController = new PIDController(kTrackKP, 0, kTrackKD);
  private PIDController reefYController = new PIDController(kTrackKP, 0, kTrackKD);

  private double reefX;
  private double reefY;

  /** Creates a new Camera. */
  public Camera() {
    reefXController.setTolerance(kTrackTolerance);
    reefYController.setTolerance(kTrackTolerance);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Reef TX", LimelightHelpers.getTX("limelight-ll"));
    SmartDashboard.putNumber("Reef TY", LimelightHelpers.getTY("limelight-ll"));
    SmartDashboard.putNumber("Reef PID X", reefX);
    SmartDashboard.putNumber("Reef PID Y", reefY);
    SmartDashboard.putNumber("Reef ID", LimelightHelpers.getFiducialID(kLimeLightReef));

    SmartDashboard.putNumber("Robot X", drivetrain.getState().Pose.getX());
    SmartDashboard.putNumber("Robot Y", drivetrain.getState().Pose.getY());
  }

  public double AutoReefX(double position) {
    double tx = LimelightHelpers.getTX("limelight-ll"); // Get April Tag X
    double dx = drivetrain.getState().Pose.getX();
    double llid = LimelightHelpers.getFiducialID(ReefLL);
    int id = (int)llid; // stupid dumb limelight devs using doubles insted of ints :(
  
    switch (id) {
      default:
        
        break;
      case kReefABRed:
        
        break;
      case kReefCDRed:
        
        break;
      case kReefEFRed:
        
        break;
      case kReefGHRed:
        
        break;
      case kReefIJRed:
        
        break;
      case kReefKLRed:
        
        break;
      case kReefABBlue:
        
        break;
      case kReefCDBlue:
        
        break;
      case kReefEFBlue:
        
        break;
      case kReefGHBlue:
        
        break;
      case kReefIJBlue:
        
        break;
      case kReefKLBlue:
        
        break;
    }
    return 0;
  }

  public double MoveReefX(double position) {
    double tx = LimelightHelpers.getTX("limelight-ll"); // Get April Tag X

    // Calculate pid
    reefX = tx - position;
    reefX = MathUtil.clamp(reefX, 0.2, 0.2);

    return reefX;
  }

  public double MoveReefY(double position) {
    double ty = LimelightHelpers.getTY("limelight-ll"); // Get April Tag Y

    // Calculate pid
    reefY = ty - position;
    reefY = MathUtil.clamp(reefY, 0.2, 0.2);

    return reefY;
  }
}
