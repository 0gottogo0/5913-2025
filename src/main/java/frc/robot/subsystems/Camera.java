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
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  private PIDController reefXController = new PIDController(kTrackKP, 0, kTrackKD);
  private PIDController reefYController = new PIDController(kTrackKP, 0, kTrackKD);

  private double reefX;
  private double reefY;

  /** Creates a new Camera. */
  public Camera() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    double tx = LimelightHelpers.getTX(kLimeLightReef);
    double ty = LimelightHelpers.getTY(kLimeLightReef);
    reefX = reefXController.calculate(tx);
    reefY = reefXController.calculate(ty);

    SmartDashboard.putNumber("Reef TX", LimelightHelpers.getTX(kLimeLightReef));
    SmartDashboard.putNumber("Reef TY", LimelightHelpers.getTY(kLimeLightReef));
    SmartDashboard.putNumber("Reef PID X", reefX);
    SmartDashboard.putNumber("Reef PID Y", reefY);
    SmartDashboard.putNumber("Reef ID", LimelightHelpers.getFiducialID(kLimeLightReef));

    SmartDashboard.putNumber("Robot X", drivetrain.getState().Pose.getX());
    SmartDashboard.putNumber("Robot Y", drivetrain.getState().Pose.getY());
  }

  public double AutoReefX(double position) {
    double tx = LimelightHelpers.getTX(kLimeLightReef); // Get April Tag X
    double ty = LimelightHelpers.getTY(kLimeLightReef); // Get April Tag Y
    double llid = LimelightHelpers.getFiducialID(kLimeLightReef);
    int id = (int)llid; // stupid dumb limelight devs using doubles insted of ints :(
                        // Its not their fault
  
    switch (id) {
      default:
        reefX = reefXController.calculate(position, tx);
        break;
      case kReefABRed:
        reefX = reefXController.calculate(position, tx);
        break;
      case kReefCDRed:
        reefX = 0.5 * ty + position;
        break;
      case kReefEFRed:
        reefX = -0.5 * ty + position;
        break;
      case kReefGHRed:
        reefX = -reefXController.calculate(position, tx);
        break;
      case kReefIJRed:
        reefX = -0.5 * ty + position;
        break;
      case kReefKLRed:
        reefX = 0.5 * ty + position;
        break;
      case kReefABBlue:
        reefX = reefXController.calculate(position, tx);
        break;
      case kReefCDBlue:
        reefX = 0.5 * ty + position;
        break;
      case kReefEFBlue:
        reefX = -0.5 * ty + position;
        break;
      case kReefGHBlue:
        reefX = -reefXController.calculate(position, tx);
        break;
      case kReefIJBlue:
        reefX = -0.5 * ty + position;
        break;
      case kReefKLBlue:
        reefX = 0.5 * ty + position;
        break;
    }
    return reefX;
  }

  public double MoveReefX(double position) {
    double tx = LimelightHelpers.getTX(kLimeLightReef); // Get April Tag X
    
    LimelightHelpers.setLEDMode_ForceOn(kLimeLightReef);

    // Calculate pid
    reefXController.setSetpoint(position);
    //reefX = reefXController.calculate(position, tx);
    reefX = reefX / 1.6;
    

    return -reefX;
  }

  public double MoveReefY(double position) {
    double ty = LimelightHelpers.getTY(kLimeLightReef); // Get April Tag Y

    LimelightHelpers.setLEDMode_ForceOn(kLimeLightReef);

    // Calculate pid
    reefYController.setSetpoint(position);

    return reefY;
  }

  public void SetLEDOff() {
    LimelightHelpers.setLEDMode_ForceOff(kLimeLightReef);
  }
}
