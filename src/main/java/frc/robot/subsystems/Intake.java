// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private SparkMax intake = new SparkMax(MotorIDs.Misc.kIntakeMotor, MotorType.kBrushless);
  private SparkMaxConfig cfg = new SparkMaxConfig();
  
  private PneumaticHub PH = new PneumaticHub(PneumaticsIDs.kPHID);
  
  private DoubleSolenoid clawSolenoid = PH.makeDoubleSolenoid(PneumaticsIDs.kClawClose, PneumaticsIDs.kClawOpen);
  
  private DigitalInput beamBreak = new DigitalInput(IO.Misc.kBeamBreak);

  public boolean ignoreBeamBreak = false;
  public boolean holdAlgae = false; // Turn this varable true if we are in an algae spot

  /** Creates a new Claw. */
  public Intake() {
    cfg
      .inverted(false)
      .idleMode(IdleMode.kCoast);

    clawSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Debug
    SmartDashboard.putBoolean("Claw Is Loaded", GetBeamBreak());
  }

  // Run the intake without the beambreak
  public void RunIntake() {
    intake.set(-Speeds.kIntakeSpeedMax);
  }

  // Run the intake backwards
  public void RunIntakeReverse() {
    intake.set(Speeds.kIntakeSpeedMax);
  }

  // Run the intake with the beambreak
  public void RunIntakeWithBeam() {
    if (!ignoreBeamBreak) {
      if (!GetBeamBreak()) {
        intake.set(-Speeds.kIntakeSpeed);
      } else {
        Stop();
      }
    } else {
      RunIntake();
    }
  }

  // Eject the Algae by closing the claw and outtake
  public void EjectAlgae() {
    clawSolenoid.set(DoubleSolenoid.Value.kForward);
    intake.set(Speeds.kIntakeSpeedMax);
    holdAlgae = false;
  }

  /**
   * Open or close claw
   * @param algae false = close claw
   */
  public void Open(boolean algae) {
    if (!algae) {
      clawSolenoid.set(DoubleSolenoid.Value.kForward);
      holdAlgae = false;
      Stop();
    } else {
      clawSolenoid.set(DoubleSolenoid.Value.kReverse);
      holdAlgae = true;
      Stop();
    }
  }

  /**
   * Get the status of the beambreak
   * @return beambreak status, true = coral in claw
   */
  public boolean GetBeamBreak() {
    return !beamBreak.get();
  }

  // Stop the intake
  public void Stop() {
    if (holdAlgae) {
      intake.set(Speeds.kIntakeSpeedHoldAlgae);
    } else if (!holdAlgae) {
      intake.set(0);
    }
  }
}
