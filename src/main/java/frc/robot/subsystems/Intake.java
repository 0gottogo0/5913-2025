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

  private SparkMax intake = new SparkMax(kIntakeMotor, MotorType.kBrushless);
  private SparkMaxConfig cfg = new SparkMaxConfig();
  
  private PneumaticHub PH = new PneumaticHub(kPHID);
  
  private DoubleSolenoid clawSolenoid = PH.makeDoubleSolenoid(kClawClose, kClawOpen);
  
  private DigitalInput beamBreak = new DigitalInput(kBeamBreak);

  public boolean ignoreBeamBreak = false;
  public boolean holdAlgae = false;

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

    SmartDashboard.putBoolean("Claw Is Loaded", GetBeamBreak());
  }

  public void RunIntake() {
    intake.set(-kIntakeSpeed);
  }

  public  void RunIntakeReverse() {
    intake.set(kIntakeSpeed);
  }

  public void RunIntakeWithBeam() {
    if (!ignoreBeamBreak) {
      if (!GetBeamBreak()) {
        intake.set(-kIntakeSpeed);
      } else {
        Stop();
      }
    } else {
      intake.set(-kIntakeSpeed);
    }
  }

  public void EjectAlgae() {
    clawSolenoid.set(DoubleSolenoid.Value.kForward);
    intake.set(kIntakeSpeedMax);
    holdAlgae = false;
  }

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

  // Whenever GetBeamBreak is true, there is a coral (or algae) in the claw
  public boolean GetBeamBreak() {
    return !beamBreak.get();
  }

  public void Stop() {
    if (holdAlgae) {
      intake.set(kIntakeSpeedHoldAlgae);
    } else if (!holdAlgae) {
      intake.set(0);
    }
  }
}
