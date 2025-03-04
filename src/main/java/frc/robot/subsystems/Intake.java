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
import frc.robot.Constants.Speeds;

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
    intake.set(-Speeds.kIntakeSpeed);
  }

  // Run the intake backwards
  public void RunIntakeReverse() {
    intake.set(Speeds.kIntakeSpeed);
  }

  // Run the intake with the beambreak
  // If we arnt in the intake position we can just run the intake normally
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

  // Open the claw if algae is true
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

  // Get the status of the beambreak
  // Whenever GetBeamBreak is true, there is a coral (or algae) in the claw and the beam is broken
  public boolean GetBeamBreak() {
    return !beamBreak.get();
  }

  // Stop the intake
  // Keeps intaking slightly if we are in an algae spot
  public void Stop() {
    if (holdAlgae) {
      intake.set(Speeds.kIntakeSpeedHoldAlgae);
    } else if (!holdAlgae) {
      intake.set(0);
    }
  }
}
