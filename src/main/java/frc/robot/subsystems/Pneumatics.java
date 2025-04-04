// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {

  PneumaticHub PH = new PneumaticHub(PneumaticsIDs.kPHID);

  /** Creates a new Pneumatics. */
  public Pneumatics() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Debug
    SmartDashboard.putNumber("Pressure", PH.getPressure(0));

    // Enable the compressor
    PH.enableCompressorAnalog(PneumaticsIDs.kMinPressure, PneumaticsIDs.kMaxPressure);
  }
}
