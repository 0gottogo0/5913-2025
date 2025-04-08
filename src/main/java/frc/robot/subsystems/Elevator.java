// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

  private TalonFX elevator = new TalonFX(MotorIDs.Misc.kElevatorMotor);
  private TalonFXConfiguration cfg = new TalonFXConfiguration();

  private PIDController elevatorControllerUp = new PIDController(PID.Elevator.kElevatorKPUp, 0, 0);
  private PIDController elevatorControllerDown = new PIDController(PID.Elevator.kElevatorKPDown, 0, 0);

  private double elevatorSetpoint; 
  private boolean pidToggle;
  public boolean holdAglae; // Turn this varable true if we are in an algae spot

  /** Creates a new Elevator. */
  public Elevator() {
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    elevator.clearStickyFaults();
    elevator.getConfigurator().apply(cfg);

    elevatorSetpoint = GetPosition(); // Set to current encoder value so elevetor doesnt "snap" when first enabled

    pidToggle = true;
    holdAglae = false;

    // Wait to set current encoder value, sometimes it takes a tinsie bit
    new Thread(() -> {
      try {
          Thread.sleep(3000);
          elevatorSetpoint = GetPosition();
      } catch (Exception e) {
      }
    }).start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    double pid = 0;

    // Calculate pid TODO: Replace with elevatorFeedforward
    if (pidToggle) {
      if (elevatorSetpoint > GetPosition()) {
        pid = elevatorControllerUp.calculate(GetPosition(), elevatorSetpoint);
      } else {
        pid = elevatorControllerDown.calculate(GetPosition(), elevatorSetpoint);
      }
    }
    
    // Slow elevator if we have want to go to algae position
    if (!holdAglae) {
      pid = MathUtil.clamp(pid, -1 * Speeds.kElevatorSpeedMax, Speeds.kElevatorSpeedMax);
      elevator.set(pid);
    } else {
      pid = MathUtil.clamp(pid, -1 * Speeds.kElevatorSpeedAlgae, Speeds.kElevatorSpeedAlgae); // Slowed elevator
      elevator.set(pid);
    }
    
    // Debug
    SmartDashboard.putNumber("Elevator PID Input", pid);
    SmartDashboard.putNumber("Elevator Setpoint", elevatorSetpoint);
    SmartDashboard.putNumber("Elevator Encoder", GetPosition());
  }

  /**
   * Set setpoint
   * @param setpoint
   */
  public void Set(double setpoint) {
    elevatorSetpoint = setpoint;
  }

  /**
   * Set setpoint if true
   * @param setpoint
   * @param isTrue true = set setpoint
   */
  public void SetIfTrue(double setpoint, boolean isTrue) {
    if (isTrue) {
      elevatorSetpoint = setpoint;
    }
  }

  /**
   * Control setpoint with axis
   * @param input input value -1 to 1
   * @param sensitivity input multiplier
   * @param disablePID false = use pid
   */
  public void ManualMovement(double input, double sensitivity, boolean disablePID) {
    if (disablePID) {
      elevator.set(input);
      pidToggle = false;
    } else {
      elevatorSetpoint = elevatorSetpoint + input * sensitivity;
    }
  }

  // Stop elevator
  public void Stop() {
    elevatorSetpoint = GetPosition();
    pidToggle = true;
  }

  // Get internal motor encoder position
  public double GetPosition() {
    return elevator.getPosition().getValueAsDouble();
  }

  /**
   * Get current setpoint
   * @return current setpoint
   */
  public double GetSetpoint() {
    return elevatorSetpoint;
  }
}