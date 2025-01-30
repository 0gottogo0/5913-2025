// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    // ** Controllers **
    public static final int kDriverController = 0;
    public static final int kManipulatorController = 1;
    public static final double kManipulatorTriggerThreshold = 0.2;

    // ** Speeds **
    public static final double kArmSpeedMax = 1;
    public static final double kClawSpeedMax = 0.8;
    public static final double kClawSpeedLow = 0.1;
    public static final double kElevatorSpeedMax = 0.5;
    public static final double kElevatorSpeedHome = 1;
    public static final double kPivotSpeedMax = 1;
    public static final double kWristSpeedMax = 1;

    // ** Rate Limiters **
    public static final double kMoveSlewRateLimiter = 20;
    public static final double kRotateSlewRateLimiter = 40;

    // ** Close Looped Controllers **
    public static final double kArmKP = 0;
    public static final double kArmKD = 0;
    public static final double kElevatorKS = 0;
    public static final double kElevatorKG = 0;
    public static final double kElevatorKV = 0;
    public static final double kElevatorL4 = 0;
    public static final double kElevatorL3 = 0;
    public static final double kElevatorL2 = 0;
    public static final double kElevatorL1 = 0;
    public static final double kPivotKP = 0;
    public static final double kPivotKD = 0;
    public static final double kPivotReef = 0;
    public static final double kPivotIntake = 0;
    public static final double kPivotClimb = 0;
    public static final double kTrackKP = 0;
    public static final double kTrackKD = 0;
    public static final double kTrackDistance = 0;
    public static final double kTrackOffset = 0;
    public static final double kWristKP = 0;
    public static final double kWristKD = 0;

    // ** Motors **
    public static final int kArmMotor = 19;
    public static final int kClawMotor = 18;
    public static final int kPivotLeftMotor = 15;
    public static final int kPivotRightMotor = 16;
    public static final int kElevatorMotor = 17;
    public static final int kWristMotor = 20;

    // ** Pnumatics **
    public static final int kPnumaticModule = 14;
    public static final int kClawOpen = 0;
    public static final int kClawClose = 1;

    // ** Input **
    public static final int kBeamBreak = 1;
    public static final String kLimeLightReef = "limelight3";
}
