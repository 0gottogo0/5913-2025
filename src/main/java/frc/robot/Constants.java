// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;

public final class Constants {

    // ** Controllers **
    public static final int kDriverController = 0;
    public static final int kManipulatorController = 1;
    public static final double kTriggerThreshold = 0.5;

    // ** Speeds **
    public static final double kElevatorSpeedMax = 0.85;
    public static final double kIntakeSpeedMax = -1;
    public static final double kIntakeSpeed = 0.4;
    public static final double kPivotSpeedMax = 1;
    public static final double kWristSpeedMax = 0.15;

    // ** Rate Limiters **
    public static final double kMoveSlewRateLimiter = 25;
    public static final double kRotateSlewRateLimiter = 40;

    // ** Close Looped Controllers **

        // Arm
        public static final double kArmKP = 0;
        public static final double kArmKD = 0;
        public static final double kArmIntake = 0;
        public static final double kArmBarge = 0;
        public static final double kArmProcessor = 0;
        public static final double kArmHome = 0;
        public static final double kArmClimb = 0;
        public static final double kArmL1 = 0;
        public static final double kArmL2 = 0;
        public static final double kArmL3 = 0;
        public static final double kArmL4 = 0;
        public static final double kArmTolerance = 0;

        // Elevator
        public static final double kElevatorKP = 0.2;
        public static final double kElevatorKD = 0;
        public static final double kElevatorIntake = 0;
        public static final double kElevatorBarge = -124.6;
        public static final double kElevatorProcessor = 0;
        public static final double kElevatorHome = 0;
        public static final double kElevatorClimb = 0;
        public static final double kElevatorGround = -24;
        public static final double kElevatorL1 = 0;
        public static final double kElevatorL2 = -22;
        public static final double kElevatorL3 = -57;
        public static final double kElevatorL4 = -124.6;
        public static final double kElevatorBottomAlge = -69;
        public static final double kElevatorTopAlge = 0;
        public static final double kElevatorTolerance = 0.5;
    
        // Pivot
        public static final double kPivotKP = 0.1;
        public static final double kPivotKD = 0;
        public static final double kPivotTolerance = 0;
        public static final double kPivotL1 = 0;
        public static final double kPivotL2 = 128;
        public static final double kPivotL3 = 135;
        public static final double kPivotL4 = 140;
        public static final double kPivotIntake = 149;
        public static final double kPivotHome = 135;
        public static final double kPivotClimb = 117;
        public static final double kPivotClimbEnd = 97;
        public static final double kPivotGround = 96;
    
        // Track
        public static final double kTrackXKP = 5;
        public static final double kTrackXKI = 0.00;
        public static final double kTrackXKD = 0.00;
        public static final double kTrackYKP = 5;
        public static final double kTrackYKI = 0.00;
        public static final double kTrackYKD = 0.00;
        public static final double kTrackRotKP = 0.15;
        public static final double kTrackRotKI = 0.00;
        public static final double kTrackRotKD = 0.00;
        public static final double kTrackTolerance = 0.01;
        public static final double kTrackDistanceLeft = 3.46;
        public static final double kTrackDistanceRight = 7.41;
        public static final double kTrackOffsetLeft = 5.62;
        public static final double kTrackOffsetRight = -23.00;
        public static final double kTrackXOffsetLeft = -0.173;
        public static final double kTrackYOffsetLeft = -0.494;
        public static final Angle kTrackRotOffsetLeft = Degrees.of(0);
        public static final double kTrackXOffsetRight = 0.159;
        public static final double kTrackYOffsetRight = -0.484;
        public static final Angle kTrackRotOffsetRight = Degrees.of(0);
    
        // Wrist
        public static final double kWristKP = 0.015;
        public static final double kWristKD = 0;
        public static final double kWristTolerance = 0;
        public static final double kWristIntake = 321;
        public static final double kWristBarge = 0;
        public static final double kWristProcessor = 0;
        public static final double kWristHome = 321;
        public static final double kWristClimb = 0;
        public static final double kWristGround = 88;
        public static final double kWristL1 = 0;
        public static final double kWristL2 = 321;
        public static final double kWristL3 = 321;
        public static final double kWristL4 = 273;
        public static final double kWristAlge = 0;
        
    // ** Drivetrain **
    public static final double kTrackWidthX = 23.5;   // These are how far the center of the swerves are apart
    public static final double kTrackWidthY = 23.375; // Same but back to front
    public static final double kDrivetrainMaxSpeed = 17.1;   // ft/s
    public static final double kDrivetrainWheelDiameter = 2; // inches
    public static final double kDrivetrainGearRatio = 6.1224489;
    public static final double kDrivetrainSteerRatio = 12.8;
    public static final double kFrameWidth = 30;     // in
    public static final double kFrameLength = 29.75; // in
        
    // ** Motors **
        
        // Drivetrain
        public static final int kFrontLeftDrive = 2;
        public static final int kFrontLeftSteer = 1;
        public static final int kFrontRightDrive = 5;
        public static final int kFrontRightSteer = 3;
        public static final int kBackLeftDrive = 6;
        public static final int kBackLeftSteer = 4;
        public static final int kBackRightDrive = 7;
        public static final int kBackRightSteer = 8;

        // Misc
        public static final int kIntakeMotor = 24;
        public static final int kPivotLeftMotor = 20;
        public static final int kPivotRightMotor = 17;
        public static final int kElevatorMotor = 30;
        public static final int kWristMotor = 26;

    // ** Pnumatics **
    public static final int kPHID = 42;
    public static final int kMinPressure = 110;
    public static final int kMaxPressure = 120;
    public static final int kClawOpen = 0;
    public static final int kClawClose = 1;

    // ** Input **

        // Drivetrain
        public static final int kFrontLeftEncoder = 9;
        public static final int kFrontRightEncoder = 12;
        public static final int kBackLeftEncoder = 10;
        public static final int kBackRightEncoder = 11;
        public static final int kPigeon = 13;

        // Misc
        public static final int kBeamBreak = 9;
        public static final int kPivotEncoderID = 2;
        public static final double kPivotEncoderOffset = 0;
        public static final int kWristEncoderID = 1;
        public static final double kWristEncoderOffset = 0;
        public static final String kLimeLightReef = "limelight-ll";
}
