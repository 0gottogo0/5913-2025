// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {

    // ** Controllers **
    public static final int kDriverController = 0;
    public static final int kManipulatorController = 1;
    public static final double kTriggerThreshold = 0.5;

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

        // Arm
        public static final double kArmKP = 0;
        public static final double kArmKD = 0;
        public static final double kArmIntake = 0;
        public static final double kArmBarge = 0;
        public static final double kArmProcessor = 0;
        public static final double kArmHome = 0;
        public static final double kArmClimb = 0;
        public static final double kArmL1 = 0;
        public static final double kArmL2or3 = 0;
        public static final double kArmL4 = 0;
        public static final double kArmTolerance = 0;

        // Elevator
        public static final double kElevatorKS = 0;
        public static final double kElevatorKG = 0;
        public static final double kElevatorKV = 0;
        public static final double kElevatorIntake = 0;
        public static final double kElevatorBarge = 0;
        public static final double kElevatorProcessor = 0;
        public static final double kElevatorHome = 0;
        public static final double kElevatorClimb = 0;
        public static final double kElevatorL1 = 0;
        public static final double kElevatorL2 = 0;
        public static final double kElevatorL3 = 0;
        public static final double kElevatorL4 = 0;
        public static final double kElevatorBottomAlge = 0;
        public static final double kElevatorTopAlge = 0;
    
        // Pivot
        public static final double kPivotKP = 0;
        public static final double kPivotKD = 0;
        public static final double kPivotTolerance = 0;
        public static final double kPivotReef = 0;
        public static final double kPivotIntake = 0;
        public static final double kPivotHome = 0;
        public static final double kPivotClimb = 0;
    
        // Track
        public static final double kTrackKP = 0;
        public static final double kTrackKD = 0;
        public static final double kTrackTolerance = 0;
        public static final double kTrackDistance = 0;
        public static final double kTrackOffset = 0;
    
        // Wrist
        public static final double kWristKP = 0;
        public static final double kWristKD = 0;
        public static final double kWristTolerance = 0;
        public static final double kWristIntake = 0;
        public static final double kWristBarge = 0;
        public static final double kWristProcessor = 0;
        public static final double kWristHome = 0;
        public static final double kWristClimb = 0;
        public static final double kWristL1 = 0;
        public static final double kWristL2or3 = 0;
        public static final double kWristL4 = 0;
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
        public static final int kFrontLeftDrive = 1;
        public static final int kFrontLeftSteer = 2;
        public static final int kFrontRightDrive = 4;
        public static final int kFrontRightSteer = 5;
        public static final int kBackLeftDrive = 7;
        public static final int kBackLeftSteer = 8;
        public static final int kBackRightDrive = 10;
        public static final int kBackRightSteer = 11;

        // Misc
        public static final int kArmMotor = 25;
        public static final int kClawMotor = 24;
        public static final int kPivotLeftMotor = 21;
        public static final int kPivotRightMotor = 22;
        public static final int kElevatorMotor = 23;
        public static final int kWristMotor = 26;

    // ** Pnumatics **
    public static final int kPnumaticModule = 14;
    public static final int kClawOpen = 0;
    public static final int kClawClose = 1;

    // ** Input **

        // Drivetrain
        public static final int kFrontLeftEncoder = 3;
        public static final int kFrontRightEncoder = 6;
        public static final int kBackLeftEncoder = 9;
        public static final int kBackRightEncoder = 12;
        public static final int kPigeon = 13;

        // Misc
        public static final int kBeamBreak = 2;
        public static final int kPivotEncoderID = 0;
        public static final double kPivotEncoderOffset = 0;
        public static final int kWristEncoderID = 1;
        public static final double kWristEncoderOffset = 0;
        public static final String kLimeLightReef = "limelight3";
}
