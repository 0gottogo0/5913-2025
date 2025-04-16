// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
    public static final class Controllers {
        public static final int kDriverController = 0;
        public static final int kManipulatorController = 1;
    }

    public static final class Speeds {
        public static final double kElevatorSpeedMax = 1;
        public static final double kElevatorSpeedAlgae = 0.8;
        public static final double kIntakeSpeedMax = 1;
        public static final double kIntakeSpeed = 0.5;
        public static final double kIntakeSpeedHoldAlgae = -0.15;
        public static final double kPivotSpeedMax = 1;
        public static final double kTrackMoveMax = .3;
        public static final double kTrackMoveSlow = .2;
        public static final double kTrackRotateMax = 0.1;
        public static final double kWristSpeedMax = 0.7;
    }


    public static final class Drivetrain {
        public static final double kMoveSlewRateLimiter = 12.00;
        public static final double kRotateSlewRateLimiter = 40.00;
        public static final double kRotateMagnitude = 0.90;
        public static final double kStickDeadzone = 0.02;

        // CTRE
        public static final double kTrackWidthX = 23.5;   // These are how far the center of the swerves are apart
        public static final double kTrackWidthY = 23.375; // Same but back to front
        public static final double kDrivetrainMaxSpeed = 17.1;      // ft/s
        public static final double kDrivetrainWheelDiameter = 1.95; // inches
        public static final double kDrivetrainGearRatio = 6.1224489;
        public static final double kDrivetrainSteerRatio = 12.8;
        public static final double kFrameWidth = 30;     // inches
        public static final double kFrameLength = 29.75; // inches
    }

    public static final class PID {
        public static final class Elevator {
        public static final double kElevatorKPUp = 0.12;
        public static final double kElevatorKPDown = 0.08;
        public static final double kElevatorIntake = 0.00;
        public static final double kElevatorBarge = -45.20;
        public static final double kElevatorProcessor = 0.00;
        public static final double kElevatorHome = 0.00;
        public static final double kElevatorClimb = 0.00;
        public static final double kElevatorL1 = -0.00;
        public static final double kElevatorL2 = -8.00;
        public static final double kElevatorL3 = -20.80;
        public static final double kElevatorL4 = -45.20;
        public static final double kElevatorBottomAlgae = -13.40;
        public static final double kElevatorTopAlgae = -26.50;
        public static final double kElevatorGroundAlgae = -5.2;
        }

        public static final class Pivot {
            public static final double kPivotKP = 0.09;
            public static final double kPivotL1 = 117.30;
            public static final double kPivotL2 = 128.00;
            public static final double kPivotL3 = 135.00;
            public static final double kPivotL4 = 140.70;
            public static final double kPivotIntake = 150.60;
            public static final double kPivotProcessor = 113.80;
            public static final double kPivotHome = 135.00;
            public static final double kPivotClimb = 122.50;
            public static final double kPivotClimbEnd = 95.50;
            public static final double kPivotBottomAlgae = 127.10;
            public static final double kPivotTopAlgae = 131.70;
            public static final double kPivotGroundAlgae = 99.70;
            public static final double kPivotDeadzone = 0.05;
        }

        public static final class Track {
            public static final double kReefTrackXKP = 1.00;
            public static final double kReefTrackYKP = 0.75;
            public static final double kReefTrackRotKP = 0.04;
            public static final double kCoralTrackXKP = 0.50;
            public static final double kCoralTrackYKP = 0.50;
            public static final double kCoralTrackRotKP = 0.03;
            public static final double kTrackXOffsetLeft = -0.193;
            public static final double kTrackYOffsetLeft = -0.464;
            public static final double kTrackXOffsetCenter = -0.050;
            public static final double kTrackYOffsetCenter = -0.590;
            public static final double kTrackXOffsetRight = 0.153;
            public static final double kTrackYOffsetRight = -0.464;
            public static final double kTrackXOffsetCoral = 0.26;  
            public static final double kTrackYOffsetCoral = -0.48;
        }
        
        public static final class Wrist {
            public static final double kWristKP = 0.015;
            public static final double kWristIntake = 289.80;
            public static final double kWristBarge = 218.3;
            public static final double kWristProcessor = 85.70;
            public static final double kWristHome = 305.60;
            public static final double kWristL1 = 312.00;
            public static final double kWristL2 = 305.60;
            public static final double kWristL3 = 295.00;
            public static final double kWristL4 = 237.00;
            public static final double kWristBottomAlgae = 82.50;
            public static final double kWristTopAlgae = 88.10;
            public static final double kWristHomeAlgae = 82.50;
            public static final double kWristGroundALgae = 51.20;
        }
    }
        
    public static final class MotorIDs {
        
        public static final class Drivetrain {
            public static final int kFrontLeftDrive = 2;
            public static final int kFrontLeftSteer = 1;
            public static final int kFrontRightDrive = 5;
            public static final int kFrontRightSteer = 3;
            public static final int kBackLeftDrive = 6;
            public static final int kBackLeftSteer = 4;
            public static final int kBackRightDrive = 7;
            public static final int kBackRightSteer = 8;
        }

        public static final class Misc {
            public static final int kIntakeMotor = 24;
            public static final int kPivotLeftMotor = 20;
            public static final int kPivotRightMotor = 17;
            public static final int kElevatorMotor = 30;
            public static final int kWristMotor = 26;
        }
    }

    public static final class PneumaticsIDs {
        public static final int kPHID = 42;
        public static final int kMinPressure = 110;
        public static final int kMaxPressure = 120;
        public static final int kClawOpen = 0;
        public static final int kClawClose = 1;
    }

    public static final class IO {

        public static final class Drivetrain {
            public static final int kFrontLeftEncoder = 9;
            public static final int kFrontRightEncoder = 12;
            public static final int kBackLeftEncoder = 10;
            public static final int kBackRightEncoder = 11;
            public static final int kPigeon = 13;
        }

        public static final class Misc {
            public static final int kBeamBreak = 9;
            public static final int kPivotEncoderID = 2;
            public static final double kPivotEncoderOffset = 0;
            public static final int kWristEncoderID = 1;
            public static final double kWristEncoderOffset = -12;
        }

        public static final class Camera {
            public static final String kLimeLightReef = "limelight-reef";
            public static final String kLimeLightCoral = "limelight-coral";
        }

        public static final class CANdle {
            public static final int kCANdleID = 26;
            public static final int kTotalLightAmount = 0;
        }
    }
}
