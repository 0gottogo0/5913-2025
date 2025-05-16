// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
    public static final class Controllers {
        public static final int DRIVER_CONTROLLER = 0;
        public static final int MANIPULATOR_CONTROLLER = 1;
    }

    public static final class Speeds {
        public static final double ELEVATOR_SPEED_MAX = 1;
        public static final double ELEVATOR_SPEED_ALGAE = 0.8;
        public static final double INTAKE_SPEED_MAX = 1;
        public static final double INTAKE_SPEED = 0.55;
        public static final double INTAKE_SPEED_L1 = 0.4;
        public static final double INTAKE_SPEED_HOLD_ALGAE = -0.15;
        public static final double PIVOT_SPEED_MAX = 1;
        public static final double TRACK_MOVE_MAX = .3;
        public static final double TRACK_MOVE_SLOW = .2;
        public static final double TRACK_ROTATE_MAX = 0.1;
        public static final double WRIST_SPEED_MAX = 0.7;
    }


    public static final class Drivetrain {
        public static final double MOVE_SLEW_RATE_LIMITER = 12.00;
        public static final double ROTATE_SLEW_RATE_LIMITER = 40.00;
        public static final double ROTATE_MAGNITUDE = 0.90;
        public static final double STICK_DEADZONE = 0.02;

        // CTRE
        public static final double TRACK_WIDTH_X = 23.5;   // These are how far the center of the swerves are apart
        public static final double TRACK_WIDTH_Y = 23.375; // Same but back to front
        public static final double DRIVETRAIN_MAX_SPEED = 17.1;      // ft/s
        public static final double DRIVETRAIN_WHEEL_DIAMETER = 1.95; // inches
        public static final double DERIVETRAIN_GEAR_RATIO = 6.1224489;
        public static final double DRIVETRAIN_STEER_RATIO = 12.8;
        public static final double FRAME_WIDTH = 30;     // inches
        public static final double FRAME_LENGTH = 29.75; // inches
    }

    public static final class PID {
        public static final class Elevator {
        public static final double ELEVATOR_KP_UP = 0.12;
        public static final double ELEVATOR_KP_DOWN = 0.08;
        public static final double ELEVATOR_INTAKE = 0.00;
        public static final double ELEVATOR_BARGE = -45.20;
        public static final double ELEVATOR_PROCESSOR = 0.00;
        public static final double ELEVATOR_HOME = 0.00;
        public static final double ELEVATOR_CLIMB = 0.00;
        public static final double ELEVATOR_L1 = -4.80;
        public static final double ELEVATOR_L2 = -6.80;
        public static final double ELEVATOR_L3 = -20.00;
        public static final double ELEVATOR_L4 = -45.20;
        public static final double ELEVATOR_BOTTOM_ALGAE = -13.40;
        public static final double ELEVATOR_TOP_ALGAE = -26.50;
        public static final double ELEVATOR_GROUND_ALGAE = -4.90;
        }

        public static final class Pivot {
            public static final double PIVOT_KP = 0.09;
            public static final double PIVOT_L1 = 140.50;
            public static final double PIVOT_L2 = 128.00;
            public static final double PIVOT_L3 = 135.00;
            public static final double PIVOT_L4 = 140.70;
            public static final double PIVOT_INTAKE = 150.60;
            public static final double PIVOT_PROCESSOR = 113.80;
            public static final double PIVOT_HOME = 135.00;
            public static final double PIVOT_CLIMB = 122.50;
            public static final double PIVOT_CLIMB_END = 95.50;
            public static final double PIVOT_BOTTOM_ALGAE = 127.10;
            public static final double PIVOT_TOP_ALGAE = 131.70;
            public static final double PIVOT_GROUND_ALGAE = 101.80;
            public static final double PIVOT_GROUND_CORRAL = 108.20;
            public static final double PIVOT_DEADZONE = 0.05;
        }

        public static final class Track {
            public static final double REEF_TRACK_X_KP = 0.85;
            public static final double REEF_TRACK_Y_KP = 0.75;
            public static final double REEF_TRACK_ROT_KP = 0.04;
            public static final double CORAL_TRACK_X_KP = 0.50;
            public static final double CORAL_TRACK_Y_KP = 0.50;
            public static final double CORAL_TRACK_ROT_KP = 0.03;
            public static final double TRACK_X_OFFSET_LEFT = -0.193;
            public static final double TRACK_Y_OFFSET_LEFT = -0.464;
            public static final double TRACK_X_OFFSET_CENTER = 0.000;
            public static final double TRACK_Y_OFFSET_CENTER = -0.590;
            public static final double TRACK_X_OFFSET_RIGHT = 0.153;
            public static final double TRACK_Y_OFFSET_RIGHT = -0.464;
            public static final double TRACK_X_OFFSET_CORAL = 0.26;  
            public static final double TRACK_Y_OFFSET_CORAL = -0.48;
        }
        
        public static final class Wrist {
            public static final double WRIST_KP = 0.015;
            public static final double WRIST_INTAKE = 273.80;
            public static final double WRIST_BARGE = 218.3;
            public static final double WRIST_PROCESSOR = 85.70;
            public static final double WRIST_HOME = 297.00;
            public static final double WRIST_L1 = 37.10;
            public static final double WRIST_L2 = 291.80;
            public static final double WRIST_L3 = 280.70;
            public static final double WRIST_L4 = 222.20;
            public static final double WRIST_BOTTOM_ALGAE = 82.50;
            public static final double WRIST_TOP_ALGAE = 88.10;
            public static final double WRIST_HOME_ALGAE = 82.50;
            public static final double WRIST_GROUND_ALGAE = 45.90;
            public static final double WRIST_GROUND_CORAL = 69.40;
        }
    }
        
    public static final class MotorIDs {
        
        public static final class Drivetrain {
            public static final int FRONT_LEFT_DRIVE = 2;
            public static final int FRONT_LEFT_STEER = 1;
            public static final int FRONT_RIGHT_DRIVE = 5;
            public static final int FRONT_RIGHT_STEER = 3;
            public static final int BACK_LEFT_DRIVE = 6;
            public static final int BACK_LEFT_STEER = 4;
            public static final int BACK_RIGHT_DRIVE = 7;
            public static final int BACK_RIGHT_STEER = 8;
        }

        public static final class Misc {
            public static final int INTAKE_MOTOR = 24;
            public static final int PIVOT_LEFT_MOTOR = 20;
            public static final int PIVOT_RIGHT_MOTOR = 17;
            public static final int ELEVATOR_MOTOR = 30;
            public static final int WRIST_MOTOR = 26;
        }
    }

    public static final class PneumaticsIDs {
        public static final int PNEUMATICS_HUB = 42;
        public static final int MIN_PRESSURE = 110;
        public static final int MAX_PRESSURE = 120;
        public static final int CLAW_OPEN = 0;
        public static final int CLAW_CLOSE = 1;
    }

    public static final class IO {

        public static final class Drivetrain {
            public static final int FRONT_LEFT_ENCODER = 9;
            public static final int FRONT_RIGHT_ENCODER = 12;
            public static final int BACK_LEFT_ENCODER = 10;
            public static final int BACK_RIGHT_ENCODER = 11;
            public static final int PIGEON_IMU = 13;
        }

        public static final class Misc {
            public static final int BEAM_BREAK = 9;
            public static final int PIVOT_ENCODER = 2;
            public static final double PIVOT_ENCODER_OFFSET = 0;
            public static final int WRIST_ENCODER = 1;
            public static final double WRIST_ENCODER_OFFSET = -29.8;
        }

        public static final class Camera {
            public static final String LIMELIGHT_REEF = "limelight-reef";
            public static final String LIMELIGHT_CORAL = "limelight-coral";
        }

        public static final class CANdle {
            public static final int CANDLE = 26;
            public static final int TOTAL_LIGHT_AMOUNT = 0;
        }
    }
}
