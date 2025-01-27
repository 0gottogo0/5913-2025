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

    // ** Speeds **
    public static final double kPivotSpeedMax = 1;
    public static final double kElevetorSpeedMax = 0.5;
    public static final double kClawSpeedMax = 0.8;
    public static final double kClawSpeedLow = 0.2;

    // ** Motors **
    public static final int kPivotLeftMotor = 15;
    public static final int kPivotRightMotor = 16;
    public static final int kElevetorMotor = 17;
    public static final int kClawMotor = 18;

    // ** Pnumatics **
    public static final int kPistonModule = 14;
    public static final int kClawIn = 30;
    public static final int kClawOut = 31;

    // ** Input **
}
