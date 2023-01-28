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
    public static final double kI_speed = 1;
    public static final double kD_speed = 0;
    public static final double kP_speed = 1;

    public static final double defaultSpeed = 0.1;
    public static final double defaultDegrees = 10.0;
    public static final double maxWaitTime = 1000.0;
    public static final double defaultRampUpDistanceInches = 6.5;

    public static final int leftPrimaryMotorCANID = 1;
    public static final int leftSecondaryMotorCANID = 2;
    public static final int rightPrimaryMotorCANID = 3;
    public static final int rightSecondaryMotorCANID = 4;
}
