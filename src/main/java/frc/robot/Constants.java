// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  
  public static class SubsystemConstants {
    public static final int kDumpsterID = 21; // TEMPORARY, WILL CHANGE
  }

  public static class DriveConstants {
    public static final int kFrontLeftTurningID = 10;
    public static final int kFrontLeftDrivingID = 11;
    
    public static final int kFrontRightTurningID = 14;
    public static final int kFrontRightDrivingID = 15;
    
    public static final int kBackRightTurningID = 16;
    public static final int kBackRightDrivingID = 17;

    public static final int kBackLeftTurningID = 12;
    public static final int kBackLeftDrivingID = 13;
     
    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;

    public static final double kDrivingP = 0.1;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / ((5676 / 60) * (0.0762 * Math.PI) / 4.71);

    public static final double kDriveReduction = 4.71;
    public static final double kAngleReduction = 46.42;

    public static final int kDrivingMaxCurrent = 60;
    public static final int kTurningMaxCurrent = 30;

    public static final double kMaxLinearSpeed = 3.5; // m / sec (4.8 old)

    public static final double kWheelOffset = 12.5;

    public static final Translation2d kFrontLeftLocation = new Translation2d(kWheelOffset, kWheelOffset);
    public static final Translation2d kFrontRightLocation = new Translation2d(kWheelOffset, -kWheelOffset);
    public static final Translation2d kBackRightLocation = new Translation2d(-kWheelOffset, -kWheelOffset);
    public static final Translation2d kBackLeftLocation = new Translation2d(-kWheelOffset, kWheelOffset);
  
    public static final double kFrontLeftAngularOffset = -Math.PI / 2;
    public static final double kFrontRightAngularOffset = 0;
    public static final double kBackRightAngularOffset = Math.PI / 2;
    public static final double kBackLeftAngularOffset = 3 * Math.PI;

    public static final double kAnglePositionConversionFactor = 2 * Math.PI; // (0 - 1) to (0 - 2pi)
    public static final double kAngleVelocityConversionFactor = (2 * Math.PI) / 60.0;

    public static final double kDrivePositionConversionFactor = 0.05082576649756735; // yall know where this comes from
    public static final double kDriveVelocityConversionFactor = kDrivePositionConversionFactor / 60;

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kAnglePositionConversionFactor; // radians

    /* EXPLAINING SLEW RATE LIMITING
     * The value of the slew rate limiter is the number of units to change per second. For example,
     * a value of 2.0 means that it takes 0.5 seconds (1/2) to get from 0 to 1.
     */
    public static final double kXYSlewRate = 18.0;
    public static final double kRotationalSlewRate = 2.0;
  }
}
