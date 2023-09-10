// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.utils.*;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class FalconConstants {
    public static double kFreeSpeedRPM = 6380.0;
    public static double kTicksPerRotation = 2048.0;

    // Falcon reports velocity in ticks per 100ms. 
    public static double kVelocityTicksPerSecondConversion = 10.0;
  }

  public static class Operator {
    public static final int kDriverControllerPort = 0;
    public static final double kDeadband = 0.05;
  }

  /**
   * Drive train constants
   */
  public static class DriveTrain {
        /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double kTrackWidthMeters = 0.63; // 24.75 in
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double kWheelBaseMeters = 0.63; // 24.75 in

    public static final CANDeviceID kPigeonID = new CANDeviceID(1, kCanivoreBusName);

    public static final SwerveModuleConfiguration kSwerveConfiguration = new SwerveModuleConfiguration(
      0.10033,
      (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0),
      true,
      (14.0 / 50.0) * (10.0 / 60.0),
      false,
      FalconConstants.kFreeSpeedRPM,
      FalconConstants.kFreeSpeedRPM
    );

    public static final SwerveMotorConfiguration kDriveMotorOptions = new SwerveMotorConfiguration()
    .withCurrentLimit(80.0)
    .withNominalVoltage(12.0)
    .withMotionMagicContants((1.51 / 12), (0.27 / 12), (0.32 / 12));

    public static final SwerveMotorConfiguration kSteerMotorOptions = new SwerveMotorConfiguration()
    .withCurrentLimit(20.0)
    .withNominalVoltage(12.0)
    .withPIDConstants(0.2, 0.0, 0.1);

    public static final class SwerveModule {
        public final CANDeviceID driveMotorID;
        public final CANDeviceID steerMotorID;
        public final CANDeviceID azimuthEncoderID;
        public final double steerOffsetRadians;

        SwerveModule(CANDeviceID driveMotorID, CANDeviceID steerMotorID, CANDeviceID encoderID, double steerOffset) {
          this.driveMotorID = driveMotorID;
          this.steerMotorID = steerMotorID;
          this.azimuthEncoderID = encoderID;
          this.steerOffsetRadians = steerOffset;
        }
    };
    public static final SwerveModule kFrontLeftModule = new SwerveModule(
      new CANDeviceID(7, kCanivoreBusName),
      new CANDeviceID(8, kCanivoreBusName),
      new CANDeviceID(4, kCanivoreBusName),
      -Math.toRadians(162.158203125)// Set to 0.0. when calibrating
    );
    public static final SwerveModule kFrontRightModule = new SwerveModule(
      new CANDeviceID(1, kCanivoreBusName),
      new CANDeviceID(2, kCanivoreBusName),
      new CANDeviceID(1, kCanivoreBusName),
      -Math.toRadians(72.158203125)
    );
    public static final SwerveModule kBackLeftModule = new SwerveModule(
      new CANDeviceID(5, kCanivoreBusName),
      new CANDeviceID(6, kCanivoreBusName),
      new CANDeviceID(3, kCanivoreBusName),
      -Math.toRadians(34.887890625000004)
    );
    public static final SwerveModule kBackRightModule = new SwerveModule(
      new CANDeviceID(3, kCanivoreBusName),
      new CANDeviceID(4, kCanivoreBusName), 
      new CANDeviceID(2, kCanivoreBusName),
      -Math.toRadians(175.25390625)
    );
  }

  /**
   * Canivore bus name we use
   */
  public static final String kCanivoreBusName = "Canivore";



}
