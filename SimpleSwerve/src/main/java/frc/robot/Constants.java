// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.*;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class Overrides {
    public static final boolean kDisableMotion = false;
  }

  public static class FalconConstants {
    public static final double kFreeSpeedRPM = 6380.0;
    public static final double kTicksPerRotation = 2048.0;

    // Falcon reports velocity in ticks per 100ms.
    public static final double kVelocityTicksToTicksPerSecond = 10.0;
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
    public static final double kTrackWidthMeters = Units.inchesToMeters(24.75); // 24.75 in
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double kWheelBaseMeters = Units.inchesToMeters(24.75); // 24.75 in

    public static final SwerveDriveKinematics kSwerveKinematics = new SwerveDriveKinematics(
      // Front left
      new Translation2d(DriveTrain.kTrackWidthMeters / 2.0, DriveTrain.kWheelBaseMeters / 2.0),
      // Front right
      new Translation2d(DriveTrain.kTrackWidthMeters / 2.0, -DriveTrain.kWheelBaseMeters / 2.0),
      // Back left
      new Translation2d(-DriveTrain.kTrackWidthMeters / 2.0, DriveTrain.kWheelBaseMeters / 2.0),
      // Back right
      new Translation2d(-DriveTrain.kTrackWidthMeters / 2.0, -DriveTrain.kWheelBaseMeters / 2.0));



    public static final CANDeviceID kPigeonID = new CANDeviceID(1, kCanivoreBusName);

    public static final double kCANcoderUpdateFrequency = 100;

    public static final SwerveModuleConfiguration = new SwerveModuleConfiguration()
      .withWheelDiameter(Units.inchesToMeters(4))
      .withDriveReduction((14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0))
      .withDrivePositiveClockwise(false)
      .withSteerReduction((14.0 / 50.0) * (10.0 / 60.0))
      .withSteerPositiveClockwise(true)
      .withDriveMotorFreeSpeed(FalconConstants.kFreeSpeedRPM)
      .withSteerMotorFreeSpeed(FalconConstants.kFreeSpeedRPM)
      .withEncoderPositiveClockwise(false);

    // kF ~ 0.046 @ 0.5m/s (decent @ 1.0m/s)
    // kP ~ 0.08
    public static final SwerveMotorConfiguration kDriveMotorOptions = new SwerveMotorConfiguration()
        .withContinuousCurrentLimit(35)
        .withPeakCurrentLimit(60)
        .withPeakCurrentDuration(0.1)
        .withNominalVoltage(12.0)
        .withPIDFConstants(0.08, 0.0, 0.0, 0.048);
        // .withFeedForwardConstants((0.64 / 12), (2.0 / 12), (0.27 / 12));

    public static final SwerveMotorConfiguration kSteerMotorOptions = new SwerveMotorConfiguration()
        .withContinuousCurrentLimit(25)
        .withPeakCurrentLimit(40)
        .withPeakCurrentDuration(0.1)
        .withNominalVoltage(12.0)
        .withPIDFConstants(0.3, 0.0, 0.0, 0.0);

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
        -Math.toRadians(72.158203125));
    public static final SwerveModule kBackLeftModule = new SwerveModule(
        new CANDeviceID(5, kCanivoreBusName),
        new CANDeviceID(6, kCanivoreBusName),
        new CANDeviceID(3, kCanivoreBusName),
        -Math.toRadians(34.887890625000004));
    public static final SwerveModule kBackRightModule = new SwerveModule(
        new CANDeviceID(3, kCanivoreBusName),
        new CANDeviceID(4, kCanivoreBusName),
        new CANDeviceID(2, kCanivoreBusName),
        -Math.toRadians(175.25390625));
  }

  public static final class AutoConstants {
    /**
     * Multiplier to the other auto speed/acceleration constants when asking for the slow
     * version.
     */
    public static final double kSlowDownFactor = 0.3;
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 0.5;
    public static final double kPYController = 0.5;
    public static final double kPThetaController = 1;

    /* Constraint for the motion profilied robot angle controller */
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  /**
   * Canivore bus name we use
   */
  public static final String kCanivoreBusName = "Canivore";

}
