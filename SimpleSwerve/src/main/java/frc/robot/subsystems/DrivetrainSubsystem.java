// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.utils.SwerveModule;

public class DrivetrainSubsystem extends SubsystemBase {
  /**
   * The maximum voltage that will be delivered to the drive motors.
   * <p>
   * This can be reduced to cap the robot's maximum speed. Typically, this is
   * useful during initial testing of the robot.
   */
  public static final double MAX_VOLTAGE = 12.0;

  // The formula for calculating the theoretical maximum velocity is:
  // <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> *
  // pi
  // An example of this constant for a Mk4 L2 module with NEOs to drive is:
  // 5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() *
  // SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
  /**
   * The maximum velocity of the robot in meters per second.
   * <p>
   * This is a measure of how fast the robot should be able to drive in a straight
   * line.
   */
  public static final double MAX_VELOCITY_METERS_PER_SECOND = DriveTrain.kSwerveConfiguration
      .getDriveMotorFreeSpeedRPM() / 60.0 *
      DriveTrain.kSwerveConfiguration.getDriveReduction() *
      DriveTrain.kSwerveConfiguration.getWheelDiameter() * Math.PI;
  /**
   * The maximum angular velocity of the robot in radians per second.
   * <p>
   * This is a measure of how fast the robot can rotate in place.
   */
  // Here we calculate the theoretical maximum angular velocity. You can also
  // replace this with a measured amount.
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
      (Math.hypot(DriveTrain.kTrackWidthMeters, DriveTrain.kWheelBaseMeters) / 2.0);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      // Front left
      new Translation2d(DriveTrain.kTrackWidthMeters / 2.0, DriveTrain.kWheelBaseMeters / 2.0),
      // Front right
      new Translation2d(DriveTrain.kTrackWidthMeters / 2.0, -DriveTrain.kWheelBaseMeters / 2.0),
      // Back left
      new Translation2d(-DriveTrain.kTrackWidthMeters / 2.0, DriveTrain.kWheelBaseMeters / 2.0),
      // Back right
      new Translation2d(-DriveTrain.kTrackWidthMeters / 2.0, -DriveTrain.kWheelBaseMeters / 2.0));

  // By default we use a Pigeon for our gyroscope. But if you use another
  // gyroscope, like a NavX, you can change this.
  // The important thing about how you configure your gyroscope is that rotating
  // the robot counter-clockwise should
  // cause the angle reading to increase until it wraps back over to zero.
  private final WPI_Pigeon2 m_pigeon = new WPI_Pigeon2(DriveTrain.kPigeonID.id, DriveTrain.kPigeonID.bus);

  // These are our modules. We initialize them in the constructor.
  private final SwerveModule[] m_swerveModules;

  private SwerveModuleState[] m_moduleStates = null;
  private final SwerveDriveOdometry m_odometry;
  private final Field2d m_field = new Field2d();

  public DrivetrainSubsystem() {
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    DriveTrain.SwerveModule moduleInfo = DriveTrain.kFrontLeftModule;
    SwerveModule frontLeftModule = new SwerveModule(
        0,
        // This parameter is optional, but will allow you to see the current state of
        // the module on the dashboard.
        tab.getLayout("Front Left Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(0, 0),

        DriveTrain.kSwerveConfiguration,
        DriveTrain.kDriveMotorOptions,
        DriveTrain.kSteerMotorOptions,
        moduleInfo.driveMotorID,
        moduleInfo.steerMotorID,
        moduleInfo.azimuthEncoderID,
        moduleInfo.steerOffsetRadians);
    frontLeftModule.configure();

    // We will do the same for the other modules
    moduleInfo = DriveTrain.kFrontRightModule;
    SwerveModule frontRightModule = new SwerveModule(
        1,
        tab.getLayout("Front Right Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(2, 0),
        DriveTrain.kSwerveConfiguration,
        DriveTrain.kDriveMotorOptions,
        DriveTrain.kSteerMotorOptions,
        moduleInfo.driveMotorID,
        moduleInfo.steerMotorID,
        moduleInfo.azimuthEncoderID,
        moduleInfo.steerOffsetRadians);
    frontRightModule.configure();

    moduleInfo = DriveTrain.kBackLeftModule;
    SwerveModule backLeftModule = new SwerveModule(
        2,
        tab.getLayout("Back Left Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(4, 0),
        DriveTrain.kSwerveConfiguration,
        DriveTrain.kDriveMotorOptions,
        DriveTrain.kSteerMotorOptions,
        moduleInfo.driveMotorID,
        moduleInfo.steerMotorID,
        moduleInfo.azimuthEncoderID,
        moduleInfo.steerOffsetRadians);
    backLeftModule.configure();

    moduleInfo = DriveTrain.kBackRightModule;
    SwerveModule backRightModule = new SwerveModule(
        3,
        tab.getLayout("Back Right Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(6, 0),
        DriveTrain.kSwerveConfiguration,
        DriveTrain.kDriveMotorOptions,
        DriveTrain.kSteerMotorOptions,
        moduleInfo.driveMotorID,
        moduleInfo.steerMotorID,
        moduleInfo.azimuthEncoderID,
        moduleInfo.steerOffsetRadians);
    backRightModule.configure();

    m_swerveModules = new SwerveModule[4];
    m_swerveModules[frontLeftModule.moduleIndex] = frontLeftModule;
    m_swerveModules[frontRightModule.moduleIndex] = frontRightModule;
    m_swerveModules[backLeftModule.moduleIndex] = backLeftModule;
    m_swerveModules[backRightModule.moduleIndex] = backRightModule;

    SwerveModulePosition[] currentPositions = new SwerveModulePosition[m_swerveModules.length];
    for (SwerveModule module : m_swerveModules) {
      currentPositions[module.moduleIndex] = module.getPosition();
    }

    m_odometry = new SwerveDriveOdometry(m_kinematics, getGyroscopeRotation(), getModulePositions());
    tab.add("Position", m_field);
  }

  /**
   * Return the drive train's theoretical maximum linear velocity (m/s)
   * 
   * @return Max velocity (m/s)
   */
  public double getMaxLinearVelocity() {
    return MAX_VELOCITY_METERS_PER_SECOND;
  }

  /**
   * Return the drive train's theoretical maximum angular velcity (radians/s)
   * 
   * @return Max velocity (radians/s)
   */
  public double getMaxAngularVelocity() {
    return MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
  }

  /**
   * Return the pose of the robot based on the current odometry
   * 
   * @return Pose2d representing current pose
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Reset odometry to the given pose
   * 
   * @param pose The Pose2d to use
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(getGyroscopeRotation(), getModulePositions(), pose);
  }

  /**
   * Get the states of each swerve module
   * 
   * @return SwerveModuleState[]
   */
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : m_swerveModules) {
      states[mod.moduleIndex] = mod.getState();
    }
    return states;
  }

  /**
   * Get the positions of each module
   * 
   * @return SwerveModulePosition[]
   */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[m_swerveModules.length];
    for (SwerveModule mod : m_swerveModules) {
      positions[mod.moduleIndex] = mod.getPosition();
    }
    return positions;
  }

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the
   * robot is currently facing to the
   * 'forwards' direction.
   */
  public void zeroGyroscope() {
    m_pigeon.setYaw(0.0);
  }

  /**
   * Resets the steer motor's internal position sensor to match the absolute
   * encoder
   */
  public void resetSteerPositionSensors() {
    for (SwerveModule module : m_swerveModules) {
      module.resetSteerPositionSensor();
    }
  }

  public Rotation2d getGyroscopeRotation() {
    return Rotation2d.fromDegrees(m_pigeon.getYaw());
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    m_moduleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
            translation.getX(),
            translation.getY(),
            rotation,
            getGyroscopeRotation())
            : new ChassisSpeeds(
                translation.getX(),
                translation.getY(),
                rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(m_moduleStates, MAX_VELOCITY_METERS_PER_SECOND);

    for (SwerveModule mod : m_swerveModules) {
      mod.setDesiredState(m_moduleStates[mod.moduleIndex], isOpenLoop);
    }
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_VELOCITY_METERS_PER_SECOND);

    for (SwerveModule mod : m_swerveModules) {
      mod.setDesiredState(desiredStates[mod.moduleIndex], false);
    }
  }

  public void periodic() {
    // Update pose with odometry
    var newPose = m_odometry.update(getGyroscopeRotation(), getModulePositions());
    m_field.setRobotPose(newPose);
  }
}
