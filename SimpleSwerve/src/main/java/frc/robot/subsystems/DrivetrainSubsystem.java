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
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.*;
import frc.robot.utils.SendablePIDParameters;
import frc.robot.utils.SendableVelocityTuningParameters;
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

  /**
   * Tuning parameters put onto the dashboard.
   */
  private SendablePIDParameters m_tuningPID = new SendablePIDParameters(Constants.DriveTrain.kDriveMotorOptions.proportionalConstant,
    Constants.DriveTrain.kDriveMotorOptions.integralConstant,
    Constants.DriveTrain.kDriveMotorOptions.derivativeConstant,
    Constants.DriveTrain.kDriveMotorOptions.feedForwardConstant);
  private SendableVelocityTuningParameters m_tuningParams = new SendableVelocityTuningParameters(2.0, 2.0);

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
    ShuffleboardTab driveTrainTab = Shuffleboard.getTab("Drivetrain");
    ShuffleboardTab driveTrainConstantsTab = Shuffleboard.getTab("Drivetrains Constants");
    ShuffleboardTab fieldTab = Shuffleboard.getTab("Field Position");

    driveTrainConstantsTab.addDouble("Max Linear Velocity", () -> MAX_VELOCITY_METERS_PER_SECOND);
    driveTrainConstantsTab.addDouble("Max Angular Velocity", () -> MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);

    DriveTrain.SwerveModule moduleInfo = DriveTrain.kFrontLeftModule;
    SwerveModule frontLeftModule = new SwerveModule(
        0,
        // This parameter is optional, but will allow you to see the current state of
        // the module on the dashboard.
        driveTrainTab.getLayout("Front Left Module", BuiltInLayouts.kList)
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
        driveTrainTab.getLayout("Front Right Module", BuiltInLayouts.kList)
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
        driveTrainTab.getLayout("Back Left Module", BuiltInLayouts.kList)
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
        driveTrainTab.getLayout("Back Right Module", BuiltInLayouts.kList)
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

    m_odometry = new SwerveDriveOdometry(DriveTrain.kSwerveKinematics, getGyroscopeRotation(), getModulePositions());
    fieldTab.add("Position", m_field)
      .withPosition(0, 0)
      .withSize(9, 5);

      configureTuningTab("Tuning", frontRightModule);

}

  private void configureTuningTab(String title, SwerveModule module) {
    ShuffleboardTab tuningTab = Shuffleboard.getTab(title);

    ShuffleboardLayout pidContainer = tuningTab.getLayout("PID", BuiltInLayouts.kGrid)
      .withPosition(0, 0)
      .withSize(2, 2);
    pidContainer.add("PID", m_tuningPID);

    ShuffleboardLayout calibrationContainer = tuningTab.getLayout("Calibration", BuiltInLayouts.kGrid)
      .withPosition(0, 2)
      .withSize(2, 2);
    calibrationContainer.add("Tuning", m_tuningParams);

    // Add graphs of velocity and distance that can be used when PID tuning
    tuningTab.addDouble("Velocity", module::getStateVelocity)
      .withPosition(2, 0)
      .withSize(6, 3)
      .withWidget(BuiltInWidgets.kGraph);

    tuningTab.addDouble("Distance", module::getStateDistance)
      .withPosition(2, 3)
      .withSize(6, 3)
      .withWidget(BuiltInWidgets.kGraph);
  }

  public void reconfigurePIDFromDashboard() {
    double kP = m_tuningPID.getProportionalConstant();
    double kI = m_tuningPID.getIntegralConstant();
    double kD = m_tuningPID.getDerivativeConstant();
    double kF = m_tuningPID.getFeedForwardConstant();

    DataLogManager.log("Setting PID: " + kP + ", " + kI + ", " + kD + ", " + kF);
    configureDriveMotorPID(kP, kI, kD, kF);
  }

  public double getTuningDistanceLimit() {
    return m_tuningParams.getDistanceLimit();
  }

  public double getTuningTargetVelocity() {
    return m_tuningParams.getTargetVelocity();
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
   * robot is currently facing to the 'forwards' direction.
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

  /**
   * Resets the drive motor's internal position sensor to 0.
   */
  public void zeroDrivePositionSensors() {
    for (SwerveModule module : m_swerveModules) {
      module.zeroDrivePositionSensor();
    }

  }

  public Rotation2d getGyroscopeRotation() {
    return Rotation2d.fromDegrees(m_pigeon.getYaw());
  }

  /**
   * Basic drive method of the drive train. This method takes the desired translation (x/y) and 
   * rotation speeds. If fieldRelative is false, the translation is robot-centric; otherwise the
   * translation is relative to the field position established when the gyroscope was last reset.
   * @param translation Translation2d describing x/y speeds (m/s)
   * @param rotation double representing the rotation speed (radians/s)
   * @param fieldRelative boolean, indicates whether translation is field- or robot-relative
   * @param isOpenLoop boolean, indicates whether to use open or closed loop controls. 
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    m_moduleStates = DriveTrain.kSwerveKinematics.toSwerveModuleStates(
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
      mod.setDesiredState(m_moduleStates[mod.moduleIndex], isOpenLoop, true);
    }
  }

  /**
   * setModuleStates sets the state of the swerve modules directly. It uses velocity PID implemented
   * by the swerve motors themselves. 
   * @param desiredStates Desired SwerveModuleStates for each module.
   */
  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_VELOCITY_METERS_PER_SECOND);

    for (SwerveModule mod : m_swerveModules) {
      mod.setDesiredState(desiredStates[mod.moduleIndex], false, true);
    }
  }

  /**
   * setModuleStatesWithoutOptimization sets the state of the swerve modules directly. It uses velocity PID implemented
   * by the swerve motors themselves. This is used for calibration.
   * @param desiredStates Desired SwerveModuleStates for each module.
   */
  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStatesWithoutOptimization(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_VELOCITY_METERS_PER_SECOND);

    for (SwerveModule mod : m_swerveModules) {
      mod.setDesiredState(desiredStates[mod.moduleIndex], false, false);
    }
  }

  /**
    * Reconfigure the PID constants for closed loop control. This is useful for PID tuning.
    * @param kP double, proportional constant
    * @param kI double, integral constant
    * @param kD double, derivative constant
    * @param kF double, feed forward constant
    */
  public void configureDriveMotorPID(double kP, double kI, double kD, double kF) {
    for (SwerveModule mod : m_swerveModules) {
      mod.configureDriveMotorPID(kP, kI, kD, kF);
    }
  }

  public void periodic() {
    // Update pose with odometry
    var newPose = m_odometry.update(getGyroscopeRotation(), getModulePositions());
    m_field.setRobotPose(newPose);
  }
}
