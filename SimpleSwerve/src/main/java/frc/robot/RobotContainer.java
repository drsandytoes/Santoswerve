// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.AutoFactory;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final CommandJoystick m_controller = new CommandJoystick(Constants.Operator.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
      m_drivetrainSubsystem,
      () -> -modifyAxis(m_controller.getY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(m_controller.getX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(m_controller.getTwist()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    ));

    // m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
    //   m_drivetrainSubsystem,
    //   () -> +modifyAxis(0) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    //   () -> +modifyAxis(0) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    //   () -> -modifyAxis(0.25) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    // ));

  }

  // When the motors get enabled, we want to do a one-time reset of the steer position sensors
  public void teleopInit() {
    m_drivetrainSubsystem.resetSteerPositionSensors();
  }

  public void autonomousInit() {
    m_drivetrainSubsystem.resetSteerPositionSensors();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    m_controller.button(1).onTrue(new InstantCommand(m_drivetrainSubsystem::zeroGyroscope, m_drivetrainSubsystem)
      .ignoringDisable(true));

    m_controller.button(2).onTrue(new InstantCommand(m_drivetrainSubsystem::resetSteerPositionSensors, m_drivetrainSubsystem)
      .ignoringDisable(true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return AutoFactory.squareTrajectory(m_drivetrainSubsystem);
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = MathUtil.applyDeadband(value, Constants.Operator.kDeadband);

    // Square the axis, but preserve sign!
    value = Math.copySign(Math.pow(value, 2), value);

    return value;
  }
}
