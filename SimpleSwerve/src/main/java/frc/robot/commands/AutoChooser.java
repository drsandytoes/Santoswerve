package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.auto.AutoCommandBase;
import frc.robot.commands.auto.AutoForward;
import frc.robot.commands.auto.AutoS;
import frc.robot.commands.auto.AutoSquare;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoChooser {
    private ShuffleboardContainer m_autoTab;
    private SendableChooser<TrajectoryConfig> m_trajectorySpeedChooser;
    private SendableChooser<AutoCommandBase> m_pathChooser;

    private DrivetrainSubsystem m_driveTrain;

    public AutoChooser(DrivetrainSubsystem driveTrain) {
        m_driveTrain = driveTrain;

        m_autoTab = Shuffleboard.getTab("Auto Config");

        m_trajectorySpeedChooser = new SendableChooser<TrajectoryConfig>();
        m_trajectorySpeedChooser.addOption("Slow", slowTrajectoryConfig());
        m_trajectorySpeedChooser.setDefaultOption("Normal", standardTrajectoryConfig());

        // Add all of the command classes here
        m_pathChooser = new SendableChooser<AutoCommandBase>();
        m_pathChooser.setDefaultOption(AutoSquare.name, new AutoSquare());
        m_pathChooser.addOption(AutoS.name, new AutoS());
        m_pathChooser.addOption(AutoForward.name, new AutoForward());

        m_autoTab.add("Speed", m_trajectorySpeedChooser)
                .withPosition(0, 0)
                .withSize(2, 1);

        m_autoTab.add("Path", m_pathChooser)
                .withSize(2, 1)
                .withPosition(0, 1);
    }

    public TrajectoryConfig getTrajectoryConfig() {
        return m_trajectorySpeedChooser.getSelected();
    }

    public Command getSelectedCommand() {
        AutoCommandBase commandFactory = m_pathChooser.getSelected();
        return commandFactory.createCommand(m_driveTrain, getTrajectoryConfig());
    }

    private static TrajectoryConfig standardTrajectoryConfig() {
        return new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.DriveTrain.kSwerveKinematics);
    }

    private static TrajectoryConfig slowTrajectoryConfig() {
        return new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond * Constants.AutoConstants.kSlowDownFactor,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared
                        * Constants.AutoConstants.kSlowDownFactor)
                .setKinematics(Constants.DriveTrain.kSwerveKinematics);
    }

}
