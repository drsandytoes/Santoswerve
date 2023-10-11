package frc.robot.commands;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.auto.AutoCommand;
import frc.robot.commands.auto.AutoForward;
import frc.robot.commands.auto.AutoS;
import frc.robot.commands.auto.AutoSquare;
import frc.robot.commands.calibration.CalibrationRun;
import frc.robot.subsystems.drive.DrivetrainSubsystem;

public class AutoChooser {
    private ShuffleboardContainer m_autoTab;
    private LoggedDashboardChooser<TrajectoryConfig> m_trajectorySpeedChooser;
    private LoggedDashboardChooser<AutoCommand> m_pathChooser;

    private DrivetrainSubsystem m_driveTrain;

    public AutoChooser(DrivetrainSubsystem driveTrain) {
        m_driveTrain = driveTrain;

        m_autoTab = Shuffleboard.getTab("Auto Config");

        m_trajectorySpeedChooser = new LoggedDashboardChooser<TrajectoryConfig>("Trajectory");
        m_trajectorySpeedChooser.addOption("Slow", slowTrajectoryConfig());
        m_trajectorySpeedChooser.addDefaultOption("Normal", standardTrajectoryConfig());

        // Add all of the command classes here
        m_pathChooser = new LoggedDashboardChooser<AutoCommand>("Auto Path");
        addPath(new AutoSquare(), false);
        addPath(new AutoS(), false);
        addPath(new AutoForward(), false);

        // Calibration command is special
        CalibrationRun calibrationRunner = new CalibrationRun(driveTrain::getTuningTargetVelocity, driveTrain::getTuningDistanceLimit);
        addPath(calibrationRunner, true);

        // Add the widgets to the dashboard
        m_autoTab.add("Speed", m_trajectorySpeedChooser)
                .withPosition(0, 0)
                .withSize(2, 1);

        m_autoTab.add("Path", m_pathChooser)
                .withSize(2, 1)
                .withPosition(0, 1);
    }

    private void addPath(AutoCommand command, boolean isDefault) {
        if (isDefault) {
            m_pathChooser.addDefaultOption(command.getName(), command);
        } else {
            m_pathChooser.addOption(command.getName(), command);
        }
    }

    public TrajectoryConfig getTrajectoryConfig() {
        return m_trajectorySpeedChooser.get();
    }

    public Command getSelectedCommand() {
        AutoCommand commandFactory = m_pathChooser.get();
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
