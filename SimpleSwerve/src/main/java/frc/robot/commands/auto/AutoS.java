package frc.robot.commands.auto;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoS extends AutoCommandBase {
    public String getName() { return "S"; }

    /**
     * Autonomous command the drives in a square pattern:
     *   (0, 0) => (1m, 0) => (1m, 1m) => (0, 1m) => (0, 0)
     * @param driveTrain Drivetrain subsystem
     * @return Command to be scheduled
     */
    public Command createCommand(DrivetrainSubsystem driveTrain, TrajectoryConfig config) {
        Trajectory trajectory = trajectoryFromWaypoints(
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
        config);

        SwerveControllerCommand swerveControllerCommand = swerveCommandForTrajectory(driveTrain, trajectory);

        return simpleSwerveTrajectoryCommand(driveTrain, trajectory.getInitialPose(), swerveControllerCommand);
    }

}
