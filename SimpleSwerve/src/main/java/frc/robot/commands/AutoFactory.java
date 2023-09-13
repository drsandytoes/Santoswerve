package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoFactory {
    /**
     * Autonomous command to drive forward from (0, 0) to (1m, 1m), then to (2m, -1m), and ending att (3m, 0)
     * @param driveTrain Drive train subsystem to use
     * @return Command to be scheduled
     */
    static public Command sTrajectory(DrivetrainSubsystem driveTrain) {
        TrajectoryConfig config = standardTrajectoryConfig();

        // An example trajectory to follow.  All units in meters.
        Trajectory trajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3, 0, new Rotation2d(0)),
                config);

        SwerveControllerCommand swerveControllerCommand = swerveCommandForTrajectory(driveTrain, trajectory);

        return simpleSwerveTrajectoryCommand(driveTrain, trajectory.getInitialPose(), swerveControllerCommand);
    }

    /**
     * Autonomous command the drives 1 meter forward and 1 meter backward.
     * @param driveTrain Drivetrain subsystem
     * @return Command to be scheduled
     */
    static public Command forwardBackwardTrajectory(DrivetrainSubsystem driveTrain) {
        TrajectoryConfig config = slowTrajectoryConfig();

        // An example trajectory to follow.  All units in meters.
        Trajectory trajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(new Translation2d(1, 0)),
                // End back at the origin
                new Pose2d(2, 0, new Rotation2d(0)),
                config);

        SwerveControllerCommand swerveControllerCommand = swerveCommandForTrajectory(driveTrain, trajectory);

        return simpleSwerveTrajectoryCommand(driveTrain, trajectory.getInitialPose(), swerveControllerCommand);
    }

    /**
     * Autonomous command the drives in a square pattern:
     *   (0, 0) => (1m, 0) => (1m, 1m) => (0, 1m) => (0, 0)
     * @param driveTrain Drivetrain subsystem
     * @return Command to be scheduled
     */
    static public Command squareTrajectory(DrivetrainSubsystem driveTrain) {
        TrajectoryConfig config = slowTrajectoryConfig();

        // An example trajectory to follow.  All units in meters.
        Trajectory trajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(new Translation2d(1, 0), new Translation2d(1, 1), new Translation2d(0, 1)),
                // End back at the origin
                new Pose2d(0, 0, new Rotation2d(0)),
                config);

        SwerveControllerCommand swerveControllerCommand = swerveCommandForTrajectory(driveTrain, trajectory);

        return simpleSwerveTrajectoryCommand(driveTrain, trajectory.getInitialPose(), swerveControllerCommand);
    }

    /**
     * Generate a command group that resets the odometry of the drive train to the given initial pose
     * and then runs the given swerve command.
     * @param driveTrain The drive train subsystem
     * @param initialPose The initial pose of the robot
     * @param swerveControllerCommand The SwerveControllerCommand that will run a trajectory
     * @return
     */
    private static Command simpleSwerveTrajectoryCommand(DrivetrainSubsystem driveTrain, Pose2d initialPose,
            SwerveControllerCommand swerveControllerCommand) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> driveTrain.resetOdometry(initialPose)),
            swerveControllerCommand,
            new InstantCommand(() -> driveTrain.drive(new Translation2d(), 0, true, true))
        );
    }

    private static SwerveControllerCommand swerveCommandForTrajectory(DrivetrainSubsystem driveTrain, Trajectory exampleTrajectory) {
        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                exampleTrajectory,
                driveTrain::getPose,
                Constants.DriveTrain.kSwerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                driveTrain::setModuleStates,
                driveTrain);
        return swerveControllerCommand;
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
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared * Constants.AutoConstants.kSlowDownFactor)
            .setKinematics(Constants.DriveTrain.kSwerveKinematics);
    }



}
