package frc.robot.commands.auto;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
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

public abstract class AutoCommandBase {
    final static String name = "unknown";

    abstract public Command createCommand(DrivetrainSubsystem driveTrain, TrajectoryConfig trajectoryConfig);

    /**
     * Helper method to return a simple trajectory created from an initial and final pose, and interior
     * waypoints. Also takes a TrajectoryConfig controlling velocity and acceleration limits.
     * @param initialPose Pose2d initial pose of the robot
     * @param wayPoints List of Translation2d interior waypoints
     * @param finalPose Pose2d final pose of the robot
     * @param trajectoryConfig TrajectoryConfig with velocity and acceleration limits
     * @return Returns the computed Trajectory
     */
    Trajectory trajectoryFromWaypoints(Pose2d initialPose, List<Translation2d> wayPoints, Pose2d finalPose, TrajectoryConfig trajectoryConfig) {
        return TrajectoryGenerator.generateTrajectory(
            initialPose,
            wayPoints,
            finalPose,
            trajectoryConfig);
    }

    /**
     * Generate a command group that resets the odometry of the drive train to the given initial pose
     * and then runs the given swerve command.
     * @param driveTrain The drive train subsystem
     * @param initialPose The initial pose of the robot
     * @param swerveControllerCommand The SwerveControllerCommand that will run a trajectory
     * @return
     */
    protected Command simpleSwerveTrajectoryCommand(DrivetrainSubsystem driveTrain, Pose2d initialPose,
            SwerveControllerCommand swerveControllerCommand) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> driveTrain.resetOdometry(initialPose)),
            swerveControllerCommand,
            new InstantCommand(() -> driveTrain.drive(new Translation2d(), 0, true, true))
        );
    }

    protected SwerveControllerCommand swerveCommandForTrajectory(DrivetrainSubsystem driveTrain, Trajectory exampleTrajectory) {
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

}
