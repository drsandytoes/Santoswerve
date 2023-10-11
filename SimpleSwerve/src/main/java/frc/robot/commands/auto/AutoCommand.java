package frc.robot.commands.auto;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DrivetrainSubsystem;

public interface AutoCommand {
    public String getName();
    public Command createCommand(DrivetrainSubsystem driveTrain, TrajectoryConfig trajectoryConfig);
}
