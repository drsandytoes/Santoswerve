package frc.robot.commands.calibration;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DrivetrainSubsystem;

public class WaitForDistance extends CommandBase {
    private double m_distance = 0.0;
    private DrivetrainSubsystem m_driveTrain;

    public WaitForDistance(double distance, DrivetrainSubsystem driveTrain) {
        m_distance = distance;
        m_driveTrain = driveTrain;

        // We purposefully don't take a subsystem since we're only monitoring here.
    }

    public boolean isFinished() {
        // We're assuming we've driving straight, and uniformly. As soon as one module has
        // gone past the limit, end the command.
        SwerveModulePosition[] positions = m_driveTrain.getModulePositions();
        for (SwerveModulePosition position : positions) {
            if (position.distanceMeters >= m_distance) {
                return true;
            }
        }

        return false;
    }
}
