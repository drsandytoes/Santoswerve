package frc.robot.commands.calibration;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auto.AutoCommand;
import frc.robot.subsystems.drive.DrivetrainSubsystem;

public class CalibrationRun implements AutoCommand {
    private DoubleSupplier m_targetSpeedSupplier;
    private DoubleSupplier m_allowedDistanceSupplier;

    // We take double suppliers so the creator can give us methods that read values from the
    // dashboard.
    public CalibrationRun(DoubleSupplier targetSpeedSupplier, DoubleSupplier distanceSupplier) {
        m_targetSpeedSupplier = targetSpeedSupplier;
        m_allowedDistanceSupplier = distanceSupplier;
    }

    public String getName() { return "Calibration"; };

    public Command createCommand(DrivetrainSubsystem driveTrain, TrajectoryConfig trajectoryConfig) {
        // Get the current values to use to generate the command
        double allowedDistance = m_allowedDistanceSupplier.getAsDouble();
        double targetSpeed = m_targetSpeedSupplier.getAsDouble();

        DataLogManager.log("Allowed distance: " + allowedDistance);
        DataLogManager.log("Generating command for " + targetSpeed + "m/s for " + allowedDistance / targetSpeed  + "s");

        SwerveModuleState[] states = driveTrain.getModuleStates();
        for (SwerveModuleState state : states) {
            state.angle = new Rotation2d(0);
            state.speedMetersPerSecond = targetSpeed;
        }

        SwerveModuleState[] haltedStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            haltedStates[i] = new SwerveModuleState();
        }
        
        return new SequentialCommandGroup(
            // Zero the drive position sensors for graphing purposes
            new InstantCommand(driveTrain::zeroDrivePositionSensors, driveTrain),
            new InstantCommand(driveTrain::reconfigurePIDFromDashboard, driveTrain),

            // Tell the motors to drive at a specified velocity until the time expires
            new ParallelRaceGroup(
                new WaitCommand(allowedDistance / targetSpeed),
                new FunctionalCommand(
                    () -> {},
                    () -> driveTrain.setModuleStatesWithoutOptimization(states), 
                    interrupted -> {},
                    () -> false, // never finishes
                    driveTrain),
                new WaitForDistance(allowedDistance + 0.5, driveTrain)
            ),

            // Halt the drive train!
            new InstantCommand(() ->  driveTrain.setModuleStates(haltedStates), driveTrain)
        );
    }

    
}
