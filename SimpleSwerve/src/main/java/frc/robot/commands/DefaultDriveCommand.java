package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;

    /**
     * Create a drive command with the given suppliers and drive subsystem. The suppliers should 
     * have already applied a deadband and converted whatever input into velocities.
     * @param drivetrainSubsystem The drive subsystem to use
     * @param translationXSupplier Double supplier that provides desired forward motion in m/s
     * @param translationYSupplier Double supplier that provides leftward motion in m/s
     * @param rotationSupplier Double supplier that provides counterclockwise rotation in radians/s
     */
    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier,
            DoubleSupplier rotationSupplier) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        // Apply deadband, and then square inputs, preserving sign.
        double translationVal = m_translationXSupplier.getAsDouble();
        double strafeVal = m_translationYSupplier.getAsDouble();
        double rotationVal = m_rotationSupplier.getAsDouble();

        /* Drive */
        m_drivetrainSubsystem.drive(
                new Translation2d(translationVal, strafeVal).times(DrivetrainSubsystem.getMaxLinearVelocity()),
                rotationVal * DrivetrainSubsystem.getMaxAngularVelocity(),
                true,
                true);

    }

    @Override
    public void end(boolean interrupted) {
        // Command the drive train to stop in case nothing comes along and says otherwise
        m_drivetrainSubsystem.drive(new Translation2d(), 0, true, true);
    }
}
