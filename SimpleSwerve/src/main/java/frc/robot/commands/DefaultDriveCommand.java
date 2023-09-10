package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;

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
        double translationVal = MathUtil.applyDeadband(m_translationXSupplier.getAsDouble(),
                Constants.Operator.kDeadband);
        double strafeVal = MathUtil.applyDeadband(m_translationYSupplier.getAsDouble(), Constants.Operator.kDeadband);
        double rotationVal = MathUtil.applyDeadband(m_rotationSupplier.getAsDouble(), Constants.Operator.kDeadband);

        /* Drive */
        m_drivetrainSubsystem.drive(
                new Translation2d(translationVal, strafeVal).times(m_drivetrainSubsystem.getMaxLinearVelocity()),
                rotationVal * m_drivetrainSubsystem.getMaxAngularVelocity(),
                true,
                true);

    }

    @Override
    public void end(boolean interrupted) {
        // Command the drive train to stop in case nothing comes along and says otherwise
        m_drivetrainSubsystem.drive(new Translation2d(), 0, true, true);
    }
}
