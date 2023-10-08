package frc.robot.utils;

import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {
    @AutoLog
    public static class SwerveModuleIOInputs {
        public double drivePosition = 0.0;
        public double driveVelocity = 0.0;
        public double steerPosition = 0.0;
        public double encoderAngle = 0.0;
    }

    public void updateInputs(SwerveModuleIOInputs inputs);

    public void configure();

    public void resetSteerPositionSensor(double absoluteAngle);

    public void zeroDrivePositionSensor();

    public void configureDriveMotorPID(double kP, double kI, double kD, double kF);

    public void setAngle(double angle);

    public void setDrivePercentOutput(double percentOutput);

    public void setDriveVelocity(double velocity);

    public void setDriveVelocity(double velocity, double feedForward);

}
