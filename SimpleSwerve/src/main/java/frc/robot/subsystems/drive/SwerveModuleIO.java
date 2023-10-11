package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {
    @AutoLog
    public static class SwerveModuleIOInputs {
        public double drivePosition = 0.0;
        public double driveVelocity = 0.0;
        public double steerPosition = 0.0;
        public double encoderAngle = 0.0;
    }

    public default void updateInputs(SwerveModuleIOInputs inputs) {};

    public default void configure() {};

    public default void resetSteerPositionSensor(double absoluteAngle) {};

    public default void zeroDrivePositionSensor() {};

    public default void configureDriveMotorPID(double kP, double kI, double kD, double kF) {};

    public default void setAngle(double angle) {};

    public default void setDrivePercentOutput(double percentOutput) {};

    public default void setDriveVelocity(double velocity) {};

    public default void setDriveVelocity(double velocity, double feedForward) {};

}
