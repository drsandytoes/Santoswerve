package frc.robot.subsystems.drive;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import frc.robot.Constants.DriveTrain;

public class GyroIOPigeon2 implements GyroIO {
    public WPI_Pigeon2 m_pigeon;

    public GyroIOPigeon2() {
        // The important thing about how you configure your gyroscope is that rotating
        // the robot counter-clockwise should cause the angle reading to increase until
        // it wraps back over to zero.
        m_pigeon = new WPI_Pigeon2(DriveTrain.kPigeonID.id, DriveTrain.kPigeonID.bus);
    }

    public void updateInputs(GyroIOInputs inputs) {
        inputs.yaw = m_pigeon.getYaw();
    }

    public void setYaw(double yaw) {
        m_pigeon.setYaw(yaw);
    }
}
