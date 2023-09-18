package frc.robot.utils;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class SendableVelocityTuningParameters implements Sendable {
    protected double m_targetVelocity;
    protected double m_distanceLimit;

    public SendableVelocityTuningParameters(double targetVelocity, double distanceLimit) {
        m_targetVelocity = targetVelocity;
        m_distanceLimit = distanceLimit;
    }

    @Override
    public void initSendable(SendableBuilder builder)  {
        builder.setSmartDashboardType("RobotPreferences");
        builder.addDoubleProperty("Target Velocity", () -> m_targetVelocity, (double target) -> m_targetVelocity = target);
        builder.addDoubleProperty("Distance Limit", () -> m_distanceLimit, (double limit) -> m_distanceLimit = limit);
    }

    public synchronized double getTargetVelocity() {
        return m_targetVelocity;
    }

    public synchronized double getDistanceLimit() {
        return  m_distanceLimit;
    }
}
