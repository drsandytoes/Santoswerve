package frc.robot.utils;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class SendablePIDParameters implements Sendable {
    protected double m_kP;
    protected double m_kI;
    protected double m_kD;
    protected double m_kF;

    public SendablePIDParameters(double kP, double kI, double kD, double kF) {
        m_kP = kP;
        m_kI = kI;
        m_kD = kD;
        m_kF = kF;
    }

    @Override
    public void initSendable(SendableBuilder builder)  {
        builder.setSmartDashboardType("RobotPreferences");
        builder.addDoubleProperty("kP", () -> m_kP, (double kP) -> m_kP = kP);
        builder.addDoubleProperty("kI", () -> m_kI, (double kI) -> m_kP = kI);
        builder.addDoubleProperty("kD", () -> m_kD, (double kD) -> m_kP = kD);
        builder.addDoubleProperty("kF", () -> m_kF, (double kF) -> m_kP = kF);
    }

    public synchronized double getProportionalConstant() {
        return m_kP;
    }

    public synchronized double getIntegralConstant() {
        return m_kI;
    }

    public synchronized double getDerivativeConstant() {
        return m_kD;
    }

    public synchronized double getFeedForwardConstant() {
        return m_kF;
    }
}
