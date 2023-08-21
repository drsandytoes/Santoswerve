package frc.robot.utils;

public class SwerveMotorConfiguration {
    // PID configuration
    public double proportionalConstant = Double.NaN;
    public double integralConstant = Double.NaN;
    public double derivativeConstant = Double.NaN;

    // Motion magic configuration
    public double velocityConstant = Double.NaN;
    public double accelerationConstant = Double.NaN;
    public double staticConstant = Double.NaN;

    public double currentLimit = Double.NaN;
    public double nominalVoltage = Double.NaN;

    public SwerveMotorConfiguration() {
    }

    public SwerveMotorConfiguration withCurrentLimit(double currentLimit) {
        this.currentLimit = currentLimit;
        return this;
    }

    public SwerveMotorConfiguration withPIDConstants(double kP, double kI, double kD) {
        proportionalConstant = kP;
        integralConstant = kI;
        derivativeConstant = kD;
        return this;
    }

    public SwerveMotorConfiguration withNominalVoltage(double nominalVoltage) {
        this.nominalVoltage = nominalVoltage;
        return this;
    }

    public SwerveMotorConfiguration withMotionMagicContants(double kVelocity, double kAcceleration, double kStatic) {
        velocityConstant = kVelocity;
        accelerationConstant = kAcceleration;
        staticConstant = kStatic;
        return this;
    }

    public boolean hasPidConstants() {
        return Double.isFinite(proportionalConstant) && Double.isFinite(integralConstant) && Double.isFinite(derivativeConstant);
    }

    public boolean hasMotionMagic() {
        return Double.isFinite(velocityConstant) && Double.isFinite(accelerationConstant) && Double.isFinite(staticConstant);
    }

    public boolean hasVoltageCompensation() {
        return Double.isFinite(nominalVoltage);
    }

    public boolean hasCurrentLimit() {
        return Double.isFinite(currentLimit);
    }
}
