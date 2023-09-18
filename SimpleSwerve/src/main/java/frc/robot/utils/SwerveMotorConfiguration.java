package frc.robot.utils;

public class SwerveMotorConfiguration {
    // PIDF configuration
    public double proportionalConstant = Double.NaN;
    public double integralConstant = Double.NaN;
    public double derivativeConstant = Double.NaN;
    public double feedForwardConstant = Double.NaN;

    // Feed forward configuration
    public double staticConstant = Double.NaN;
    public double velocityConstant = Double.NaN;
    public double accelerationConstant = Double.NaN;

    // Current limits
    public double continuousCurrentLimit = Double.NaN;
    public double peakCurrentLimit = Double.NaN;
    public double peakCurrentDuration = 0.0;

    public double nominalVoltage = Double.NaN;

    public SwerveMotorConfiguration() {
    }

    public SwerveMotorConfiguration withContinuousCurrentLimit(double currentLimit) {
        this.continuousCurrentLimit = currentLimit;
        return this;
    }

    public SwerveMotorConfiguration withPeakCurrentLimit(double currentLimit) {
        this.peakCurrentLimit = currentLimit;
        return this;
    }

    public SwerveMotorConfiguration withPeakCurrentDuration(double duration) {
        this.peakCurrentDuration = duration;
        return this;
    }

    public SwerveMotorConfiguration withPIDFConstants(double kP, double kI, double kD, double kF) {
        proportionalConstant = kP;
        integralConstant = kI;
        derivativeConstant = kD;
        feedForwardConstant = kF;
        return this;
    }

    public SwerveMotorConfiguration withNominalVoltage(double nominalVoltage) {
        this.nominalVoltage = nominalVoltage;
        return this;
    }

    public SwerveMotorConfiguration withFeedForwardConstants(double kStatic, double kVelocity, double kAcceleration) {
        staticConstant = kStatic;
        velocityConstant = kVelocity;
        accelerationConstant = kAcceleration;
        return this;
    }

    public boolean hasPidConstants() {
        return Double.isFinite(proportionalConstant) && Double.isFinite(integralConstant) && Double.isFinite(derivativeConstant) && Double.isFinite(feedForwardConstant);
    }

    public boolean hasFeedForwardConstants() {
        return Double.isFinite(velocityConstant) && Double.isFinite(accelerationConstant) && Double.isFinite(staticConstant);
    }

    public boolean hasVoltageCompensation() {
        return Double.isFinite(nominalVoltage);
    }

    public boolean hasCurrentLimit() {
        // We only need a continuous current limit
        return Double.isFinite(continuousCurrentLimit);
    }
}
