package frc.robot.utils;

import java.util.Objects;

/**
 * A swerve module configuration.
 * <p>
 * A configuration represents a unique mechanical configuration of a module. For example, the Swerve Drive Specialties
 * Mk3 swerve module has two configurations, standard and fast, and therefore should have two configurations
 * ({@link SdsModuleConfigurations#MK3_STANDARD} and {@link SdsModuleConfigurations#MK3_FAST} respectively).
 */
public class SwerveModuleConfiguration {
    private double wheelDiameter;

    private double driveReduction;
    private double steerReduction;

    private boolean drivePositiveClockwise;
    private boolean steerPositiveClockwise;
    private boolean encoderPositiveClockwise;

    private double driveMotorFreeSpeedRPM;
    private double steerMotorFreeSpeedRPM;

    public SwerveModuleConfiguration() {
    }

    // /**
    //  * Creates a new module configuration.
    //  *
    //  * @param wheelDiameter  The diameter of the module's wheel in meters.
    //  * @param driveReduction The overall drive reduction of the module. Multiplying motor rotations by this value
    //  *                       should result in wheel rotations.
    //  * @param driveInverted  Whether the drive motor should be inverted. If there is an odd number of gear reductions
    //  *                       this is typically true.
    //  * @param steerReduction The overall steer reduction of the module. Multiplying motor rotations by this value
    //  *                       should result in rotations of the steering pulley.
    //  * @param steerInverted  Whether the steer motor should be inverted. If there is an odd number of gear reductions
    //  *                       this is typically true.
    //  * @param driveMotorFreeSpeed The drive motor's free speed RPM (maximum RPM)
    //  */
    // public SwerveModuleConfiguration(double wheelDiameter, double driveReduction, boolean driveInverted,
    //                            double steerReduction, boolean steerInverted, double driveMotorFreeSpeed,
    //                            double steerMotorFreeSpeedRPM) {
    //     this.wheelDiameter = wheelDiameter;
    //     this.driveReduction = driveReduction;
    //     this.driveInverted = driveInverted;
    //     this.steerReduction = steerReduction;
    //     this.steerInverted = steerInverted;
    //     this.driveMotorFreeSpeedRPM = driveMotorFreeSpeed;
    //     this.steerMotorFreeSpeedRPM = steerMotorFreeSpeedRPM;
    // }

    /**
     * Builder pattern to set wheel diameter
     * @param wheelDiameter (meters)
     * @return SwerveModuleConfiguration
     */
    public SwerveModuleConfiguration withWheelDiameter(double wheelDiameter) {
        this.wheelDiameter = wheelDiameter;
        return this;
    }

    /**
     * Builder pattern to set drive reduction
     * @param driveReduction 
     * @return SwerveModuleConfiguration
     */
    public SwerveModuleConfiguration withDriveReduction(double driveReduction) {
        this.driveReduction = driveReduction;
        return this;
    }

    /**
     * Builder pattern to set steer reduction
     * @param steerReduction 
     * @return SwerveModuleConfiguration
     */
    public SwerveModuleConfiguration withSteerReduction(double steerReduction) {
        this.steerReduction = steerReduction;
        return this;
    }

    /**
     * Builder pattern: free drive motor speed
     * @param driveMotorFreeSpeed (RPM)
     * @return SwerveModuleConfiguration
     */
    public SwerveModuleConfiguration withDriveMotorFreeSpeed(double driveMotorFreeSpeed) {
        this.driveMotorFreeSpeedRPM = driveMotorFreeSpeed;
        return this;
    }

    /**
     * Builder pattern: free steer motor speed
     * @param steerMotorFreeSpeed (RPM)
     * @return SwerveModuleConfiguration
     */
    public SwerveModuleConfiguration withSteerMotorFreeSpeed(double steerMotorFreeSpeed) {
        this.steerMotorFreeSpeedRPM = steerMotorFreeSpeed;
        return this;
    }

    /**
     * Builder pattern: drive direction
     * @param drivePositiveClockwise (boolean)
     * @return SwerveModuleConfiguration
     */
    public SwerveModuleConfiguration withDrivePositiveClockwise(boolean drivePositiveClockwise) {
        this.drivePositiveClockwise = drivePositiveClockwise;
        return this;
    }

    /**
     * Builder pattern: steer direction
     * @param steerPositiveClockwise (boolean)
     * @return SwerveModuleConfiguration
     */
    public SwerveModuleConfiguration withSteerPositiveClockwise(boolean steerPositiveClockwise) {
        this.steerPositiveClockwise = steerPositiveClockwise;
        return this;
    }

    /**
     * Builder pattern: encoder inversion
     * @param encoderPositiveClockwise (boolean)
     * @return SwerveModuleConfiguration
     */
    public SwerveModuleConfiguration withEncoderPositiveClockwise(boolean encoderPositiveClockwise]) {
        this.encoderPositiveClockwise = encoderPositiveClockwise;
        return this;
    }

    /**
     * Gets the diameter of the wheel in meters.
     */
    public double getWheelDiameter() {
        return wheelDiameter;
    }

    /**
     * Gets the overall reduction of the drive system.
     * <p>
     * If this value is multiplied by drive motor rotations the result would be drive wheel rotations.
     */
    public double getDriveReduction() {
        return driveReduction;
    }

    /**
     * Gets if the drive motor should be inverted.
     */
    public boolean isDrivePositiveClockwise() {
        return drivePositiveClockwise;
    }

    /**
     * Gets the overall reduction of the steer system.
     * <p>
     * If this value is multiplied by steering motor rotations the result would be steering pulley rotations.
     */
    public double getSteerReduction() {
        return steerReduction;
    }

    /**
     * Gets if the steering motor should be inverted.
     */
    public boolean isSteerPositiveClockwise() {
        return steerPositiveClockwise;
    }

    /**
     * Gest if the encoder should be inverted
     */
    public boolean isEncoderPositiveClockwise() {
        return encoderPositiveClockwise;
    }

    /**
     * Gets the drive motor free speed RPM
     */
    public double getDriveMotorFreeSpeedRPM() {
        return driveMotorFreeSpeedRPM;
    }

    /**
     * Gets the steer motor free speed RPM
     */
    public double getSteerMotorFreeSpeedRPM() {
        return steerMotorFreeSpeedRPM;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        SwerveModuleConfiguration that = (SwerveModuleConfiguration) o;
        return Double.compare(that.getWheelDiameter(), getWheelDiameter()) == 0 &&
                Double.compare(that.getDriveReduction(), getDriveReduction()) == 0 &&
                isDriveInverted() == that.isDriveInverted() &&
                Double.compare(that.getSteerReduction(), getSteerReduction()) == 0 &&
                isSteerInverted() == that.isSteerInverted();
    }

    @Override
    public int hashCode() {
        return Objects.hash(
                getWheelDiameter(),
                getDriveReduction(),
                isDriveInverted(),
                getSteerReduction(),
                isSteerInverted()
        );
    }

    @Override
    public String toString() {
        return "ModuleConfiguration{" +
                "wheelDiameter=" + wheelDiameter +
                ", driveReduction=" + driveReduction +
                ", driveInverted=" + driveInverted +
                ", steerReduction=" + steerReduction +
                ", steerInverted=" + steerInverted +
                '}';
    }
}
