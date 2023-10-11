package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.robot.Constants;
import frc.robot.utils.ModuleStateUtils;
import frc.robot.utils.SwerveModuleConfiguration;
import frc.robot.utils.SwerveMotorConfiguration;

public class SwerveModule {
    public final int moduleIndex;
    protected final SwerveModuleConfiguration m_configuration;

    protected final SwerveModuleIO m_io;
    protected final SwerveModuleIOInputsAutoLogged m_inputs;
    protected SimpleMotorFeedforward m_driveFeedForward;

    private double m_referenceAngleRadians;
    private double m_commandedSpeedMetersPerSecond;

    private SimpleMotorFeedforward m_driveFeedforward;

    public SwerveModule(int moduleIndex, SwerveModuleIO io, SwerveModuleConfiguration configuration, 
        SwerveMotorConfiguration driveConfiguration, SwerveMotorConfiguration steerConfiguration,
        ShuffleboardLayout container) {

        this.moduleIndex = moduleIndex;
        m_io = io;
        m_inputs = new SwerveModuleIOInputsAutoLogged();
        m_configuration = configuration;

        if (driveConfiguration.hasFeedForwardConstants()) {
            m_driveFeedforward = new SimpleMotorFeedforward(driveConfiguration.staticConstant, driveConfiguration.velocityConstant, driveConfiguration.accelerationConstant);
        }

        addDashboardEntries(container);
    }

    public void periodic() {
        m_io.updateInputs(m_inputs);
        Logger.getInstance().processInputs(String.format("SwerveModule/%d", moduleIndex), m_inputs);
    }

    public void configure() {
        m_io.configure();
    }

    public void configureDriveMotorPID(double kP, double kI, double kD, double kF) {
        m_io.configureDriveMotorPID(kP, kI, kD, kF);
    }

    /**
     * Synchronize the steer motor's internal position with the absolute encoder by setting the 
     * motor's internal sensor to the angle read by the absolute encoder. This should only be called
     * when the motor is relatively idle.
     */
    public void resetSteerPositionSensor() {
        double absoluteAngle = getAbsoluteAngle();
        m_io.resetSteerPositionSensor(absoluteAngle);
        m_referenceAngleRadians = absoluteAngle;
    }

    public void zeroDrivePositionSensor() {
        m_io.zeroDrivePositionSensor();
    }


    /**
     * Get the absolute angle as reported by the absolute encoder
     * @return angle in radians [0, 2*pi]
     */
    protected double getAbsoluteAngle() {
        double angle = m_inputs.encoderAngle;

        // This probably isn't necessary. The CANCoder always reports [0, 360]
        angle = ModuleStateUtils.positiveModulus(angle, 2.0 * Math.PI);

        return angle;
    }

    /**
     * Get the velocity and angle of the module as a SwerveModuleState object
     * @return SwerveModuleState for this module
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getStateVelocity(), new Rotation2d(getStateAngle()));
    }

    /**
     * Set the desired state of the module
     * @param desiredState SwerveModuleState 
     * @param isOpenLoop boolean, Controls whether open or closed loop control is used
     * @param optimize boolean,  Indicates whether  angle optimization is allowed. This should be true except for calibration.
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop, boolean optimize) {
        /* This is a custom optimize function, since default WPILib optimize assumes continuous 
        * controller which CTRE and Rev onboard is not 
        */
        if (optimize) {
            desiredState = ModuleStateUtils.optimize(desiredState, getState().angle); 
        }
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    /**
     * Set the speed of the module based on the desired state.
     * @param desiredState SwerveModuleState that is desired
     * @param isOpenLoop Is the system being driven open loop from a joystick
     */
    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        double speed = Constants.Overrides.kDisableMotion ? 0 : desiredState.speedMetersPerSecond;
        m_commandedSpeedMetersPerSecond = speed;
        if (isOpenLoop) {
            double percentOutput = speed / getMaxVelocity();
            m_io.setDrivePercentOutput(percentOutput);
        }
        else {
            if (m_driveFeedforward != null) {
                m_io.setDriveVelocity(speed, m_driveFeedforward.calculate(speed));
            } else {
                m_io.setDriveVelocity(speed);
            }
        }
    }

    protected void setAbsoluteAngle(double angle) {
        m_referenceAngleRadians = angle;
        m_io.setAngle(angle);
    }

    private void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (getMaxVelocity() * 0.01)) ? new Rotation2d(m_referenceAngleRadians) : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        m_referenceAngleRadians = angle.getRadians();
        m_io.setAngle(angle.getRadians());
    }

    /**
     * Add dashboard entries for the module to given container
     * @param container ShuffleboardContainer to use
     */
    private void addDashboardEntries(ShuffleboardContainer container) {
        container.addNumber("Absolute Encoder Angle", () -> Math.toDegrees(getAbsoluteAngle()));
        container.addNumber("Current Angle", () -> Math.toDegrees(getStateAngle()));
        container.addNumber("Target Angle", () -> Math.toDegrees(getReferenceAngle()));
        container.addNumber("Current Velocity", this::getStateVelocity);
        container.addNumber("Commanded Velocity", () -> m_commandedSpeedMetersPerSecond);
    }

    /**
     * Return the current velocity of the module as measured by the drive motor.
     * @return Current wheel velocity in m/s.
     */
    protected double getStateVelocity() {
        return m_inputs.driveVelocity;
    }

    /**
     * Return the current distance the wheel has traveled as measured by the drive motor.
     * @return Current wheel distance in meters
     */
    protected double getStateDistance() {
        return m_inputs.drivePosition;
    }

    /**
     * Return the current SwerveModulePosition for this module
     * @return SwerveModulePosition for this module
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getStateDistance(), new Rotation2d(getStateAngle()));
    }

    /**
     * Returns the raw state angle from the motor. Useful for plotting when trying to debug motor
     * turning discontinuities.
     * @return double, raw sensor position in radians
     */
    private double getStateAngle() {
        return m_inputs.steerPosition;
    }

    /**
     * Return the target angle (set point) of the wheel
     * @return Target angle in radians
     */
    protected double getReferenceAngle() {
        return m_referenceAngleRadians;
    }

    /**
     * Gets the drive motor free velocity in m/s
     */
    protected double getMaxVelocity() {
        return m_configuration.getDriveMotorFreeSpeedRPM() / 60.0 * m_configuration.getDriveReduction() * m_configuration.getWheelDiameter() * Math.PI;
    }

    /**
     * Gets the azimuth motor free speed angular velocity in radians/s
     */
    protected double getMaxAngularVelocity() {
        // v = rw => w = v / r
        double linearSpeed =  m_configuration.getSteerMotorFreeSpeedRPM() / 60.0 * m_configuration.getSteerReduction() * m_configuration.getWheelDiameter() * Math.PI;
        double diameter = Math.hypot(Constants.DriveTrain.kTrackWidthMeters, Constants.DriveTrain.kWheelBaseMeters);

        return linearSpeed / diameter / 2.0;
    }

}

