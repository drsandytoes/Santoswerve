package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.robot.Constants;
import frc.robot.Constants.FalconConstants;

public class SwerveModule {
    private static final int ENCODER_RESET_ITERATIONS = 500;
    private static final double ENCODER_RESET_MAX_ANGULAR_VELOCITY = Math.toRadians(0.5);
    private int resetIteration = 0;

    public final int moduleIndex;

    private static final int CAN_TIMEOUT_MS = 250;
    private static final int STATUS_FRAME_GENERAL_PERIOD_MS = 250;
    
    private final double m_steerMotorSensorPositionCoefficient;
    private final double m_steerMotorSensorVelocityCoefficient;

    private final double m_driveMotorSensorPositionCoefficient;
    private final double m_driveMotorSensorVelocityCoefficient;

    private WPI_TalonFX m_driveMotor;
    private WPI_TalonFX m_steerMotor;
    private WPI_CANCoder m_steerEncoder;

    private double m_steerOffset;

    private SwerveModuleConfiguration m_configuration;
    private SwerveMotorConfiguration m_driveMotorOptions, m_steerMotorOptions;

    private double m_referenceAngleRadians;
    private double m_commandedAngleRadians;
    private double m_commandedSpeedMetersPerSecond;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.DriveTrain.kDriveMotorOptions.staticConstant, Constants.DriveTrain.kDriveMotorOptions.velocityConstant, Constants.DriveTrain.kDriveMotorOptions.staticConstant);

    public SwerveModule(int moduleIndex, ShuffleboardLayout container, SwerveModuleConfiguration configuration, SwerveMotorConfiguration driveMotorOptions, SwerveMotorConfiguration steerMotorOptions, CANDeviceID driveMotorID, CANDeviceID steerMotorID, CANDeviceID steerEncodeID, double steerOffset) {
        this.moduleIndex = moduleIndex;

        m_driveMotor = new WPI_TalonFX(driveMotorID.id, driveMotorID.bus);
        m_steerMotor = new WPI_TalonFX(steerMotorID.id, steerMotorID.bus);
        m_steerEncoder = new WPI_CANCoder(steerEncodeID.id, steerEncodeID.bus);
        m_steerOffset = steerOffset;
        m_configuration = configuration;

        m_driveMotorOptions = driveMotorOptions;
        m_steerMotorOptions = steerMotorOptions;

        // Used to convert the motor's reported position from native units to radians
        // Falcon's sensor position is in ticks. To convert the reported sensor position to an angle in radians:
        // radians = sensor-reading * (1 revolution / ticksPerRevolution) * (2pi radians / 1 revolution) * gear reduction
        m_steerMotorSensorPositionCoefficient = 2.0 * Math.PI / FalconConstants.kTicksPerRotation * m_configuration.getSteerReduction();

        // Used to convert the motor's reported velocity from native units to radians/second
        m_steerMotorSensorVelocityCoefficient = m_steerMotorSensorPositionCoefficient * FalconConstants.kVelocityTicksPerSecondConversion;

        // Used to convert the motor's reported position from native units to meters
        // Falcon's sensor position is in ticks. To conver the reported sensor position to a distance in meters:
        // meters = sensor-reading * (1 revolution / ticksPerRevolution) * (pi * wheelDiameterInMeters / revolution) * gear reduction
        m_driveMotorSensorPositionCoefficient = Math.PI * m_configuration.getWheelDiameter() * m_configuration.getDriveReduction() / FalconConstants.kTicksPerRotation;

        // Used to convert the motor's reported velocity from native units to meters/second
        m_driveMotorSensorVelocityCoefficient = m_driveMotorSensorPositionCoefficient * FalconConstants.kVelocityTicksPerSecondConversion;

        addDashboardEntries(container);
    }

    public void configure() {
        configureCANCoder();
        configureDriveMotor();
        configureSteerMotor();
    }

    public void configureCANCoder() {
        CANCoderConfiguration config = new CANCoderConfiguration();
        config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        config.magnetOffsetDegrees = Math.toDegrees(m_steerOffset);
        config.sensorDirection = false;

        m_steerEncoder.configAllSettings(config, 250);
        m_steerEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10, 250);
    }

    public void configureDriveMotor() {
        TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

        if (m_driveMotorOptions.hasPidConstants()) {
            motorConfiguration.slot0.kP = m_driveMotorOptions.proportionalConstant;
            motorConfiguration.slot0.kI = m_driveMotorOptions.integralConstant;
            motorConfiguration.slot0.kD = m_driveMotorOptions.derivativeConstant;
        }

        if (m_driveMotorOptions.hasVoltageCompensation()) {
            motorConfiguration.voltageCompSaturation = m_driveMotorOptions.nominalVoltage;
        }

        if (m_driveMotorOptions.hasCurrentLimit()) {
            motorConfiguration.supplyCurrLimit.currentLimit = m_driveMotorOptions.currentLimit;
            motorConfiguration.supplyCurrLimit.enable = true;
        }

        m_driveMotor.configAllSettings(motorConfiguration);

        if (m_driveMotorOptions.hasVoltageCompensation()) {
            // Enable voltage compensation
            m_driveMotor.enableVoltageCompensation(true);
        }

        m_driveMotor.setNeutralMode(NeutralMode.Brake);

        m_driveMotor.setInverted(m_configuration.isDriveInverted() ? TalonFXInvertType.Clockwise : TalonFXInvertType.CounterClockwise);
        m_driveMotor.setSensorPhase(true);

        // Reduce CAN status frame rates
        m_driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, STATUS_FRAME_GENERAL_PERIOD_MS, CAN_TIMEOUT_MS);
    }

    public void configureSteerMotor() {
        TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
        if (m_steerMotorOptions.hasPidConstants()) {
            motorConfiguration.slot0.kP = m_steerMotorOptions.proportionalConstant;
            motorConfiguration.slot0.kI = m_steerMotorOptions.integralConstant;
            motorConfiguration.slot0.kD = m_steerMotorOptions.derivativeConstant;
        }
        if (m_steerMotorOptions.hasMotionMagic()) {
            if (m_steerMotorOptions.hasVoltageCompensation()) {
                motorConfiguration.slot0.kF = (1023.0 * m_steerMotorSensorVelocityCoefficient / m_steerMotorOptions.nominalVoltage) * m_steerMotorOptions.velocityConstant;
            } else {
                assert(false);
                // TODO: What should be done if no nominal voltage is configured? Use a default voltage?
            }

            // TODO: Make motion magic max voltages configurable or dynamically determine optimal values
            motorConfiguration.motionCruiseVelocity = 2.0 / m_steerMotorOptions.velocityConstant / m_steerMotorSensorVelocityCoefficient;
            motorConfiguration.motionAcceleration = (8.0 - 2.0) / m_steerMotorOptions.accelerationConstant / m_steerMotorSensorVelocityCoefficient;
        }
        if (m_steerMotorOptions.hasVoltageCompensation()) {
            motorConfiguration.voltageCompSaturation = m_steerMotorOptions.nominalVoltage;
        }
        if (m_steerMotorOptions.hasCurrentLimit()) {
            motorConfiguration.supplyCurrLimit.currentLimit = m_steerMotorOptions.currentLimit;
            motorConfiguration.supplyCurrLimit.enable = true;
        }

        m_steerMotor.configAllSettings(motorConfiguration, CAN_TIMEOUT_MS);

        if (m_steerMotorOptions.hasVoltageCompensation()) {
            m_steerMotor.enableVoltageCompensation(true);
        }
        CtreUtils.checkCtreError(m_steerMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, CAN_TIMEOUT_MS),
        "Set selected sensor");
        m_steerMotor.setSensorPhase(true);
        m_steerMotor.setInverted(m_configuration.isSteerInverted() ? TalonFXInvertType.CounterClockwise : TalonFXInvertType.Clockwise);
        m_steerMotor.setNeutralMode(NeutralMode.Brake);

        double currentAngle = getAbsoluteAngle();
        resetSteerPositionSensor();
        m_referenceAngleRadians = currentAngle; // MDS
        System.out.println("Init angle: " + currentAngle);
        System.out.println("Init angle: " + Math.toDegrees(currentAngle));

        double sensorPosition = m_steerMotor.getSelectedSensorPosition() * m_steerMotorSensorPositionCoefficient;
        System.out.println("Post-set motor sensor position: " + Math.toDegrees(sensorPosition));

        // Reduce CAN status frame rates
        m_steerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, STATUS_FRAME_GENERAL_PERIOD_MS, CAN_TIMEOUT_MS);

        // MDS: Set initial position (even though motor disabled?)
        // if (m_steerMotorOptions.hasPidConstants() || m_steerMotorOptions.hasMotionMagic()) {
        //     m_steerMotor.set(TalonFXControlMode.Position, currentAngle / m_steerMotorSensorPositionCoefficient);
        // }
    }

    /**
     * Synchronize the steer motor's internal position with the absolute encoder by setting the 
     * motor's internal sensor to the angle read by the absolute encoder. This should only be called
     * when the motor is relatively idle.
     */
    public void resetSteerPositionSensor() {
        double absoluteAngle = getAbsoluteAngle();
        CtreUtils.checkCtreError(m_steerMotor.setSelectedSensorPosition(absoluteAngle / m_steerMotorSensorPositionCoefficient),
        "Reset motor sensor position");
    }


    /**
     * Get the absolute angle as reported by the absolute encoder
     * @return angle in radians [0, 2*pi]
     */
    public double getAbsoluteAngle() {
        double angle = Math.toRadians(m_steerEncoder.getAbsolutePosition());
        angle %= 2.0 * Math.PI;
        if (angle < 0.0) {
            angle += 2.0 * Math.PI;
        }

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
     * @param isOpenLoop not used
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        desiredState = ModuleStateUtils.optimize(desiredState, getState().angle); 
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
            m_driveMotor.set(ControlMode.PercentOutput, percentOutput);
        }
        else {
            double velocity = speed / m_driveMotorSensorVelocityCoefficient;
            m_driveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(speed));
        }
    }

    private void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (getMaxVelocity() * 0.01)) ? new Rotation2d(m_referenceAngleRadians) : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        m_steerMotor.set(ControlMode.Position, angle.getRadians() / m_steerMotorSensorPositionCoefficient);
        m_referenceAngleRadians = angle.getRadians();
        m_commandedAngleRadians = m_referenceAngleRadians;
    }

    /**
     * Add dashboard entries for the module to given container
     * @param container ShuffleboardContainer to use
     */
    private void addDashboardEntries(ShuffleboardContainer container) {
        container.addNumber("Absolute Encoder Angle", () -> Math.toDegrees(getAbsoluteAngle()))
            .withPosition(0, 0);
        container.addNumber("Current Angle", () -> Math.toDegrees(getStateAngle()))
            .withPosition(0, 1);
        container.addNumber("Commanded Angle", () -> Math.toDegrees(m_commandedAngleRadians))
            .withPosition(0, 2);
        container.addNumber("Target Angle", () -> Math.toDegrees(getReferenceAngle()))
            .withPosition(0, 3);
        container.addNumber("Current Velocity", this::getStateVelocity)
            .withPosition(0, 4);
        container.addNumber("Commanded Velocity", () -> m_commandedSpeedMetersPerSecond)
            .withPosition(0, 5);
    }

    /**
     * Return the current velocity of the module as measured by the drive motor.
     * @return Current wheel velocity in m/s.
     */
    public double getStateVelocity() {
        return m_driveMotor.getSelectedSensorVelocity() * m_driveMotorSensorVelocityCoefficient;
    }

    /**
     * Return the current distance the wheel has traveled as measured by the drive motor.
     * @return Current wheel distance in meters
     */
    public double getStateDistance() {
        double distance = m_driveMotor.getSelectedSensorPosition() * m_driveMotorSensorPositionCoefficient;
        return distance;
    }

    /**
     * Return the current SwerveModulePosition for this module
     * @return SwerveModulePosition for this module
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getStateDistance(), new Rotation2d(getStateAngle()));
    }

    /**
     * Return the current angle of the wheel as measured by the steer motor.
     * @return Current wheel angle in radians [0, 2*pi]
     */
    public double getStateAngle() {
        double motorAngleRadians = m_steerMotor.getSelectedSensorPosition() * m_steerMotorSensorPositionCoefficient;
        motorAngleRadians %= 2.0 * Math.PI;
        if (motorAngleRadians < 0.0) {
            motorAngleRadians += 2.0 * Math.PI;
        }

        return motorAngleRadians;
    }

    /**
     * Return the target angle (set point) of the wheel
     * @return Target angle in radians
     */
    public double getReferenceAngle() {
        return m_referenceAngleRadians;
    }

    /**
     * Set the target angle of the wheel
     * @param referenceAngleRadians Target / reference angle in radians
     */
    public void setReferenceAngle(double referenceAngleRadians) {
        double currentAngleRadians = m_steerMotor.getSelectedSensorPosition() * m_steerMotorSensorPositionCoefficient;

        // When the module has not been rotating for a while, reset the motor's position back to the angle read
        // from the absolute encoder. This seems to be needed when the motors are first enabled, but also helps
        // with drift over time?
        if (m_steerMotor.getSelectedSensorVelocity() * m_steerMotorSensorPositionCoefficient < ENCODER_RESET_MAX_ANGULAR_VELOCITY) {
            if (++resetIteration >= ENCODER_RESET_ITERATIONS) {
                resetIteration = 0;
                double absoluteAngle = getAbsoluteAngle();
                m_steerMotor.setSelectedSensorPosition(absoluteAngle / m_steerMotorSensorPositionCoefficient);
                currentAngleRadians = absoluteAngle;
            }
        } else {
            resetIteration = 0;
        }

        double currentAngleRadiansMod = currentAngleRadians % (2.0 * Math.PI);
        if (currentAngleRadiansMod < 0.0) {
            currentAngleRadiansMod += 2.0 * Math.PI;
        }

        // The reference angle has the range [0, 2pi) but the Falcon's encoder can go above that
        double adjustedReferenceAngleRadians = referenceAngleRadians + currentAngleRadians - currentAngleRadiansMod;
        if (referenceAngleRadians - currentAngleRadiansMod > Math.PI) {
            adjustedReferenceAngleRadians -= 2.0 * Math.PI;
        } else if (referenceAngleRadians - currentAngleRadiansMod < -Math.PI) {
            adjustedReferenceAngleRadians += 2.0 * Math.PI;
        }
        
        TalonFXControlMode controlMode = m_steerMotorOptions.hasMotionMagic() ? TalonFXControlMode.MotionMagic : TalonFXControlMode.Position;
        m_steerMotor.set(controlMode, adjustedReferenceAngleRadians / m_steerMotorSensorPositionCoefficient);

        m_referenceAngleRadians = referenceAngleRadians;
        m_commandedAngleRadians = adjustedReferenceAngleRadians;
    }

    /**
     * Gets the drive motor free velocity in m/s
     */
    public double getMaxVelocity() {
        return m_configuration.getDriveMotorFreeSpeedRPM() / 60.0 * m_configuration.getDriveReduction() * m_configuration.getWheelDiameter();
    }

    /**
     * Gets the azimuth motor free speed angular velocity in radians/s
     */
    public double getMaxAngularVelocity() {
        // v = rw => w = v / r
        double linearSpeed =  m_configuration.getSteerMotorFreeSpeedRPM() / 60.0 * m_configuration.getSteerReduction() * m_configuration.getWheelDiameter();
        double diameter = Math.hypot(Constants.DriveTrain.kTrackWidthMeters, Constants.DriveTrain.kWheelBaseMeters);

        return linearSpeed / diameter / 2.0;
    }

}

