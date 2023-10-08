package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants;
import frc.robot.Constants.FalconConstants;

public class SwerveModuleIOFalcon implements SwerveModuleIO {
    private class FalconMath {
        private final double m_steerMotorSensorPositionCoefficient;
        private final double m_steerMotorSensorVelocityCoefficient;
    
        private final double m_driveMotorSensorPositionCoefficient;
        private final double m_driveMotorSensorVelocityCoefficient;
    
        public FalconMath(SwerveModuleConfiguration configuration) {
            // Used to convert the motor's reported position from native units to radians
            // Falcon's sensor position is in ticks. To convert the reported sensor position to an angle in radians:
            // radians = sensor-reading * (1 revolution / ticksPerRevolution) * (2pi radians / 1 revolution) * gear reduction
            m_steerMotorSensorPositionCoefficient = 2.0 * Math.PI / FalconConstants.kTicksPerRotation * configuration.getSteerReduction();

            // Used to convert the motor's reported velocity from native units to radians/second
            m_steerMotorSensorVelocityCoefficient = m_steerMotorSensorPositionCoefficient * FalconConstants.kVelocityTicksToTicksPerSecond;

            // Used to convert the motor's reported position from native units to meters
            // Falcon's sensor position is in ticks. To conver the reported sensor position to a distance in meters:
            // meters = sensor-reading * (1 revolution / ticksPerRevolution) * (pi * wheelDiameterInMeters / revolution) * gear reduction
            m_driveMotorSensorPositionCoefficient = Math.PI * configuration.getWheelDiameter() * configuration.getDriveReduction() / FalconConstants.kTicksPerRotation;

            // Used to convert the motor's reported velocity from native units to meters/second
            m_driveMotorSensorVelocityCoefficient = m_driveMotorSensorPositionCoefficient * FalconConstants.kVelocityTicksToTicksPerSecond;
        }

        /**
         * Convert a Falcon position (ticks) of the module angle to the
         * real-world position of the module.
         * @param falconAngle Falcon ticks of the azimuth motor
         * @return double, angle of the module (radians)
         */
        public double absoluteAngleFromFalconAngle(double falconAngle) {
            return falconAngle * m_steerMotorSensorPositionCoefficient;
        }

        /**
         * Convert an absolute module angle to a falcon angle (position)
         * @param absAngle Absolute angle in radians
         * @return double, Falcon ticks corresponding to the angle
         */
        public double falconAngleFromAbsoluteAngle(double absAngle) {
            return absAngle / m_steerMotorSensorPositionCoefficient;
        }

        /**
         * Convert a Falcon position (ticks) to a drive position.
         * @param shaftPosition Falcon rotation (ticks)
         * @return double, Robot distance (m)
         */
        public double drivePositionFromFalconPosition(double shaftPosition) {
            return shaftPosition * m_driveMotorSensorPositionCoefficient;
        }

        /**
         * Convert a drive position to a Falcon position (ticks)
         * @param drivePosition Distance in m
         * @return double, Falcon ticks corresponding to the position
         */
        public double falconPositionFromDrivePosition(double drivePosition) {
            return drivePosition / m_driveMotorSensorPositionCoefficient;
        }

        /**
         * Convert a Falcon shaft velocity (in Falcon units) to a robot velocity.
         * @param shaftVelocity shaft velocity (ticks/100ms)
         * @return double, robot velocity
         */
        public double driveVelocityFromFalconVelocity(double shaftVelocity) {
            return shaftVelocity * m_driveMotorSensorVelocityCoefficient;
        }

        /**
         * Convert a robot velocity to a Falcon velocity, adjusting for Falcon's
         * weird velocity units and the gear ratio.
         * @param driveVelocity drive velocity in m/s
         * @return double, Falcon velocity in Falcon velocity units (ticks/100ms)
         */
        public double falconVelocityFromDriveVelocity(double driveVelocity) {
            return driveVelocity / m_driveMotorSensorVelocityCoefficient;
        }

    }

    private static final int CAN_TIMEOUT_MS = 250;
    private static final int STATUS_FRAME_GENERAL_PERIOD_MS = 250;

    private WPI_TalonFX m_driveMotor;
    private WPI_TalonFX m_steerMotor;
    private WPI_CANCoder m_steerEncoder;

    private final FalconMath m_falconMath;

    private SwerveModuleConfiguration m_configuration;
    private SwerveMotorConfiguration m_driveMotorOptions, m_steerMotorOptions;

    private double m_steerOffset;

    public SwerveModuleIOFalcon(SwerveModuleConfiguration configuration, SwerveMotorConfiguration driveMotorOptions, SwerveMotorConfiguration steerMotorOptions, CANDeviceID driveMotorID, CANDeviceID steerMotorID, CANDeviceID steerEncodeID, double steerOffset) {
        m_falconMath = new FalconMath(configuration);

        m_driveMotor = new WPI_TalonFX(driveMotorID.id, driveMotorID.bus);
        m_steerMotor = new WPI_TalonFX(steerMotorID.id, steerMotorID.bus);
        m_steerEncoder = new WPI_CANCoder(steerEncodeID.id, steerEncodeID.bus);
        m_steerOffset = steerOffset;
        m_configuration = configuration;

        m_driveMotorOptions = driveMotorOptions;
        m_steerMotorOptions = steerMotorOptions;

    }

    public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.encoderAngle  = Math.toRadians(m_steerEncoder.getAbsolutePosition());

        inputs.driveVelocity = m_falconMath.driveVelocityFromFalconVelocity(m_driveMotor.getSelectedSensorVelocity());
        inputs.drivePosition = m_falconMath.drivePositionFromFalconPosition(m_driveMotor.getSelectedSensorPosition());

        inputs.steerPosition = m_falconMath.absoluteAngleFromFalconAngle(m_steerMotor.getSelectedSensorPosition());
    }

    public void configure() {
        configureCANCoder();
        configureDriveMotor();
        configureSteerMotor();
    }

    public void setDrivePercentOutput(double percentOutput) {
        m_driveMotor.set(ControlMode.PercentOutput, percentOutput);
    }

    public void setDriveVelocity(double velocity) {
        double falconVelocity = m_falconMath.falconVelocityFromDriveVelocity(velocity);
        m_driveMotor.set(ControlMode.Velocity, falconVelocity);

    }

    public void setDriveVelocity(double velocity, double feedForward) {
        double falconVelocity = m_falconMath.falconVelocityFromDriveVelocity(velocity);
        m_driveMotor.set(ControlMode.Velocity, falconVelocity, DemandType.ArbitraryFeedForward, feedForward);
    }

    public void setAngle(double angle) {
        m_steerMotor.set(ControlMode.Position, m_falconMath.falconAngleFromAbsoluteAngle(angle));
    }

    public void zeroDrivePositionSensor() {
        CtreUtils.checkCtreError(m_driveMotor.setSelectedSensorPosition(0, 0, CAN_TIMEOUT_MS),
        "Reset drive motor sensor position");
    }

    /**
     * Synchronize the steer motor's internal position with the absolute encoder by setting the 
     * motor's internal sensor to the angle read by the absolute encoder. This should only be called
     * when the motor is relatively idle.
     */
    public void resetSteerPositionSensor(double absoluteAngle) {
        double steerMotorAngle = m_falconMath.falconAngleFromAbsoluteAngle(absoluteAngle);

        // Using a timeout forces the call to wait (up to the timeout) for this to happen.
        CtreUtils.checkCtreError(m_steerMotor.setSelectedSensorPosition(steerMotorAngle, 0, CAN_TIMEOUT_MS),
        "Reset steer motor sensor position");
        m_steerMotor.set(ControlMode.Position, steerMotorAngle);

        System.out.println("Steer position sensor is now: " + m_steerMotor.getSelectedSensorPosition());
    }

    /**
     * Reconfigure the PID constants for closed loop control. This is useful for PID tuning.
     * @param kP double, proportional constant
     * @param kI double, integral constant
     * @param kD double, derivative constant
     * @param kF double, feed forward constant
     */
    public void configureDriveMotorPID(double kP, double kI, double kD, double kF) {
        m_driveMotor.config_kP(0, kP);
        m_driveMotor.config_kI(0, kI);
        m_driveMotor.config_kD(0, kD);
        m_driveMotor.config_kF(0, kF);
    }




    /** Internal methods **/

    protected void configureCANCoder() {
        CANCoderConfiguration config = new CANCoderConfiguration();
        config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        config.magnetOffsetDegrees = Math.toDegrees(m_steerOffset);
        config.sensorDirection = false;

        m_steerEncoder.configAllSettings(config, 250);
        m_steerEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10, 250);
    }

    protected void configureDriveMotor() {
        TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

        if (m_driveMotorOptions.hasPidConstants()) {
            motorConfiguration.slot0.kP = m_driveMotorOptions.proportionalConstant;
            motorConfiguration.slot0.kI = m_driveMotorOptions.integralConstant;
            motorConfiguration.slot0.kD = m_driveMotorOptions.derivativeConstant;
            motorConfiguration.slot0.kF = m_driveMotorOptions.feedForwardConstant;
        }

        if (m_driveMotorOptions.hasVoltageCompensation()) {
            motorConfiguration.voltageCompSaturation = m_driveMotorOptions.nominalVoltage;
        }

        if (m_driveMotorOptions.hasCurrentLimit()) {
            motorConfiguration.supplyCurrLimit.currentLimit = m_driveMotorOptions.continuousCurrentLimit;
            if (Double.isFinite(m_driveMotorOptions.peakCurrentLimit)) {
                motorConfiguration.supplyCurrLimit.triggerThresholdCurrent =  m_driveMotorOptions.peakCurrentLimit;
                motorConfiguration.supplyCurrLimit.triggerThresholdTime = m_driveMotorOptions.peakCurrentDuration;
            }
            motorConfiguration.supplyCurrLimit.enable = true;
        }

        m_driveMotor.configFactoryDefault();
        m_driveMotor.configAllSettings(motorConfiguration);

        if (m_driveMotorOptions.hasVoltageCompensation()) {
            // Enable voltage compensation
            m_driveMotor.enableVoltageCompensation(true);
        }

        CtreUtils.checkCtreError(m_driveMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, CAN_TIMEOUT_MS),
        "Set selected sensor");

        m_driveMotor.setNeutralMode(NeutralMode.Brake);

        m_driveMotor.setInverted(m_configuration.isDriveInverted() ? TalonFXInvertType.Clockwise : TalonFXInvertType.CounterClockwise);
        m_driveMotor.setSensorPhase(true);

        // Reduce CAN status frame rates
        m_driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, STATUS_FRAME_GENERAL_PERIOD_MS, CAN_TIMEOUT_MS);
    }

    protected void configureSteerMotor() {
        TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
        if (m_steerMotorOptions.hasPidConstants()) {
            motorConfiguration.slot0.kP = m_steerMotorOptions.proportionalConstant;
            motorConfiguration.slot0.kI = m_steerMotorOptions.integralConstant;
            motorConfiguration.slot0.kD = m_steerMotorOptions.derivativeConstant;
            motorConfiguration.slot0.kF = m_steerMotorOptions.feedForwardConstant;
        }

        if (m_steerMotorOptions.hasVoltageCompensation()) {
            motorConfiguration.voltageCompSaturation = m_steerMotorOptions.nominalVoltage;
        }
        if (m_steerMotorOptions.hasCurrentLimit()) {
            motorConfiguration.supplyCurrLimit.currentLimit = m_steerMotorOptions.continuousCurrentLimit;
            if (Double.isFinite(m_steerMotorOptions.peakCurrentLimit)) {
                motorConfiguration.supplyCurrLimit.triggerThresholdCurrent =  m_steerMotorOptions.peakCurrentLimit;
                motorConfiguration.supplyCurrLimit.triggerThresholdTime = m_steerMotorOptions.peakCurrentDuration;
            }
            motorConfiguration.supplyCurrLimit.enable = true;
        }

        m_steerMotor.configFactoryDefault();
        m_steerMotor.configAllSettings(motorConfiguration, CAN_TIMEOUT_MS);

        if (m_steerMotorOptions.hasVoltageCompensation()) {
            m_steerMotor.enableVoltageCompensation(true);
        }
        CtreUtils.checkCtreError(m_steerMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, CAN_TIMEOUT_MS),
        "Set selected sensor");
        m_steerMotor.setSensorPhase(true);
        m_steerMotor.setInverted(m_configuration.isSteerInverted() ? TalonFXInvertType.CounterClockwise : TalonFXInvertType.Clockwise);
        m_steerMotor.setNeutralMode(NeutralMode.Brake);

        // Reduce CAN status frame rates
        m_steerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, STATUS_FRAME_GENERAL_PERIOD_MS, CAN_TIMEOUT_MS);
    }

}
