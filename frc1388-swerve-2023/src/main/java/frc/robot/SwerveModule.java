package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorTimeBase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {

    private final WPI_TalonFX m_driveMotor;
    private final double WHEEL_DIAMETER_INCHES = 4.0;
    private final double METERS_PER_WHEEL_ROTATION = Math.PI * WHEEL_DIAMETER_INCHES / Constants.inchesPerMeter;
    private final double WHEEL_ROTATIONS_PER_METER = 1 / METERS_PER_WHEEL_ROTATION;
    private final double MOTOR_ROTATIONS_PER_WHEEL_ROTATION = 6.75;
    private final double SENSOR_UNITS_PER_MOTOR_ROTATION = 2048;
    private final double SECONDS_PER_100MS = 0.1;

    /**
     * (meters / second) times this constant results in (sensor units / 100ms)
     * <p>
     * (i think the math works)
     */
    private final double SENSOR_CYCLE_SECONDS_PER_100MS_METERS = WHEEL_ROTATIONS_PER_METER * MOTOR_ROTATIONS_PER_WHEEL_ROTATION * SENSOR_UNITS_PER_MOTOR_ROTATION * SECONDS_PER_100MS;

    private final double DISTANCE_PER_ENCODER_TICK = (1 / SENSOR_UNITS_PER_MOTOR_ROTATION) * (1 / MOTOR_ROTATIONS_PER_WHEEL_ROTATION) * METERS_PER_WHEEL_ROTATION;
    private final WPI_TalonFX m_rotationMotor;

    private final CANCoder m_canCoder;
    private final CANCoderConfiguration m_canCoderConfig = new CANCoderConfiguration();

    private final PIDController m_rotationPID;

    private final String m_name;

    private final double m_encoderOffset;

    public SwerveModule(WPI_TalonFX driveMotor, WPI_TalonFX rotationMotor, CANCoder canCoder, double encoderOffset, String name) {
        m_driveMotor = driveMotor;
        m_driveMotor.configFactoryDefault();
        m_driveMotor.setNeutralMode(NeutralMode.Brake);
        m_driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        m_driveMotor.config_kF(0, 0); 
        m_driveMotor.config_kP(0, 0.001);// form 2022 swerve
        m_driveMotor.config_kI(0, 0);
        m_driveMotor.config_kD(0, 0);

        m_rotationMotor = rotationMotor;
        m_rotationMotor.configFactoryDefault();
        m_rotationMotor.setNeutralMode(NeutralMode.Brake);
        m_rotationMotor.setInverted(true);
        // m_rotationMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);

        m_rotationPID = new PIDController(
           0.007,
            0,
            0
        );
        m_rotationPID.setTolerance(5);
        m_rotationPID.enableContinuousInput(0, 360);

        // m_rotationMotor.setControlFramePeriod(ControlFrame.Control_3_General, 20); XXX look into this

        m_canCoder = canCoder;
        // m_canCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
        // m_canCoderConfig.magnetOffsetDegrees(encoderOffset);
        m_canCoder.configAllSettings(m_canCoderConfig);
        m_encoderOffset = encoderOffset;
        
        m_name = name;
    }

    public void setSwerveModuleStates(SwerveModuleState inputState) {
        Rotation2d rotation = new Rotation2d(Math.toRadians(getRotationAngle()));
        SwerveModuleState swerveModuleState = SwerveModuleState.optimize(inputState, rotation);
        // setDriveSpeed(swerveModuleState.speedMetersPerSecond);
        // setRotationPosition(swerveModuleState.angle.getDegrees());

        setDriveSpeed(swerveModuleState.speedMetersPerSecond);
        setRotationPosition(swerveModuleState.angle.getDegrees());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            m_driveMotor.getSelectedSensorPosition() * DISTANCE_PER_ENCODER_TICK,
            // new Rotation2d()
            new Rotation2d(Math.toRadians(getRotationAngle()))
        );
    }

    /**
     * speed of the motor
     * @param inputSpeed is in meters / second
     */
    public void setDriveSpeed(double inputSpeed) {
        // m_driveMotor.set(ControlMode.Velocity, inputSpeed * SENSOR_CYCLE_SECONDS_PER_100MS_METERS);        
        SmartDashboard.putNumber(m_name + " set speed ", inputSpeed);
        m_driveMotor.set(inputSpeed / 3);
    }

    /**
     * set angle of the module
     * @param angle is in degrees
     */
    public void setRotationPosition(double angle) {
        // SmartDashboard.putNumber(m_name + "encoder value", getRotationAngle());
        SmartDashboard.putNumber(m_name + " set angle ", angle);
        m_rotationMotor.set(m_rotationPID.calculate(getRotationAngle(), angle));
        // m_rotationMotor.set(m_rotationPID.calculate(getRotationAngle(), 0 + 90));
    }

    public double getRotationAngle() {
        return (m_canCoder.getAbsolutePosition
        () - m_encoderOffset + 36000) % 360;
    }

    public void periodic() {
        // System.out.println("encoder angle: " + getRotationAngle() + "\t    motor sensor pos: " + m_rotationMotor.getSelectedSensorPosition());
        // System.out.println("swerve moduleing");
        // m_driveMotor.set(0.2);
        // SmartDashboard.putNumber(m_name + "raw encoder value", m_canCoder.getPosition());
        SmartDashboard.putNumber(m_name + " modified encoder value ", getRotationAngle());
        SmartDashboard.putNumber(m_name + " distance in meters wheel has traveled", m_driveMotor.getSelectedSensorPosition() * DISTANCE_PER_ENCODER_TICK);
        // SmartDashboard.putNumber(m_name + " raw encoder value ", m_canCoder.getPosition());

    }

}
