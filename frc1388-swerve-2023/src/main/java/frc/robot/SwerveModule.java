package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorTimeBase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {

    private final WPI_TalonFX m_driveMotor;

    private final WPI_TalonFX m_rotationMotor;

    private final CANCoder m_canCoder;

    private final CANCoderConfiguration m_canCoderConfig = new CANCoderConfiguration();

    public SwerveModule(WPI_TalonFX driveMotor, WPI_TalonFX rotationMotor, CANCoder canCoder) {
        m_driveMotor = driveMotor;
        m_driveMotor.configFactoryDefault();
        m_driveMotor.setNeutralMode(NeutralMode.Brake);
        m_driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        m_driveMotor.config_kF(0, 0.055); // from frc1388-2023
        m_driveMotor.config_kP(0, 0.03);
        m_driveMotor.config_kI(0, 0.0001);
        m_driveMotor.config_kD(0, 0);

        m_rotationMotor = rotationMotor;
        m_rotationMotor.configFactoryDefault();
        m_rotationMotor.setNeutralMode(NeutralMode.Brake);
        m_rotationMotor.configSelectedFeedbackSensor(RemoteFeedbackDevice.valueOf(0));
        m_rotationMotor.config_kF(0, 0);
        m_rotationMotor.config_kP(0, 0);
        m_rotationMotor.config_kI(0, 0);
        m_rotationMotor.config_kD(0, 0);

        m_canCoder = canCoder;
        m_canCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
        m_canCoder.configAllSettings(m_canCoderConfig);
    }

    public void setSwerveModuleStates(SwerveModuleState inputState) {
        Rotation2d rotation = new Rotation2d(Math.toRadians(getRotationAngle()));
        SwerveModuleState swerveModuleState = SwerveModuleState.optimize(inputState, rotation);

        setDriveSpeed(swerveModuleState.speedMetersPerSecond);
        setRotationPosition(swerveModuleState.angle.getDegrees());
    }

    /**
     * speed of the motor
     * @param inputSpeed is in meters / second
     */
    public void setDriveSpeed(double inputSpeed) {
        // TODO: math to input speed, velocity is in sensor units / 100 ms
        m_driveMotor.set(ControlMode.Velocity, inputSpeed);
    }

    /**
     * set angle of the module
     * @param angle is in degrees
     */
    public void setRotationPosition(double angle) {
        m_rotationMotor.set(ControlMode.Position, angle);
    }

    public double getRotationAngle() {
        return m_canCoder.getPosition();
    }

}
