package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import frc.robot.robotmath.Vector2D;

public class SwerveMod {

    private static final double ticksPerRotationSteer = 2048 * 12.8;
    private static final double ticksPerRotationDrive = 2048 * 8.14;


    public TalonFX m_steer;
    public TalonFX m_drive;
    public CANCoder m_absEncoder;

    public Vector2D m_pos;

    private boolean inverted;

    private double m_cancoderOffset;
    private double m_falconOffset;

    private int m_steerID;

    public SwerveMod(int steerID, int driveID, double x, double y, double cancoderOffset, double falconOffset) {
        m_steerID = steerID;
        m_steer = new TalonFX(steerID);
        m_drive = new TalonFX(driveID);
        m_absEncoder = new CANCoder(steerID);

        m_pos = new Vector2D(x, y);

        // Configure encoders/PID
        m_steer.configFactoryDefault();
        m_steer.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
        m_steer.configNeutralDeadband(0.001, 30);
		m_steer.config_kF(0, 0, 30);
		m_steer.config_kP(0, 1.1, 30);
		m_steer.config_kI(0, 0.0, 30);
		m_steer.config_kD(0, 0.0, 30);
        m_steer.configMotionCruiseVelocity(40000, 30);
		m_steer.configMotionAcceleration(40000, 30);

        m_drive.configFactoryDefault();
        m_drive.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
        m_drive.setSelectedSensorPosition(0);
        m_drive.configNeutralDeadband(0.001, 30);
		m_drive.config_kF(0, 0, 30);
		m_drive.config_kP(0, 0.5, 30);
		m_drive.config_kI(0, 0.01, 30);
		m_drive.config_kD(0, 0.1, 30);
        m_drive.configMotionCruiseVelocity(100, 30);
		m_drive.configMotionAcceleration(400, 30);
        
        
        m_drive.setInverted(true);
        m_steer.setInverted(false);
        m_drive.setNeutralMode(NeutralMode.Brake);
        m_steer.setNeutralMode(NeutralMode.Brake);

        m_absEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        m_absEncoder.configMagnetOffset(0);
        m_absEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);


        // Power Management
        // Cap off our current at 39 amps. If we go above 40 amps, the breaker will flip
        SupplyCurrentLimitConfiguration currentLimit = new SupplyCurrentLimitConfiguration(true, 39, 39, 0 );
        m_drive.configSupplyCurrentLimit(currentLimit);
        // Enable voltage compensation to prevent variable behavior when the battery gets low/poor 
        m_drive.configVoltageCompSaturation(12.5);
        m_drive.enableVoltageCompensation(true);

//        m_steer.configVoltageCompSaturation(12.0);
//        m_steer.enableVoltageCompensation(true);

        m_cancoderOffset = cancoderOffset;
        m_falconOffset = falconOffset;

        inverted = false;

        resetSteerSensor();
    }

    public void resetSteerSensor()
    {
        // Align the falcon to the cancoder
        double pos = m_absEncoder.getAbsolutePosition() - m_cancoderOffset;
        pos = pos / 360.0 * ticksPerRotationSteer;
        m_steer.setSelectedSensorPosition( pos );
        m_steer.set(TalonFXControlMode.MotionMagic, pos);

    }

    public void drive(double power) {
        m_drive.set(TalonFXControlMode.PercentOutput, power * (inverted ? -1.0 : 1.0));
    }

    // Rotates to angle given in radians
    public void rotateToRad(double angle) {
        rotate((angle - Math.PI * 0.5) / (2 * Math.PI) * ticksPerRotationSteer);
    }

    // Rotates to a position given in ticks
    public void rotate(double toAngle) {
        double motorPos = m_steer.getSelectedSensorPosition();

        // The number of full rotations the motor has made
        int numRot = (int) Math.floor(motorPos / ticksPerRotationSteer);

        // The target motor position dictated by the joystick, in motor ticks
        double joystickTarget = numRot * ticksPerRotationSteer + toAngle;
        double joystickTargetPlus = joystickTarget + ticksPerRotationSteer;
        double joystickTargetMinus = joystickTarget - ticksPerRotationSteer;

        // The true destination for the motor to rotate to
        double destination;

        // Determine if, based on the current motor position, it should stay in the same
        // rotation, enter the next, or return to the previous.
        if (Math.abs(joystickTarget - motorPos) < Math.abs(joystickTargetPlus - motorPos)
                && Math.abs(joystickTarget - motorPos) < Math.abs(joystickTargetMinus - motorPos)) {
            destination = joystickTarget;
        } else if (Math.abs(joystickTargetPlus - motorPos) < Math.abs(joystickTargetMinus - motorPos)) {
            destination = joystickTargetPlus;
        } else {
            destination = joystickTargetMinus;
        }

        // If the target position is farther than a quarter rotation away from the
        // current position, invert its direction instead of rotating it the full
        // distance
        if (Math.abs(destination - motorPos) > ticksPerRotationSteer / 4.0) {
            inverted = true;
            if (destination > motorPos)
                destination -= ticksPerRotationSteer / 2.0;
            else
                destination += ticksPerRotationSteer / 2.0;
        } else {
            inverted = false;
        }

        m_steer.set(TalonFXControlMode.MotionMagic, destination);

        
    }

    // Returns an angle in radians
    public double getSteerAngle() {
        return m_steer.getSelectedSensorPosition() / ticksPerRotationSteer * Math.PI * 2;
    }


    public double getDrivePosition() {
        return m_drive.getSelectedSensorPosition() / ticksPerRotationSteer * (3*Math.PI);
    }

    public double getDriveVelocity() {
        return m_drive.getSelectedSensorVelocity() * 10 / ticksPerRotationDrive * (4*Math.PI);
    }

    


}
