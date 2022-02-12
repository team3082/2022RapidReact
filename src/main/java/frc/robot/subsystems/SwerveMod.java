package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class SwerveMod {

    private static final double ticksPerRot = 2048 * 12.8;

    private TalonFX m_steer;
    private TalonFX m_drive;

    public double m_xPos;
    public double m_yPos;

    private boolean inverted;

    public SwerveMod(int steerId, int driveId, double x, double y) {
        m_steer = new TalonFX(steerId);
        m_drive = new TalonFX(driveId);
        // Configure encoders/PID
        m_steer.configFactoryDefault();
        m_drive.configFactoryDefault();

        m_steer.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 20);
        // m_steer.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.QuadEncoder, 0, 20);
        m_steer.configFactoryDefault();
        m_steer.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
        m_steer.configNeutralDeadband(0.001, 30);
		m_steer.config_kF(0, 0, 30);
		m_steer.config_kP(0, 0.5, 30);
		m_steer.config_kI(0, 0.01, 30);
		m_steer.config_kD(0, 0.1, 30);
        m_steer.configMotionCruiseVelocity(20000, 30);
		m_steer.configMotionAcceleration(40000, 30);
        m_drive.configFactoryDefault();
        m_drive.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
        m_drive.configNeutralDeadband(0.001, 30);
		m_drive.config_kF(0, 0, 30);
		m_drive.config_kP(0, 0.5, 30);
		m_drive.config_kI(0, 0.01, 30);
		m_drive.config_kD(0, 0.1, 30);
        m_drive.configMotionCruiseVelocity(100, 30);
		m_drive.configMotionAcceleration(400, 30);

    }

    public void drive(double power) {
        m_drive.set(TalonFXControlMode.PercentOutput, power * (inverted ? -1.0 : 1.0));
    }

    // Rotates to angle given in radians
    public void rotateToRad(double angle) {
        rotate(angle / (2 * Math.PI) * ticksPerRot);
    }

    // Rotates to a position given in ticks
    public void rotate(double toAngle) {
        double motorPos = m_steer.getSelectedSensorPosition();

        // The number of full rotations the motor has made
        int numRot = (int) Math.floor(motorPos / ticksPerRot);

        // The target motor position dictated by the joystick, in motor ticks
        double joystickTarget = numRot * ticksPerRot + toAngle;
        double joystickTargetPlus = joystickTarget + ticksPerRot;
        double joystickTargetMinus = joystickTarget - ticksPerRot;

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
        if (Math.abs(destination - motorPos) > ticksPerRot / 4.0) {
            inverted = true;
            if (destination > motorPos)
                destination -= ticksPerRot / 2.0;
            else
                destination += ticksPerRot / 2.0;
        } else {
            inverted = false;
        }

        m_steer.set(TalonFXControlMode.MotionMagic, destination);
    }

}
