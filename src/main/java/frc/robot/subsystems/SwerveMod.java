package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class SwerveMod {

    private static final double ticksPerRot = 2048*12.8;
    
    private TalonFX m_steer; 
    private TalonFX m_drive;

    public double m_xPos;
    public double m_yPos;
    

    private boolean inverted;

    public SwerveMod(int steerId, int driveId, double x,double y){
        m_steer = new TalonFX(steerId);
        m_drive = new TalonFX(driveId);
        // configure encoders/PID
        m_steer.configFactoryDefault();
        m_drive.configFactoryDefault();

        m_steer.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 20);
        //m_steer.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.QuadEncoder, 0, 20);

        m_steer.config_kF(0, 0, 10);
		m_steer.config_kP(0, 0.3, 10);
		m_steer.config_kI(0, 0, 10);
		m_steer.config_kD(0, 0, 10);
    }
    
    public void drive(double power){
        m_drive.set(TalonFXControlMode.PercentOutput, power * (inverted?-1.0:1.0));
    }

    //rotates to angle given in radians
    public void rotateToRad(double angle){
        rotate(angle / (2 * Math.PI) * ticksPerRot);
    }

    //rotates to a position given in ticks
    public void rotate(double angle){
        double motorPos = m_steer.getSelectedSensorPosition();
    
        int numRot = (int)Math.floor(motorPos/ticksPerRot);
        double joystickTarget = numRot*ticksPerRot+angle;
        double joystickTargetPlus = joystickTarget + ticksPerRot;
        double joystickTargetMinus = joystickTarget - ticksPerRot;
        
        double destination;
    
        if(Math.abs(joystickTarget-motorPos)<Math.abs(joystickTargetPlus-motorPos) && Math.abs(joystickTarget-motorPos)<Math.abs(joystickTargetMinus-motorPos)){
            destination = joystickTarget;
        } else if(Math.abs(joystickTargetPlus-motorPos)<Math.abs(joystickTargetMinus-motorPos)){
            destination = joystickTargetPlus;
        } else {
            destination = joystickTargetMinus;
        }

        if(Math.abs(destination-motorPos) > ticksPerRot/4.0){
            inverted = true;
            if(destination>motorPos)
                destination -= ticksPerRot/2.0;
            else
                destination += ticksPerRot/2.0;
        } else {
            inverted = false;
        }

        m_steer.set(TalonFXControlMode.Position, destination);
    }

}
