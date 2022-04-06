package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.robotmath.RMath;
import frc.robot.robotmath.RTime;

public class Pigeon {

    private static Pigeon2 m_pigeon;

    // Proportional constant
    private static double m_Kp = 0;
    // Integral constant
    private static double m_Ki = 0;
    // Derivative constant
    private static double m_Kd = 0;

    private static double m_max = 0;
    private static double m_scale = 0;
    private static double m_targetAngle = 0;
    private static double m_deadband = 0;

    private static double m_ISum = 0;
    private static double m_lastError = 0;
    
    
    // Is the pigeon actively correcting our angle?
    private static boolean m_active = false;
    
    // Setting this to true will make the Pigeon's 
    // PID values visable and editable from the network
    final static boolean m_tuningmode = false;
    private static NetworkTable m_nt;


    public static void init() {
        m_pigeon = new Pigeon2(0);
        m_pigeon.configFactoryDefault();
        
        // Start deactivated so we don't spin on boot
        stop();


        if(m_tuningmode) {
            m_nt = NetworkTableInstance.getDefault().getTable("PID");
            m_nt.getEntry("P").setDouble(0.0);
            m_nt.getEntry("I").setDouble(0.0);
            m_nt.getEntry("D").setDouble(0.0);
        }
    }


    // Disable angle correction and zero our data
    public static void stop() {
        m_active = false;
        m_lastError = 0;
        m_ISum = 0;
        m_targetAngle = 0;
    }


    private static void updatePIDNT() {
        m_Kp = m_nt.getEntry("P").getDouble(m_Kp * 180.0) / 180.0;
        m_Ki = m_nt.getEntry("I").getDouble(m_Ki * 180.0) / 180.0; 
        m_Kd = m_nt.getEntry("D").getDouble(m_Kd * 180.0) / 180.0;
    }

    public static void zero(){
        m_pigeon.setYaw(0, 10);
    }

    public static void setYaw(double deg) {
        m_pigeon.setYaw(deg, 10);
    }

    // Local to the robot, not the world
    // Pitch, rotates around the X, left to right, axis
    // Tilts forward and backward
    public static double getPitch() {
        return m_pigeon.getPitch();
    }

    // Local to the robot, not the world
    // Yaw, rotates around the Y, up and down, axis
    public static double getRotation() {
        return m_pigeon.getYaw();
    }

    // Local to the robot, not the world
    // Roll, rotates around the Z, forward and backward, axis
    // Tilts left and right
    public static double getRoll() {
        return m_pigeon.getRoll();
    }


    public static void initPID(double p, double i, double d, double max, double scale, double deadband) {
        m_lastError = 0;
        m_ISum = 0;

        // Scaling down by 180 lets us have larger, more sane, PID values
        // that are relative to the rotation count, rather than degrees
        m_Kp = p / 180.0;
        m_Ki = i / 180.0;
        m_Kd = d / 180.0;

        Pigeon.m_max = max;
        Pigeon.m_scale = scale;
        Pigeon.m_deadband = deadband;

        // If we're in tuning mode, make sure the network gets our new values
        if(m_tuningmode) {
            m_nt.getEntry("P").setDouble(p);
            m_nt.getEntry("I").setDouble(i);
            m_nt.getEntry("D").setDouble(d);
        }

    }

    public static void setTargetAngle(double target) {
        Pigeon.m_active = true;
        Pigeon.m_targetAngle = target;

        if(Math.abs(Pigeon.m_targetAngle - target) > 6.0) {
            // Only reset these if our new target is significant
            //m_lastError = 0;
            //m_ISum = 0;
        }
    }


    public static double getError() {
        double currentAngle = getRotation();
        double error = calculateDestinationPID(currentAngle) - currentAngle;
        return error;
    }

    public static boolean atSetpoint() {
        return Math.abs(getError()) <= m_deadband;
    }

    private static double calculateDestinationPID(double pigAng) {
        // This will return the closest angle to our current position, 
        // that is at the same position of m_targetAngle

        // Example:
        //   If we have a target like 723, and our current angle is 350,
        //   this will respond with the closest angle to 350 
        //   with the same position as 723, which would be 363 
        
        return RMath.targetAngleAbsolute(pigAng, m_targetAngle, 360);
    }


    public static double correctTurnWithPID() {
        
        // If we're not actively correcting, don't turn at all
        if(!m_active)
            return 0;

        // This assumes that we're calling correctTurnWithPID every frame
        double dt = RTime.deltaTime();
        

        // If we're in tuning mode, check the network for new values
        if(m_tuningmode)
            updatePIDNT();


        double error = getError();
        
        // Trapezoidal sum to approximate the integral of the error  
        m_ISum += dt*(error + m_lastError)/2;

        // Approximation for the derivative of the error
        double derivative = (error - m_lastError) / dt;
        
        // Remember our current error for next time's calculations
        m_lastError = error;

        double Pout = m_Kp * error;
        double Iout = m_Ki * m_ISum;
        double Dout = m_Kd * derivative;
        double correctionPower =  Pout + Iout + Dout;
        
        if (Math.abs(error)>m_deadband) { // || Math.abs(correctionPower) < m_scale * 0.01) {
            
            // Correction power cannot be greater than or less than +m_max & -m_max
            if (correctionPower>m_max) {
                correctionPower = m_max;
            } else if (correctionPower<-m_max) {
                correctionPower = -m_max;
            }
            
            //System.out.printf("%.02f - %.02f : %.02f\n", error, correctionPower, currentAngle);
            //System.out.println(error + " " + correctionPower + " : " + currentAngle);
        } else {
            correctionPower = 0;
        }


        // Scale down correction power so we don't spin the robot like crazy
        correctionPower *= m_scale;

        
        // If we've come to a rest and our error is close to 0, we're done. 
        // Zero our sums
        if(Math.abs(derivative) < m_scale * 0.01) {
           // correctionPower = 0;
           // m_ISum = 0;
           // m_lastError = 0;
        }


        return correctionPower;
    }

}
