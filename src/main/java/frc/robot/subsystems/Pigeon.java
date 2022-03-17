package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Pigeon {

    private static Pigeon2 pigeon;


    private static double m_lastError = 0;
    private static double m_ISum = 0;
    private static double m_Kp = 0;
    private static double m_Ki = 0;
    private static double m_Kd = 0;
    private static double m_max = 0;
    private static double m_targetAngle;
    private static double m_deadband = 0;

    private static NetworkTable m_nt;

    final static boolean m_tuningmode = true;


    public static void init() {
        pigeon = new Pigeon2(0);
        pigeon.configFactoryDefault();

        if(m_tuningmode)
        {
            m_nt = NetworkTableInstance.getDefault().getTable("PID");
            m_nt.getEntry("P").setDouble(0.0);
            m_nt.getEntry("I").setDouble(0.0);
            m_nt.getEntry("D").setDouble(0.0);
        }
    }

    private static void updatePIDNT()
    {
        m_Kp = m_nt.getEntry("P").getDouble(m_Kp);
        m_Ki = m_nt.getEntry("I").getDouble(m_Ki); 
        m_Kd = m_nt.getEntry("D").getDouble(m_Kd);
    }

    public static void zero(){
        pigeon.setYaw(0, 10);
    }

    public static double getRotation() {
        return pigeon.getYaw();
    }

    public static double getRoll() {
        return pigeon.getRoll();
    }

    public static double getPitch() {
        return pigeon.getPitch();
    }

    public static void initPID(double p, double i, double d, double max, double deadband) {
        m_lastError = 0;
        m_ISum = 0;
        m_Kp = p;
        m_Ki = i;
        m_Kd = d;

        Pigeon.m_max = max;
        Pigeon.m_deadband = deadband;

        if(m_tuningmode) {
            m_nt.getEntry("P").setDouble(m_Kp);
            m_nt.getEntry("I").setDouble(m_Ki);
            m_nt.getEntry("D").setDouble(m_Kd);
        }

    }

    public static void setTargetAngle(double target) {
        Pigeon.m_targetAngle = target;
    }

    private static double calculateDestinationPID(double pigAng)
    {
        // The number of full rotations the bot has made
        int numRot = (int) Math.floor(pigAng / 360);

        // The target pigeon angle
        double target = numRot * 360 + m_targetAngle;
        double targetPlus = target + 360;
        double targetMinus = target - 360;

        // The true destination for the bot to rotate to
        double destination;

        // Determine if, based on the current angle, it should stay in the same
        // rotation, enter the next, or return to the previous.
        if (Math.abs(target - pigAng) < Math.abs(targetPlus - pigAng)
                && Math.abs(target - pigAng) < Math.abs(targetMinus - pigAng)) {
            destination = target;
        } else if (Math.abs(targetPlus - pigAng) < Math.abs(targetMinus - pigAng)) {
            destination = targetPlus;
        } else {
            destination = targetMinus;
        }

        return destination;
    }


    public static double correctTurnWithPID(double dt) {
        
        if(m_tuningmode)
            updatePIDNT();

        double currentAngle = getRotation();
        double error = calculateDestinationPID(currentAngle) - currentAngle;
        
        m_ISum += dt*(error + m_lastError)/2;
        double derivative = (error - m_lastError) / dt;
        
        m_lastError = error;

        double Pout = m_Kp * error / 180.0;
        double Iout = m_Ki * m_ISum;
        double Dout = m_Kd * derivative / 180.0;
        double correctionPower =  Pout + Iout + Dout;
        
        if (Math.abs(error)>m_deadband) {
            if (correctionPower>m_max) {
                correctionPower = m_max;
            } else if (correctionPower<-m_max) {
                correctionPower = -m_max;
            }
            System.out.println(correctionPower + " : " + currentAngle);
        } else {
            correctionPower = 0;
        }

        return correctionPower;
    }

}
