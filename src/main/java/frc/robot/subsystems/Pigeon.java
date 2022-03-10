package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

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


    public static void init() {
        pigeon = new Pigeon2(0);
        pigeon.configFactoryDefault();
        //NEEDS TUNING!!
        initPID(0.08, 0, 0.02, 0.18, 1.0);
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
        
        double currentAngle = getRotation();
        double error = calculateDestinationPID(currentAngle) - currentAngle;
        
        m_ISum += dt*(error + m_lastError)/2;
        double derivative = (error - m_lastError) / dt;
        
        m_lastError = error;

        double Pout = m_Kp * error;
        double Iout = m_Ki * m_ISum;
        double Dout = m_Kd * derivative;
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
