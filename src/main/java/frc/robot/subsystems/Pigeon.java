package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

public class Pigeon {

    private static Pigeon2 pigeon;


    private static double lastError = 0;
    private static double ISum = 0;
    private static double Kp = 0;
    private static double Ki = 0;
    private static double Kd = 0;
    private static double max = 0;
    private static double targetAngle;
    private static double deadband = 0;


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
        lastError = 0;
        ISum = 0;
        Kp = p;
        Ki = i;
        Kd = d;

        Pigeon.max = max;
        Pigeon.deadband = deadband;
    }

    public static void setTargetAngle(double target) {
        Pigeon.targetAngle = target;
    }

    private static double calculateDestinationPID(double pigAng)
    {
        // The number of full rotations the bot has made
        int numRot = (int) Math.floor(pigAng / 360);

        // The target pigeon angle
        double target = numRot * 360 + targetAngle;
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
        
        ISum += dt*(error + lastError)/2;
        double derivative = (error - lastError) / dt;
        
        lastError = error;

        double Pout = Kp * error;
        double Iout = Ki * ISum;
        double Dout = Kd * derivative;
        double correctionPower =  Pout + Iout + Dout;
        
        if (Math.abs(error)>deadband) {
            if (correctionPower>max) {
                correctionPower = max;
            } else if (correctionPower<-max) {
                correctionPower = -max;
            }
            System.out.println(correctionPower + " : " + currentAngle);
        } else {
            correctionPower = 0;
        }

        return correctionPower;
    }

}
