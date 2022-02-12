package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

public class Pigeon {

    private static Pigeon2 pigeon;

    public static void init() {
        pigeon = new Pigeon2(0);
        pigeon.configFactoryDefault();
    }

    public static void zero() {
        //pigeon.zeroGyroBiasNow();
        pigeon.setYaw(0);
    }

    public static double getRotation() {
        return pigeon.getYaw();
    }

}
