package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

public class Pigeon {
    
    private static Pigeon2 pigeon;

    public static void init() {
        pigeon = new Pigeon2(0);
    }

    public static void zero(){
        pigeon.zeroGyroBiasNow();
    }

    public static double getRotation() {
        return pigeon.getYaw();
    }

}
