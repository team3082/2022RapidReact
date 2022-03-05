 package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class AutoAlign {

    private static NetworkTable m_networkTable;

    public static void init(){
        m_networkTable = NetworkTableInstance.getDefault().getTable("vis");
    }

    public static boolean hasTarget(){
        return m_networkTable.getEntry("target").getBoolean(false);
    }

    public static double getAngle(){
        return m_networkTable.getEntry("angle").getDouble(0.0);
    }

    public static double getDistance(){
        return m_networkTable.getEntry("distance").getDouble(12.5);
    }

}
