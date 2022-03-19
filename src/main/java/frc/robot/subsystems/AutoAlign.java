 package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class AutoAlign {

    private static NetworkTable m_networkTable;

    private static NetworkTableEntry nt_hub_seen;
    private static NetworkTableEntry nt_hub_ang;
    private static NetworkTableEntry nt_hub_dist;


    public static void init(){
        m_networkTable = NetworkTableInstance.getDefault().getTable("vis");
        nt_hub_seen = m_networkTable.getEntry("hub_seen");
        nt_hub_ang  = m_networkTable.getEntry("hub_ang");
        nt_hub_dist = m_networkTable.getEntry("hub_dist");
    
    }

    public static boolean hasTarget(){
        return nt_hub_seen.getNumber(0).intValue()==1;
    }

    public static double getAngle(){
        return nt_hub_ang.getDouble(0.0);
    }

    public static double getDistance(){
        return nt_hub_dist.getDouble(12.5);
    }

    public static void update() {

        if(!hasTarget())
        {
            System.out.println("_N");
            return;    
        }
        System.out.println("Y_");
        double ang  = getAngle();
        double dist = getDistance();
        nt_hub_seen.setDouble(0);

        double target_ang = Pigeon.getRotation() - ang;
        Pigeon.setTargetAngle(target_ang);

    }
}
