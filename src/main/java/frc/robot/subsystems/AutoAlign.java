 package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class AutoAlign {

    private static NetworkTable m_networkTable;

    private static NetworkTableEntry nt_hub_seen;
    private static NetworkTableEntry nt_hub_ang;
    private static NetworkTableEntry nt_hub_dist;
    private static NetworkTableEntry nt_hub_dist_avg;

    private static boolean m_hubSeen = false;
    public static double m_distAvg = 0;
    private static double m_targetAngle = 0;

    public static void init(){
        m_networkTable = NetworkTableInstance.getDefault().getTable("vis");
        nt_hub_seen = m_networkTable.getEntry("hub_seen");
        nt_hub_ang  = m_networkTable.getEntry("hub_ang");
        nt_hub_dist = m_networkTable.getEntry("hub_dist");
        nt_hub_dist_avg = m_networkTable.getEntry("hub_dist_avg");
    
    }

    public static boolean hasTarget(){
        return nt_hub_seen.getBoolean(false);
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
            m_hubSeen = false;
            return;    
        }

        m_targetAngle = getAngle();
        double dist   = getDistance();
        
        
        m_distAvg = m_distAvg * 0.8 + dist * 0.2;
        nt_hub_dist_avg.setDouble(m_distAvg);
        
        m_hubSeen = true;
        nt_hub_seen.setBoolean(false);
    }

    public static void setAngle() {

        if(m_hubSeen)
        {
            double absang = Pigeon.getRotation() - m_targetAngle;
            Pigeon.setTargetAngle(absang);
        }
    }
}
