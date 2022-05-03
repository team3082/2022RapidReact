 package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.robotmath.RTime;
import frc.robot.robotmath.Vector2D;

public class AutoAlign {

    private static boolean kDisabled = false;


    private static NetworkTable m_networkTable;
    private static NetworkTableEntry nt_hub_seen;
    private static NetworkTableEntry nt_hub_ang;
    private static NetworkTableEntry nt_hub_dist;

    public static boolean m_hubSeen  = false;
    public  static double  m_distAvg  = 10.0;
    private static double  m_hubAngle = 0.0;

    private static double m_lastupdatetime = 0;

    private static final double k_innerBand = 10.0;
    private static final double k_outerBand = 27.0;

    public static void init() {

        m_lastupdatetime = 0;
        
        m_hubSeen = false;
        m_hubAngle = 0.0;
        m_distAvg = 10.0;
        

        if(disabled()) return;
        m_networkTable = NetworkTableInstance.getDefault().getTable("vis");
        nt_hub_seen = m_networkTable.getEntry("hub_seen");
        nt_hub_ang  = m_networkTable.getEntry("hub_ang");
        nt_hub_dist = m_networkTable.getEntry("hub_dist");
        
    }

    //public static boolean hasTarget(){
    //    return nt_hub_seen.getBoolean(false);
    //}

    public static double getAngle(){
        return m_hubAngle;
    }

    public static double getDistance(){
        return m_distAvg;
    }

    public static void update() {
        if(disabled()) return;


        // If we don't have new data, we can ignore this frame
        boolean hastarget = nt_hub_seen.getBoolean(false);
        if(!hastarget)
        {
            // Haven't seen the hub this frame
            m_hubSeen = false;
            return;    
        }

        // Toggle "hub_seen" back to false so we don't mistakenly recv this frame twice
        // We do this immediately incase we recv new data during this frame
        m_hubSeen = true;
        nt_hub_seen.setBoolean(false);

        

        // New data
        m_hubAngle  = nt_hub_ang.getDouble(0.0);
        double dist = nt_hub_dist.getDouble(10.5) + 2.0;

        //Check for bad data
        /*
        if(dist < k_innerBand || dist > k_outerBand){
            m_hubSeen = false;
            return;
        }
        */

        // Time since last recv from the Pi
        double curtime = RTime.now();
        double deltatime = curtime - m_lastupdatetime;
        m_lastupdatetime = curtime;
        
        // dt * 2 so half a second is our "full replacement" period
        double scale = deltatime * 2;

        // While the shooter is active, discard new data
        // It shakes the camera way too much most of the time 
        //if(Shooter.active())
        //    scale *= 0.75;

        // Cap off our scale at 1 
        if(scale > 1.0)
            scale = 1.0;


        
        // Combine our last avg with our new dist.
        // Longer the interval between updates, the more bias our new data has
        m_distAvg = m_distAvg * (1.0 - scale) + dist * (scale);

        //Update odometry position
        double a = Pigeon.getRotation() - m_hubAngle;
        a = (Math.PI*0.5) + Math.PI*(a/180.0);
        SwervePosition.setPosition(new Vector2D(Math.cos(a)*m_distAvg, Math.sin(a)*m_distAvg).mul(-12));
    }

    public static void setAngle() {
        if(disabled()) return;


        if(m_hubSeen)
        {
            double absang = Pigeon.getRotation() - m_hubAngle;
            Pigeon.setTargetAngle(absang);
        }
    }


    
    private static int m_disabledPrintCount = 0;
    private static boolean disabled() {
        if(kDisabled) {
            if(m_disabledPrintCount++ < 20) System.out.println("ERROR: VISION DISABLED!!!!");
            return true;
        }
        return false;
    }
}
