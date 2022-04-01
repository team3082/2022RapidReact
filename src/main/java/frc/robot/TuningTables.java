package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.robotmath.RTime;
import frc.robot.subsystems.AutoAlign;
import frc.robot.subsystems.SwervePosition;

// Temporary class for pushing data to the network for debugging purposes
// !!DISABLE BEFORE COMPETITION!!
public class TuningTables {
    
    private static final boolean kEnabled = false; 

	private static NetworkTable m_nt;
	private static NetworkTableEntry m_ix;
	private static NetworkTableEntry m_iy;
	private static NetworkTableEntry m_ax;
	private static NetworkTableEntry m_ay;
	private static NetworkTableEntry m_vx;
	private static NetworkTableEntry m_vy;
	private static NetworkTableEntry m_dt;
    private static NetworkTableEntry m_hub_dist_avg;
    
    public static void init() {

        if(!kEnabled)
            return;

		m_nt = NetworkTableInstance.getDefault().getTable("tuning");
        
		m_ix = m_nt.getEntry("ix");
		m_iy = m_nt.getEntry("iy");
		m_ax = m_nt.getEntry("ax");
		m_ay = m_nt.getEntry("ay");
		m_vx = m_nt.getEntry("vx");
		m_vy = m_nt.getEntry("vy");
		m_dt = m_nt.getEntry("dt");
        m_hub_dist_avg = m_nt.getEntry("hub_dist_avg");

		
		m_ix.setDouble(0);
		m_iy.setDouble(0);
		m_ax.setDouble(0);
		m_ay.setDouble(0);
		m_vx.setDouble(0);
		m_vy.setDouble(0);
		m_dt.setDouble(0);
    }

    public static void update() {

        if(!kEnabled)
            return;

        m_ix.setDouble(SwervePosition.positionInt.x);
		m_iy.setDouble(SwervePosition.positionInt.y);
		m_ax.setDouble(SwervePosition.positionAcc.x);
		m_ay.setDouble(SwervePosition.positionAcc.y);
		m_vx.setDouble(SwervePosition.absVelocity.x);
		m_vy.setDouble(SwervePosition.absVelocity.y);

		m_dt.setDouble(RTime.getDeltaTime());

        m_hub_dist_avg.setDouble(AutoAlign.m_distAvg);

    }
}
