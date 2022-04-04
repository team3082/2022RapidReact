package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.robotmath.RTime;
import frc.robot.robotmath.Vector2D;
import frc.robot.subsystems.AutoAlign;
import frc.robot.subsystems.SwervePosition;

// Temporary class for pushing data to the network for debugging purposes
// !!DISABLE BEFORE COMPETITION!!
public class TuningTables {
    

	private static final double kShooterAngle = 30.0;


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
    
	private static NetworkTable m_nt_set;
	private static NetworkTableEntry m_shooter_angle;

    
    public static void init() {

        if(!kEnabled)
            return;

		m_nt = NetworkTableInstance.getDefault().getTable("tuning_info");
		m_nt_set = NetworkTableInstance.getDefault().getTable("tuning_set");
        
		m_ix = m_nt.getEntry("ix");
		m_iy = m_nt.getEntry("iy");
		m_ax = m_nt.getEntry("ax");
		m_ay = m_nt.getEntry("ay");
		m_vx = m_nt.getEntry("vx");
		m_vy = m_nt.getEntry("vy");
		m_dt = m_nt.getEntry("dt");
        m_hub_dist_avg = m_nt.getEntry("hub_dist_avg");

		m_shooter_angle = m_nt_set.getEntry("shooter_ang");
		
		m_ix.setDouble(0);
		m_iy.setDouble(0);
		m_ax.setDouble(0);
		m_ay.setDouble(0);
		m_vx.setDouble(0);
		m_vy.setDouble(0);
		m_dt.setDouble(0);

		m_shooter_angle.setDouble(kShooterAngle);
    }

    public static void update() {

        if(!kEnabled)
            return;

		Vector2D vel = SwervePosition.getAbsVelocity();
		Vector2D abspos = SwervePosition.getAbsPosition();
		Vector2D pos = SwervePosition.getPosition();
        m_ix.setDouble(pos.x);
		m_iy.setDouble(pos.y);
		m_ax.setDouble(abspos.x);
		m_ay.setDouble(abspos.y);
		m_vx.setDouble(vel.x);
		m_vy.setDouble(vel.y);

		m_dt.setDouble(RTime.getDeltaTime());

        m_hub_dist_avg.setDouble(AutoAlign.m_distAvg);

    }


	public static double getShooterAngle() {
		if(kEnabled)
			return m_shooter_angle.getDouble(kShooterAngle);	
		return kShooterAngle;
	}
}
