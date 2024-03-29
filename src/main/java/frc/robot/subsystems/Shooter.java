package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXPIDSetConfiguration;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.TuningTables;
import frc.robot.robotmath.RTime;
import frc.robot.robotmath.Vector2D;

public class Shooter {

    public enum ShooterMode {
        Stopped(),
        Revving(),
        Firing(),
        Eject(),
    }

    public enum HandoffMode {
        Stopped(),
        Live(),
        Dead(),
    }


	public static final double kShooterAngle = 24.0;


    // falcon / wheel 
    private static final double kShooterBeltRatio = 1; // 0.95/0.64


    //  (Rotations)  v  (1  _mins) v  ( 1 _secs  )  v  ( falc rev) v ( 2048 _enc)
    //  (  Minutes)  ^  (60 _secs) ^  (10 _100 ms)  ^  (wheel rev) ^ ( 1 _rots  ) 
    // 
    private static final double kRPMToVel = 2048.0 * kShooterBeltRatio / 60.0 / 10.0;
    private static final double kVelToRPM = 10.0 * 60.0 / 2048.0 / kShooterBeltRatio;
    
    

    // Only fire when within this many RPM of our target
    private static final double kDeadbandRPM = 18.0;

    // When firing automatically, keep the handoff on only for kHandoffLifespan seconds
    private static final double kHandoffLifespan = 0.2;
    private static final double kHandoffDeadtime = 0.4;

    // Handoff speeds
    private static final double kHandoffPullSpeed   = 1.0;
    private static final double kHandoffEjectSpeed  = -1.0;
    private static final double kHandoffRetainSpeed = -0.6;
    // Slowly run the handoff backwards when stopped
    private static final double kHandoffStopSpeed = 0.0;


    public static VictorSPX m_handoff;
    public static TalonFX m_flywheel;

    private static ShooterMode m_mode    = ShooterMode.Stopped;
    private static double m_targetSpeed  = 0;

    private static HandoffMode m_handoffMode = HandoffMode.Stopped;
    private static double m_handoffLiveTime     = 0;

    private static NetworkTable m_nt;
    private static NetworkTableEntry nt_target_rpm;
    private static NetworkTableEntry nt_current_rpm;
    private static NetworkTableEntry nt_at_setpoint;
    private static NetworkTableEntry nt_temperature;
    
    public static void init() {

        // Setup the handoff
        m_handoff = new VictorSPX(8);
        m_handoff.configFactoryDefault();
        
        m_handoff.setInverted(false);
        
        // Set the handoff to break to prevent balls from slipping
        m_handoff.setNeutralMode(NeutralMode.Coast);



        // Set up the flywheel
        m_flywheel = new TalonFX(10);
        m_flywheel.configFactoryDefault();
        
        m_flywheel.setInverted(true);
        
        // Coast on neutral so we don't fight the momentum 
        m_flywheel.setNeutralMode(NeutralMode.Coast);
        
        // Configure the PID for our velocity control
        if(false)
        {
            m_flywheel.config_kP(0, 0.3);
            m_flywheel.config_kI(0, 0.011);
            m_flywheel.config_kD(0, 0);
            m_flywheel.config_kF(0, 0);
        }
        else
        {
            m_flywheel.config_kP(0, 0.3);
            m_flywheel.config_kI(0, 0.001);
            m_flywheel.config_kD(0, 0);
            m_flywheel.config_kF(0, 1023.0 / 20000.0);

        }
        
        // Use a long loop period so we rev up to speed gently
        m_flywheel.configClosedLoopPeriod(0, 1000);
        
        // Cap off our current at 39 amps. If we go above 40 amps, the breaker will flip
        SupplyCurrentLimitConfiguration currentLimit = new SupplyCurrentLimitConfiguration(true, 40, 40, 0 );
        m_flywheel.configSupplyCurrentLimit(currentLimit);
        
        // Enable voltage compensation to prevent variable behavior when the battery gets low/poor 
        m_flywheel.configVoltageCompSaturation(12.2);
        m_flywheel.enableVoltageCompensation(true);
        
        // Create a network table for the shooter so we can feed the driver info
        m_nt = NetworkTableInstance.getDefault().getTable("shooter");
        
        nt_target_rpm  = m_nt.getEntry("target_rpm");
        nt_current_rpm = m_nt.getEntry("current_rpm");
        nt_at_setpoint = m_nt.getEntry("at_setpoint");
        nt_temperature = m_nt.getEntry("temperature");
        
        nt_target_rpm.setDouble(0.0);
        nt_current_rpm.setDouble(0.0);
        nt_at_setpoint.setBoolean(false);
        
        // Zero our vars
        m_mode         = ShooterMode.Stopped;
        m_targetSpeed  = 0;
        m_handoffLiveTime = 0;
        m_handoffMode  = HandoffMode.Stopped;

    }


    private static void zeroHandoffVars() {
        m_handoffMode = HandoffMode.Stopped;
        m_handoffLiveTime = 0;
    }

    private static void setVelocity(double vel) {
        // Are we switching from winding down to speeding up?
        //boolean startShot = m_flywheel.getControlMode() == ControlMode.PercentOutput;

        // Pump in the vel AFTER checking our mode as to not overwrite
        m_flywheel.set(TalonFXControlMode.Velocity, vel);
        
    }


    public static void update() {
        
        boolean reachedSetpoint = atSetpoint();

        switch(m_mode) {
            
            case Firing:
                double now = RTime.now();
                
                // Rev the flywheel up to our set velocity
                setVelocity(m_targetSpeed);
                
                
                // Pass in the ball only when it's at the setpoint
                
                // What state should the handoff be in?
                switch(m_handoffMode) {
                    case Stopped:
                        // Wait until at setpoint and then make it live
                        if(reachedSetpoint) {
                            m_handoffMode = HandoffMode.Live;
                            m_handoffLiveTime = now + kHandoffLifespan;
                        }
                        break;
                    case Live:
                        // Wait until we are over our lifespan and then die
                        if(now > m_handoffLiveTime) {
                            m_handoffMode = HandoffMode.Dead;
                            m_handoffLiveTime = now + kHandoffDeadtime;
                        }
                        break;
                    case Dead:
                        // Wait until we are over our death time and at setpoint
                        // and then COME BACK LIKE A PHOENIX!!
                        if(now > m_handoffLiveTime && reachedSetpoint) {
                            m_handoffMode = HandoffMode.Live;
                            m_handoffLiveTime = now + kHandoffDeadtime;
                        }
                        break;
                    
                }

                // Apply the handoff's mode
                switch(m_handoffMode) {
                    case Stopped:
                        // We dont want to run the handoff if we're just starting
                        // The ball can jam the shooter wheel!
                        m_handoff.set(ControlMode.PercentOutput, kHandoffRetainSpeed);
                        break;
                    case Dead:
                        // Run the handoff backwards to prevent balls from shooting early
                        m_handoff.set(ControlMode.PercentOutput, kHandoffEjectSpeed);
                        // Run the intake to prevent balls from pushing eachother out
                        Intake.slowRetain();   
                        break;
                    case Live:
                        m_handoff.set(ControlMode.PercentOutput, kHandoffPullSpeed);
                        // When we're running the handoff, run the intake too incase a ball is stuck!
                        Intake.setEnabled(true);
                        break;                     
                }

                break;

            case Revving:
                // Rev the flywheel up to our set velocity
                setVelocity(m_targetSpeed);

                // Don't run the handoff
                m_handoff.set(ControlMode.PercentOutput, kHandoffStopSpeed);
                zeroHandoffVars();
                break;
            
            case Eject:
                // Run the shooter forward and the handoff backwards
                m_flywheel.set(TalonFXControlMode.PercentOutput, 0.8);
                m_handoff.set(ControlMode.PercentOutput, kHandoffEjectSpeed);
                zeroHandoffVars();
                m_targetSpeed = 0.0;
                break;

            case Stopped:
                // If we just set our velocity controller to 0, it would forcefully attempt to stop the shooter
                // By setting our output to 0, we disable the controller and allow the wheel to coast
                // This should help us maintain the health of our belts
                m_flywheel.set(TalonFXControlMode.PercentOutput, 0.0);
                m_handoff.set(ControlMode.PercentOutput, kHandoffStopSpeed);
                zeroHandoffVars();
                m_targetSpeed = 0.0;
                break;
        }

        // Update our network entries
        nt_target_rpm.setDouble(m_targetSpeed * Shooter.kVelToRPM);
        nt_current_rpm.setDouble(m_flywheel.getSelectedSensorVelocity() * kVelToRPM);
        nt_at_setpoint.setBoolean(reachedSetpoint);
        nt_temperature.setDouble(getTemperature());
        
    }


    public static void setRPMForDist(double dist_ft)
    {
        // Take half a foot off our shot so that we bank it against the wall
        //dist_ft -= 0.5;
        dist_ft -= 0.1;

        // Big Gray Wheel
        final double wheel_radius_ft = 8.0 /* inch diameter */ / 2.0 / 12.0;
        
        final double grav_ftps = -32.1740486; 
        /*final*/ double shooter_angle = (TuningTables.getShooterAngle()) * Math.PI / 180.0;
        final Vector2D shooter_dir = new Vector2D(Math.sin(shooter_angle), Math.cos(shooter_angle));
        final Vector2D hub_pos_ft  = new Vector2D(0, 8.0 + 8.0/12.0);

        // We sink the target lower as we get further away to try to avoid balls bouncing out
        // From sink_dist_begin_ft out to sink_dist_ft, we sink from hub_pos_ft down by sink_height_ft
        final double sink_dist_begin_ft = 10.0;
        final double sink_dist_ft       = 25.0;
        final double sink_height_ft     = 0.125; 


        Vector2D target = new Vector2D( 0, (dist_ft - sink_dist_begin_ft ) / (sink_dist_ft - sink_dist_begin_ft) * -sink_height_ft ).add(hub_pos_ft);
        
        Vector2D shooter_pos_ft = new Vector2D(-dist_ft, 2);
        Vector2D delta = target.sub(shooter_pos_ft);

        // The speed the ball should be at when it comes out of the shooter
        //        
        // hub_pos_ft.y = (shooter_pos_ft.y) + (speed)(shooter_dir.y)t + (0.5)(grav_ftps)(t^2)
        // hub_pos_ft.x = (shooter_pos_ft.x) + (speed)(shooter_dir.x)t
        // 
        double speed = delta.x / ( shooter_dir.x * Math.sqrt( 2.0 * ( delta.y - delta.x*shooter_dir.y/shooter_dir.x ) / grav_ftps ) );

        // We treat the shooter like a planetary gearbox
        final double hood_radius_ft = 11.5 / 12.0;
        final double ball_compressed_radius_ft = 0.5 * (hood_radius_ft - wheel_radius_ft);
        final double path_radius_ft = wheel_radius_ft + ball_compressed_radius_ft;
        
        // I think this has something to do with the compression and/or math to real life tuning magic 
        final double magic_multiplier = 0.8893; // AAAA!

        // hood_to_ball * ball_to_wheel
        // ball cancels out
        // hood_to_wheel
        final double hood_to_wheel_ratio = hood_radius_ft / wheel_radius_ft * magic_multiplier;

        double freq = speed / (path_radius_ft * Math.PI) * hood_to_wheel_ratio;
        double rpm = freq * 60.0;
        

        // If our calculations failed, were very small, or negative, don't run the shooter
        if(Double.isInfinite(rpm) 
        || Double.isNaN(rpm) 
        || rpm <= 2) {
            stop();
        }
        else {
            // Pump it into the wheel!
            revTo(rpm);
        }        
    }

    

    public static void stop() {
        m_mode = ShooterMode.Stopped;
        m_targetSpeed = 0;
    } 

    public static void eject() {
        m_mode = ShooterMode.Eject;
    }

    public static void fire() {
        // We can only begin firing if we were already revving
        if(m_mode == ShooterMode.Revving)
            m_mode = ShooterMode.Firing;
    }

    public static void revTo(double rpm) {
        m_targetSpeed = rpm * kRPMToVel;
        m_mode = ShooterMode.Revving;
    }


    // Is the shooter up to speed yet?
    public static boolean atSetpoint()
    {
        final double vel_deadband = kDeadbandRPM * Shooter.kRPMToVel;
        double vel = m_flywheel.getSelectedSensorVelocity();

        double err = Math.abs(vel - m_targetSpeed);
        return err < vel_deadband;
    }

    // Is the shooter currently revving?
    public static boolean revving() {
        return m_mode == ShooterMode.Revving;
    }

    // Is the shooter trying to fire?
    public static boolean firing() {
        return m_mode == ShooterMode.Firing;
    }

    // Temp in C
    public static double getTemperature() {
        return m_flywheel.getTemperature();
    }
}
