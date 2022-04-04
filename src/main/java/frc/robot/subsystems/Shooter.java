package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import frc.robot.TuningTables;
import frc.robot.robotmath.Vector2D;

public class Shooter {
    private static VictorSPX m_handoff;
    public static TalonFX m_flywheel;

    public static double m_targetSpeed = 0;
    
    // falcon / wheel 
    public static final double kShooterBeltRatio = 1; // 0.95/0.64


    //  (Rotations)  v  (1  _mins) v  ( 1 _secs  )  v  ( falc rev) v ( 2048 _enc)
    //  (  Minutes)  ^  (60 _secs) ^  (10 _100 ms)  ^  (wheel rev) ^ ( 1 _rots  ) 
    // 
    public static final double kRPMToVel = 2048.0 * kShooterBeltRatio / 60.0 / 10.0;
    public static final double kVelToRPM = 10.0 * 60.0 / 2048.0 / kShooterBeltRatio;
    
    

    // Only fire when within this many RPM of our target
    private static final double kDeadbandRPM = 6.0;

    private static boolean m_active = false;
    private static boolean m_firing = false;


    public static void init() {
        m_handoff = new VictorSPX(8);
        m_flywheel = new TalonFX(10);
        m_handoff.configFactoryDefault();
        m_flywheel.configFactoryDefault();

        m_handoff.setInverted(false);
        m_flywheel.setInverted(true);

        m_flywheel.setNeutralMode(NeutralMode.Coast);
        m_flywheel.config_kP(0, 0.2);
        m_flywheel.config_kI(0, 0.015);
        m_flywheel.config_kD(0, 0);
        m_flywheel.configClosedLoopPeriod(0, 1200);
        
        m_flywheel.configVoltageCompSaturation(12.2);
        m_flywheel.enableVoltageCompensation(true);

        m_handoff.setNeutralMode(NeutralMode.Brake);


        m_targetSpeed = 0;
        m_active = false;
        m_firing = false;

    }

    public static void update() {
        if(m_active) {

            if(m_firing) {
                if(Pigeon.atSetpoint() && Shooter.atSetpoint()) {  
                    setHandoffEnabled(true);
                }
            }
            else {
                setHandoffEnabled(false);
            }

        }
        else {
            setHandoffEnabled(false);
        }
    }

    public static void setShooterSpeed(double shooter_control) {
        m_flywheel.set(ControlMode.PercentOutput, shooter_control);
    }

    public static void setShooterRPM(double rpm){
        m_flywheel.set(ControlMode.Velocity, rpm*kRPMToVel);
    }

    public static void setHandoffEnabled(boolean enabled) {
        m_handoff.set(ControlMode.PercentOutput, enabled?1:0);
    }

    // Distance in feet
    // Mass in kg
    public static void setRPMForDist(double dist_ft, double kp)
    {
        // Approximate masses
        
        // Big Gray
        final double wheel_radius_ft = 8 /* inch diameter */ / 2.0 / 12.0;
        
        // Small old swerve
        //final double wheel_radius_ft = 6 /* inch diameter */ / 2.0 / 12.0;
        
        final double grav_ftps = -32.1740486; 
        /*final*/ double shooter_angle = (TuningTables.getShooterAngle()) * Math.PI / 180.0;
        final Vector2D shooter_dir = new Vector2D(Math.sin(shooter_angle), Math.cos(shooter_angle));
        final Vector2D hub_pos_ft = new Vector2D(0, 8 + 8/12 + 0.6);
        
        Vector2D shooter_pos_ft = new Vector2D(-dist_ft - 1, 2);
        Vector2D delta = hub_pos_ft.sub(shooter_pos_ft);

        // The speed the ball should be at when it comes out of the shooter
        //        
        // hub_pos_ft.y = (shooter_pos_ft.y) + (speed)(shooter_dir.y)t + (0.5)(grav_ftps)(t^2)
        // hub_pos_ft.x = (shooter_pos_ft.x) + (speed)(shooter_dir.x)t
        // 
        double speed = delta.x / ( shooter_dir.x * Math.sqrt( 2.0 * ( delta.y - delta.x*shooter_dir.y/shooter_dir.x ) / grav_ftps ) );

        //final double wheel_mass_kg = 0.275;
        //final double wheel_mass_kg = 0.2 * 2;
        //final double ball_mass_kg = 0.225;

        // Angular velocity of shooter
        // Wheel I: MR^2 
        // Ball  I: 2MR^2/3
        // 
        // (0.5)(I_w)(w_w^2) = (0.5)(m_b)(v_b^2) + (0.5)(I_b)(w_b^2) 
        //
        //double w = Math.sqrt((10.0/6.0) * ball_mass_kg * (speed * speed) / (wheel_mass_kg * (wheel_radius_ft*wheel_radius_ft)));
        // Shooter rotations per minute
        //double f = w / (2.0 * Math.PI) * 60.0; 


        // We treat the shooter like a planetary gearbox
        //
        final double hood_radius_ft = 11.5 / 12.0;
        final double ball_compressed_radius_ft = 0.5 * (hood_radius_ft - wheel_radius_ft);
        final double path_radius_ft = wheel_radius_ft + ball_compressed_radius_ft;

        // hood_to_ball * ball_to_wheel
        // ball cancels out
        // hood_to_wheel
        final double hood_to_wheel_ratio = hood_radius_ft / wheel_radius_ft;

        double freq = speed / (path_radius_ft * Math.PI) * hood_to_wheel_ratio;
        double rpm = freq * 60;
        
        // Tweak our RPM by some tuning value "kp"
        rpm *= kp;

        // Convert to encoder ticks per 100 ms
        double vel = rpm * kRPMToVel;

        
        // If our calculations failed, were very small, or negative, don't run the shooter
        if(Double.isInfinite(vel) 
        || Double.isNaN(vel) 
        || vel <= 2048) {
            stopVelocityControl();
        }
        else
        {
            // Pump it into the wheel!
            m_targetSpeed = vel;
            m_flywheel.set(TalonFXControlMode.Velocity, m_targetSpeed);
            m_active = true;
        }        
    }

    public static boolean atSetpoint()
    {
        final double vel_deadband = kDeadbandRPM * Shooter.kRPMToVel;
        double vel = m_flywheel.getSelectedSensorVelocity();

        double err = Math.abs(vel - m_targetSpeed);
        return err < vel_deadband;
    }

    public static void stopVelocityControl() {
        // If we just set our velocity controller to 0, it would forcefully attempt to stop the shooter
        // By setting our output to 0, we disable the controller and allow the wheel to coast
        // This should help us maintain the health of our belts
        m_flywheel.set(TalonFXControlMode.PercentOutput, 0.0);
        m_active = false;
    } 


    public static void eject() {
        m_flywheel.set(TalonFXControlMode.PercentOutput, 0.8);
        m_handoff.set(ControlMode.PercentOutput, -1);
        m_active = true;
    }


    public static void fire(boolean fire) {
        m_firing = fire;
    }

    // Is the shooter currently revving?
    public static boolean active() {
        return m_active;
    }

    public static boolean firing() {
        return m_firing;
    }
}
