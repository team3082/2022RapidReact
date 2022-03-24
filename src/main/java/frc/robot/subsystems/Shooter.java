package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import org.opencv.core.Mat;

import frc.robot.robotmath.Vector2D;

public class Shooter {
    private static VictorSPX m_handoff;
    public static TalonFX m_flywheel;

    public static double m_targetSpeed = 0;
    
    public static final double kToRPM = 1.0 / 2048.0 * 10.0 * 60.0 * (0.64/0.95);


    public static void init() {
        m_handoff = new VictorSPX(11);
        m_flywheel = new TalonFX(10);
        m_handoff.configFactoryDefault();
        m_flywheel.configFactoryDefault();

        m_handoff.setInverted(true);
        m_flywheel.setInverted(true);

        m_flywheel.setNeutralMode(NeutralMode.Coast);
        m_flywheel.config_kP(0, 0.0007);
        m_flywheel.config_kI(0, 0.004);
        m_flywheel.config_kD(0, 0);
        m_flywheel.configClosedLoopPeriod(0, 1750);
        // 17,760

        m_targetSpeed = 0;
    }

    public static void setShooterSpeed(double shooter_control) {
        m_flywheel.set(ControlMode.PercentOutput, shooter_control);
    }

    //WARNING: MIGHT BE WRONG
    public static void setShooterRPM(double rpm){
        m_flywheel.set(ControlMode.Velocity, (rpm*2048)/(60*10));
    }

    public static void setHandoffEnabled(Boolean shooter_intake_control) {
        if (shooter_intake_control) {
            m_handoff.set(ControlMode.PercentOutput, -1);
        } else {
            m_handoff.set(ControlMode.PercentOutput, 0);
        }
    }

    // Distance in feet
    // Mass in kg
    public static void setRPMForDist(double dist_ft, double p)
    {
        // Approximate masses
        final double wheel_mass_kg = 0.275;
        final double ball_mass_kg = 0.225;

        final double wheel_radius_ft = 8 /* inch diameter */ / 2.0 / 12.0;

        final double shooter_gear_ratio = 0.95/0.64;
        final double grav_ftps = -32.2; 
        final double shooter_angle = (90 - 11) * Math.PI / 180.0;
        final Vector2D shooter_dir = new Vector2D(Math.cos(shooter_angle), Math.sin(shooter_angle));
        final Vector2D hub_pos_ft = new Vector2D(0, 8 + 8/12);
        
        Vector2D bot_pos_ft = new Vector2D(-dist_ft, 2);
        Vector2D delta = hub_pos_ft.sub(bot_pos_ft);

        // The speed the ball should be at when it comes out of the shooter
        double speed = delta.x / ( shooter_dir.x * Math.sqrt( 2.0 * ( delta.y - delta.x*shooter_dir.y/shooter_dir.x ) / grav_ftps ) );

        // Angular velocity of shooter
        // Wheel I: MR^2 
        // Ball  I: 2MR^2/3
        // 
        // (0.5)(I_w)(w_w^2) = (0.5)(m_b)(v_b^2) + (0.5)(I_b)(w_b^2) 
        double w = Math.sqrt((10.0/6.0) * ball_mass_kg * (speed * speed) / (wheel_mass_kg * (wheel_radius_ft*wheel_radius_ft)));
        
        // Shooter rotations per second
        double f = w / (2.0 * Math.PI); 
        
        // Falcon rotations per second
        double ff = f * shooter_gear_ratio;        

        // Encoder ticks per 100 ms
        double enc = ff * 2048.0 / 10.0 * p;

        // Pump it into the wheel!
        m_targetSpeed = enc;
        m_flywheel.set(TalonFXControlMode.Velocity, m_targetSpeed);

        
        if(Double.isNaN(m_targetSpeed) || m_targetSpeed <= 2048)
            m_flywheel.set(TalonFXControlMode.PercentOutput, 0.0);
        
    }

    public static boolean atSetpoint()
    {
        final double vel_deadband = 50.0 / Shooter.kToRPM;
        double vel = m_flywheel.getSelectedSensorVelocity();

        double err = Math.abs(vel - m_targetSpeed);

        return err < vel_deadband;
    }


}