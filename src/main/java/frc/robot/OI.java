package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.robotmath.Vector2D;
import frc.robot.subsystems.AutoAlign;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveManager;
public class OI {
    static Joystick m_joystick;
    static NetworkTable m_nt;

    public static void init(){
        m_nt = NetworkTableInstance.getDefault().getTable("shooter");
        m_nt.getEntry("target_dist").setDouble(0.0);
        m_joystick = new Joystick(0);
        m_nt.getEntry("kp").setDouble(1.00);

    }

    public static void joystickInput(){


        double boost = m_joystick.getRawAxis(3);
        boost = boost * 0.7 + 0.3;


        Vector2D drive = new Vector2D(m_joystick.getRawAxis(0), -m_joystick.getRawAxis(1));

        double rotate = 0;
        if(!m_joystick.getRawButton(1)){
           
            rotate = m_joystick.getRawAxis(4);
            rotate = Math.pow(rotate,2) * boost * Math.signum(rotate);

            /*
            Vector2D angle = new Vector2D(m_joystick.getRawAxis(4), -m_joystick.getRawAxis(5));
            if(angle.mag() > 0.75)
            {
                
                double ang = angle.atanDeg();
                Pigeon.setTargetAngle(ang); 
                //System.out.println("Angle: " + ang);
                //rotate = m_joystick.getRawAxis(4);
                //rotate *= boost;
                //if(Math.abs(rotate)<0.1)
                //    rotate = 0;
            }
            */
        } else {
            AutoAlign.setAngle();
            rotate = Pigeon.correctTurnWithPID();
            //System.out.println(AutoAlign.getAngle());
        }
        
        if(Math.abs(rotate) < 0.03)
            rotate = 0;

        
        
        drive = drive.mul(boost);
        if(drive.mag() < 0.1){
            drive = new Vector2D(0, 0); 
        }
        



        SwerveManager.rotateAndDrive(rotate, drive);
        
        if(m_joystick.getRawButton(4))
        {
            Pigeon.zero();
            Pigeon.setTargetAngle(0);
        }


        //Shooter.setShooterSpeed(m_joystick.getRawAxis(2));
        Intake.setEnabled(m_joystick.getRawButton(6));
        

        double kp = m_nt.getEntry("kp").getDouble(1.00);
        if(m_joystick.getRawButton(5)) {
            Shooter.setRPMForDist(AutoAlign.m_distAvg, kp);
            if(rotate == 0 && Shooter.atSetpoint())
                Shooter.setHandoffEnabled(true);
//            else
//                Shooter.setHandoffEnabled(false);

        }
        else {
            Shooter.stopVelocityControl();
            Shooter.setHandoffEnabled(false);
        }




        //Shooter.setRPMForDist(m_nt.getEntry("target_dist").getDouble(0.0), kp);
        m_nt.getEntry("target_rpm").setDouble(Shooter.m_targetSpeed * Shooter.kVelToRPM);
        m_nt.getEntry("current_rpm").setDouble(Shooter.m_flywheel.getSelectedSensorVelocity() * Shooter.kVelToRPM);
        m_nt.getEntry("at_setpoint").setBoolean(Shooter.atSetpoint());


    }
}
