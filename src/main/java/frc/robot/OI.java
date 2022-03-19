package frc.robot;
import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
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
        m_nt.getEntry("rpm").setDouble(0.0);
        m_joystick = new Joystick(0);
    }

    public static void joystickInput(){


        double boost = m_joystick.getRawAxis(3);
        boost = boost * 0.8 + 0.2;


        double x = m_joystick.getRawAxis(0);
        double y = -1* m_joystick.getRawAxis(1);
        double rotate = 0.0;

        if(!m_joystick.getRawButton(1)){
            double angx = -m_joystick.getRawAxis(4);
            double angy = -m_joystick.getRawAxis(5);
            if(Math.hypot(angx, angy) > 0.75)
            {

                double ang = Math.atan2(angx,angy) * 180.0 / Math.PI;
                Pigeon.setTargetAngle(ang); 
                System.out.println("'               " + ang);
                //rotate = m_joystick.getRawAxis(4);
                //rotate *= boost;
                //if(Math.abs(rotate)<0.1)
                //    rotate = 0;
            }
        } else {
            AutoAlign.update();
            //System.out.println(AutoAlign.getAngle());
        }
        rotate = -Pigeon.correctTurnWithPID(0.02);


        x *= boost;
        y *= boost;
        if(Math.hypot(x, y) < 0.1){
            x = 0;
            y = 0;
        }

        if(Math.abs(rotate) < 0.1)
            rotate = 0;

        SwerveManager.rotateAndDrive(rotate, x, y);
        
        if(m_joystick.getRawButton(4)) {
            Pigeon.zero();
        }

        Shooter.setHandoffEnabled(m_joystick.getRawButton(5));
        //Shooter.setShooterSpeed(m_joystick.getRawAxis(2));
        Shooter.setShooterRPM(m_nt.getEntry("rpm").getDouble(0.0));
        Intake.setEnabled(m_joystick.getRawButton(6));
    }
}
