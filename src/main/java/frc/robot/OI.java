package frc.robot;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.SwerveManager;
public class OI {
    static Joystick m_joystick;

    public static void init(){
        m_joystick = new Joystick(0);
    }

    public static void joystickInput(){


        double boost = m_joystick.getRawAxis(3);
        boost = boost * 0.7 + 0.3;


        double x = m_joystick.getRawAxis(4);
        double y = -1* m_joystick.getRawAxis(5);
        double rotate = m_joystick.getRawAxis(0);

        x *= boost;
        y *= boost;
        rotate *= boost;

        if(Math.hypot(x, y) < 0.1){
            x = 0;
            y = 0;
        }
        if(Math.abs(rotate)<0.1)
            rotate = 0;

        SwerveManager.rotateAndDrive(rotate, x, y);
        
        if(m_joystick.getRawButton(4)) {
            Pigeon.zero();
        }
    }
}
