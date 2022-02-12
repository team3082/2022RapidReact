package frc.robot;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.SwerveManager;
public class OI {
    static Joystick m_joystick;

    public static void init(){
        m_joystick = new Joystick(0);
    }

    public static void joystickInput(){

        double x = m_joystick.getRawAxis(4);
        double y = -1* m_joystick.getRawAxis(5);
        // rotate is inverted because negative values cause clockwise rotation in rotateAndDrive
        double rotate = -1 * m_joystick.getRawAxis(0);

        if(Math.hypot(x, y) < 0.1){
            x = 0;
            y = 0;
        }
        if(Math.abs(rotate)<0.1)
            rotate = 0;

        SwerveManager.rotateAndDrive(rotate/1.5, x/1.5, y/1.5);

    }
}
