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
        double rotate = m_joystick.getRawAxis(0);

        SwerveManager.rotateAndDrive(rotate, x, y);

    }
}
