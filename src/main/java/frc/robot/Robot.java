package frc.robot;

import org.ejml.equation.Variable;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.*;

public class Robot extends TimedRobot {

  final double ticksPerRot = 2048*12.8;
  Joystick m_joystick;
  
  SwerveMod frontLeft;
  SwerveMod frontRight;

  SwerveMod[] m_array;

  @Override
  public void robotInit() {
    SwerveManager.init();
    // m_joystick = new Joystick(0);
    // frontLeft = new SwerveMod(0, 7);
    // frontRight = new SwerveMod(0,3);

    m_array = new SwerveMod[]{frontLeft, frontRight};
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    double x = m_joystick.getRawAxis(0);
    double y = -1* m_joystick.getRawAxis(1);
 double drive = Math.hypot(x, y);

for(int i =0; i<m_array.length;i++){
m_array[i].drive(0.3*drive);

}
   

    
   
    // if(drive > 0.1){
    //   frontLeft.drive(0.3 * drive);
    //   double joystickPos;
    //   joystickPos = ticksPerRot/2 + Math.atan2(y, x) / (2 * Math.PI) * ticksPerRot;
    //   frontLeft.rotate(joystickPos);
    // }
  }

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}
}
  