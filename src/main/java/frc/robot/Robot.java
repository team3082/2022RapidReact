package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.SwerveMod;

public class Robot extends TimedRobot {

  final double ticksPerRot = 2048*12.8;
  Joystick m_joystick;

  SwerveMod swerve;

  @Override
  public void robotInit() {
    m_joystick = new Joystick(0);
    swerve = new SwerveMod(2, 4);
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

    if(drive > 0.1){
      swerve.drive(0.3 * drive);
      double joystickPos;
      joystickPos = ticksPerRot/2 + Math.atan2(y, x) / (2 * Math.PI) * ticksPerRot;
      swerve.rotate(joystickPos);
    }
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
  