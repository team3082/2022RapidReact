package frc.robot;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.auto.BasicAuto;
import frc.robot.subsystems.*;

public class Robot extends TimedRobot {

  @Override
  public void robotInit() {
    SwerveManager.init();
    OI.init();
    Pigeon.init();
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void teleopInit() {
    SwerveManager.zeroSteeringEncoders();
  }

  @Override
  public void teleopPeriodic() {
    OI.joystickInput();
  }

  @Override
  public void testInit() {
    Pigeon.zero();
  }

  @Override
  public void testPeriodic() {

  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    Pigeon.zero();
    SwerveManager.pointWheels(0);
    BasicAuto.init();
  }

  @Override
  public void autonomousPeriodic() {
    BasicAuto.update();    
  }
}
