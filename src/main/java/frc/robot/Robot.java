package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.auto.BasicAuto;
import frc.robot.subsystems.*;

public class Robot extends TimedRobot {

  @Override
  public void robotInit() {
    SwerveManager.init();
    OI.init();
    Pigeon.init();
    Pigeon.initPID(0.08, 0, 0.02, 0.18, 1.0);
    Shooter.init();
    Intake.init();
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
    System.out.println("auto");
    BasicAuto.init();    
  }

  @Override
  public void autonomousPeriodic() {
    BasicAuto.update();    
  }
}
