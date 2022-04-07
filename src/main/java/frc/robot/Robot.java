package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.auto.BasicAuto;
import frc.robot.robotmath.RTime;
import frc.robot.subsystems.*;

public class Robot extends TimedRobot {


	@Override
	public void robotInit() {
		RTime.init();
		SwerveManager.init();
		Pigeon.init();
		Pigeon.initPID(1.5, 0.1, 0, 1.0, 0.75, 3.0);
		Shooter.init();
		Intake.init();
		AutoAlign.init();
		SwervePosition.init();
		Climber.init();
		OI.init();
		
		TuningTables.init();
	}

	@Override
	public void robotPeriodic() {
		Climber.update();
	}

	@Override
	public void teleopInit() {
		SwerveManager.zeroSteeringEncoders();
		
		// FIXME: Remove before competition
//		Shooter.stop();
//		SwervePosition.init();
	}

	@Override
	public void teleopPeriodic() {
		RTime.update();

		SwervePosition.update();
		AutoAlign.update();
		
		TuningTables.update();

		OI.joystickInput();
		Shooter.update();
	}

	@Override
	public void testInit() {
		System.out.println("Test Init!");
		
		RTime.init();
		TuningTables.init();
		Shooter.init();
		
		Pigeon.zero();
		SwerveManager.zeroSteeringEncoders();

		TestMode.init();
	}

	@Override
	public void testPeriodic() {
		//SwerveManager.calibrationTest();
		
		RTime.update();
		
		TestMode.update();
		TuningTables.update();

		Shooter.update();
	}

	@Override
	public void disabledInit() {
	}

	@Override
	public void disabledPeriodic() {
	}

	@Override
	public void autonomousInit() {

		SwervePosition.init();

		SwerveManager.pointWheels(0);
		System.out.println("auto");
		BasicAuto.init();
	}

	@Override
	public void autonomousPeriodic() {
		RTime.update();
		SwervePosition.update();
		AutoAlign.update();

		BasicAuto.update();

		Shooter.update();
		TuningTables.update();
	}
}
