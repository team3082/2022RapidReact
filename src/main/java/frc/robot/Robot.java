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
		Pigeon.initPID(3.0, 3.0, 0, 1.0, 0.25, 1.0);
		Shooter.init();
		Intake.init();
		AutoAlign.init();
		SwervePosition.init();
		OI.init();
		
		TuningTables.init();
	}

	@Override
	public void robotPeriodic() {
	}

	@Override
	public void teleopInit() {
		SwerveManager.zeroSteeringEncoders();
		
		// FIXME: Remove before competition
		Shooter.stopVelocityControl();
		SwervePosition.init();
	}

	@Override
	public void teleopPeriodic() {
		RTime.update();
		SwervePosition.updateVelocity();
		SwervePosition.updatePositionIntegration();
		SwervePosition.updatePositionAccumulation();
		
		TuningTables.update();

		OI.joystickInput();
		AutoAlign.update();

	}

	@Override
	public void testInit() {
		System.out.println("Test Init!");
		Pigeon.zero();
		SwerveManager.zeroSteeringEncoders();
	}

	@Override
	public void testPeriodic() {
		//SwerveManager.calibrationTest();
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
		SwervePosition.init();

		SwerveManager.pointWheels(0);
		System.out.println("auto");
		BasicAuto.init();
	}

	@Override
	public void autonomousPeriodic() {
		RTime.update();
		SwervePosition.updateVelocity();
		SwervePosition.updatePositionIntegration();
		SwervePosition.updatePositionAccumulation();

		BasicAuto.update();

		TuningTables.update();

	}
}
