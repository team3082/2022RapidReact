package frc.robot;

import javax.print.attribute.standard.PrinterInfo;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import frc.CANMap;
import frc.controllermaps.LogitechF310;
import frc.robot.robotmath.RMath;
import frc.robot.subsystems.Shooter;

public class TestMode {

    ///
    // This was created as a "ground truth" for robot functionality.
    // It should *not* be smart.
    // It should *not* interract with any other systems.
    // It should be forever self isolated.
    ///


    private static Joystick m_controller;
    
    // Climber
    private static TalonFX m_climberHook;
    private static TalonSRX m_climberScrew;


	private static NetworkTable m_nt;
	private static NetworkTableEntry m_shooter_dist;


    public static void init() {

        // Controller
        m_controller = new Joystick(0);

        // Climber
        m_climberHook = new TalonFX(CANMap.CLIMBER_HOOK);
        m_climberScrew = new TalonSRX(CANMap.CLIMBER_SCREW);

        
        m_nt = NetworkTableInstance.getDefault().getTable("test_mode");
        m_shooter_dist = m_nt.getEntry("shooter_dist");
    
        m_shooter_dist.setDouble(0);

    }

    public static void disable() {

    }

    public static void update() {

        double left  = m_controller.getRawAxis(LogitechF310.AXIS_LEFT_Y);
        double right = m_controller.getRawAxis(LogitechF310.AXIS_RIGHT_Y);

        if(Math.abs(left) < 0.01) left = 0;
        if(Math.abs(right) < 0.01) right = 0;
        left = RMath.smoothJoystick1(left);
        right = RMath.smoothJoystick1(right);

        m_climberHook.set(ControlMode.PercentOutput, right);
        m_climberScrew.set(ControlMode.PercentOutput, left);

        if(m_controller.getRawButton(LogitechF310.BUTTON_Y))
        {
            m_climberScrew.setSelectedSensorPosition(0);
            m_climberHook.setSelectedSensorPosition(0);
        }

        Shooter.setRPMForDist(m_shooter_dist.getDouble(0));
    }
    
}
