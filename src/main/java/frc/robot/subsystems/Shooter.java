package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class Shooter {
  static VictorSPX shooter_intake_victor;
  static TalonFX shooter_falcon;
  
  static public void init() 
  {
    shooter_intake_victor = new VictorSPX(11);
    shooter_falcon = new TalonFX(10);
    shooter_intake_victor.configFactoryDefault();
    shooter_falcon.configFactoryDefault();
    shooter_intake_victor.setInverted(true);
    shooter_falcon.setInverted(true);
    shooter_falcon.setNeutralMode(NeutralMode.Coast);
    shooter_falcon.setNeutralMode(NeutralMode.Coast);
    shooter_falcon.config_kP(0, 1);
    shooter_falcon.config_kI(0, 0);
    shooter_falcon.config_kD(0, 0);
    shooter_falcon.configClosedLoopPeriod(0, 1750);
    // 17,760
  }

  static public void shooter(double shooter_control) 
  {
    shooter_falcon.set(ControlMode.PercentOutput, shooter_control);
    System.out.printf("%.02f\n", shooter_falcon.getSelectedSensorVelocity() / 2048 * 0.5 * 10 * 60);
  }
  static public void shooter_intake(Boolean shooter_intake_control)
  {
    if(shooter_intake_control)
    {
      shooter_intake_victor.set(ControlMode.PercentOutput, -1);
    }
    else
    {
      shooter_intake_victor.set(ControlMode.PercentOutput, 0);
    }
  }

}