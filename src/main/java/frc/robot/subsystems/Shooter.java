package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class Shooter {
  private static VictorSPX m_handoff;
  private static TalonFX m_flywheel;
  
  public static void init() {
    m_handoff = new VictorSPX(11);
    m_flywheel = new TalonFX(10);
    m_handoff.configFactoryDefault();
    m_flywheel.configFactoryDefault();
    m_handoff.setInverted(true);
    m_flywheel.setInverted(true);
    m_flywheel.setNeutralMode(NeutralMode.Coast);
    m_flywheel.setNeutralMode(NeutralMode.Coast);
    m_flywheel.config_kP(0, 1);
    m_flywheel.config_kI(0, 0);
    m_flywheel.config_kD(0, 0);
    m_flywheel.configClosedLoopPeriod(0, 1750);
    // 17,760
  }

  public static void setShooterSpeed(double shooter_control) {
    m_flywheel.set(ControlMode.PercentOutput, shooter_control);
  }

  //WARNING: MIGHT BE WRONG
  public static void setShooterRPM(double rpm){
    m_flywheel.set(ControlMode.Velocity, (rpm*2048)/(60*10));
  }

  public static void setHandoffEnabled(Boolean shooter_intake_control) {
    if (shooter_intake_control) {
      m_handoff.set(ControlMode.PercentOutput, -1);
    } else {
      m_handoff.set(ControlMode.PercentOutput, 0);
    }
  }

}