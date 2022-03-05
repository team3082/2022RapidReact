// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class Intake {
  static VictorSPX intake_victor;
  
  static public void init() 
  {
    intake_victor = new VictorSPX(12);
    intake_victor.configFactoryDefault();
    intake_victor.setInverted(true);
  }

  static public void variable_throttle_intake(double power) 
  {
    intake_victor.set(ControlMode.PercentOutput, power);
  }
  static public void full_throttle_intake(Boolean intake_control)
  {
    if(intake_control)
    {
      intake_victor.set(ControlMode.PercentOutput, 0.75);
    }
    else
    {
      intake_victor.set(ControlMode.PercentOutput, 0);
    }
    
  }

}
