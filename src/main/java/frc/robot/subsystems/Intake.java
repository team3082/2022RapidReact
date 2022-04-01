package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class Intake {
    private static VictorSPX m_intakeVictor;

    public static void init() {
        m_intakeVictor = new VictorSPX(11);
        m_intakeVictor.configFactoryDefault();
        m_intakeVictor.setInverted(true);
    }

    public static void setSpeed(double power) {
        m_intakeVictor.set(ControlMode.PercentOutput, power);
    }

    public static void setEnabled(Boolean intake_control) {
        if (intake_control)
            m_intakeVictor.set(ControlMode.PercentOutput, 1);
        else
            m_intakeVictor.set(ControlMode.PercentOutput, 0);
    }

}
