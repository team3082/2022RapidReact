package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.robotmath.Vector2D;

public class Climber {

    private static TalonFX hookMotor;
    private static TalonSRX tiltMotor;

    private static boolean climbing;

    public static void init(){
        hookMotor = new TalonFX(9999);

        hookMotor.configFactoryDefault();
        hookMotor.setInverted(false);
        hookMotor.setNeutralMode(NeutralMode.Brake);
        hookMotor.setSelectedSensorPosition(0);
        hookMotor.config_kF(0, 0, 30);
        hookMotor.config_kP(0, 0.1, 30);
        hookMotor.config_kI(0, 0, 30);
        hookMotor.config_kD(0, 0.05, 30);


        tiltMotor = new TalonSRX(9999);

        tiltMotor.configFactoryDefault();
        tiltMotor.setInverted(true);
        tiltMotor.setNeutralMode(NeutralMode.Brake);
        tiltMotor.setSelectedSensorPosition(0);
    }

    public static boolean isClimbing(){
        return climbing;
    }

    public static boolean isLimitHit(){
        return tiltMotor.getSensorCollection().isFwdLimitSwitchClosed();
    }

    public static void startClimb(){
        climbing = true;
        setHook(0);
        setTilt(0);
        Pigeon.stop();
        Shooter.stop();
        SwerveManager.rotateAndDrive(0, new Vector2D(0,0));
        Intake.setEnabled(false);
    }

    public static void stopClimb(){
        climbing = false;
        setHook(0);
        setTilt(0);
    }

    //4 controlling motors:
    public static void setHook(double pow){
        hookMotor.set(ControlMode.PercentOutput, pow);
    }
    public static void setTilt(double pow){
        tiltMotor.set(ControlMode.PercentOutput, pow);
    }

    //Auto climbing
    //Called in OI
    public static void climb(){
        //Hook should start at up position
    }

}
