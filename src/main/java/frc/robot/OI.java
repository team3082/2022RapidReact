package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.robotmath.Vector2D;
import frc.robot.subsystems.AutoAlign;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveManager;
import frc.robot.subsystems.SwervePosition;

public class OI {
    
    static Joystick m_driverStick;
    
    static final int kMoveX = 0;
    static final int kMoveY = 1;
    static final int kRotateX = 4;
    static final int kBoost = 2;

    static final int kPigeonZero = 8;
    static final int kShooterRev = 6;
    static final int kShooterFire = 5;
    static final int kManualHandoff = 3;
    static final int kIntakePull = 3;
    

    static Joystick m_operatorStick;
    
    static final int kEject1 = 5;
    static final int kEject2 = 6;
    

    static NetworkTable m_nt;

    public static void init(){
        m_nt = NetworkTableInstance.getDefault().getTable("shooter");
        m_driverStick = new Joystick(0);
        m_operatorStick = new Joystick(1);
    }

    public static void joystickInput(){

        // Driver: (L Stick) Drives
        Vector2D drive = new Vector2D(m_driverStick.getRawAxis(kMoveX), -m_driverStick.getRawAxis(kMoveY));

        // Driver: (R trigger) Boosts
        double boost = m_driverStick.getRawAxis(kBoost);
        boost = boost * 0.7 + 0.3;

        // While shooting, slow us waaay down
        if(Shooter.firing())
            boost *= 0.1 / 0.4;

        // Joystick deadzone for drive
        if(drive.mag() < 0.05)
            drive = new Vector2D(0, 0); 
        else
            drive = drive.mul(boost);

        //Driver: (R stick X) Rotates
        double rotate = -m_driverStick.getRawAxis(kRotateX);
        //Joystick deadzone for rotation
        if(Math.abs(rotate) < 0.05)
            rotate = 0;
        else
            rotate = Math.pow(rotate,2) * boost * Math.signum(rotate);

        //Old rotation method (Rotates to angle of stick)
        /*
        Vector2D angle = new Vector2D(m_driverJoystick.getRawAxis(4), -m_driverJoystick.getRawAxis(5));
        if(angle.mag() > 0.75)
        {
            
            double ang = angle.atanDeg();
            Pigeon.setTargetAngle(ang); 
            //System.out.println("Angle: " + ang);
            //rotate = m_driverJoystick.getRawAxis(4);
            //rotate *= boost;
            //if(Math.abs(rotate)<0.1)
            //    rotate = 0;
        }
        */
        //System.out.println(AutoAlign.getAngle());
        
        // Driver: (START) zeros pigeon
        if(m_driverStick.getRawButton(kPigeonZero)){
            Pigeon.zero();
            Pigeon.setTargetAngle(0);
        }
        
        // Operator: (L & R  bumper together) Eject
        if(m_operatorStick.getRawButton(kEject1) && m_operatorStick.getRawButton(kEject2)) {
            Intake.eject();
            Shooter.eject();
        } else {
 
            // Driver: (R Trigger) Intakes
            Intake.setEnabled(m_driverStick.getRawAxis(kIntakePull) > 0.075);
            
            // Driver: (R Bumper) Revs to set rpm and does NOT auto align
            boolean shooterRev = m_driverStick.getRawButton(kShooterRev);
            // Driver: (L Bumper)...
            //  Aligns to hub, 
            //  Overrides rev and revs to distance from hub, 
            //  & Activates handoff when at speed
            boolean shooterAutoFire = m_driverStick.getRawButton(kShooterFire);
            // Driver: (X) Manually activates handoff
            boolean manualHandoff = m_driverStick.getRawButton(kManualHandoff);

            if(shooterAutoFire) {
                if(AutoAlign.m_hubSeen){
                    //Use hub
                    AutoAlign.setAngle();
                    Shooter.setRPMForDist(AutoAlign.m_distAvg, 1.0);
                } else {
                    //Use odomotry (KINDA SUS)
                    Vector2D direction = new Vector2D(0,0).sub(SwervePosition.getPosition());
                    Pigeon.setTargetAngle(direction.atanDeg());
                    Shooter.setRPMForDist(direction.mag(), 1.0);
                }
                rotate = Pigeon.correctTurnWithPID();
            } else if (shooterRev){
                Shooter.setShooterRPM(3000);
        } else {
                Shooter.stopVelocityControl();
                Pigeon.stop();
            }

            Shooter.fire(shooterAutoFire);
            if(manualHandoff)
                Shooter.setHandoffEnabled(true);
        }
        SwerveManager.rotateAndDrive(rotate, drive);

        //NT stuff for debugging/tuning only. Not actually used
        m_nt.getEntry("target_rpm").setDouble(Shooter.m_targetSpeed * Shooter.kVelToRPM);
        m_nt.getEntry("current_rpm").setDouble(Shooter.m_flywheel.getSelectedSensorVelocity() * Shooter.kVelToRPM);
        m_nt.getEntry("at_setpoint").setBoolean(Shooter.atSetpoint());
    }
}
