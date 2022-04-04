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

public class OI {
    
    static final int kMoveX = 0;
    static final int kMoveY = 1;
    static final int kRotateX = 4;
    static final int kBoost = 2;

    static final int kPigeonZero = 4;
    static final int kShooterRev = 6;
    static final int kShooterFire = 5;
    static final int kIntakePull = 3;
    static final int kEject = 8;
    
    
    static Joystick m_joystick;
    static NetworkTable m_nt;



    public static void init(){
        m_nt = NetworkTableInstance.getDefault().getTable("shooter");
        m_joystick = new Joystick(0);

    }

    public static void joystickInput(){

        // Driver: (L Stick) Drives
        Vector2D drive = new Vector2D(m_driverJoystick.getRawAxis(0), -m_driverJoystick.getRawAxis(1));

        double boost = m_joystick.getRawAxis(kBoost);
        boost = boost * 0.7 + 0.3;

        // While shooting, slow us waaay down
        if(Shooter.firing())
            boost *= 0.1 / 0.4;


        Vector2D drive = new Vector2D(m_joystick.getRawAxis(kMoveX), -m_joystick.getRawAxis(kMoveY));

        double rotate = -m_joystick.getRawAxis(kRotateX);
        rotate = Math.pow(rotate,2) * boost * Math.signum(rotate);

            if(Math.abs(rotate) < 0.03)
                rotate = 0;

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

            
        if(Math.abs(rotate) < 0.03)
            rotate = 0;

        
        
        drive = drive.mul(boost);
        if(drive.mag() < 0.1){
            drive = new Vector2D(0, 0); 
        }
        
        
        if(m_joystick.getRawButton(kPigeonZero))
        {
            Pigeon.zero();
            Pigeon.setTargetAngle(0);
        }


        
        if(m_joystick.getRawButton(kEject)) {
            Intake.eject();
            Shooter.eject();
        }
        else {

            

            //Shooter.setShooterSpeed(m_joystick.getRawAxis(2));
            Intake.setEnabled(m_joystick.getRawAxis(kIntakePull) > 0.075);
            


            boolean shooterrev = m_joystick.getRawButton(kShooterRev);
            boolean shooterfire = m_joystick.getRawButton(kShooterFire);

            
            if(shooterrev || shooterfire) {
                AutoAlign.setAngle();
                if(AutoAlign.m_hubSeen)
                    Shooter.setRPMForDist(AutoAlign.m_distAvg, 1.0);
                rotate = Pigeon.correctTurnWithPID();
            }
            else {
                Shooter.stopVelocityControl();
                Pigeon.stop();
            }

            if(shooterfire) {
                Shooter.fire(true);
            }
            else
                Shooter.fire(false);

        }
        SwerveManager.rotateAndDrive(rotate, drive);


        //Shooter.setRPMForDist(m_nt.getEntry("target_dist").getDouble(0.0), kp);
        m_nt.getEntry("target_rpm").setDouble(Shooter.m_targetSpeed * Shooter.kVelToRPM);
        m_nt.getEntry("current_rpm").setDouble(Shooter.m_flywheel.getSelectedSensorVelocity() * Shooter.kVelToRPM);
        m_nt.getEntry("at_setpoint").setBoolean(Shooter.atSetpoint());
    }
}
