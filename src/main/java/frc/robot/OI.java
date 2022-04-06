package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import frc.LogitechF310;
import frc.robot.robotmath.Vector2D;
import frc.robot.subsystems.AutoAlign;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveManager;
import frc.robot.subsystems.SwervePosition;

public class OI {
    
    static Joystick m_driverStick;
    
    static final int kMoveX         = LogitechF310.AXIS_LEFT_X;
    static final int kMoveY         = LogitechF310.AXIS_LEFT_Y;
    static final int kRotateX       = LogitechF310.AXIS_RIGHT_X;
    static final int kBoost         = LogitechF310.AXIS_LEFT_TRIGGER;

    static final int kPigeonZero    = LogitechF310.BUTTON_START;
    static final int kShooterRev    = LogitechF310.BUTTON_RIGHT_BUMPER;
    static final int kShooterFire   = LogitechF310.BUTTON_LEFT_BUMPER;
    static final int kShooterManual = LogitechF310.BUTTON_X;
    static final int kIntakePull    = LogitechF310.AXIS_RIGHT_TRIGGER;
    static final int kEject         = LogitechF310.BUTTON_BACK;

    static Joystick m_operatorStick;

    static final int kBigHook    = LogitechF310.AXIS_RIGHT_Y;
    static final int kTiltHook   = LogitechF310.AXIS_LEFT_Y; 
    static final int kStartClimb = LogitechF310.BUTTON_START;
    static final int kStopClimb  = LogitechF310.BUTTON_BACK;


    
    public static void init(){
        m_driverStick = new Joystick(0);
        m_operatorStick = new Joystick(1);
    }

    public static void joystickInput(){

        //If climbing stop everything.
        // Operator: Stops climb
        if(Climber.isClimbing()){
            if(m_operatorStick.getRawButton(kStopClimb))
                Climber.stopClimb();
            else
                Climber.climb();
                return;
        }
        
        // Operator: Starts climb
        if(m_operatorStick.getRawButton(kStartClimb)){
            Climber.startClimb();
            Climber.climb();
            return;
        }

        // Operator: Controlls big hook
        if(m_operatorStick.getRawAxis(kBigHook) > 0.2)
            Climber.setHook(m_operatorStick.getRawAxis(kBigHook));

        // Operator: Controlls tilting hook
        if(m_operatorStick.getRawAxis(kTiltHook) > 0.2)
            Climber.setHook(m_operatorStick.getRawAxis(kTiltHook));

        // Driver: Drives
        Vector2D drive = new Vector2D(m_driverStick.getRawAxis(kMoveX), -m_driverStick.getRawAxis(kMoveY));

        // Driver: Boosts
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

        //Driver: Rotates
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
        
        // Driver: zeros pigeon
        if(m_driverStick.getRawButton(kPigeonZero)){
            Pigeon.zero();
            Pigeon.setTargetAngle(0);
        }
        
        // Driver: Eject
        if(m_driverStick.getRawButton(kEject)) {
            Intake.eject();
            Shooter.eject();
            Pigeon.stop();
        } else {
 
            // Driver: Intakes
            Intake.setEnabled(m_driverStick.getRawAxis(kIntakePull) > 0.075);
            
            // Driver: Revs to set rpm and does NOT auto align
            boolean shooterRev = m_driverStick.getRawButton(kShooterRev);
            // Driver: ...
            //  Aligns to hub, 
            //  Overrides rev and revs to distance from hub, 
            //  & Activates handoff when at speed
            boolean shooterAutoFire = m_driverStick.getRawButton(kShooterFire);
            // Driver: Manually fires the shooter with a set rpm
            boolean manualFire = m_driverStick.getRawButton(kShooterManual);
            final double manualFireRPM = 2500.0;

            // Always use odometry for auto firing.
            // Odomotry gets instantly updated when the camera sees the hub,
            //  so this should be the same as using vision except it should work when we can't see the hub

            // We only ever want to be aligning if we're in auto fire mode 
            if(shooterAutoFire && !manualFire) {

                double heading = Pigeon.getRotation() / 180.0 * Math.PI;
                Vector2D robotPos = SwervePosition.getPosition();
                Vector2D shooterOffset = new Vector2D(Math.cos(heading), Math.sin(heading)).mul(12 + 5/8);
                Vector2D shooterPos = robotPos.add(shooterOffset);
                
                // Multiply by -1 so that we're pointing towards 0
                Vector2D dir = shooterPos.mul(-1).norm();
                
                double ang = dir.atanDeg();
                Pigeon.setTargetAngle(ang);
                //System.out.println(ang);
                rotate = Pigeon.correctTurnWithPID();
            } else {
                Pigeon.stop();
            }
                
            // Manually firing takes precedence over auto firing
            // Auto firing takes precedence over revving

            if (manualFire) {
                // Manually fire the shooter to our set rpm
                Shooter.revTo(manualFireRPM);
                Shooter.fire();
            } else if(shooterAutoFire) {
                // Rev and fire the shooter for our position on the field
                Shooter.setRPMForDist(SwervePosition.getPosition().mag() / 12.0);
                Shooter.fire();   
            } else if (shooterRev) {
                // Rev the shooter to a set rpm, somewhere about the middle of the field
                Shooter.revTo(manualFireRPM);
            } else {
                Shooter.stop();
            }
            
        }

        // Swerving and a steering! Zoom!
        SwerveManager.rotateAndDrive(rotate, drive);

    }
}
