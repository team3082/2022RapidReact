package frc.robot;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import frc.controllermaps.GuitarHero;
import frc.controllermaps.LogitechF310;
import frc.controllermaps.PS3;
import frc.robot.robotmath.RMath;
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

    static final int kPigeonZero    = LogitechF310.BUTTON_Y;
    static final int kShooterRev    = LogitechF310.BUTTON_RIGHT_BUMPER;
    static final int kShooterFire   = LogitechF310.BUTTON_LEFT_BUMPER;
    static final int kShooterManual = LogitechF310.BUTTON_X;
    static final int kIntakePull    = LogitechF310.AXIS_RIGHT_TRIGGER;
    static final int kEject         = LogitechF310.BUTTON_B;

    static Joystick m_operatorStick;

    // True: Logitech, False: PS3
    static final boolean useLogitechOperator = true;

    // LOGITECH OPERATOR
    static final int kLBigHook    = LogitechF310.AXIS_RIGHT_Y;
    static final int kLTiltHook   = LogitechF310.AXIS_LEFT_Y; 
    static final int kLStartClimb = LogitechF310.BUTTON_START;
    static final int kLStopClimb  = LogitechF310.BUTTON_BACK;
    static final int kLZeroClimber= LogitechF310.BUTTON_Y;

    //PS3 OPERATOR
    static final int kPBigHook    = PS3.AXIS_RIGHT_Y;
    static final int kPTiltHook   = PS3.AXIS_LEFT_Y; 
    static final int kPStartClimb = PS3.BUTTON_START;
    static final int kPStopClimb  = PS3.BUTTON_SELECT;
    static final int kPZeroClimber= PS3.BUTTON_TRIANGLE;

    //GUITAR HERO OPERATOR
    // static final int kGBigHookUp   = GuitarHero.BUTTON_TOP_G;
    // static final int kGBigHookDown = GuitarHero.BUTTON_TOP_R;
    // static final int kGTiltHookOut = GuitarHero.BUTTON_TOP_Y;
    // static final int kGTiltHookIn  = GuitarHero.BUTTON_TOP_B; 
    // static final int kGHookPower   = GuitarHero.AXIS_WHAMMY_BAR;
    // static final int kGStartClimb  = GuitarHero.BUTTON_START;
    // static final int kGStopClimb   = GuitarHero.BUTTON_BACK;
    // static final int kGZeroClimber = GuitarHero.BUTTON_TOP_O;

    
    public static void init(){
        m_driverStick = new Joystick(0);
        m_operatorStick = new Joystick(useLogitechOperator?1:2);
        NetworkTableInstance.getDefault().getTable("help").getEntry("dist").setDouble(0);
    }

    public static void joystickInput(){

        //If climbing stop everything.
        // Operator: Stops climb
        if(Climber.isClimbing()){
            if(m_operatorStick.getRawButton(useLogitechOperator ? kLStopClimb : kPStopClimb))
                Climber.stopClimb();
            else
                Climber.climb();
                return;
        }
        // Operator: Starts climb
        if(m_operatorStick.getRawButton(useLogitechOperator ? kLStartClimb : kPStartClimb)){
            Climber.startClimb();
            Climber.climb();
            return;
        }

        if(m_operatorStick.getRawButton(2)){
            Climber.startHighBarClimb();
            Climber.climb();
            return;
        }
        
        if(m_operatorStick.getRawButton(useLogitechOperator ? kLZeroClimber : kPZeroClimber)){
          Climber.zero();
        }

        // Operator: Controls big hook
        if(m_operatorStick.getRawButton(1)){
            Climber.gotoStart();
        } else if(Math.abs(m_operatorStick.getRawAxis(useLogitechOperator ? kLBigHook : kPBigHook)) > 0.05)
            Climber.setHook(m_operatorStick.getRawAxis(useLogitechOperator ? kLBigHook : kPBigHook));
        else
            Climber.setHook(0);

        //Operator: Controls tilting hook
        if(Math.abs(m_operatorStick.getRawAxis(useLogitechOperator ? kLTiltHook : kPTiltHook)) > 0.05)
            Climber.setScrew(m_operatorStick.getRawAxis(useLogitechOperator ? kLTiltHook : kPTiltHook));
        else
            Climber.setScrew(0);

        /*
        if (useLogitechOperator) {
            // LOGITECH OPERATOR
            // Operator: Controls big hook
            if(Math.abs(m_operatorStick.getRawAxis(kLBigHook)) > 0.05)
                Climber.setHook(m_operatorStick.getRawAxis(kLBigHook));
            else
                Climber.setHook(0);

            //Operator: Controls tilting hook
            if(Math.abs(m_operatorStick.getRawAxis(kLTiltHook)) > 0.05)
                Climber.setScrew(m_operatorStick.getRawAxis(kLTiltHook));
            else
                Climber.setScrew(0);
        } else {
            // GUITAR HERO OPERATOR
            double bigHookAxis = (m_operatorStick.getRawButton(kGBigHookUp) ? 1 : 0) - (m_operatorStick.getRawButton(kGBigHookDown) ? 1 : 0);
            double tiltHookAxis = (m_operatorStick.getRawButton(kGTiltHookOut) ? 1 : 0) - (m_operatorStick.getRawButton(kGTiltHookIn) ? 1 : 0);
            bigHookAxis *= (m_operatorStick.getRawAxis(kGHookPower) + 1.0) / 2.0;
            tiltHookAxis *= (m_operatorStick.getRawAxis(kGHookPower) + 1.0) / 2.0;

            // Operator: Controls big hook
            if(Math.abs(bigHookAxis) > 0.05)
                Climber.setHook(bigHookAxis);
            else
                Climber.setHook(0);

            // Operator: Controls tilting hook
            if(Math.abs(tiltHookAxis) > 0.05)
                Climber.setScrew(tiltHookAxis);
            else
                Climber.setScrew(0);
        }
        */

        // Driver: Drives
        Vector2D drive = RMath.smoothJoystick2(m_driverStick.getRawAxis(kMoveX), -m_driverStick.getRawAxis(kMoveY));
        // Driver: Boosts
        double boost = m_driverStick.getRawAxis(kBoost);
        final double defaultSpeed = 0.5;
        boost = boost * (1.0 - defaultSpeed) + defaultSpeed;

        // While shooting, slow us waaay down
        if(Shooter.firing())
            boost *= 0.1 / 0.4;

        // Joystick deadzone for drive
        if(drive.mag() < 0.01)
            drive = new Vector2D(0, 0); 
        else
            drive = drive.mul(boost);

        //Driver: Rotates
        double rotate = -m_driverStick.getRawAxis(kRotateX);
        rotate = RMath.smoothJoystick1(rotate) * 0.3;

        //Joystick deadzone for rotation
        if(Math.abs(rotate) < 0.01)
            rotate = 0;

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
            final double manualFireRPM = 2000.0;

            // Always use odometry for auto firing.
            // Odomotry gets instantly updated when the camera sees the hub,
            //  so this should be the same as using vision except it should work when we can't see the hub

            // We only ever want to be aligning if we're in auto fire mode 
            if(shooterAutoFire && !manualFire) {

                //double heading = Pigeon.getRotation() / 180.0 * Math.PI;
                //Vector2D robotPos = SwervePosition.getPosition();
                //Vector2D shooterOffset = new Vector2D(Math.cos(heading), Math.sin(heading)).mul(12 + 5/8);
                //Vector2D shooterPos = robotPos;//robotPos.add(shooterOffset);
                
                // Multiply by -1 so that we're pointing towards 0
                //Vector2D dir = shooterPos.mul(-1).norm();
                AutoAlign.setAngle();

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
                //Shooter.setRPMForDist(NetworkTableInstance.getDefault().getTable("help").getEntry("dist").getDouble(0));
                //Shooter.setRPMForDist(SwervePosition.getPosition().mag() / 12.0);
                Shooter.revTo(manualFireRPM);
                if(Pigeon.atSetpoint())
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
