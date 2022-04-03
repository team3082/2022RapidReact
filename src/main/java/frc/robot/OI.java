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
    static Joystick m_driverJoystick;
    static Joystick m_operatorJoystick;
    static NetworkTable m_nt;

    public static void init(){
        m_nt = NetworkTableInstance.getDefault().getTable("shooter");
        m_nt.getEntry("target_dist").setDouble(0.0);
        m_driverJoystick = new Joystick(0);
        m_operatorJoystick = new Joystick(1);
        m_nt.getEntry("kp").setDouble(1.00);

    }

    public static void joystickInput(){

        // Driver: (L Stick) Drives
        Vector2D drive = new Vector2D(m_driverJoystick.getRawAxis(0), -m_driverJoystick.getRawAxis(1));

        // Driver: (L Trigger) Boosts Speed
        double boost = m_driverJoystick.getRawAxis(3);
        boost = boost * 0.7 + 0.3;

        drive = drive.mul(boost);
        if(drive.mag() < 0.1){
            drive = new Vector2D(0, 0); 
        }

        double rotate = 0;
        // Driver: (A) Auto Aligns
        if(!m_driverJoystick.getRawButton(1)){
            // Driver: (R Stick) Manually Rotates
            rotate = m_driverJoystick.getRawAxis(4);
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
        } else {
            AutoAlign.setAngle();
            rotate = Pigeon.correctTurnWithPID();
            //System.out.println(AutoAlign.getAngle());
        }

        SwerveManager.rotateAndDrive(rotate, drive);
        
        // Driver: (Y) Zeros Pigeon
        if(m_driverJoystick.getRawButton(4))
        {
            Pigeon.zero();
            Pigeon.setTargetAngle(0);
        }


        // Driver:   (R Bumper) Intakes
        // Operator: [R Bumper] Reverses Intake
        if(!m_operatorJoystick.getRawButton(6)){
            //Driver
            Intake.setEnabled(m_driverJoystick.getRawButton(6));
        } else {
            //Operator
            Intake.setSpeed(-0.75);
        }

        // Operator: [A, B, Y] Shoots based on presets
        //      A ~ 15ft
        //      B ~ 20ft
        //      Y ~ 25ft
        // Driver:   (L Bumper) Shoots based on vision
        // Operator: [L Bumper] Reverses Shooter & Handoff
        double kp = m_nt.getEntry("kp").getDouble(1.00);
        if (m_operatorJoystick.getRawButton(1)) {
            //Shoot for 10ft
            Shooter.setRPMForDist(10, kp);
            if(Shooter.atSetpoint())
                Shooter.setHandoffEnabled(true);
            
        } else if (m_operatorJoystick.getRawButton(2)) {
            //Shoot for 15ft
            Shooter.setRPMForDist(15, kp);
            if(Shooter.atSetpoint())
                Shooter.setHandoffEnabled(true);
        
        } else if (m_operatorJoystick.getRawButton(4)) {
            //Shoot for 20ft
            Shooter.setRPMForDist(20, kp);
            if(Shooter.atSetpoint())
                Shooter.setHandoffEnabled(true);
            
        } else if(m_driverJoystick.getRawButton(5)) {
            //Shoot Based on Vision Distance
            Shooter.setRPMForDist(AutoAlign.m_distAvg, kp);
            if(rotate == 0 && Shooter.atSetpoint())
                Shooter.setHandoffEnabled(true);
        
        } else if (m_operatorJoystick.getRawButton(5)){
            // Reverse
            Shooter.reverse();
        } else {
            Shooter.stopVelocityControl();
            Shooter.setHandoffEnabled(false);
        }
        //VALUES TO LOOK AT FOR TUNING SHOOTER
        m_nt.getEntry("target_rpm").setDouble(Shooter.m_targetSpeed * Shooter.kVelToRPM);
        m_nt.getEntry("current_rpm").setDouble(Shooter.m_flywheel.getSelectedSensorVelocity() * Shooter.kVelToRPM);
        m_nt.getEntry("at_setpoint").setBoolean(Shooter.atSetpoint());
    }
}
