package frc.robot.subsystems;

import frc.robot.robotmath.RTime;
import frc.robot.robotmath.Vector2D;

public class SwervePosition {

    public static double[] previousDrivePosition;
    public static Vector2D positionInt;
    public static Vector2D positionAcc;
    public static Vector2D absVelocity;
    public static Vector2D lastVelocity;
    
    public static void init() {
        previousDrivePosition = new double[]{0, 0, 0, 0};
        positionInt  = new Vector2D();
        positionAcc  = new Vector2D();
        absVelocity  = new Vector2D();
        lastVelocity = new Vector2D();
    }

    public static void updatePositionAccumulation(){
        Vector2D deltapos = new Vector2D();

        for (int i = 0; i < 4; i++) {
            double angle = SwerveManager.getSteerAngle(i);
            double currentDriveMotorDistance = SwerveManager.getDrivePosition(i);
            double driveMotor0Change = currentDriveMotorDistance - previousDrivePosition[i];
            previousDrivePosition[i] = currentDriveMotorDistance;
            deltapos.x += Math.sin(angle)*driveMotor0Change;
            deltapos.y += Math.cos(angle)*driveMotor0Change;
        }

        // 4 Swerve modules, so divide by 4
        deltapos = deltapos.div(4);

        // Rotate the change in position to be local to the field
        double heading = Pigeon.getRotation() * Math.PI / 180.0;
        deltapos = deltapos.rotate(heading);

        // Add our new movement to our current position 
        positionAcc = positionAcc.add(deltapos);
    }



    public static void updateVelocity() {
        Vector2D vel = new Vector2D(0,0);
        for (int i = 0; i < 4; i++) {
            double driveMotorSpeed = SwerveManager.getDriveVelocity(i);
            double angle = SwerveManager.getSteerAngle(i);
            vel.x += Math.sin(angle)*driveMotorSpeed;
            vel.y += Math.cos(angle)*driveMotorSpeed;
        }

        vel.x /= 4.0;
        vel.y /= 4.0;

        double heading = Pigeon.getRotation()*Math.PI/180; 

        // Rotate our velocity to be local to the field
        vel = vel.rotate(heading);

        vel.x *= -1;

        lastVelocity = absVelocity; 
        absVelocity = vel;
    }

    public static void updatePositionIntegration() {
        positionInt = positionInt.add(absVelocity.add(lastVelocity).mul(0.5*RTime.getDeltaTime()));
    }


    //returns array of the robot's angle and distance in INCHES based of manual calculations
    public static double[] getPositionPolar(){
        
        Vector2D pos = getPosition();
        double distance = pos.mag();
        double angle = pos.atan()*(180/Math.PI);

        return new double[]{angle, distance};
    }

    public static Vector2D getPosition(){
        return positionInt;
    }

    //returns array of the robot's angle and distance in INCHES based of of sensor velocity
    // public static double[] angleDistancePositionVelocity(){
        
    //     double[] pos = new double[]{xPosition, yPosition};
        
    //     double distance = Math.sqrt(Math.pow(pos[0], 2) + Math.pow(pos[1], 2));
    //     double angle = Math.atan(pos[1]/pos[0])*(180/Math.PI);

    //     return new double[]{angle, distance};
        
    // }

}