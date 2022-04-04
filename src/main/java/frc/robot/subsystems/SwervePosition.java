package frc.robot.subsystems;

import frc.robot.robotmath.RTime;
import frc.robot.robotmath.Vector2D;

public class SwervePosition {

    private static Vector2D position;
    private static Vector2D localVelocity;
    private static Vector2D absVelocity;
    private static Vector2D lastAbsVelocity;
    
    private static Vector2D positionOffset;

    public static void init() {
        localVelocity   = new Vector2D();
        absVelocity     = new Vector2D();
        lastAbsVelocity = new Vector2D();
        position        = new Vector2D();
        positionOffset  = new Vector2D();
    }

    public static void update() {

        // Derive our velocity 
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

        localVelocity = vel;

        // Rotate our velocity to be local to the field
        vel = vel.rotate(heading);

        vel.x *= -1;

        lastAbsVelocity = absVelocity; 
        absVelocity = vel;


        // Integrate our velocity to find our position
        position = position.add(absVelocity.add(lastAbsVelocity).mul(0.5*RTime.getDeltaTime()));

    }


    //returns array of the robot's angle and distance in INCHES based of manual calculations
    public static double[] getPositionPolar(){
        
        Vector2D pos = getPosition();
        double distance = pos.mag();
        double angle = pos.atan()*(180/Math.PI);

        return new double[]{angle, distance};
    }

    public static Vector2D getAbsPosition(){
        return position;
    }

    public static Vector2D getPosition(){
        return position.add(positionOffset);
    }

    public static Vector2D getAbsVelocity() {
        return absVelocity;
    }

    public static void updateOffsetForPosition(Vector2D newnPosition){
        positionOffset = newnPosition.sub(position);
    }
    //returns array of the robot's angle and distance in INCHES based of of sensor velocity
    // public static double[] angleDistancePositionVelocity(){
        
    //     double[] pos = new double[]{xPosition, yPosition};
        
    //     double distance = Math.sqrt(Math.pow(pos[0], 2) + Math.pow(pos[1], 2));
    //     double angle = Math.atan(pos[1]/pos[0])*(180/Math.PI);

    //     return new double[]{angle, distance};
        
    // }

}