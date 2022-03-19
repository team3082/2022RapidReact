package frc.robot.subsystems;

public class SwervePosition {

    public static double[] previousDriveDistance;
    public static double[] previousDriveDistanceVelocity; 
    public static double xPosition = 0, yPosition = 0;
    public static double lastXVel = 0, lastYVel = 0;

    public static void init() {
        previousDriveDistance = new double[]{0, 0, 0, 0};
        previousDriveDistanceVelocity = new double[]{0, 0, 0, 0};
        xPosition = 0;
        yPosition = 0;
        lastXVel = 0;
        lastYVel = 0;
    }

    public static void coordinatePositionManual(){
        for (int i = 0; i < 4; i++) {
            double currentDriveMotorDistance = SwerveManager.getManualDistance(i);
            double driveMotor0Change = currentDriveMotorDistance - previousDriveDistance[i];
            previousDriveDistance[i] = currentDriveMotorDistance;
            xPosition += Math.sin(SwerveManager.getAngle(i))*driveMotor0Change;
            yPosition += Math.cos(SwerveManager.getAngle(i))*driveMotor0Change;
        }
    }

    /*public static void coordinatePositionVelocity(){
        for (int i = 0; i < 4; i++) {
            double driveMotorDistance = SwerveManager.getVelocityDistance(i);
            double number = (driveMotorDistance+previousDriveDistanceVelocity[i])/2*0.02;
            xPosition += Math.sin(SwerveManager.getAngle(i))*number*0.02;
            yPosition += Math.cos(SwerveManager.getAngle(i))*number*0.02;
            previousDriveDistanceVelocity[i] = driveMotorDistance;
        }
    }
*/


    public static double[] absVel(){
        double yVel = 0;
        double xVel = 0;
        for (int i = 0; i < 4; i++) {
            double driveMotorSpeed = SwerveManager.getVelocityDistance(i);
            double angle = SwerveManager.getAngle(i);
            xVel += Math.sin(angle)*driveMotorSpeed;
            yVel += Math.cos(angle)*driveMotorSpeed;
        }

        xVel /= 4.0;
        yVel /= 4.0;

        double heading = Pigeon.getPitch()*Math.PI/180; 

        double xVelAbs = xVel *  Math.cos(heading) + yVel * Math.sin(heading);
        double yVelAbs = xVel * -Math.sin(heading) + yVel * Math.cos(heading);

        return new double[]{xVelAbs, yVelAbs};
    }

    public static double[] velocityPosition() {
        double[] array = absVel();
        double xVel = array[0];
        double yVel = array[1];

        xPosition += (xVel+lastXVel)/2*0.02;
        yPosition += (yVel+lastYVel)/2*0.02;

        lastXVel = xVel;
        lastYVel = yVel;
        
        return new double[]{xPosition, yPosition};
    }

    //double relMoveX = moveX *  Math.cos(heading) + moveY * Math.sin(heading);
    //double relMoveY = moveX * -Math.sin(heading) + moveY * Math.cos(heading);

    //returns array of the robot's angle and distance in INCHES based of manual calculations
    public static double[] angleDistancePositionManual(){
        
        double[] pos = new double[]{xPosition, yPosition};
        
        double distance = Math.sqrt(Math.pow(pos[0], 2) + Math.pow(pos[1], 2));
        double angle = Math.atan(pos[1]/pos[0])*(180/Math.PI);

        return new double[]{angle, distance};
        
    }

    //returns array of the robot's angle and distance in INCHES based of of sensor velocity
    public static double[] angleDistancePositionVelocity(){
        
        double[] pos = new double[]{xPosition, yPosition};
        
        double distance = Math.sqrt(Math.pow(pos[0], 2) + Math.pow(pos[1], 2));
        double angle = Math.atan(pos[1]/pos[0])*(180/Math.PI);

        return new double[]{angle, distance};
        
    }

}