package frc.robot.auto;

import frc.robot.robotmath.RTime;
import frc.robot.robotmath.Vector2D;
import frc.robot.subsystems.AutoAlign;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveManager;
import frc.robot.subsystems.SwervePosition;



public class NotBasicAuto {

    public static AutoFrame[] frames;
    public static int current_frame = 0;
    public static double[] start = new double[4];

    public void update() {
        if (frames[current_frame].update()) {
            frames[++current_frame].start();
        }
    }

    interface AutoFrame {
        public void start();
        public boolean update();//returns true moving to next instruction
    }
    class Move implements AutoFrame{

        Vector2D move;
        double movedistance;

        public Move(double movex, double movey, double distance){
            this.move.x = movex;
            this.move.y = movey;
            this.movedistance = distance; // do unit conversion from meters to ticks here
        }
        public void start(){
            for(int i = 0; i < 4; i++){
                start[i] = SwerveManager.getEncoderPos(i); 
            }
        }
        public boolean update(){
            double avgDist = 0;
            for(int i = 0; i < 4; i++){
                avgDist += Math.abs(start[i]-SwerveManager.getEncoderPos(i));
            }
            avgDist /= 4;
            if(avgDist >= this.movedistance){
                return true;
            } 
            return false;
        }
    }
    class Rotate implements AutoFrame{

        double targetAngle;

        public Rotate(double angle){
            this.targetAngle = angle;
        }
        public void start(){
            Pigeon.setTargetAngle(this.targetAngle);
        }
        public boolean update(){
            if(Pigeon.atSetpoint()){
                return true;
            }
            return false;

        }
    }
}

