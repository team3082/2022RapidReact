package frc.robot.auto;

import frc.robot.Robot;
import frc.robot.robotmath.Vector2D;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveManager;
import frc.robot.subsystems.SwervePosition;

public class BasicAuto {
    
    // INSTruction
    enum INST 
    {
        MOVE(),
        MOVETOCOORD(),
        ROTATE(),
        INTAKE(),
        SHOOT(),
    }

    public static class AutoFrame
    {
        static public AutoFrame Rotate(double angle)
        {
            AutoFrame frame = new AutoFrame();
            frame.instruction = INST.ROTATE;
            frame.targetangle = angle;
            return frame;
        }
        
        static public AutoFrame Move(double movex, double movey, double distance)
        {
            AutoFrame frame = new AutoFrame();
            frame.instruction = INST.MOVE;
            frame.move.x = movex;
            frame.move.y = movey;
            frame.movedistance = distance; // do unit conversion from meters to ticks here
            return frame;
        }
        static public AutoFrame MoveToCoord(Vector2D dest)
        {
            AutoFrame frame = new AutoFrame();
            frame.instruction = INST.MOVETOCOORD;
            frame.move = dest;
            return frame;
        }
        static public AutoFrame Intake(boolean intakeOn)
        {
            AutoFrame frame = new AutoFrame();
            frame.instruction = INST.INTAKE;
            frame.intakeOn = intakeOn;
            return frame;
        }
        static public AutoFrame Shoot(double rpm)
        {
            AutoFrame frame = new AutoFrame();
            frame.instruction = INST.SHOOT;
            frame.rpm = rpm;
            return frame;
        }


        private AutoFrame() {} 

        INST instruction;
        
        double movedistance;
        Vector2D move;
        boolean intakeOn;
        double rpm;

        double targetangle;
    }
    
    private static boolean isDone = false;
    private static double[] start = new double[4];

    private static AutoFrame[] instructions;
    private static int index;

    public static void init(){
        index = 0;
        isDone = false;
        instructions = new AutoFrame[]
        {
            AutoFrame.MoveToCoord(new Vector2D(24, 24))
        };
    }

    public static void update(){
        if (isDone)
            return;
        switch(instructions[index].instruction){
            case ROTATE:
                double pow = -Pigeon.correctTurnWithPID(Robot.kDefaultPeriod);
                if(pow == 0){
                    nextInstruction();
                } else {
                    SwerveManager.rotateAndDrive(pow, Vector2D.kZero);
                }
                break;
            case MOVE:
                double avgDist = 0;
                for(int i = 0; i < 4; i++){
                    avgDist += Math.abs(start[i]-SwerveManager.getEncoderPos(i));
                }
                avgDist /= 4;
                if(avgDist >= instructions[index].movedistance){
                    nextInstruction();
                } else {
                    SwerveManager.rotateAndDrive(0, instructions[index].move);
                }
                break;
            case MOVETOCOORD:
                //if(Math.hypot(instructions[index].move.x - SwervePosition.xPosition, instructions[index].move.y - SwervePosition.yPosition) < 12){
                if (instructions[index].move.sub(SwervePosition.position).mag() < 12) {
                    nextInstruction();
                }
                break;
        }
    }

    private static void nextInstruction(){
        index++;
        System.out.println("next");
        if (instructions.length <= index){
            isDone = true;
            SwerveManager.rotateAndDrive(0, Vector2D.kZero);
            return;
        }
        switch(instructions[index].instruction){
            case ROTATE:
                Pigeon.setTargetAngle(instructions[index].targetangle);
                
                break;
            case MOVE:
                for(int i = 0; i < 4; i++){
                    start[i] = SwerveManager.getEncoderPos(i); 
                }
                break;
            case INTAKE:
               Intake.setEnabled(instructions[index].intakeOn); 
               break;
            case SHOOT:
                Shooter.setShooterRPM(instructions[index].rpm);
               break;
            case MOVETOCOORD:
                Vector2D direction = instructions[index].move.sub(SwervePosition.position).norm();
                Vector2D move = direction.div(3.0);
                SwerveManager.rotateAndDrive(0, move);
                break;
        }
    } 
}
