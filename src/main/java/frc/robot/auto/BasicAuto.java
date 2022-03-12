package frc.robot.auto;

import frc.robot.Robot;
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
            frame.movex = movex;
            frame.movey = movey;
            frame.movedistance = distance; // do unit conversion from meters to ticks here
            return frame;
        }
        static public AutoFrame MoveToCoord(double xDest, double yDest)
        {
            AutoFrame frame = new AutoFrame();
            frame.instruction = INST.MOVETOCOORD;
            frame.movex = xDest;
            frame.movey = yDest;
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
        double movex;
        double movey;
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
        instructions = new AutoFrame[]
        {
            AutoFrame.MoveToCoord(24, 24)
        };
    }

    public static void update(){
        if (isDone)
            return;
        switch(instructions[index].instruction){
            case ROTATE:
                double pow = Pigeon.correctTurnWithPID(Robot.kDefaultPeriod);
                if(pow == 0){
                    nextInstruction();
                } else {
                    SwerveManager.rotateAndDrive(pow, 0, 0);
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
                    SwerveManager.rotateAndDrive(0, instructions[index].movex, instructions[index].movey);
                }
                break;
            case MOVETOCOORD:
                if(Math.hypot(instructions[index].movex - SwervePosition.xPosition, instructions[index].movey - SwervePosition.yPosition) < 12){
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
            SwerveManager.rotateAndDrive(0, 0, 0);
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
                double moveX = instructions[index].movex - SwervePosition.xPosition;
                double moveY = instructions[index].movey - SwervePosition.yPosition;
                double hyp = Math.hypot(moveX, moveY);
                moveX /= hyp*3;
                moveY /= hyp*3;
                SwerveManager.rotateAndDrive(0, moveX, moveY);
                break;
        }
    } 
}
