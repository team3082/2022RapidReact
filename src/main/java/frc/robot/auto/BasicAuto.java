package frc.robot.auto;

import frc.robot.Robot;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.SwerveManager;

public class BasicAuto {
    
    // INSTruction
    enum INST 
    {
        MOVE(),
        ROTATE(),
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

        private AutoFrame() {} 

        INST instruction;
        
        double movedistance;
        double movex;
        double movey;

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
        }
    } 

}
