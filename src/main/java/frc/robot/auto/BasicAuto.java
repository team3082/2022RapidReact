package frc.robot.auto;

import frc.robot.robotmath.RTime;
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
        MOVETOANDLOOKAT(),
        LOOKAT(),
        ROTATE(),
        INTAKE(),
        REVFORDIST(),
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
        static public AutoFrame MoveToAndLookAt(double x, double y)
        {
            AutoFrame frame = new AutoFrame();
            frame.instruction = INST.MOVETOANDLOOKAT;
            frame.move = new Vector2D(x,y);
            return frame;
        }        
        static public AutoFrame LookAt(double x, double y)
        {
            AutoFrame frame = new AutoFrame();
            frame.instruction = INST.LOOKAT;
            frame.move = new Vector2D(x,y);
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
        static public AutoFrame MoveTo(double x, double y)
        {
            AutoFrame frame = new AutoFrame();
            frame.instruction = INST.MOVETOCOORD;
            frame.move = new Vector2D(x,y);
            return frame;
        }
        static public AutoFrame Intake(boolean intakeOn)
        {
            AutoFrame frame = new AutoFrame();
            frame.instruction = INST.INTAKE;
            frame.intakeOn = intakeOn;
            return frame;
        }
        static public AutoFrame RevForDist(double dist)
        {
            AutoFrame frame = new AutoFrame();
            frame.instruction = INST.REVFORDIST;
            frame.dist = dist;
            return frame;
        }
        static public AutoFrame Shoot(){
            AutoFrame frame = new AutoFrame();
            frame.instruction = INST.SHOOT;
            frame.stopTime = Double.MAX_VALUE;
            return frame;
        }


        private AutoFrame() {} 

        INST instruction;

        double stopTime;
        
        double movedistance;
        Vector2D move;
        boolean intakeOn;
        double dist;

        double targetangle;
    }
    
    private static boolean isDone = false;
    private static double[] start = new double[4];

    private static AutoFrame[] instructions;
    private static int index;

    public static void init(){
        index = 0;
        isDone = false;
        threeBallTohuman();        
        beginInstruction();
    }

    public static void basic(){
        instructions = new AutoFrame[]
        {
            // Intake
            AutoFrame.MoveToAndLookAt(36, 56),
            AutoFrame.MoveToAndLookAt(36-117, 56-15),
            AutoFrame.Rotate(135),
            //AutoFrame.BeginAutoAlign(),
            AutoFrame.MoveTo(36-117-42, 56-15+154),

        };
    }

    public static void threeBallTohuman() {
        //NOT FINISHED!!!!
        Pigeon.setYaw(90);
        SwervePosition.setPosition(new Vector2D(-86, -32));
        instructions = new AutoFrame[]
        {
            AutoFrame.Intake(true),
            AutoFrame.MoveToAndLookAt(-133, -39),
            AutoFrame.Intake(false),
            AutoFrame.RevForDist(Math.hypot(-133, -39)),
            AutoFrame.LookAt(0, 0),
            AutoFrame.Shoot(),
            AutoFrame.RevForDist(Math.hypot(-133, -39)),
            AutoFrame.Shoot()

        };
    }

    public static void update(){
        if (isDone)
            return;
        double pow = 0;
        pow = Pigeon.correctTurnWithPID();
        Vector2D movement = new Vector2D(0,0);
        switch(instructions[index].instruction){
            case ROTATE:
                //if(pow == 0){
                    nextInstruction();
                //} else {
                    //SwerveManager.rotateAndDrive(pow, new Vector2D());
                //}
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
                }
                break;
            case MOVETOCOORD:
                //if(Math.hypot(instructions[index].move.x - SwervePosition.xPosition, instructions[index].move.y - SwervePosition.yPosition) < 12){
                if (instructions[index].move.sub(SwervePosition.getPosition()).mag() < 1.2) {
                    nextInstruction();
                }
                else {
                    Vector2D direction = instructions[index].move.sub(SwervePosition.getPosition()).norm();
                    movement = direction.mul(0.1);
                }
                break;
            case MOVETOANDLOOKAT:
                if (instructions[index].move.sub(SwervePosition.getPosition()).mag() < 1.2) {
                    nextInstruction();
                }
                else {
                    Vector2D direction = instructions[index].move.sub(SwervePosition.getPosition()).norm();
                    movement = direction.mul(0.1);
                    
                    Pigeon.setTargetAngle(direction.atanDeg());
                    //pow = -Pigeon.correctTurnWithPID();
                }
                break;
            case LOOKAT:
                Vector2D direction = instructions[index].move.sub(SwervePosition.getPosition()).norm();
                movement = new Vector2D(0,0);

                Pigeon.setTargetAngle(direction.atanDeg());
                //pow = -Pigeon.correctTurnWithPID();
                
                if(Pigeon.atSetpoint())
                    nextInstruction();
                break;
            case SHOOT:
                if(RTime.now() >= instructions[index].stopTime){
                    Shooter.stop();
                    nextInstruction();
                    return;
                }
                Shooter.fire();
                if(Shooter.atSetpoint()){
                    instructions[index].stopTime = RTime.now() + 0.5;   
                }
                break;
            default:
                break;
        }

        SwerveManager.rotateAndDrive(pow, movement);
    }

    private static void nextInstruction(){
        index++;
        System.out.println("next");
        if (instructions.length <= index){
            isDone = true;
            SwerveManager.rotateAndDrive(0, new Vector2D());
            return;
        }

        beginInstruction();
    }

    private static void beginInstruction() {
        
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
                nextInstruction();
                break;
            case REVFORDIST:
                Shooter.setRPMForDist(instructions[index].dist);
                nextInstruction();
                break;
            default:
                break;
        }
    } 
}
