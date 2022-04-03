package frc.robot.auto;

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

    public static void 3ballTohuman() {
        Pigeon.setYaw(-90);
        SwervePosition.positionInt = new Vector2D(-86, -32);
        instructions = new AutoFrame[]
        {
            // Intake
            AutoFrame.LookAt(0, 0),
            AutoFrame.MoveToAndLookAt(36-117, 56-15),
            AutoFrame.Rotate(135),
            //AutoFrame.BeginAutoAlign(),
            AutoFrame.MoveTo(36-117-42, 56-15+154),

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
                break;
            case SHOOT:
                Shooter.setShooterRPM(instructions[index].rpm);
                break;
            case MOVETOCOORD:
                break;
            case MOVETOANDLOOKAT:
                break;
        }
    } 
}
