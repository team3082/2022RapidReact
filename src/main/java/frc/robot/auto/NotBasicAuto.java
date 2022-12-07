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
    public static void backupAndShoot() {
        Pigeon.setYaw(0);
        SwervePosition.setPosition(new Vector2D(0,-92));
        frames = new AutoFrame[]{
            new RevForDist(0,-136),
            new MoveTo(0,-136),
            new SetIntake(true),
            new LookAt(0, 0),
            new AutoRev(0, -100),
            new Shoot(),
            new SetIntake(false)
        };
    }

    public static void twoBallLeft(){
        Pigeon.setYaw(140);
        SwervePosition.setPosition(new Vector2D(-44,-84));
        frames = new AutoFrame[]{
            new MoveTo(-32, -72),
            new Wait(1),
            new SetIntake(true),
            new RevForDist(-85,-130),
            new MoveToAndLookAt(-85,-130),//Ball at -85,-130
            new Wait(1),
            new SetIntake(false),
            new LookAt(0, 0),
            new AutoRev(-85,-130),
            new Shoot(),
            new Wait(1),
            new AutoRev(-85,-130),
            new Shoot()
        };
    }

    public static void threeBall() {
        //NOT FINISHED!!!!
        Pigeon.setYaw(-90);
        SwervePosition.setPosition(new Vector2D(86, -39));
        frames = new AutoFrame[]
        {
            new MoveTo(82, -39),
            new Wait(1),
            new SetIntake(true),
            //Ball at (150,-27)
            new MoveToAndLookAt(140, -39),
            new Wait(1),
            new SetIntake(false),
            new MoveTo(136, -140),
            //3rd ball at (87, -125)
            new RevForDist(120, -175),
            new MoveTo(120, -175),
            new LookAt(0, 0),
            new AutoRev(120, -175),
            new Shoot(),
            new Wait(1),
            new AutoRev(105, -158),
            new Shoot(),
            new SetIntake(true),
            new RevForDist(87, -125),
            new MoveTo(87, -125),
            new SetIntake(false),
            new LookAt(0, 0), 
            new AutoRev(87, -125),
            new Shoot(),
            //Human player at (118, -282)
            // new SetIntake(true),
            // new RevForDist(90, -275),
            // new MoveToAndLookAt(90, -275),
            // new LookAt(0,0),
            // new Shoot()

        };
    }


    public static AutoFrame[] frames;

    private static int current_frame = 0;
    private static double[] start = new double[4];

    private static boolean intakeOn;
    private static double rotSpeed;
    private static Vector2D movement;

    public void update() {
        rotSpeed = Pigeon.correctTurnWithPID();
        movement = new Vector2D(0,0);

        if (frames[current_frame].update()) {
            if (++current_frame == frames.length - 1)
                return;
            frames[current_frame].start();
        }

        Intake.setEnabled(intakeOn);
        SwerveManager.rotateAndDrive(rotSpeed, movement);
    }


    interface AutoFrame {
        public void start();
        public boolean update();//returns true moving to next instruction
    }


    static class Move implements AutoFrame {
        Vector2D move;
        double movedistance;

        public Move(double movex, double movey, double distance) {
            this.move.x = movex;
            this.move.y = movey;
            this.movedistance = distance; // do unit conversion from meters to ticks here
        }
        public void start() {
            for(int i = 0; i < 4; i++){
                start[i] = SwerveManager.getEncoderPos(i); 
            }
        }
        public boolean update() {
            double avgDist = 0;
            for(int i = 0; i < 4; i++) {
                avgDist += Math.abs(start[i] - SwerveManager.getEncoderPos(i));
            }
            avgDist /= 4;
            if(avgDist >= this.movedistance) {
                return true;
            } 
            return false;
        }
    }

    static class Rotate implements AutoFrame {
        double targetAngle;

        public Rotate(double angle) {
            this.targetAngle = angle;
        }
        public void start() {
            Pigeon.setTargetAngle(this.targetAngle);
        }
        public boolean update() {
            if(Pigeon.atSetpoint()) {
                return true;
            }
            return false;
        }
    }

    static class LookAt extends Rotate {
        public LookAt(double x, double y) {
            super(new Vector2D(x, y).sub(SwervePosition.getPosition()).norm().atanDeg());
        }
    }

    static class MoveTo implements AutoFrame {
        Vector2D move;

        public MoveTo(double x, double y) {
            this.move = new Vector2D(x, y);
        }
        public void start() {}
        public boolean update() {
            //if(Math.hypot(instructions[index].move.x - SwervePosition.xPosition, instructions[index].move.y - SwervePosition.yPosition) < 12){
            if (move.sub(SwervePosition.getPosition()).mag() < 1.2) {
                return true;
            }
            else {
                Vector2D direction = move.sub(SwervePosition.getPosition()).norm();
                movement = direction.mul(0.3);
            }
            return false;
        }
    }
    
    static class MoveToAndLookAt extends MoveTo {
        public MoveToAndLookAt(double x, double y) {
            super(x, y);
        }
        @Override
        public void start() {
            Vector2D direction = move.sub(SwervePosition.getPosition()).norm();
            Pigeon.setTargetAngle(direction.atanDeg());
        }
    }

    static class SetIntake implements AutoFrame {
        boolean on;

        public SetIntake(boolean on) {
            this.on = on;
        }
        public void start() {
            intakeOn = on;
        }
        public boolean update() {
            return true;
        }
    }

    static class RevForDist implements AutoFrame {
        double dist;

        public RevForDist(double x, double y) {
            this.dist = Math.hypot(x, y) / 12.0;
        }
        public void start() {
            Shooter.setRPMForDist(this.dist);
        }
        public boolean update() {
            return true;
        }
    }

    static class AutoRev extends RevForDist {
        public AutoRev(double x, double y) {
            super(x, y);
        }
        @Override
        public void start() {
            if (AutoAlign.m_hubSeen) 
                Shooter.setRPMForDist(AutoAlign.m_distAvg);
            else
                Shooter.setRPMForDist(this.dist);
        }
    }

    static class Shoot implements AutoFrame {
        double stopTime = Double.MAX_VALUE;

        public void start() {}
        public boolean update() {
            if(this.stopTime == Double.MAX_VALUE && Shooter.atSetpoint()){
                this.stopTime = RTime.now() + 1;
            }
            if(RTime.now() >= this.stopTime){
                Shooter.stop();
                return true;
            }
            Shooter.fire();
            return false;
        }
    }

    static class DoAutoAlign implements AutoFrame {
        boolean aaHubSeen = false;

        public void start() {}
        public boolean update() {
            if (AutoAlign.m_hubSeen && !this.aaHubSeen) {
                this.aaHubSeen = true;
                AutoAlign.setAngle();
            }
            if (this.aaHubSeen && Pigeon.atSetpoint())
                return true;
            
            return false;
        }
    }

    static class Wait implements AutoFrame {
        double seconds;
        double stopTime;

        public Wait(double seconds) {
            this.seconds = seconds;
        }
        public void start() {
            this.stopTime = RTime.now() + stopTime;
        }
        public boolean update() {
            if (RTime.now() > this.stopTime)
                return true;
            return false;
        }
    }
}

