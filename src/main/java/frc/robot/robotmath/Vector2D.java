package frc.robot.robotmath;

import edu.wpi.first.math.Vector;
import frc.robot.subsystems.SwerveManager;

public class Vector2D {
    public double x;
    public double y; 
    
    public Vector2D(double x, double y){
        this.x = x;
        this.y = y;
    }
   
    public Vector2D add(Vector2D rhs){
        return new Vector2D(x + rhs.x, y + rhs.y);
    }
    public Vector2D sub(Vector2D rhs){
        return new Vector2D(x - rhs.x, y - rhs.y);
    }
    public Vector2D mul(double scale){
        return new Vector2D(x * scale, y * scale);
    }
    public Vector2D div(double scale){
        return new Vector2D(x / scale, y / scale);
    }
    public double mag(){
        return Math.hypot(x, y);
    }
    public Vector2D norm(){
        // x/=mag();
        // y/= mag();
       
        return new Vector2D(x/mag(), y/mag());
    }

    public Vector2D rotate(double angle){
        double rotateY = x * -Math.sin(angle) + y * Math.cos(angle);
        double rotateX = x *  Math.cos(angle) + y * Math.sin(angle);
        return new Vector2D(rotateX, rotateY);
    }
   
    

}
