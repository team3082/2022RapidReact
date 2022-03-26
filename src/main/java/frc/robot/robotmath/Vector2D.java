package frc.robot.robotmath;

public class Vector2D {
    public static final Vector2D kZero = new Vector2D();
    
    public double x;
    public double y; 
    
    public Vector2D() {
        x = 0;
        y = 0;
    }

    public Vector2D(double x, double y){
        this.x = x;
        this.y = y;
    }
    /**
    * Add another vector to this one
    * @param rhs The vector on the right hand side of the equation
    */
    public Vector2D add(Vector2D rhs){
        return new Vector2D(x + rhs.x, y + rhs.y);
    }
    /**
    * Subtract another vector from this one
    * @param rhs The vector on the right hand side of the equation
    */
    public Vector2D sub(Vector2D rhs){
        return new Vector2D(x - rhs.x, y - rhs.y);
    }
    /**
    * Multiply this vector by a scalar
    * @param scale The scalar to multiply the vector by
    */
    public Vector2D mul(double scale){
        return new Vector2D(x * scale, y * scale);
    }
    /**
    * Divide this vector by a scalar
    * @param scale The scalar to divide the vector by
    */
    public Vector2D div(double scale){
        return new Vector2D(x / scale, y / scale);
    }
    /**
    * Calculate the magnitude of this vector
    */
    public double mag(){
        return Math.hypot(x, y);
    }
    /**
    * Calculate a normalized version of this vector
    */
    public Vector2D norm(){
        // x/=mag();
        // y/= mag();
        double m = mag();
        return div(m);
    }

    /**
    * Calculate a rotated version of this vector using a rotation matrix
    @param angle The angle, in radians, by which to rotate the vector
    */
    public Vector2D rotate(double angle){
        double rotateX = x *  Math.cos(angle) + y * Math.sin(angle);
        double rotateY = x * -Math.sin(angle) + y * Math.cos(angle);
        return new Vector2D(rotateX, rotateY);
    }

    /**
    * Calculate the angle of the vector using Math.atan2
    */
    public double atan(){
        return Math.atan2(y, x);
    }

}
