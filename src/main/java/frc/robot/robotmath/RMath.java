package frc.robot.robotmath;

public class RMath {

    public static double angleDiff(double current_angle, double target_angle, double angle_range) {

        target_angle = target_angle % 360;
        if(target_angle < 0)
            target_angle += 360;

        // The number of full rotations the bot has made
        int numRot = (int) Math.floor(current_angle / angle_range);

        // The target pigeon angle
        double target = numRot * angle_range + target_angle;
        double targetPlus = target + angle_range;
        double targetMinus = target - angle_range;

        // The true destination for the bot to rotate to
        double destination;

        // Determine if, based on the current angle, it should stay in the same
        // rotation, enter the next, or return to the previous.
        if (Math.abs(target - current_angle) <= Math.abs(targetPlus - current_angle)
         && Math.abs(target - current_angle) <= Math.abs(targetMinus - current_angle)) {
            destination = target;
        } else if (Math.abs(targetPlus - current_angle) < Math.abs(targetMinus - current_angle)) {
            destination = targetPlus;
        } else {
            destination = targetMinus;
        }

        return destination;
    }

}
