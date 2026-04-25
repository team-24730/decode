package org.firstinspires.ftc.teamcode.rusteze.utility;

public class RegressionContainer {
    public static double getFlywheelRPM(double distance) {

        if (distance < 0) {
            return 2433;
        } else if (distance > 240) {
            return 6000;
        } else {
            return (14.70231 * distance) + (2433.47347);
        }
    }

    public static double getHoodPosition(double distance) {
        if (distance <= 30.9) {
            return 0.97;
        } else if (distance >= 98.3) {
            return 0.25;
        } else {
            return (0.000149593 * Math.pow(distance, 2))  +  (-0.029811 * distance)  +  (1.73453);
        }
    }
}