package org.firstinspires.ftc.teamcode.rusteze;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConstants {
    public static int TARGET_RPM = 3690;
    public static double K_V = 0.0002; // 0.0002
    public static double K_P = 0.005; // 0.001
    public static double TARGET_HOOD = 0.4 ; // 0.25 - 0.97
    public enum Color {
        RED,
        BLUE
    }
    public static Color teamColor = Color.RED;
    public static boolean turretInitialized = false;

    public int getTargetRpm() {
        return TARGET_RPM;
    }
    public double getkV() {
        return K_V;
    }
    public double getkP() {
        return K_P;
    }
    public double getTargetHood() { return TARGET_HOOD; }
}
