package org.firstinspires.ftc.teamcode.rusteze;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConstants {
    public static int TARGET_RPM = 0;
    public static double K_V = 0.0002;
    public static double K_P = 0.001;
    public static double TARGET_HOOD = 0.25;

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
