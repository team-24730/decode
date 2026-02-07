package org.firstinspires.ftc.teamcode.rusteze;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Lift {
    public Servo leftLift;
    public Servo rightLift;
    public boolean extended;

    public Lift(HardwareMap hwMap) {
        leftLift = hwMap.get(Servo.class, "leftLift");
        rightLift = hwMap.get(Servo.class, "rightLift");
        extended = false;
    }

    public void retract() {
        //if (extended) {
            leftLift.setPosition(0.15);
            rightLift.setPosition(0.15);
            extended = false;
        //}
    }

    public void extend() {
        //if (!extended) {
            leftLift.setPosition(0.78);
            rightLift.setPosition(0.78);
            extended = true;
        //}
    }
}
