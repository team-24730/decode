package org.firstinspires.ftc.teamcode.rusteze;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    public Drivetrain drivetrain;
    public Intake intake;
    public Transfer transfer;
    public Outtake outtake;

    public Robot(HardwareMap hwMap) {
        drivetrain = new Drivetrain(hwMap);
        intake = new Intake(hwMap);
        transfer = new Transfer(hwMap);
        outtake = new Outtake(hwMap);
    }
}
