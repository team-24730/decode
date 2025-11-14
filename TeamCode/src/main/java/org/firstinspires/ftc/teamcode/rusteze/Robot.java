package org.firstinspires.ftc.teamcode.rusteze;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    public Drivetrain drivetrain;
    public Intake intake;
    public Transfer transfer;
    public Outtake outtake;

    public boolean intakeRunning = false;

    long lastIntakeTime = 0;
    boolean turnShooterOffFlag = false;

    public Robot(HardwareMap hwMap) {
        drivetrain = new Drivetrain(hwMap);
        intake = new Intake(hwMap);
        transfer = new Transfer(hwMap);
        outtake = new Outtake(hwMap);
    }

    public void update() {
        if (intakeRunning) {
            intake.setPower(1);
            transfer.setPower(0.3);
            lastIntakeTime = System.currentTimeMillis();
        } else if (System.currentTimeMillis() - lastIntakeTime < 500) {
            intake.disable();
            transfer.setPower(-0.3);
            outtake.setPower(-0.35);
            turnShooterOffFlag = true;
        } else {
            intake.disable();
            transfer.disable();
            if (turnShooterOffFlag) {
                outtake.setFlywheelTarget(0);
                outtake.setPower(0);
                turnShooterOffFlag = false;
            }
        }


    }
}
