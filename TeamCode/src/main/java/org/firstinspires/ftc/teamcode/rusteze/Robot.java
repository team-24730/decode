package org.firstinspires.ftc.teamcode.rusteze;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    public Drivetrain drivetrain;
    public Intake intake;
    public Transfer transfer;
    public Outtake outtake;

    public boolean intakeRunning = false;
    public boolean dontTurnOffTransfer = false;

    long lastIntakeTime = 0;
    boolean turnShooterOffFlag = false;

    public Robot(HardwareMap hwMap) {
        drivetrain = new Drivetrain(hwMap);
        intake = new Intake(hwMap);
        transfer = new Transfer(hwMap);
        outtake = new Outtake(hwMap);
    }

    public void setOuttakeTarget(double targetRPM) {
        outtake.setTarget(targetRPM);
        outtake.setPower(0);
        outtake.useControlSystem = true;
    }

    public void setOuttakeRawPower(double power) {
        outtake.setPower(power);
        outtake.setTarget(0);
        outtake.useControlSystem = false;
    }

    public void update() {

        // Intake logic
        if (intakeRunning) {
            intake.setPower(1);
            transfer.setPower(0.6);
            lastIntakeTime = System.currentTimeMillis();
        } else if (System.currentTimeMillis() - lastIntakeTime < 500) {
            intake.setPower(-0.1);
            transfer.setPower(-0.5);
            setOuttakeRawPower(-0.35);
            turnShooterOffFlag = true;
            dontTurnOffTransfer = true;
        } else {
            intake.disable();
            if (turnShooterOffFlag) {
                setOuttakeTarget(0);
                transfer.disable();
                turnShooterOffFlag = false;
            }
            dontTurnOffTransfer = false;
        }

        // Outtake logic

        outtake.update();

    }
}
