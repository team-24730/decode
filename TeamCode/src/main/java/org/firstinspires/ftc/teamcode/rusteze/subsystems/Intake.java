package org.firstinspires.ftc.teamcode.rusteze.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    DcMotorEx intake;
    Servo clutch;
    public boolean clutchEnabled;
    long timestamp;


    public enum State {
        INTAKE,
        REVERSE,
        TRANSFER,
        IDLE
    }
    State state = State.IDLE;

    public Intake(HardwareMap hwMap) {
        intake = hwMap.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        clutch = hwMap.get(Servo.class, "clutch");
    }

    public State getState() { return state; }

    public void setPower(double power) { intake.setPower(power); }

    public void enable() {
        state = State.INTAKE;
        if (clutchEnabled) {
            intake.setPower(0.2);
        } else {
            intake.setPower(1);
        }
        timestamp = System.currentTimeMillis();
        disengageClutch();
    }

    public void reverse() {
        state = State.REVERSE;
        disengageClutch();
        intake.setPower(-1);
    }

    public void enableTransfer() {
        intake.setPower(1);
        engageClutch();
        state = State.TRANSFER;
    }

    public void autoFarTransfer() {
        intake.setPower(0.6);
        engageClutch();
        state = State.TRANSFER;
    }

    public void autoTransfer() {
        intake.setPower(0.8);
        engageClutch();
        state = State.TRANSFER;
    }

    public void disable() {
        state = State.IDLE;
        intake.setPower(0);
    }

    public void engageClutch() { clutch.setPosition(0.6); clutchEnabled = true; }

    public void disengageClutch () { clutch.setPosition(0.8); clutchEnabled = false; }

    public void update() {
        if (state == State.INTAKE && System.currentTimeMillis() - timestamp > 300) { // allow some time for disengagement before intaking
            intake.setPower(1);
        }
    }
}
