package org.firstinspires.ftc.teamcode.rusteze;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake {
    DcMotorEx intake;
    Servo clutch;
    public boolean isEnabled;
    long timestamp;


    public enum State {
        INTAKE,
        REENGAGING,
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
        disengageClutch();
        intake.setPower(0);
        timestamp = System.currentTimeMillis(); // delay intaking
    }

    public void enableTransfer() {
        intake.setPower(1);
        engageClutch();
        state = State.TRANSFER;
    }

    public void disable() {
        intake.setPower(0);
        state = State.IDLE;
    }

    public void engageClutch() { clutch.setPosition(0.6); }

    public void disengageClutch () { clutch.setPosition(0.8); }

    public void update() {
        /*if (state == State.REENGAGING && System.currentTimeMillis() - timestamp > 250) { // reengage after a delay
            engageClutch();
        }
        if (state == State.REENGAGING && System.currentTimeMillis() - timestamp > 1000) { // allow one second and a half for the clutch to reengage
            intake.setPower(0);
            state = State.IDLE;
        } */

        if (state == State.INTAKE && System.currentTimeMillis() - timestamp > 150) { // allow some time for disengagement before intaking
            intake.setPower(1);
        }
    }
}
