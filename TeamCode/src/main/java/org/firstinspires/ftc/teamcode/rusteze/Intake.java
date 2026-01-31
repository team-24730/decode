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
        intake.setPower(1);
        disengageClutch();
    }

    public void enableTransfer() {
        intake.setPower(1);
        state = State.TRANSFER;
    }

    public void disable() {
        if (state == State.INTAKE) {
            state = State.REENGAGING;
            intake.setPower(0.1);
            engageClutch();
            timestamp = System.currentTimeMillis();
        } else if (state == State.TRANSFER) {
            intake.setPower(0);
            state = State.IDLE;
        } else {
            state = State.IDLE;
            intake.setPower(0);
        }
    }

    public void engageClutch() { clutch.setPosition(1); }

    public void disengageClutch () { clutch.setPosition(0); }

    public void update() {
        if (state == State.REENGAGING && System.currentTimeMillis() - timestamp > 500) {
            intake.setPower(0);
            state = State.IDLE;
        }
    }
}
