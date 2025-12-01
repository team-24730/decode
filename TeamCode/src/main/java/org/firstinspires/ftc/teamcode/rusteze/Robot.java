package org.firstinspires.ftc.teamcode.rusteze;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    public Drivetrain drivetrain;
    public Intake intake;
    public Transfer transfer;
    public Outtake outtake;

    public enum State {
        IDLE,
        INTAKE,
        INTAKE_REVERSE,
        OUTTAKE_IDLE,
        OUTTAKE_SHOOT
    }
    State robotState = State.IDLE;

    private double drivetrainAxial = 0;
    private double drivetrainLateral = 0;
    private double drivetrainRotation = 0;
    private boolean drivetrainSlowEnabled = false;
    private boolean drivetrainEnabled = false;

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

    public void setDrivetrainTarget(double y, double x, double rx, boolean slowEnabled) {
        drivetrainAxial = y;
        drivetrainLateral = x;
        drivetrainRotation = rx;
        drivetrainSlowEnabled = slowEnabled;
        drivetrainEnabled = true;
    }

    public State getState() {
        return robotState;
    }

    public void setState(State state) {
        this.robotState = state;
    }

    public void update() {

        switch (robotState) {
            case IDLE:

                if (drivetrainEnabled) {
                    drivetrain.setMotorPowers(drivetrainAxial, drivetrainLateral, drivetrainRotation, drivetrainSlowEnabled);
                }

                intake.setPower(0);
                transfer.setPower(0);
                setOuttakeRawPower(0);

                break;

            case INTAKE:

                if (drivetrainEnabled) {
                    drivetrain.setMotorPowers(drivetrainAxial, drivetrainLateral, drivetrainRotation, drivetrainSlowEnabled);
                }

                intake.setPower(1);
                transfer.setPower(0.6);

                lastIntakeTime = System.currentTimeMillis();

                break;

            case INTAKE_REVERSE:

                if (drivetrainEnabled) {
                    drivetrain.setMotorPowers(drivetrainAxial, drivetrainLateral, drivetrainRotation, drivetrainSlowEnabled);
                }

                intake.setPower(-0.1);
                transfer.setPower(-0.5);
                setOuttakeRawPower(-0.35);


                // Exit condition
                if (System.currentTimeMillis() - lastIntakeTime > 500) {
                    setState(State.IDLE);
                }

                break;

            case OUTTAKE_IDLE:
                if (drivetrainEnabled) {
                    drivetrain.setMotorPowers(drivetrainAxial, drivetrainLateral, drivetrainRotation, drivetrainSlowEnabled);
                }
                transfer.setPower(0);
                break;

            case OUTTAKE_SHOOT:
                if (drivetrainEnabled) {
                    drivetrain.setMotorPowers(drivetrainAxial, drivetrainLateral, drivetrainRotation, drivetrainSlowEnabled);
                }
                transfer.setPower(1);
                break;


            default:
                robotState = State.IDLE;
        }

        outtake.update();

        /*

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

         */

    }
}
