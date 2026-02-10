package org.firstinspires.ftc.teamcode.rusteze.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.rusteze.RobotConstants;

public class Outtake {
    private DcMotorEx flywheelTop;
    private DcMotorEx flywheelBottom;
    private Servo hood;

    public double currentRPM = 0;

    public double targetRPM = 0;
    public boolean useControlSystem = true;
    private double rawPower = 0;

    private final int TICKS_PER_REVOLUTION = 28;

    RobotConstants constants = new RobotConstants();

    public Outtake(HardwareMap hwMap) {
        flywheelTop = hwMap.get(DcMotorEx.class, "flywheelTop");
        flywheelBottom = hwMap.get(DcMotorEx.class, "flywheelBottom");
        hood = hwMap.get(Servo.class, "hood");

        flywheelTop.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheelTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelBottom.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelBottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelBottom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelBottom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setTarget(double target) {
            targetRPM = target;
    }

    public void setHood(double target) {
        hood.setPosition(target);
    }

    public void setPower(double power) {
        rawPower = power;
    }

    public void update() {

        double flywheelPower;
        if (useControlSystem) {
            // Calculate control system variables
            currentRPM = flywheelBottom.getVelocity() / TICKS_PER_REVOLUTION * 60;                                  // Convert ticks per second to revolutions per minute
            double errorRPM = targetRPM - currentRPM;
            double kV = constants.getkV();
            double kP = constants.getkP();

            double vComponent = targetRPM * kV;                                                                         // Calculate feedforward (SVA) V component
            double pComponent = errorRPM * kP;                                                                          // Calculate feedback (PID) P component

            flywheelPower = Math.min(1, Math.max(vComponent + pComponent, 0));                                          // Calculate and clamp flywheel power between 0 and 1
        } else {
            flywheelPower = rawPower;
        }

        // Set motors to calculated power
        flywheelTop.setPower(flywheelPower);
        flywheelBottom.setPower(flywheelPower);
    }
}
