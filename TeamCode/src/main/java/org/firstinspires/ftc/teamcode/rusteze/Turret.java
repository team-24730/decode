package org.firstinspires.ftc.teamcode.rusteze;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret {
    public DcMotorEx turret;
    private double targetPosition;
    private final double DEGREES_PER_TICK = 0.18; // 360/2000
    PID pid = new PID(0.05, 0, 0);
    final double MAX_PID_OUTPUT = 0.4;

    public Turret(HardwareMap hwMap) {
        turret = hwMap.get(DcMotorEx.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        if (!RobotConstants.turretInitialized) {
            turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getPosition() {
        return turret.getCurrentPosition() * DEGREES_PER_TICK;
    }

    public double getTargetPosition() {
        return turret.getCurrentPosition();
    }

    public void initializeTurret() {
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RobotConstants.turretInitialized = true;
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setTargetPosition(double targetPosition) {
        double negativeOverflow = targetPosition - 360;
        double positiveOverflow = targetPosition + 360;

        if (positiveOverflow - getPosition() < targetPosition - getPosition() && positiveOverflow < 370) {
            this.targetPosition = positiveOverflow;
        } else if (negativeOverflow - getPosition() < targetPosition - getPosition() && negativeOverflow > -10) {
            this.targetPosition = negativeOverflow;
        } else {
            this.targetPosition = targetPosition;
        }
    }

    public void update() {
        if (RobotConstants.turretInitialized) {

            // VERY IMPORTANT: OVERRIDE SO TURRET DOES NOT DESTROY WIRES
            if (getPosition() < -10) {
                RobotConstants.turretInitialized = false;
                turret.setPower(0);
            } else if (getPosition() > 370) {
                RobotConstants.turretInitialized = false;
                turret.setPower(0);
            }

            double turretPower = pid.calculate(targetPosition - getPosition());
            turret.setPower(Math.max(Math.min(turretPower, MAX_PID_OUTPUT), -MAX_PID_OUTPUT));

        } else {
            turret.setPower(0);
            turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }
}
