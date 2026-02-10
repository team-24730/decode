package org.firstinspires.ftc.teamcode.rusteze.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.rusteze.utility.PID;
import org.firstinspires.ftc.teamcode.rusteze.RobotConstants;

public class Turret {
    public DcMotorEx turret;
    private double targetPosition;
    private final double DEGREES_PER_TICK = 0.18; // 360/2000
    PID pid = new PID(0.05, 0, 0);
    final double MAX_PID_OUTPUT = 0.4;
    final double hysteresis = 5; // extra degrees at the end of the turret range
    final double emergencyStop = 10; // amount of degrees the turret can go past 0 or 360 before it disables

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
        return targetPosition;
    }

    public void initializeTurret() {
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RobotConstants.turretInitialized = true;
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setTargetPosition(double targetPosition) {
        /*double moduloTargetPosition = targetPosition % 360;

        double negativeOverflow = moduloTargetPosition - 360;
        double positiveOverflow = moduloTargetPosition + 360;

        if (positiveOverflow - getPosition() < moduloTargetPosition - getPosition() && positiveOverflow < 370) {
            this.targetPosition = positiveOverflow;
        } else if (negativeOverflow - getPosition() < moduloTargetPosition - getPosition() && negativeOverflow > -10) {
            this.targetPosition = negativeOverflow;
        } else {
            this.targetPosition = moduloTargetPosition;
        } */

        if (targetPosition > 360) {
            this.targetPosition = 360;
        } else if (targetPosition < 0) {
            this.targetPosition = 0;
        } else {
            this.targetPosition = targetPosition;
        }
    }

    public void setSmartTargetPosition(double targetPos) {
        targetPosition = targetPos % 360;   // Use modulo operator to wrap values past 360
        if (targetPosition < 0) {           // Make sure only positive values (0-360) result
            targetPosition += 360;
        }

        double currentPosition = getPosition();
        double negativeOverflow = targetPosition - 360; // Make an overflow 360 degrees below the target
        double positiveOverflow = targetPosition + 360; // Make an overflow 360 degrees above the target
        if (negativeOverflow >= -hysteresis && Math.abs(currentPosition - negativeOverflow) < Math.abs(currentPosition - targetPosition)) {
            // Make sure overflow is no more than 10 degrees away // Set to overflow value if the overflow is closer than the normal 0-360 degree target
            targetPosition = negativeOverflow;
        } else if (Math.abs(positiveOverflow) <= 360 + hysteresis && Math.abs(currentPosition - positiveOverflow) < Math.abs(currentPosition - targetPosition)) {
            // Make sure overflow is no more than 10 degrees away // Set to overflow value if the overflow is closer than the normal 0-360 degree target
            targetPosition = positiveOverflow;
        }
    }

    public void update() {
        if (RobotConstants.turretInitialized) {

            // VERY IMPORTANT: OVERRIDE SO TURRET DOES NOT DESTROY WIRES
            if (getPosition() < -emergencyStop) {
                RobotConstants.turretInitialized = false;
                turret.setPower(0);
            } else if (getPosition() > 360 + emergencyStop) {
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
