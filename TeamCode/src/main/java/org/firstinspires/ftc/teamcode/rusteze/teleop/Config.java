package org.firstinspires.ftc.teamcode.rusteze.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.rusteze.RobotConstants;
import org.firstinspires.ftc.teamcode.rusteze.subsystems.Turret;


@TeleOp(name="Config")
public class Config extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private RobotConstants constants = new RobotConstants();
    DcMotorEx turret;
    DcMotorEx intake;


    @Override
    public void runOpMode() {

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        runtime.reset();
        while(opModeIsActive()) {

            if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) {
                if (constants.teamColor == RobotConstants.Color.BLUE) {
                    constants.teamColor = RobotConstants.Color.RED;
                } else {
                    constants.teamColor = RobotConstants.Color.BLUE;
                }
            }

            if (Math.abs(gamepad1.left_stick_x) > 0.05) {
                turret.setPower(gamepad1.left_stick_x * 0.3);
            } else {
                turret.setPower(0);
            }

            if (gamepad1.y) {
                RobotConstants.turretInitialized = false;
            }

            telemetry.addLine("Press a to change color");
            telemetry.addData("Current Color", constants.teamColor);
            telemetry.addLine("Use left joystick to move turret");
            telemetry.addData("Press y to deinitialize turret", RobotConstants.turretInitialized);
            telemetry.update();


        }
    }
}
