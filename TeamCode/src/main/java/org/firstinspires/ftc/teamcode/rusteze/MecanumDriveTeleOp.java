package org.firstinspires.ftc.teamcode.rusteze;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="MecanumDriveTeleOp")
public class MecanumDriveTeleOp extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    double outtakeTarget = 0.0;

    @Override
    public void runOpMode() {

        Robot robot = new Robot(hardwareMap);

        waitForStart();
        runtime.reset();
        while(opModeIsActive()) {

            robot.drivetrain.setMotorPowers(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            if (gamepad2.x) {
                robot.outtake.setFlywheelTarget(2000);
            }

            if (gamepad2.y) {
                robot.outtake.setFlywheelTarget(3500);
            }

            if (gamepad2.b) {
                robot.outtake.setFlywheelTarget(6000);
            }

            if (gamepad2.a) {
                robot.outtake.setFlywheelTarget(0);
            }

            if (gamepad2.right_bumper) {
                robot.transfer.setPower(1);
            }

            robot.intakeRunning = gamepad2.left_bumper;

            robot.update();

            telemetry.update();

        }
    }
}