package org.firstinspires.ftc.teamcode.rusteze;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="MecanumDriveTeleOp")
public class MecanumDriveTeleOp extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    Drivetrain drivetrain;

    @Override
    public void runOpMode() {

        drivetrain = new Drivetrain(hardwareMap);

        waitForStart();
        runtime.reset();
        while(opModeIsActive()) {

            drivetrain.setMotorPowers(-gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);

        }
    }
}
