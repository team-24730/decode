package org.firstinspires.ftc.teamcode.rusteze;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="MecanumDriveTeleOp")
public class MecanumDriveTeleOp extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    Drivetrain drivetrain;
    Transfer transfer;
    Intake intake;
    Outtake outtake;
    double outtakeTarget = 0.0;

    @Override
    public void runOpMode() {

        drivetrain = new Drivetrain(hardwareMap);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        outtake = new Outtake(hardwareMap);

        waitForStart();
        runtime.reset();
        while(opModeIsActive()) {

            drivetrain.setMotorPowers(-gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);

            //intake.setPower(-gamepad2.left_stick_y);

            transfer.setPower(-gamepad2.right_stick_y);

            outtakeTarget += -gamepad2.left_stick_y * 10;
            outtake.setFlywheelTarget(outtakeTarget);

            telemetry.addData("Target RPM", outtakeTarget * 10);
            telemetry.update();

        }
    }
}
