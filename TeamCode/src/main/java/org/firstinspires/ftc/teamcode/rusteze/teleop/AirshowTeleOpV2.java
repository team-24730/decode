package org.firstinspires.ftc.teamcode.rusteze.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.rusteze.subsystems.RobotV2;


@TeleOp(name="Airshow TeleOp")
public class AirshowTeleOpV2 extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private int rpmTarget = 3000;

    @Override
    public void runOpMode() {
        RobotV2 robot = new RobotV2(hardwareMap);

        waitForStart();
        runtime.reset();
        while(opModeIsActive()) {

            // DRIVETRAIN CODE
            robot.drivetrain.setMotorPowers(-gamepad1.left_stick_y * 0.5, gamepad1.left_stick_x * 0.5, gamepad1.right_stick_x * 0.6, false);

            // SIMPLE INTAKING CODE
            if (gamepad1.left_bumper && robot.getState() == RobotV2.State.IDLE) {
                robot.setState(RobotV2.State.INTAKE);
                robot.intake.enable();
                robot.intake.closeGate();
            }
            if (!gamepad1.left_bumper && robot.getState() == RobotV2.State.INTAKE) {
                robot.setState(RobotV2.State.IDLE);
                robot.intake.disable();
            }

            // SIMPLE SHOOTING CODE
            if (gamepad1.right_bumper) {
                robot.setState(RobotV2.State.SHOOTING);
                robot.outtake.useControlSystem = true;
                robot.outtake.setHood(0.97);
                robot.outtake.setTarget(rpmTarget);
            }
            if (!gamepad1.right_bumper && robot.getState() == RobotV2.State.SHOOTING) {
                robot.setState(RobotV2.State.IDLE);
                robot.outtake.setTarget(0);
            }
            if (gamepad1.dpadUpWasPressed()) {
                rpmTarget += 50;
            }
            if (gamepad1.dpadDownWasPressed()) {
                rpmTarget -= 50;
            }

            // SIMPLE TRANSFER CODE
            if (robot.getState() == RobotV2.State.SHOOTING && gamepad1.left_bumper) {
                robot.intake.openGate();
                robot.intake.enable();
            }
            if (robot.getState() == RobotV2.State.SHOOTING && !gamepad1.left_bumper) {
                robot.intake.disable();
            }

            robot.update();
            telemetry.addLine("NEW CONTROLS");
            telemetry.addLine("Left Bumper: Intake/Transfer (Hold)");
            telemetry.addLine("Right Bumper: Spin Up Flywheel (Hold)");
            telemetry.addLine("");
            telemetry.addLine("");
            telemetry.addLine("OTHER INFORMATION");
            telemetry.addData("Current State", robot.getState());
            telemetry.addData("Flywheel Speed", rpmTarget);
            telemetry.addLine("Change Flywheel Speed with Dpad up and Dpad down");
            telemetry.update();
        }
    }
}
