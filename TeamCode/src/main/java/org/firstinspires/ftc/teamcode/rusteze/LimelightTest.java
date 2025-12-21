package org.firstinspires.ftc.teamcode.rusteze;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@Disabled
@TeleOp(name="Limelight Test")
public class LimelightTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private Limelight3A limelight;
    private Robot robot;

    private double lastTx = 0;
    private double k_P = 0.045;

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(8);
        limelight.start();

        robot = new Robot(hardwareMap);

        waitForStart();
        runtime.reset();
        while(opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            if (result != null) {
                if (result.isValid()) {

                    Pose3D botpose = result.getBotpose();
                    telemetry.addData("tx", result.getTx());
                    lastTx = result.getTx();
                    telemetry.addData("ty", result.getTy());
                    telemetry.addData("Botpose", botpose.toString());
                } else {
                    telemetry.addLine("Invalid result");
                    lastTx = 0;
                }
            } else {
                telemetry.addLine("Null result");
                lastTx = 0;
            }

            if (gamepad1.a) {
                robot.setDrivetrainTarget(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x + lastTx * k_P, gamepad1.right_bumper);
            } else {
                robot.setDrivetrainTarget(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.right_bumper);
            }

            telemetry.addData("Time", runtime);

            robot.update();
            telemetry.update();

        }
    }
}
