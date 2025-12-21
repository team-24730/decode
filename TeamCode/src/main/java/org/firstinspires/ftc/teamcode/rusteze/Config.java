package org.firstinspires.ftc.teamcode.rusteze;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Config")
public class Config extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private RobotConstants constants = new RobotConstants();


    @Override
    public void runOpMode() {



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


            telemetry.addLine("Press a to change color");
            telemetry.addData("Current Color", constants.teamColor);
            telemetry.update();


        }
    }
}
