package org.firstinspires.ftc.teamcode.rusteze;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Template")
//@Disabled
public class TemplateOpMode extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();



    @Override
    public void runOpMode() {



        waitForStart();
        runtime.reset();
        while(opModeIsActive()) {



        }
    }
}
