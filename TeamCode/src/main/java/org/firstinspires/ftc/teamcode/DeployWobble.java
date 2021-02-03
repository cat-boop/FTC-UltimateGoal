package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class DeployWobble extends LinearOpMode {
    public void runOpMode(){

        DcMotor wobble = null;
        wobble = hardwareMap.get(DcMotor.class, "wobble");

        wobble.setMode (DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobble.setTargetPosition(-228);
        wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        wobble.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();


        while(opModeIsActive()) {
            //tick -248

            wobble.setPower(0.5);

            telemetry.addData("tick", wobble.getCurrentPosition());
            telemetry.update();
            }

        }
    }

