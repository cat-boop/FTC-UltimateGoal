package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp
public class TestWobble extends LinearOpMode {

        public void runOpMode(){

            DcMotor wobble = null;
            wobble = hardwareMap.get(DcMotor.class, "wobble");

            wobble.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            wobble.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            waitForStart();


            while(opModeIsActive()) {
                //tick -248
            if (gamepad1.right_trigger > 0) {
                wobble.setPower(gamepad1.right_trigger);
            }
            if (gamepad1.left_trigger > 0) {
                wobble.setPower(-gamepad1.left_trigger);
            }
            if (gamepad1.right_trigger == 0 && gamepad1.left_trigger == 0){
                wobble.setPower(0);
            }

            telemetry.addData("tick", wobble.getCurrentPosition());
            telemetry.update();

            }

        }
    }


