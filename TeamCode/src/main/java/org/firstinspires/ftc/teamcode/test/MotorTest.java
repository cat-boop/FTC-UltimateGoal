package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "MotorTest")
public class MotorTest extends LinearOpMode {

    DcMotor testMotor = null;

    @Override
    public void runOpMode() {
        //right_bumper

        testMotor = hardwareMap.get(DcMotor.class, "motor1");

        waitForStart();

        while (opModeIsActive()){
            if (gamepad1.right_trigger > 0){
                testMotor.setPower(gamepad1.right_trigger);
            }
            else {
                testMotor.setPower((-gamepad1.left_trigger));
            }

            telemetry.addData("right_trigger" , gamepad1.right_trigger);
            telemetry.addData("left_trigger" , gamepad1.left_trigger);
            telemetry.update();
        }

    }
}