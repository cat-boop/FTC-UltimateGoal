package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "MotorTest")
public class MotorTest extends LinearOpMode {

    DcMotor test_motor = null;

    @Override
    public void runOpMode() {
        //right_bumper

        test_motor = hardwareMap.get(DcMotor.class, "motor1");

        waitForStart();

        while (opModeIsActive()){
            if (gamepad1.right_trigger > 0){

                test_motor.setPower(gamepad1.right_trigger);
            }
            if (gamepad1.left_trigger > 0 ){

                test_motor.setPower((-gamepad1.left_trigger));
            }

            telemetry.addData("right_trigger" , gamepad1.right_trigger );
            telemetry.addData("left_trigger" , gamepad1.left_trigger );

            telemetry.update();
        }

    }
}