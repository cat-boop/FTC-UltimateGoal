package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class CRServoTest extends LinearOpMode {
    Servo continuousServoLeft = null;
    Servo continuousServoRight = null;

    double power = 0.5, currentPowerLeft, currentPowerRight;

    @Override
    public void runOpMode() {
        continuousServoLeft = hardwareMap.get(Servo.class, "continuousServoLeft");
        continuousServoRight = hardwareMap.get(Servo.class, "continuousServoRight");

        telemetry.addData("Initialized", "wait for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.right_trigger > 0) {
                currentPowerLeft = power + gamepad1.right_trigger / 2;
                currentPowerRight = power - gamepad1.right_trigger / 2;
            }
            else if (gamepad1.left_trigger >= 0) {
                currentPowerLeft = power - gamepad1.left_trigger / 2;
                currentPowerRight = power + gamepad1.left_trigger / 2;
            }
            continuousServoLeft.setPosition(currentPowerLeft);
            continuousServoRight.setPosition(currentPowerRight);
            sleep(50);

            telemetry.addData("current power left", currentPowerLeft);
            telemetry.addData("current power right", currentPowerRight);
            telemetry.update();
        }
    }
}
