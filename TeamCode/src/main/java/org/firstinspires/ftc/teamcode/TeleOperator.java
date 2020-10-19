package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOp")
public class TeleOperator extends LinearOpMode {

    Hardware robot = new Hardware();
    Gyroscope gyroscope = new Gyroscope();

    Servo servo;
    double servo_position = 0, INCREMENT = 0.01, MIN_POS = 0, MAX_POS = 1;

    @Override
    public void runOpMode() {
        telemetry.addData("Status:", "Initializing");
        telemetry.update();

        robot.init(hardwareMap);
        gyroscope.init(hardwareMap);
        servo = hardwareMap.get(Servo.class, "servo");

        telemetry.addData("Status:", "Initialized");
        telemetry.update();

        waitForStart();

        while (!isStopRequested()) {
            robot.setPower(gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);

            if (gamepad1.right_bumper) {
                servo_position = Math.min(MAX_POS, servo_position + INCREMENT);
                sleep(50);
            }
            else if (gamepad1.left_bumper) {
                servo_position = Math.max(MIN_POS, servo_position - INCREMENT);
                sleep(50);
            }
            servo.setPosition(servo_position);

            telemetry.addData("Right stick x value:", gamepad1.right_stick_x);
            telemetry.addData("Right stick y value:", gamepad1.right_stick_y);
            telemetry.addData("Left stick y value:", gamepad1.left_stick_y);
            telemetry.addData("Heading:", gyroscope.getAngle());
            telemetry.update();
        }
        robot.setPower(0, 0, 0);
    }
}
