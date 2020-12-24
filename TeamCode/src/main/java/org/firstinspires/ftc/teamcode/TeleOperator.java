package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOp")
public class TeleOperator extends LinearOpMode {

    Hardware robot = new Hardware();
    Gyroscope gyroscope = new Gyroscope();

    double majorPosition = 0, INCREMENT = 0.05;
    static final int CYCLE_MS = 50;

    @Override
    public void runOpMode() {
        telemetry.addData("Status:", "Initializing");
        telemetry.update();

        robot.init(hardwareMap);
        gyroscope.init(hardwareMap);

        telemetry.addData("Status:", "Initialized");
        telemetry.update();

        waitForStart();

        while (!isStopRequested()) {
            //robot.setPower(gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);
            robot.setPower(-gamepad1.right_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);

            if (gamepad1.right_trigger > 0) {
                robot.intakeMajor.setPower(gamepad1.right_trigger);
                robot.intakeMinor.setPower(gamepad1.right_trigger);
            }
            else {
                robot.intakeMajor.setPower(-gamepad1.left_trigger);
                robot.intakeMinor.setPower(-gamepad1.left_trigger);
            }

            if (gamepad1.right_bumper) robot.servoMinor.setPosition(robot.MINOR_MAX);
            if (gamepad1.left_bumper) robot.servoMinor.setPosition(robot.MINOR_MIN);

            if (gamepad1.dpad_up) majorPosition = Math.min(robot.MAJOR_MAX, majorPosition + INCREMENT);
            if (gamepad1.dpad_down) majorPosition = Math.max(robot.MAJOR_MIN, majorPosition - INCREMENT);

            robot.servoMajor.setPosition(majorPosition);
            sleep(CYCLE_MS);

            telemetry.addData("major servo position", majorPosition);
            telemetry.addData("Heading:", gyroscope.getAngle());
            telemetry.addData("position", "encoders %5d :%5d :%5d", robot.rightFront.getCurrentPosition(), robot.rightRear.getCurrentPosition(), robot.leftRear.getCurrentPosition());
            telemetry.update();

        }
        robot.setPower(0, 0, 0);
    }
}
