package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp")
public class TeleOperator extends LinearOpMode {

    Hardware robot = new Hardware();
    Gyroscope gyroscope = new Gyroscope();

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
            robot.setPower(gamepad1.right_stick_y, -gamepad1.right_stick_x, -gamepad1.left_stick_x);
            telemetry.addData("Right stick x value:", gamepad1.right_stick_x);
            telemetry.addData("Right stick y value:", gamepad1.right_stick_y);
            telemetry.addData("Left stick y value:", gamepad1.left_stick_y);
            telemetry.addData("Heading:", gyroscope.getAngle());
            telemetry.update();
        }
        robot.setPower(0, 0, 0);
    }
}
