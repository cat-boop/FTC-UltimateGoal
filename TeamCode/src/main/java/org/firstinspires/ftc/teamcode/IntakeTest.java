package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Intake")
public class IntakeTest extends LinearOpMode {

    Hardware robot = new Hardware();
    Gyroscope gyroscope = new Gyroscope();

    DcMotor intakeMajor = null;
    DcMotor intakeMinor = null;

    Servo servo;
    double servo_position = 0, INCREMENT = 0.01, MIN_POS = 0, MAX_POS = 1;

    @Override
    public void runOpMode() {
        telemetry.addData("Status:", "Initializing");
        telemetry.update();

        intakeMajor = hardwareMap.get(DcMotor.class, "intakeMajor");
        intakeMinor = hardwareMap.get(DcMotor.class, "intakeMinor");

        intakeMinor.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status:", "Initialized");
        telemetry.update();

        waitForStart();

        while (!isStopRequested()) {
            //robot.setPower(gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);
            //robot.setPower(-gamepad1.right_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);

            if (gamepad1.right_trigger >= 0) {
                intakeMajor.setPower(-gamepad1.right_trigger);
                //intakeMinor.setPower(gamepad1.right_trigger);
            }
            if (gamepad1.left_trigger >= 0) {
                //intakeMajor.setPower(-gamepad1.left_trigger);
                intakeMinor.setPower(-gamepad1.left_trigger);
            }


            //telemetry.addData("Right stick x value:", gamepad1.right_stick_x);
            //telemetry.addData("Right stick y value:", gamepad1.right_stick_y);
            //telemetry.addData("Left stick y value:", gamepad1.left_stick_y);
            //telemetry.addData("Heading:", gyroscope.getAngle());
            //telemetry.addData("position", "encoders %5d :%5d", robot.rightFront.getCurrentPosition(), robot.rightRear.getCurrentPosition());
            //telemetry.update();
        }
        robot.setPower(0, 0, 0);
    }
}
