package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp")
public class TeleOperator extends LinearOpMode {

    Hardware robot = new Hardware();
    Gyroscope gyroscope = new Gyroscope();

    double shooterLiftPosition = 0, INCREMENT = 0.025;
    static final int CYCLE_MS = 50;

    boolean previousState = false;

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
            robot.setPower(-gamepad1.left_stick_y, -gamepad1.right_stick_x, -gamepad1.left_stick_x);

            if (gamepad1.right_trigger > 0) {
                robot.intakeMajor.setPower(gamepad1.right_trigger);
                robot.intakeMinor.setPower(gamepad1.right_trigger);
                shooterLiftPosition = 0;
                robot.ringLift.setPosition(robot.RING_LIFT_MIN);
            }
            else {
                robot.intakeMajor.setPower(-gamepad1.left_trigger);
                robot.intakeMinor.setPower(-gamepad1.left_trigger);
            }

            if (gamepad1.right_bumper) shooterLiftPosition = 0.35;
            if (gamepad1.left_bumper) shooterLiftPosition = Math.max(robot.SHOOTER_LIFT_MIN, shooterLiftPosition - INCREMENT);

            if (gamepad1.dpad_up) {
                //majorPosition = Math.min(robot.MAJOR_MAX, majorPosition + INCREMENT);
                shooterLiftPosition = robot.SHOOTER_LIFT_MAX;
                robot.ringLift.setPosition(robot.RING_LIFT_MAX);
                //robot.ringLift.setPosition(shooterLiftPosition);
            }

            if (gamepad1.dpad_down) {
                //majorPosition = Math.max(robot.MAJOR_MIN, majorPosition - INCREMENT);
                shooterLiftPosition = robot.SHOOTER_LIFT_MIN;
                robot.ringLift.setPosition(robot.RING_LIFT_MIN);
                //robot.shooterLift.setPosition(shooterLiftPosition);
            }

            if (gamepad1.b) {
                robot.ringPusher.setPosition(robot.RING_PUSHER_MAX);
                //while (robot.ringPusher.getPosition() != robot.PING_PUSHER_MAX);
                sleep(200);
                robot.ringPusher.setPosition(robot.RING_PUSHER_MIN);
            }

            if (gamepad1.a) {
                previousState = !previousState;
                robot.shooterDo(previousState);
                sleep(150);
            }

            if (gamepad1.x) robot.servoMinor.setPosition(robot.MINOR_MIN);
            if (gamepad1.y) robot.servoMinor.setPosition(robot.MINOR_MAX);

            robot.shooterLift.setPosition(shooterLiftPosition);
            sleep(CYCLE_MS);

            telemetry.addData("major servo position", shooterLiftPosition);
            telemetry.addData("state", previousState);
            telemetry.addData("Heading:", gyroscope.getAngle());
            telemetry.addData("position", "encoders %5d :%5d :%5d", robot.rightFront.getCurrentPosition(), robot.rightRear.getCurrentPosition(), robot.leftRear.getCurrentPosition());
            telemetry.update();

        }
        robot.setPower(0, 0, 0);
    }
}
