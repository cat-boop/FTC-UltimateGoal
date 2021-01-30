package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Objects;

import static org.firstinspires.ftc.teamcode.Hardware.encoders;

@TeleOp(name = "TeleOp")
public class TeleOperator extends LinearOpMode {

    Hardware robot = new Hardware();
    Gyroscope gyroscope = new Gyroscope();

    double shooterLiftPosition = 0, INCREMENT = 0.025, liftPower = 0.2;
    static final int CYCLE_MS = 50;

    boolean previousState = false, needUpLift = false, needDownLift = false;

    ElapsedTime time = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status:", "Initializing");
        telemetry.update();

        robot.init(hardwareMap);
        gyroscope.init(hardwareMap);

        telemetry.addData("Status:", "Initialized");
        telemetry.update();

        waitForStart();

        time.reset();

        while (!isStopRequested()) {
            //robot.setPower(gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);
            robot.setPower(-gamepad1.left_stick_y, -gamepad1.right_stick_x, -gamepad1.left_stick_x);

            if (gamepad1.right_trigger > 0) {
                robot.intake.setPower(gamepad1.right_trigger);
                shooterLiftPosition = robot.SHOOTER_LIFT_MIN;
                robot.liftDown();
            }
            else robot.intake.setPower(-gamepad1.left_trigger);


            if (gamepad1.right_bumper) shooterLiftPosition = 0.35;
            if (gamepad1.left_bumper) shooterLiftPosition = Math.max(robot.SHOOTER_LIFT_MIN, shooterLiftPosition - INCREMENT);

            if (gamepad1.dpad_up) {
                //majorPosition = Math.min(robot.MAJOR_MAX, majorPosition + INCREMENT);
                shooterLiftPosition = robot.SHOOTER_LIFT_MAX;
                needUpLift = true;
                time.reset();
                //robot.ringLift.setPosition(shooterLiftPosition);
            }

            if (gamepad1.dpad_down) {
                //majorPosition = Math.max(robot.MAJOR_MIN, majorPosition - INCREMENT);
                shooterLiftPosition = robot.SHOOTER_LIFT_MIN;
                needDownLift = true;
                time.reset();
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

            if (gamepad1.x) robot.deployWobble();
            if (gamepad1.y) robot.grabWobble();

            if (needUpLift) {
                if (time.seconds() > 5) {
                    needUpLift = false;
                    robot.ringLift.setPower(0);
                }
                else robot.ringLift.setPower(liftPower);
            }

            if (needDownLift) {
                if (time.seconds() > 5) {
                    needDownLift = false;
                    robot.ringLift.setPower(0);
                }
                else robot.ringLift.setPower(-liftPower);
            }

            telemetry.addData("major servo position", shooterLiftPosition);
            telemetry.addData("state", previousState);
            telemetry.addData("Heading:", gyroscope.getAngle());
            telemetry.addData("position", "encoders %5d :%5d :%5d", Objects.requireNonNull(encoders.get("encoder")).getCurrentPosition(),
                    Objects.requireNonNull(encoders.get("leftEncoder")).getCurrentPosition(),
                    Objects.requireNonNull(encoders.get("rightEncoder")).getCurrentPosition());
            telemetry.addData("shooter lift position", encoders.get("liftEncoder").getCurrentPosition());
            telemetry.addData("wobble motor position", encoders.get("wobble").getCurrentPosition());
            telemetry.update();

        }
        robot.setPower(0, 0, 0);
    }
}
