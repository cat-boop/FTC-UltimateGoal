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

    double shooterLiftPosition = robot.SHOOTER_ANGLE_MAX, INCREMENT = 0.025, liftPower = 0.2;
    static final int CYCLE_MS = 50;

    boolean previousStateShooter = false, previousStatePusher = false, previousStateWobble = false;

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
            robot.setPower(-gamepad1.right_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            //robot.setPower(-gamepad2.left_stick_y, -gamepad2.right_stick_x, -gamepad2.left_stick_x);

            if (gamepad1.right_trigger > 0) {
                robot.wobble.setPower(gamepad1.right_trigger);
                shooterLiftPosition = robot.SHOOTER_ANGLE_MIN;
                //needDownLift = true;
                //time.reset();
            }
            else robot.wobble.setPower(-gamepad1.left_trigger);

            if (gamepad1.right_bumper) robot.intake.setPower(1);
            if (gamepad1.left_bumper) robot.intake.setPower(-1);
            if (!gamepad1.right_bumper && !gamepad1.left_bumper) robot.intake.setPower(0);//

            if (gamepad1.a) {
                previousStateWobble = !previousStateWobble;
                if (previousStateWobble) robot.grabWobble();
                else robot.deployWobble();
                sleep(150);
            }

//            if (gamepad1.b) {
//                robot.ringPusher.setPosition(robot.RING_PUSHER_MOVE);
//                //while (robot.ringPusher.getPosition() != robot.PING_PUSHER_MAX);
//                sleep(200);
//                robot.ringPusher.setPosition(robot.RING_PUSHER_STOP);
//            }
//
//            if (gamepad1.a) {
//                previousState = !previousState;
//                if (previousState) robot.ringPusher.setPosition(robot.RING_PUSHER_MOVE);
//                else robot.ringPusher.setPosition(robot.RING_PUSHER_STOP);
//                robot.shooterDo(previousState);
//                sleep(150);
//            }

            if (gamepad1.x) robot.deployWobble();
            if (gamepad1.y) robot.grabWobble();



            if (gamepad2.right_trigger > 0) robot.ringLift.setPower(gamepad2.right_trigger);
            else robot.ringLift.setPower(-gamepad2.left_trigger);

            if (gamepad2.right_bumper) {
                shooterLiftPosition = Math.max(robot.SHOOTER_ANGLE_MIN, shooterLiftPosition - INCREMENT);
                sleep(150);
            }
            if (gamepad2.left_bumper) {
                shooterLiftPosition = Math.min(robot.SHOOTER_ANGLE_MAX, shooterLiftPosition + INCREMENT);
                sleep(150);
            }

            robot.shooterAngle.setPosition(shooterLiftPosition);

            if (gamepad2.a) {
                previousStateShooter = !previousStateShooter;
                robot.shooterDo(previousStateShooter);
                sleep(200);
            }

            if (gamepad2.b) {
                previousStatePusher = !previousStatePusher;
                if (previousStatePusher) robot.ringPusher.setPosition(robot.RING_PUSHER_MOVE);
                else robot.ringPusher.setPosition(robot.RING_PUSHER_STOP);
                sleep(200);
            }

            telemetry.addData("shooter angle position", shooterLiftPosition);
            telemetry.addData("state", previousStateShooter);
            telemetry.addData("Heading:", gyroscope.getAngle());
            telemetry.addData("position", "encoders %5d :%5d :%5d", Objects.requireNonNull(encoders.get("encoder")).getCurrentPosition(),
                    Objects.requireNonNull(encoders.get("leftEncoder")).getCurrentPosition(),
                    -Objects.requireNonNull(encoders.get("rightEncoder")).getCurrentPosition());
            telemetry.addData("shooter lift position", encoders.get("liftEncoder").getCurrentPosition());
            telemetry.addData("wobble motor position", encoders.get("wobble").getCurrentPosition());
            telemetry.update();

        }
        robot.setPower(0, 0, 0);
    }
}
