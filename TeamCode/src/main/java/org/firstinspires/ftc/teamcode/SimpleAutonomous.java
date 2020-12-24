package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous()
public class SimpleAutonomous extends LinearOpMode {

    Hardware robot = new Hardware();
    Gyroscope gyroscope = new Gyroscope();
    ElapsedTime timer = new ElapsedTime();

    static final double     LENGTH_PER_TIC = 0.0030326975625;

    double time_start;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        gyroscope.init(hardwareMap);

        telemetry.addData("Status", "initialize");
        telemetry.addData("Status", "wait for start");
        telemetry.update();

        waitForStart();
        timer.reset();

        turnForTime(90);
        robot.setPower(0, 0, 0);
    }


    public void driveForTime(double time, double move) {
        double current_angle = gyroscope.getAngle();
        time_start = timer.seconds();

        while (timer.seconds() - time_start < time && opModeIsActive()) {
            double turn = gyroscope.turnTo(current_angle, 0);
            current_angle = gyroscope.getAngle();

            robot.setPower(move, turn, 0);
            telemetry.addData("Current time", timer.seconds());
            telemetry.update();
        }
        robot.setPower(0, 0, 0);
    }

    public void turnForTime(double target_angle) {
        double current_angle = gyroscope.getAngle();
        ElapsedTime time_at_target = new ElapsedTime();

        while ( (Math.abs(gyroscope.format(target_angle, current_angle)) > 2 || time_at_target.seconds() < 1) && opModeIsActive()) {

            if (Math.abs(gyroscope.format(target_angle, current_angle)) > 4) {
                time_at_target.reset();
            }

            double power = gyroscope.turnTo(current_angle, target_angle);
            current_angle = gyroscope.getAngle();
            robot.setPower(0, power, 0);

            telemetry.addData("Current angle", current_angle);
            telemetry.addData("Time at target", time_at_target.seconds());
            telemetry.addData("Error", gyroscope.format(target_angle, current_angle));
            telemetry.addData("Power", power);
            telemetry.update();
        }
        robot.setPower(0, 0, 0);
    }

    public void driveByTicks(double speed, int inches) {
        //int target_position = robot.rightFront.getCurrentPosition() + (int) (inches/LENGTH_PER_TIC);
        int target_position = robot.rightRear.getCurrentPosition() + (int) (inches / LENGTH_PER_TIC);
        int sign = target_position / Math.abs(target_position);

        //robot.rightFront.setTargetPosition(target_position);
        robot.rightRear.setTargetPosition(target_position);

        // Turn On RUN_TO_POSITION
        //robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.setPower(speed * sign, 0, 0);

        while (opModeIsActive() &&
                (Math.abs(robot.rightRear.getCurrentPosition()) < target_position * sign /* && robot.rightDrive.isBusy() */ )) {

            // Display it for the driver.
            telemetry.addData("Path1",  "Running to %7d", target_position /*,  newRightTarget */);
            telemetry.addData("Path2",  "Running at %7d",
                    Math.abs(robot.rightRear.getCurrentPosition()) /*,
                                            robot.rightDrive.getCurrentPosition() */ );
            //telemetry.addData("")
            telemetry.addData("angle", gyroscope.getAngle());
            telemetry.update();
        }

        // Stop all motion;
        robot.setPower(0, 0, 0);
        robot.reset();
    }
}
