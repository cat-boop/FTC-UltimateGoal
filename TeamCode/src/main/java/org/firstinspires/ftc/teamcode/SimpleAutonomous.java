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

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        gyroscope.init(hardwareMap);

        telemetry.addData("Status", "initialize");
        telemetry.addData("Status", "wait for start");
        telemetry.update();

        waitForStart();
        timer.reset();

        turnForTime(8);
        robot.setPower(0, 0, 0);
    }


    public void turnForTime(double targetAngle) {
        double currentAngle = gyroscope.getAngle();
        ElapsedTime timeAtTarget = new ElapsedTime();

        while ((Math.abs(gyroscope.format(targetAngle, currentAngle)) > 1 || timeAtTarget.seconds() < 1) && opModeIsActive()) {

            if (Math.abs(gyroscope.format(targetAngle, currentAngle)) > 2) {
                timeAtTarget.reset();
            }

            double power = gyroscope.turnTo(currentAngle, targetAngle);
            currentAngle = gyroscope.getAngle();
            robot.setPower(0, power, 0);

            telemetry.addData("Current angle", currentAngle);
            telemetry.addData("Time at target", timeAtTarget.seconds());
            telemetry.addData("Error", gyroscope.format(targetAngle, currentAngle));
            telemetry.addData("Power", power);
            telemetry.update();
        }
        robot.setPower(0, 0, 0);
    }
}
