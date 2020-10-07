package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous()
public class SimpleAutonomous extends LinearOpMode {

    Hardware hardware = new Hardware();
    Gyroscope gyroscope = new Gyroscope();
    ElapsedTime timer = new ElapsedTime();

    double time_start;

    @Override
    public void runOpMode() {
        hardware.init(hardwareMap);
        gyroscope.init(hardwareMap);

        waitForStart();
        timer.reset();


        for (int i = 0; i < 4 && !isStopRequested(); i++) {
            //движение прямо
            driveForTime(2, 0.5, 0, 0);

            //поворот на 90 градусов
            turnForTime(90 * (i + 1));

            sleep(200);
        }


        hardware.setPower(0, 0, 0);
    }


    public void driveForTime(double time, double move, double turn, double sideways) {
        time_start = timer.seconds();
        while (timer.seconds() - time_start < time && opModeIsActive()) {
            hardware.setPower(move, turn, sideways);
            telemetry.addData("Current time", timer.seconds());
            telemetry.update();
        }
        hardware.setPower(0, 0, 0);
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
            hardware.setPower(0, power, 0);

            telemetry.addData("Current angle", current_angle);
            telemetry.addData("Time at target", time_at_target.seconds());
            telemetry.addData("Error", gyroscope.format(target_angle, current_angle));
            telemetry.addData("Power", power);
            telemetry.update();
        }
        hardware.setPower(0, 0, 0);
    }
}
