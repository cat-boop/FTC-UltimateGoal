package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous()
public class SimpleAutonomous extends LinearOpMode {

    Hardware hardware = new Hardware();
    ElapsedTime timer = new ElapsedTime();

    double time_start;

    @Override
    public void runOpMode() {
        hardware.init(hardwareMap);

        waitForStart();
        timer.reset();

        for (int i = 0; i < 4 && !isStopRequested(); i++) {
            //движение прямо
            driveForTime(4, 0.5, 0, 0);

            //поворот на 90 градусов
            driveForTime(2, 0, 0.5, 0);
        }
        hardware.setPower(0, 0, 0);
    }

    public void driveForTime(double time, double move, double turn, double sideways) {
        time_start = timer.seconds();
        while (timer.seconds() - time_start < time && !isStopRequested()) {
            hardware.setPower(move, turn, sideways);
        }
    }
}
