/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import java.lang.Math;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.vision.Camera;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Drive By Encoder")
//@Disabled
public class DriveByEncoder extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware robot   = new Hardware();   // Use a Pushbot's hardware
    Gyroscope gyroscope = new Gyroscope();
    Camera camera = new Camera();
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 2400 ;    // eg:

    static final double     LENGTH_PER_TIC = 0.0030326975625;

    int numberOfNone = 0, numberOfOne = 0, numberOfFour = 0;
    final double DRIVE_SPEED = 0.3;
    //static final double     WHEEL_DIAMETER_INCHES   = 1.1614173228346456692913385826772 * 2;     // For figuring circumference
    //static final double     COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * Math.PI);

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        msStuckDetectStop = 2500;
        robot.init(hardwareMap);
        gyroscope.init(hardwareMap);
        camera.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Initialized");    //
        telemetry.update();

        waitForStart();


        for (int i = 0; i < 1000; i++) {
            int number = camera.getNumberOfRing();
            if (number == 0) numberOfNone++;
            if (number == 1) numberOfOne++;
            if (number == 4) numberOfFour++;
        }

        driveByTicks(0.5, 0, 50, true, false);
        while (gyroscope.getAngle() < 5 && opModeIsActive()) robot.setPower(0, 0.1, 0);
        robot.setPower(0, 0, 0);

        robot.shooterDo(true);
        sleep(500);

        for (int i = 0; i < 3; i++) {
            robot.ringPusher.setPosition(robot.RING_PUSHER_MAX);
            sleep(300);
            robot.ringPusher.setPosition(robot.RING_PUSHER_MIN);
            sleep(750);

            if (numberOfNone >= numberOfOne && numberOfNone >= numberOfFour) telemetry.addData("Number of rings", 0);
            if (numberOfOne >= numberOfNone && numberOfOne >= numberOfFour) telemetry.addData("Number of rings", 1);
            if (numberOfFour >= numberOfNone && numberOfFour >= numberOfOne) telemetry.addData("Number of rings", 4);
            telemetry.update();
        }
        robot.shooterDo(false);

        while (gyroscope.getAngle() >= 0 && opModeIsActive()) robot.setPower(0, -0.1, 0);
        robot.setPower(0, 0, 0);
        robot.reset();

        if (numberOfNone >= numberOfOne && numberOfNone >= numberOfFour) {
            telemetry.addData("Number of rings", 0);
            telemetry.update();
            driveByTicks(DRIVE_SPEED, -10, 0, false, true);
            driveByTicks(DRIVE_SPEED, 0, 20, true, false);

            robot.servoMinor.setPosition(robot.MINOR_MAX);
            sleep(500);

            driveByTicks(1, 0, -1, true, false);
            //driveByTicks(DRIVE_SPEED, 10, 0, false, true);
            //driveByTicks(DRIVE_SPEED, 0, 10, true, false);
        }

        if (numberOfOne >= numberOfNone && numberOfOne >= numberOfFour) {
            telemetry.addData("Number of rings", 1);
            telemetry.update();
            driveByTicks(DRIVE_SPEED, 5, 0, false, true);
            driveByTicks(DRIVE_SPEED, 0, 40, true, false);

            robot.servoMinor.setPosition(robot.MINOR_MAX);
            sleep(500);

            driveByTicks(1, 0, -1, true, false);

            driveByTicks(DRIVE_SPEED, 0, -10, true, false);
            //driveByTicks(DRIVE_SPEED, 10, 0, false, true);
            //driveByTicks(DRIVE_SPEED, 0, 10, true, false);
        }

        if (numberOfFour >= numberOfNone && numberOfFour >= numberOfOne) {
            telemetry.addData("Number of rings", 4);
            telemetry.update();
            driveByTicks(DRIVE_SPEED, -10, 0, false, true);
            driveByTicks(DRIVE_SPEED, 0, 65, true, false);

            robot.servoMinor.setPosition(robot.MINOR_MAX);
            sleep(500);

            driveByTicks(1, 0, -1, true, false);
            driveByTicks(DRIVE_SPEED, 0, -40, true, false);
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public void driveByTicks(double speed, double x, double y, boolean xIsZero, boolean yIsZero) {
        int target_position_x = robot.leftRear.getCurrentPosition() + (int) (x / LENGTH_PER_TIC);

        int target_left = robot.rightRear.getCurrentPosition() + (int) (y / LENGTH_PER_TIC);
        int target_right = robot.rightFront.getCurrentPosition() + (int) (y / LENGTH_PER_TIC);

        int sign_x = (x >= 0 ? 1 : -1);
        int sign_y = (y >= 0 ? 1 : -1);

        double y_multiplier = 1, x_multiplier = 1;
        if (Math.abs(x) >= Math.abs(y)) {
            if (yIsZero) y_multiplier = 0;
            else y_multiplier = Math.abs(x / y);
        }
        if (Math.abs(y) >= Math.abs(x)) {
            if (xIsZero) x_multiplier = 0;
            else x_multiplier = Math.abs(y / x);
        }

        robot.rightRear.setTargetPosition(target_left);
        robot.rightFront.setTargetPosition(target_right);
        robot.leftRear.setTargetPosition(target_position_x);

        robot.setPower(speed * sign_y * y_multiplier, 0, speed * sign_x * x_multiplier);

        while (opModeIsActive() &&
                ( ((Math.abs(robot.rightRear.getCurrentPosition()) < target_left * sign_y) &&
                  (Math.abs(robot.rightFront.getCurrentPosition()) < target_right * sign_y)) ||
                (Math.abs(robot.leftRear.getCurrentPosition()) < target_position_x * sign_x)) ) {

            // Display it for the driver.
            telemetry.addData("Path1",  "Running to x%7d  y%7d", target_position_x, target_left);
            telemetry.addData("Path2",  "Running at %7d %7d %7d",
                    robot.rightRear.getCurrentPosition(), robot.rightFront.getCurrentPosition(), robot.leftRear.getCurrentPosition());
            //telemetry.addData("")
            telemetry.addData("angle", gyroscope.getAngle());
            telemetry.addData("sign y", sign_y);
            telemetry.addData("y_multiplier", y_multiplier);
            telemetry.addData("sign x", sign_x);
            telemetry.addData("x_multiplier", x_multiplier);
            telemetry.update();
        }

        // Stop all motion;
        robot.setPower(0, 0, 0);
        robot.reset();
    }
    public void driveForward(double speed, double inches) {
        int target_left = robot.rightRear.getCurrentPosition() + (int) (inches / LENGTH_PER_TIC);
        int target_right = robot.rightFront.getCurrentPosition() + (int) (inches / LENGTH_PER_TIC);

        int sign_y = target_left / Math.abs(target_left);

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
}
