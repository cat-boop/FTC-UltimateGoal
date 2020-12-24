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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@TeleOp(name="Drive By Encoder")
//@Disabled
public class DriveByEncoder extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware robot   = new Hardware();   // Use a Pushbot's hardware
    Gyroscope gyroscope = new Gyroscope();
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 2400 ;    // eg:

    static final double     LENGTH_PER_TIC = 0.0030326975625;

    //static final double     WHEEL_DIAMETER_INCHES   = 1.1614173228346456692913385826772 * 2;     // For figuring circumference
    //static final double     COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * Math.PI);

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        gyroscope.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Initialized");    //
        telemetry.update();


        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d",
                          -robot.rightFront.getCurrentPosition());
                          //-robot.rightRear.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        driveByTicks(0.1, 20, 40);
        //driveByTicks(0.3, 30);
        //driveByTicks(0.3, -30);
        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        //encoderDrive(DRIVE_SPEED,  20,  48, 15);  // S1: Forward 47 Inches with 5 Sec timeout
        //encoderDrive(TURN_SPEED,   12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        //encoderDrive(DRIVE_SPEED, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public void driveByTicks(double speed, double x, double y) {
        int target_position_x = robot.leftRear.getCurrentPosition() + (int) (x / LENGTH_PER_TIC);

        int target_left = robot.rightRear.getCurrentPosition() + (int) (y / LENGTH_PER_TIC);
        int target_right = robot.rightFront.getCurrentPosition() + (int) (y / LENGTH_PER_TIC);

        int sign_x = target_position_x / Math.abs(target_position_x);
        int sign_y = target_left / Math.abs(target_left);

        double y_multiplier = (y >= x ? 1 : x / y);
        double x_multiplier = (x >= y ? 1 : y / x);

        robot.rightRear.setTargetPosition(target_left);
        robot.rightFront.setTargetPosition(target_right);
        robot.leftRear.setTargetPosition(target_position_x);

        telemetry.addData("sign y", sign_y);
        telemetry.addData("y_multiplier", y_multiplier);
        telemetry.update();

        robot.setPower(speed * sign_y * y_multiplier, 0, speed * sign_x * x_multiplier);

        while (opModeIsActive() &&
                (Math.abs(robot.rightRear.getCurrentPosition()) < target_left * sign_y) &&
                (Math.abs(robot.rightFront.getCurrentPosition()) < target_right * sign_y) &&
                (Math.abs(robot.rightRear.getCurrentPosition()) < target_position_x * sign_x)) {

            // Display it for the driver.
            telemetry.addData("Path1",  "Running to x%7d  y%7d", target_position_x, target_left);
            telemetry.addData("Path2",  "Running at %7d %7d %7d",
                    robot.rightRear.getCurrentPosition(), robot.rightFront.getCurrentPosition(), robot.leftRear.getCurrentPosition());
            //telemetry.addData("")
            telemetry.addData("angle", gyroscope.getAngle());
            telemetry.update();
        }

        // Stop all motion;
        robot.setPower(0, 0, 0);
        robot.rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
