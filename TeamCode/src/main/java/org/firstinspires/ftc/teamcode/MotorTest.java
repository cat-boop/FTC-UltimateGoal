package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "MotorTest")
public class MotorTest extends LinearOpMode {

    DcMotor test_motor = null;

    static final double     COUNTS_PER_MOTOR        = 288;
    static final double     COUNTS_PER_ROTATION    =   4;

    @Override
    public void runOpMode() {
        test_motor = hardwareMap.get(DcMotor.class, "test_motor");

        test_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        test_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();

        encoderDrive(0.1, 1);
    }

    public void encoderDrive(double speed, double number_of_rotation) {
        int newTarget;
        if (opModeIsActive()) {
            newTarget = test_motor.getCurrentPosition() + (int)(number_of_rotation * COUNTS_PER_MOTOR);
            test_motor.setTargetPosition(newTarget);

            test_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            test_motor.setPower(Math.abs(speed));

            while (opModeIsActive() && test_motor.isBusy()) {
                telemetry.addData("Current position", test_motor.getCurrentPosition());
                telemetry.update();
            }

            test_motor.setPower(0);

            test_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
