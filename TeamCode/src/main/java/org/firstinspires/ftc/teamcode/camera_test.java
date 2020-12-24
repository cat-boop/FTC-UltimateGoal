package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

/*
 * This sample demonstrates how to stream frames from Vuforia to the dashboard. Make sure to fill in
 * your Vuforia key below and select the 'Camera' preset on top right of the dashboard. This sample
 * also works for UVCs with slight adjustments.
 */
@Autonomous
public class camera_test extends LinearOpMode {

    // TODO: fill in
    public static final String VUFORIA_LICENSE_KEY = "AQ8d61z/////AAABmQOvoeIUQ0/Uu+f90TxEKtIlNRde7rT6J6ctjV73Fws/c643Fzge4icrjPg6sMFUTua69xRHhX28Ey9MPWO6LaEr/b1QY1hk/4QgzOqJZ5Pwwi6ypXjD0nn4DvLW39UJpQd8xI2W0DGD42e5U9ywo02V+aKhCDqx//kcXQ5yKbBd4NWND5mXrSmRR8uUBlOPpa+MI00ILBdjgJMSaBasD64iK3MncnNUgAtdg+KRA5dllaGxiIHBeoMQrhstwKa2h+3YaNGspzWKJpR2x/2QKw1k1o91rHPcNi1CZQH955ImXu6RK6wG6XonDIjK4ZzysKjAoscjGwhf/r342zm6+psHdx/JjXWe/KEDQEnTQPZJ";

    @Override
    public void runOpMode() throws InterruptedException {
        // gives Vuforia more time to exit before the watchdog notices
        msStuckDetectStop = 2500;

        VuforiaLocalizer.Parameters vuforiaParams = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        vuforiaParams.vuforiaLicenseKey = VUFORIA_LICENSE_KEY;
        vuforiaParams.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(vuforiaParams);

        FtcDashboard.getInstance().startCameraStream(vuforia, 0);

        waitForStart();

        while (opModeIsActive());
    }
}
