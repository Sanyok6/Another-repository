package org.firstinspires.ftc.teamcode.drive.opmode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.LinearSlide;

@Config
@TeleOp(name="Linear Slide Tune", group="Tuning")
public class LinearSlideTune extends LinearOpMode {

    public static double Kp;
    public static double Ki;
    public static double Kd;

    @Override
    public void runOpMode() {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        LinearSlide linear_slide = new LinearSlide(hardwareMap.dcMotor.get("ls_motor"));

        Kp = linear_slide.Kp;
        Ki = linear_slide.Ki;
        Kd = linear_slide.Kd;

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.dpad_up) {
                linear_slide.setTarget(600);
            } else if (gamepad1.dpad_down) {
                linear_slide.setTarget(200);
            } else if (gamepad1.a) {
                linear_slide.setTarget(0);
            }

            linear_slide.moveTowardsTarget();

            linear_slide.Kp = Kp;
            linear_slide.Ki = Ki;
            linear_slide.Kd = Kd;

            telemetry.addData("Linear slide position: ", linear_slide.motor.getCurrentPosition());
            telemetry.update();
        }
    }
}
