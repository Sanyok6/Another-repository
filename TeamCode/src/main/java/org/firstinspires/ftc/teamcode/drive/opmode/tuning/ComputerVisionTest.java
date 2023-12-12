package org.firstinspires.ftc.teamcode.drive.opmode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.ComputerVision;

@Config
@TeleOp(name="Computer Vision Test", group="Tuning")
public class ComputerVisionTest extends LinearOpMode {

    public static double ax1 = 0;
    public static double ay1 = 25;
    public static double ax2 = 125;
    public static double ay2 = 150;

    public static double bx1 = 380;
    public static double by1 = 10;
    public static double bx2 = 510;
    public static double by2 = 100;

    public static double cx1 = 830;
    public static double cy1 = 0;
    public static double cx2 = 960;
    public static double cy2 = 120;

    @Override
    public void runOpMode() {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());


        ComputerVision vision = new ComputerVision(hardwareMap, true, telemetry);

        while (!isStarted()) {
            ComputerVision.points = new double[][][] {{{ax1, ay1}, {ax2, ay2}}, {{bx1, by1}, {bx2, by2}}, {{cx1, cy1}, {cx2, cy2}}};

            telemetry.addData("Location", vision.pipeline.location);
            telemetry.update();
        }

    }


}
