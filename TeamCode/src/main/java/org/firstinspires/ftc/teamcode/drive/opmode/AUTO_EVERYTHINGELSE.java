package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.ComputerVision;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.LinearSlide;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(group = "drive")
public class AUTO_EVERYTHINGELSE extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        ComputerVision vision = new ComputerVision(hardwareMap, true, telemetry);

        CRServo li_servo = hardwareMap.crservo.get("li_servo");
        CRServo ri_servo = hardwareMap.crservo.get("ri_servo");

        Servo to_servo = hardwareMap.servo.get("to_servo");
        Servo bo_servo = hardwareMap.servo.get("bo_servo");
        Servo outtake_rotate = hardwareMap.servo.get("outtake_rotate");
        Servo intake_lift = hardwareMap.servo.get("intake_lift");
        DistanceSensor dist = hardwareMap.get(DistanceSensor.class, "dist");

        LinearSlide linear_slide = new LinearSlide(hardwareMap.dcMotor.get("ls_motor"));

        while (!isStarted()) {
            telemetry.addData("Detected location: ", vision.pipeline.location);
            telemetry.update();
        }

        to_servo.setPosition(0.95);
        bo_servo.setPosition(0);
        outtake_rotate.setPosition(0.885);

        Pose2d startingPose = new Pose2d(-60, -35, 0);
        drive.setPoseEstimate(startingPose);

        Trajectory traj;

        if (vision.pipeline.location == 1) {
            traj = drive.trajectoryBuilder(new Pose2d(-60, -35, 0))
                    .lineToSplineHeading(new Pose2d(-28, -36, Math.toRadians(90)))
                    .build();
            drive.followTrajectory(traj);
        } else if (vision.pipeline.location == 2) {
            traj = drive.trajectoryBuilder(new Pose2d(-60, -35, 0))
                    .lineToSplineHeading(new Pose2d(-25, -34, Math.toRadians(0)))
                    .build();
            drive.followTrajectory(traj);
        } else {
            traj = drive.trajectoryBuilder(new Pose2d(-60, -35, 0))
                    .lineToSplineHeading(new Pose2d(-28, -35, Math.toRadians(270)))
                    .build();
            drive.followTrajectory(traj);
        }

        sleep(500);

        li_servo.setPower(1);
        ri_servo.setPower(-1);

        sleep(2000);

        li_servo.setPower(0);
        ri_servo.setPower(0);

        traj = drive.trajectoryBuilder(traj.end())
                .back(1)
                .build();
        drive.followTrajectory(traj);

    }
}
