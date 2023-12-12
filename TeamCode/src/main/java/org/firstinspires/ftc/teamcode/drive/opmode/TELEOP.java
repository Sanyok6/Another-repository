package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.LinearSlide;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.ToggleButton;

@TeleOp
public class TELEOP extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // li_servo stands for "left intake servo"
        // We use the CRServo class because the intake servos are constant rotation servos
        CRServo li_servo = hardwareMap.crservo.get("li_servo");
        CRServo ri_servo = hardwareMap.crservo.get("ri_servo");

        // to_servo stands for "top outtake servo", bo_servo stands for "bottom outtake servo"
        Servo to_servo = hardwareMap.servo.get("to_servo");
        Servo bo_servo = hardwareMap.servo.get("bo_servo");
        Servo outtake_rotate = hardwareMap.servo.get("outtake_rotate");

        Servo airplane_servo = hardwareMap.servo.get("airplane_servo");

        Servo intake_lift = hardwareMap.servo.get("intake_lift");

        DistanceSensor dist = hardwareMap.get(DistanceSensor.class, "dist");

        LinearSlide linear_slide = new LinearSlide(hardwareMap.dcMotor.get("ls_motor"));

        // setup toggle buttons for controlling outtake
        ToggleButton x_toggle = new ToggleButton();
        ToggleButton b_toggle = new ToggleButton();
        ToggleButton y_toggle = new ToggleButton();

        // vibrate the controller for extra driver feedback when button toggled
        x_toggle.onToggle = () -> gamepad2.rumble(200);
        b_toggle.onToggle = () -> gamepad2.rumble(200);
        y_toggle.onToggle = () -> gamepad2.rumble(200);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(new Pose2d());

        waitForStart();

        while (opModeIsActive()) {
            Pose2d poseEstimate = drive.getPoseEstimate();

            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y/(gamepad1.right_trigger > 0 ? 2.5 : 1.75),
                    -gamepad1.left_stick_x/(gamepad1.right_trigger > 0 ? 2.5 : 1.75)
            ).rotated(-poseEstimate.getHeading());

            if (gamepad1.a && dist.getDistance(DistanceUnit.CM) > 6) {
                input = new Vector2d(
                        -0.2,
                        -0
                ).rotated(-poseEstimate.getHeading());
            }

            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -gamepad1.right_stick_x/2
                    )
            );

            drive.update();

            if (gamepad1.y) {
                drive.setPoseEstimate(new Pose2d());
            }

            // this controls the intake
            if (gamepad2.left_trigger > 0 || gamepad2.right_trigger > 0) {
                li_servo.setPower(-1);
                ri_servo.setPower(1);
            } else if (gamepad2.left_bumper || gamepad2.right_bumper) {
                li_servo.setPower(1);
                ri_servo.setPower(-1);
            } else {
                li_servo.setPower(0);
                ri_servo.setPower(0);
            }

            // this controls linear slide
            if (gamepad2.dpad_up) {
                linear_slide.manualMove(0.8);
            } else if (gamepad2.dpad_down) {
                linear_slide.manualMove(-0.5);
            }  else if (gamepad2.a) {
                linear_slide.setTarget(0);
            } else {
                linear_slide.moveTowardsTarget();
            }

            // this controls outtake
            x_toggle.updateState(gamepad2.x);
            b_toggle.updateState(gamepad2.b);
            y_toggle.updateState(gamepad2.y);

            to_servo.setPosition(x_toggle.toggled ? 0.95 : 0.85);
            bo_servo.setPosition(b_toggle.toggled ? 0.3 : 0);
            outtake_rotate.setPosition(y_toggle.toggled ? 0.4 : 0.885);

            intake_lift.setPosition(gamepad1.dpad_left ? 0.15 : 0);


            // this controls airplane launcher
            if (gamepad1.dpad_left && gamepad2.dpad_left) {
                airplane_servo.setPosition(0.9);
            } else {
                airplane_servo.setPosition(1);
            }

            telemetry.update();
        }
    }
}
