package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class LinearSlide {
    public DcMotor motor;

    double target = 0;

    // constants for PID controller
    // TODO: When motion profiling is implemented, Kd should be lowered to about 0.0003
    public double Kp = 0.0065;
    public double Ki = 0.0035;
    public double Kd = 0.0006;

    // variables for PID controller
    double integralSum = 0;
    double prevError = 0;
    ElapsedTime timer = new ElapsedTime();

    // stores whether the slide is being controlled manually or not
    enum ControlMode {
        MANUAL,
        AUTO
    }
    ControlMode controlMode = ControlMode.AUTO;

    public LinearSlide(DcMotor motor) {
        this.motor = motor;
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setTarget(int target) {
        controlMode = ControlMode.AUTO;

        // reset PID variables
        integralSum = 0;
        prevError = 0;
        timer.reset();

        this.target = target;
    }

    public void moveTowardsTarget() {
        if (controlMode == ControlMode.AUTO) {
            // implements PID controller to move the linear slide towards the target position
            // TODO: motion profile linear slide movement
            double error = target - motor.getCurrentPosition();
            double derivative = (error - prevError) / timer.seconds();
            integralSum = integralSum + (error * timer.seconds());

            double motorPower = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
            motor.setPower(motorPower);

            prevError = error;

            timer.reset();
        } else {
            // TODO: use PID to maintain linear slide position even during manual control
            motor.setPower(0.2);
        }
    }

    // returns whether the linear slide reached the target position
    public Boolean reachedTarget() {
        return Math.abs(target-motor.getCurrentPosition()) < 20;
    }

    // allows the linear slide to be moved manually, without the help of a PID controller
    // TODO: motion profile manual linear slide movement
    public void manualMove(double motorPower) {
        controlMode = ControlMode.MANUAL;

        // makes sure the linear slide is in bounds
        if ((motorPower > 0 && motor.getCurrentPosition() < 825) || (motorPower < 0 && motor.getCurrentPosition() > 0)) {
            motor.setPower(motorPower);
        } else {
            motor.setPower(0.2);
        }
    }


}

