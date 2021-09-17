package org.firstinspires.ftc.teamcode.SimpleXDrive;

import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="SimpleXDrive", group="Drive")
public class SimpleXDrive extends OpMode {
    public DcMotor[] xMotors = null;
    public DcMotor[] yMotors = null;
    RevIMU imu = null;

    double[] position = new Double[0, 0]

    @Override
    public void init() {
        xMotors = new DcMotor[] {
            getMotor("motor0", false),
            getMotor("motor1", true)
        };
        yMotors = new DcMotor[] {
                getMotor("motor2", false),
                getMotor("motor3", true)
        };

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start() {
        imu.init();

        telemetry.addData("Status", "Started");
    }

    @Override
    public void loop() {
        double h = imu.getHeading();
        telemetry.addData("h", h);

        double xSpeed = gamepad1.left_stick_x;
        double ySpeed = gamepad1.left_stick_y;
        double hSpeed = gamepad1.right_stick_x;

        double[] normalizedSpeeds = normalizeVector(h, xSpeed, ySpeed);

        xSpeed = normalizedSpeeds[0];
        ySpeed = normalizedSpeeds[1];

        xMotors[0].setPower(xSpeed-hSpeed);
        xMotors[1].setPower(xSpeed+hSpeed);
        yMotors[0].setPower(ySpeed-hSpeed);
        yMotors[1].setPower(ySpeed+hSpeed);
    }

    public DcMotor getMotor(String name, boolean inverted) {
        Motor motor = new Motor(hardwareMap, name);
        motor.setInverted(inverted);

        return motor.motor;
    }

    public double[] normalizeVector(double h, double x, double y) {
        double angle = Math.toRadians(h);
        double yComponent = (-(x * Math.sin(angle)) + (y * Math.cos(angle)));
        double xComponent = ((x * Math.cos(angle)) + (y * Math.sin(angle)));

        double[] vector = new double[]{xComponent, yComponent};
        return vector;
    }

}
