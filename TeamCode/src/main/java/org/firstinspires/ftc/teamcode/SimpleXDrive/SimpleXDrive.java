package org.firstinspires.ftc.teamcode.SimpleXDrive;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

@TeleOp(name="SimpleXDrive", group="Drive")
public class SimpleXDrive extends OpMode {
    public DcMotor[] xMotors;
    public DcMotor[] yMotors;
    RevIMU imu = null;

    static final double TRACK_WIDTH = 10.4;
    static final double TICKS_TO_INCHES = (Math.PI*3.54331)/1120;
    static final double CENTER_WHEEL_OFFSET = 5.5;

    MotorEx encoderLeft, encoderRight, encoderPerp;

    HolonomicOdometry holOdom;

    Double[] position = new Double[]{0.0, 0.0};

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

        encoderLeft = new MotorEx(hardwareMap, "motor2");
        encoderRight = new MotorEx(hardwareMap, "motor3");
        encoderPerp = new MotorEx(hardwareMap, "motor0");

        holOdom = new HolonomicOdometry(
                () -> encoderLeft.getCurrentPosition() * TICKS_TO_INCHES,
                () -> encoderRight.getCurrentPosition() * TICKS_TO_INCHES,
                () -> encoderPerp.getCurrentPosition() * TICKS_TO_INCHES,
                TRACK_WIDTH, CENTER_WHEEL_OFFSET
        );

        imu = new RevIMU(hardwareMap, "imu");

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start() {
        holOdom.updatePose(new Pose2d(1, 2, new Rotation2d(0)));
        imu.init();

        telemetry.addData("Status", "Started");
    }

    @Override
    public void loop() {
        holOdom.updatePose();

        Pose2d pose = holOdom.getPose();

        position[0] = pose.getX();
        position[1] = pose.getY();

        telemetry.addData("position", "%1f, %1f", position[0], position[1]);

        telemetry.addData("left", encoderLeft.getCurrentPosition());
        telemetry.addData("right", encoderRight.getCurrentPosition());
        telemetry.addData("center", encoderPerp.getCurrentPosition());

        double h = -imu.getHeading();
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
