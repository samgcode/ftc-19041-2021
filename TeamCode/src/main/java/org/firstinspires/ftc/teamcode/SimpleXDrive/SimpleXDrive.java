package org.firstinspires.ftc.teamcode.SimpleXDrive;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

import org.firstinspires.ftc.teamcode.Subsystems.*;
import org.firstinspires.ftc.teamcode.Commands.*;

@TeleOp(name="SimpleXDrive", group="Drive")
public class SimpleXDrive extends CommandOpMode {
    public DcMotor[] xMotors;
    public DcMotor[] yMotors;
    MotorEx encoderLeft, encoderRight, encoderPerp;
    RevIMU imu = null;
    HolonomicOdometry holOdom;
    DriveSubsystem driveSubsystem;
    SetDriveSpeedCommand setSpeedCommand;

    static final double TRACK_WIDTH = 10.4;
    static final double TICKS_TO_INCHES = (Math.PI*3.54331)/1120;
    static final double CENTER_WHEEL_OFFSET = 5.5;

    Double[] position = new Double[]{0.0, 0.0};

    @Override
    public void init() {
        driveSubsystem = new DriveSubsystem(hardwareMap);

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

        Vector speed = new Vector(gamepad1.left_stick_x, gamepad1.left_stick_y);
        speed = normalizeVector(speed.x, speed.y, h)

        double xSpeed = speed.x;
        double ySpeed = speed.y;
        double hSpeed = gamepad1.right_stick_x;

        setSpeedCommand = new SetDriveSpeedCommand(
                    xSpeed, ySpeed, hSpeed
                );

        schedule(setSpeedCommand);
    }

    public double[] normalizeVector(Vector vector) {
                double angle = Math.toRadians(vector.h);
                double yComponent = (-(vector.x * Math.sin(angle)) + (vector.y * Math.cos(angle)));
                double xComponent = ((vector.x * Math.cos(angle)) + (vector.y * Math.sin(angle)));

                return new Vector(xComponent, yComponent, vector.h);
            }
}
