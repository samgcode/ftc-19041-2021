package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Utils.Vector;

public class DriveSubsystem extends SubsystemBase {
    DcMotor[] xMotors, yMotors;
    Vector speed = new Vector(0, 0, 0);
    HardwareMap hardwareMap;

    public DriveSubsystem(HardwareMap hMap_) {
        hardwareMap = hMap_;

        xMotors = new DcMotor[] {
            getMotor("motor0", false),
            getMotor("motor1", true)
        };
        yMotors = new DcMotor[] {
            getMotor("motor2", false),
            getMotor("motor3", true)
        };
    }

    public void setSpeed(Vector newSpeed) {
        speed = newSpeed;
    }

    @Override
    public void periodic() {

        double xSpeed = speed.x;
        double ySpeed = speed.y;
        double hSpeed = speed.h;

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
}