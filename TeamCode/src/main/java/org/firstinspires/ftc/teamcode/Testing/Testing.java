package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Testing", group="Testing")

public class Testing extends OpMode {
    private DcMotor[] motors = null;
    private Servo[] servos = null;
    private int selectedMotor = 0;
    private int selectedServo = 0;
    private float motorPower = 0;
    private Float servoPosition = 0f;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        motors = new DcMotor[]{
                getMotor("motor0", 1),
                getMotor("motor1", 1),
                getMotor("motor2", 1),
                getMotor("motor3", 1)
        };
        servos = new Servo[]{
                getServo("servo0"),
                getServo("servo1"),
                getServo("servo2"),
                getServo("servo3"),
                getServo("servo4"),
                getServo("servo5")
        };
    }

    @Override
    public void loop() {
        motorPower = gamepad1.right_trigger;

        float servoX = gamepad1.left_stick_x;
        float servoY = -gamepad1.left_stick_y;

        servoPosition = (float)(Math.atan(servoX/servoY) * 180/Math.PI);
        if(servoPosition.isNaN()) {
            servoPosition = 0f;
        }

        servoPosition = mapF(servoPosition, -90f, 90f, 0f, 1f);

        updateGamepad();

        motors[selectedMotor].setPower(motorPower);
        servos[selectedServo].setPosition((double)servoPosition);

        telemetry.addData("Selected motor (^ D v) : ", selectedMotor);
        telemetry.addData("motorSpeed (Y to invert): ", motorPower);
        telemetry.addData("Selected servo (< D > ): ", selectedServo);
        telemetry.addData("servo position (LS): ", (double)servoPosition);
    }


    public DcMotor getMotor(String name, Integer direction) {
        DcMotor motor = hardwareMap.get(DcMotor.class, name);
        if(direction == -1) {
            motor.setDirection(DcMotor.Direction.REVERSE);
        } else {
            motor.setDirection(DcMotor.Direction.FORWARD);
        }

        return motor;
    }

    public Servo getServo(String name) {
        Servo servo = hardwareMap.get(Servo.class, name);
        servo.scaleRange(0, 1);
        return servo;
    }

    private boolean up = false;
    private boolean down = false;
    private boolean left = false;
    private boolean right = false;
    private float time = 0;

    public void updateGamepad() {
        if(gamepad1.dpad_up && up == false) {
            selectedMotor++;
            up = true;
            time = (float)runtime.time();
        }
        if(gamepad1.dpad_down && down == false) {
            selectedMotor--;
            down = true;
            time = (float)runtime.time();
        }

        if(gamepad1.dpad_right && right == false) {
            selectedServo++;
            right = true;
            time = (float)runtime.time();
        }
        if(gamepad1.dpad_left && left == false) {
            selectedServo--;
            left = true;
            time = (float)runtime.time();
        }

        if(runtime.time()-time > 0.25) {
            up = false;
            down = false;
            left = false;
            right = false;
        }

        selectedMotor = Range.clip(selectedMotor, 0, 3);
        selectedServo = Range.clip(selectedServo, 0, 5);
    }

    public float mapF(float value, float min, float max, float newMin, float newMax) {
        return (value-min)/(min-max)*(newMin-newMax)+newMin;
    }
}
