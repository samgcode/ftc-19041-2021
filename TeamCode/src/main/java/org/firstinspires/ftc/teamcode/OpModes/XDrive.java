package org.firstinspires.ftc.teamcode.OpModes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.*;
import org.firstinspires.ftc.teamcode.Commands.*;

@TeleOp(name="XDrive", group="Drive")
public class XDrive extends CommandOpMode {
    DriveSubsystem driveSubsystem;
    CustomOdometrySubsystem odometrySubsystem;

    SetDriveSpeedCommand setSpeedCommand;

    @Override
    public void initialize() {
        driveSubsystem = new DriveSubsystem(hardwareMap);
        odometrySubsystem = new CustomOdometrySubsystem(hardwareMap, telemetry);

        setSpeedCommand = new SetDriveSpeedCommand(driveSubsystem, gamepad1, odometrySubsystem);

        schedule(setSpeedCommand);

        telemetry.addData("Status", "Initialized");

    }
}