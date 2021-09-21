package org.firstinspires.ftc.teamcode.SimpleXDrive;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.*;
import org.firstinspires.ftc.teamcode.Commands.*;

//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

@TeleOp(name="SimpleXDrive", group="Drive")
public class SimpleXDrive extends CommandOpMode {
    public DcMotor[] xMotors;
    public DcMotor[] yMotors;

    DriveSubsystem driveSubsystem;
    OdometrySubsystem odometrySubsystem;

    SetDriveSpeedCommand setSpeedCommand;

    @Override
    public void initialize() {
        driveSubsystem = new DriveSubsystem(hardwareMap);
        odometrySubsystem = new OdometrySubsystem(hardwareMap, telemetry);

        setSpeedCommand = new SetDriveSpeedCommand(driveSubsystem, gamepad1, odometrySubsystem);

        schedule(setSpeedCommand);

        telemetry.addData("Status", "Initialized");

    }
}
