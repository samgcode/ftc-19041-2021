package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.function.DoubleSupplier;

import org.firstinspires.ftc.teamcode.Subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.Utils.Vector;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;

public class SetDriveSpeedCommand extends CommandBase {
    DriveSubsystem driveSubsystem;
    Gamepad gamepad;
    OdometrySubsystem odometrySubsystem;


    public SetDriveSpeedCommand(DriveSubsystem subsystem_, Gamepad gamepad_, OdometrySubsystem odometrySubsystem_) {
        driveSubsystem = subsystem_;
        gamepad = gamepad_;
        odometrySubsystem = odometrySubsystem_;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        double h = odometrySubsystem.getPosition().h;

        Vector normalizedSpeeds = Vector.normalizeVector(new Vector(gamepad.left_stick_x, gamepad.left_stick_y, h));
        Vector speed = new Vector(normalizedSpeeds.x, normalizedSpeeds.y, gamepad.right_stick_x);

        driveSubsystem.setSpeed(speed);
    }
}
