package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.Gamepad;

import dev.nextftc.robot.Mechanism;


public class Drivetrain implements Mechanism {
    private final Follower follower;

    public Drivetrain(Follower follower) {
        this.follower = follower;
    }

    @Override
    public void periodic() {
        follower.update();
    }

    public Command drive(Gamepad gamepad1) {
        DrivePowers drivePowers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x,
                follower.pose().heading()
        );

        return infinite(() -> follower.manual(drivePowers));
    }

}
