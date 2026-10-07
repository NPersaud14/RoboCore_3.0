package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.data.Alliance;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.Set;

import dev.nextftc.hardware.RobotController;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.NextRobot;
import lombok.Getter;
import lombok.Setter;

public class Robot implements NextRobot {

    @Getter
    private final Intake intake = new Intake();
    @Getter
    @Setter
    private Alliance alliance;
    private Drivetrain drivetrain;
    private Follower follower;

    public Robot() {
    }

    public Follower getFollower() {
        if (follower == null) {
            follower = Constants.create(RobotController.hardwareMap());
        }
        return follower;
    }

    public Drivetrain getDrivetrain() {
        if (drivetrain == null) {
            drivetrain = new Drivetrain(getFollower());
        }
        return drivetrain;
    }

    @NonNull
    @Override
    public Set<Mechanism> getMechanisms() {
        return Set.of(drivetrain, intake);
    }

}
