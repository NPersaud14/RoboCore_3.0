package org.firstinspires.ftc.teamcode;

import com.pedropathing.controllers.filters.KalmanFilter;

import org.firstinspires.ftc.teamcode.mechanisms.Intake;

import java.util.Collections;
import java.util.Set;

import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.NextRobot;

public class Robot implements NextRobot {

    Intake intake = new Intake();

    @Override
    public Set<Mechanism> getMechanisms() {return Set.of(intake);}

}
