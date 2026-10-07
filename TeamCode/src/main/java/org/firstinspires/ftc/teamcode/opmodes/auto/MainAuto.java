package org.firstinspires.ftc.teamcode.opmodes.auto;

import org.firstinspires.ftc.teamcode.Robot;

import dev.nextftc.robot.opmode.NextAutonomous;
import dev.nextftc.robot.opmode.NextOpMode;

@NextAutonomous(name = "MainAuto", preselectTeleop = "Main")
public class MainAuto extends NextOpMode {
    private final Robot robot;

    public MainAuto(Robot robot) {
        super(robot);
        this.robot = robot;
    }


}
