package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;

import dev.nextftc.hardware.RobotController;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;

public class Intake implements Mechanism {
    NextMotor motor = new NextMotor(RobotController.expansionHub(), 0);

    public Command run() {
        return instant(() -> motor.setThrottle(1));
    }

    public Command stop() {
        return instant(() -> motor.setThrottle(0));
    }
}
