package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.Robot;

import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.opmode.BulkReadHook;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;

@NextTeleop(name = "Main")
public class MainTeleOp extends NextOpMode {

    private final Follower follower;

    public MainTeleOp(Robot robot) {
        super(robot, BulkReadHook.INSTANCE);
        follower = robot.getFollower();
    }

    @Override
    public void start() {
        super.start();
        Telemetry.addBackend(PanelsTelemetry.INSTANCE.getFtcTelemetry());
    }

    @Override
    public void periodic() {
        follower.update();
    }

    @Override
    public void end() {

    }

    @Override
    public void disabledPeriodic() {

    }
}
