package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.controllers.filters.KalmanFilter;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import dev.nextftc.robot.Mechanism;

/**
 * KalmanPositionFuser fuses high-frequency local odometry (Pinpoint) with
 * absolute global vision (Limelight 3A) to provide a more accurate robot pose.
 * <p>
 * This class implements the NextFTC Mechanism interface, allowing its periodic
 * method to be called automatically by the robot loop.
 */
public class KalmanPositionFuser implements Mechanism {
    private final KalmanFilter xFilter;
    private final KalmanFilter yFilter;
    private final KalmanFilter headingFilter;

    private final GoBildaPinpointDriver pinpoint;
    private final Limelight3A limelight;
    private final Follower follower;

    private double lastPinpointX;
    private double lastPinpointY;
    private double lastPinpointHeading;

    /**
     * Creates a new KalmanPositionFuser.
     *
     * @param pinpoint  The GoBilda Pinpoint Driver (used for prediction).
     * @param limelight The Limelight 3A (used for absolute correction).
     * @param follower  The PedroPathing Follower (updated with the fused pose).
     */
    public KalmanPositionFuser(GoBildaPinpointDriver pinpoint, Limelight3A limelight, Follower follower) {
        this.pinpoint = pinpoint;
        this.limelight = limelight;
        this.follower = follower;

        // Tune these values:
        // Lower modelCovariance = more trust in Pinpoint's relative movements.
        // Lower dataCovariance = more trust in Limelight's absolute global position.
        this.xFilter = new KalmanFilter(0.01, 0.5);
        this.yFilter = new KalmanFilter(0.01, 0.5);
        this.headingFilter = new KalmanFilter(0.001, 0.1);

        // Initialize state
        pinpoint.update();
        this.lastPinpointX = pinpoint.getPosX(DistanceUnit.INCH);
        this.lastPinpointY = pinpoint.getPosY(DistanceUnit.INCH);
        this.lastPinpointHeading = pinpoint.getHeading(AngleUnit.DEGREES);

        xFilter.reset(lastPinpointX, 1, 1);
        yFilter.reset(lastPinpointY, 1, 1);
        headingFilter.reset(lastPinpointHeading, 1, 1);
    }

    /**
     * Called periodically by the NextFTC robot loop.
     * Performs the Kalman update and pushes the result to PedroPathing.
     */
    @Override
    public void periodic() {
        // 1. Prediction: Get movement since last loop from Pinpoint
        pinpoint.update();
        double currentX = pinpoint.getPosX(DistanceUnit.INCH);
        double currentY = pinpoint.getPosY(DistanceUnit.INCH);
        double currentHeading = pinpoint.getHeading(AngleUnit.DEGREES);

        double dx = currentX - lastPinpointX;
        double dy = currentY - lastPinpointY;
        double dHeading = currentHeading - lastPinpointHeading;

        lastPinpointX = currentX;
        lastPinpointY = currentY;
        lastPinpointHeading = currentHeading;

        // 2. Correction: Apply Limelight data if available
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();

            // Convert Limelight meters to inches for PedroPathing
            double llX = botpose.getPosition().x * 39.3701;
            double llY = botpose.getPosition().y * 39.3701;
            double llHeading = botpose.getOrientation().getYaw();

            // Kalman Fusion
            xFilter.update(dx, llX);
            yFilter.update(dy, llY);

            // Normalize heading to take the shortest path around the circle
            double normalizedLLHeading = normalizeAngle(llHeading, headingFilter.state());
            headingFilter.update(dHeading, normalizedLLHeading);
        } else {
            // No vision: Move the filter forward based on Pinpoint deltas alone
            xFilter.update(dx, xFilter.state());
            yFilter.update(dy, yFilter.state());
            headingFilter.update(dHeading, headingFilter.state());
        }

        // 3. Update PedroPathing logic
        // PedroPathing Poses use Radians for heading internally.
        follower.setPose(new Pose(
                xFilter.state(),
                yFilter.state(),
                Math.toRadians(headingFilter.state())
        ));
    }

    /**
     * Adjusts the measurement angle to be within +/- 180 degrees of the current state
     * to prevent the Kalman Filter from jumping the wrong way across the 0/360 boundary.
     */
    private double normalizeAngle(double target, double current) {
        double delta = target - current;
        while (delta > 180) delta -= 360;
        while (delta < -180) delta += 360;
        return current + delta;
    }

    /**
     * Returns the current fused pose.
     */
    public Pose getFusedPose() {
        return new Pose(xFilter.state(), yFilter.state(), Math.toRadians(headingFilter.state()));
    }
}
