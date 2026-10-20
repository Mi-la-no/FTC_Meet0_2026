
package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

@TeleOp(name = "Pollen Search Auto", group = "Autonomous")
public class Limelight extends OpMode {

    private Follower follower;
    private Limelight3A limelight;

    private final ElapsedTime autoTimer = new ElapsedTime();
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime lostTimer = new ElapsedTime();

    private enum SearchState {
        SEARCHING,
        CONFIRMING,
        TRACKING
    }

    private SearchState state = SearchState.SEARCHING;

    // Search behavior
    private static final double SEARCH_TURN = -0.10;
    private static final double CONFIRM_SECONDS = 0.45;
    private static final double LOST_GRACE_SECONDS = 0.80;

    // Faster movement
    private static final double FORWARD_POWER = 0.34;
    private static final double ALIGN_POWER = 0.22;
    private static final double TURNING_FORWARD_POWER = 0.14;

    // Vision steering
    private static final double TURN_KP = 0.015;
    private static final double MAX_TURN = 0.28;
    private static final double LEFT_BOUNDARY = -6.0;
    private static final double RIGHT_BOUNDARY = 6.0;
    private static final double CENTER_TOLERANCE = 2.0;

    private double lastTargetAngle = 0.0;
    private double lastForwardCommand = 0.0;
    private double lastTurnCommand = 0.0;

    private boolean sawPollenDuringConfirmation = false;

    // Summary of every Pollen detection in one Limelight result.
    private static class VisionObservation {
        int pollenCount = 0;
        int leftCount = 0;
        int centerCount = 0;
        int rightCount = 0;

        double leftAngleSum = 0.0;
        double centerAngleSum = 0.0;
        double rightAngleSum = 0.0;

        int rawDetectionCount = 0;
        double targetAngle = 0.0;
        int bestSectorCount = 0;
        String bestSector = "NONE";
    }

    @Override
    public void init() {

        // Reuse the same drivetrain configuration as your TeleOp.
        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(
                Teleop.startingPose == null
                        ? new Pose()
                        : Teleop.startingPose
        );

        follower.update();

        // Use ONLY the Limelight neural detector.
        limelight = hardwareMap.get(
                Limelight3A.class, "limelight");

        // Pipeline 0 must be your trained neural detector.
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addLine("Pollen Search Auto Ready");
        telemetry.addLine("PedroPathing drivetrain initialized");
        telemetry.addLine("Limelight neural detector enabled");
        telemetry.addLine("No internal autonomous time cutoff");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();

        autoTimer.reset();
        stateTimer.reset();
        lostTimer.reset();

        state = SearchState.SEARCHING;
    }

    @Override
    public void loop() {

        follower.update();

        VisionObservation vision = readPollen();

        double forward = 0.0;
        double turn = 0.0;
        String behavior;

        switch (state) {

            case SEARCHING:

                // Rotate slowly until Pollen enters the camera view.
                if (vision.pollenCount > 0) {

                    lastTargetAngle = vision.targetAngle;

                    // Stop searching and pause to confirm the detection.
                    state = SearchState.CONFIRMING;
                    stateTimer.reset();

                    sawPollenDuringConfirmation = true;
                    lostTimer.reset();

                    behavior = "Pollen found - pausing to confirm";
                } else {

                    forward = 0.0;
                    turn = SEARCH_TURN;

                    behavior = "Searching for Pollen";
                }

                break;

            case CONFIRMING:

                // Remain still during the confirmation period.
                forward = 0.0;
                turn = 0.0;

                if (vision.pollenCount > 0) {
                    sawPollenDuringConfirmation = true;
                    lastTargetAngle = vision.targetAngle;
                    lostTimer.reset();
                }

                if (stateTimer.seconds() >= CONFIRM_SECONDS) {

                    if (sawPollenDuringConfirmation) {

                        state = SearchState.TRACKING;
                        lostTimer.reset();

                        behavior = "Pollen confirmed - approaching";

                    } else {

                        state = SearchState.SEARCHING;

                        behavior = "Detection not confirmed";
                    }

                } else {

                    behavior = "Waiting for stable detection";
                }

                break;

            case TRACKING:

                if (vision.pollenCount > 0) {

                    // Use the freshest location estimate from the camera.
                    lastTargetAngle = vision.targetAngle;
                    lostTimer.reset();

                    double angleError =
                            Math.abs(lastTargetAngle);

                    // Keep turning toward Pollen while making progress.
                    turn = -Range.clip(
                            lastTargetAngle * TURN_KP,
                            -MAX_TURN,
                            MAX_TURN
                    );

                    if (angleError < CENTER_TOLERANCE) {
                        turn = 0.0;
                    }

                    // Drive faster when the target is near the center.
                    // Continue moving slowly while making large turns.
                    if (angleError > 25.0) {

                        forward = TURNING_FORWARD_POWER;
                        behavior = "Turning toward Pollen";

                    } else if (angleError > 10.0) {

                        forward = ALIGN_POWER;
                        behavior = "Aligning and approaching";

                    } else {

                        forward = FORWARD_POWER;
                        behavior = "Approaching Pollen";
                    }

                } else {

                    // Do NOT immediately resume spinning when a detection
                    // briefly disappears. Hold position and wait to reacquire.
                    forward = 0.0;
                    turn = 0.0;

                    if (lostTimer.seconds() >= LOST_GRACE_SECONDS) {

                        state = SearchState.SEARCHING;

                        behavior = "Pollen lost - resuming search";

                    } else {

                        behavior = "Detection lost - waiting to reacquire";
                    }
                }

                break;

            default:

                state = SearchState.SEARCHING;
                behavior = "Resetting search";
                break;
        }

        lastForwardCommand = forward;
        lastTurnCommand = turn;

        // Use the working PedroPathing mecanum drivetrain.
        follower.setTeleOpDrive(
                forward,
                0.0,
                turn,
                false
        );

        telemetry.addData("State", state);
        telemetry.addData("Behavior", behavior);

        telemetry.addData("Raw detector results",
                vision.rawDetectionCount);
        telemetry.addData("Pollen detections",
                vision.pollenCount);

        telemetry.addData("Left / Center / Right",
                "%d / %d / %d",
                vision.leftCount,
                vision.centerCount,
                vision.rightCount);

        telemetry.addData("Best sector",
                vision.bestSector);
        telemetry.addData("Best sector count",
                vision.bestSectorCount);

        telemetry.addData("Target horizontal angle",
                "%.2f degrees", lastTargetAngle);

        telemetry.addData("Forward command",
                "%.2f", lastForwardCommand);
        telemetry.addData("Turn command",
                "%.2f", lastTurnCommand);

        // Informational only: this is NOT a cutoff.
        telemetry.addData("Elapsed time",
                "%.1f seconds", autoTimer.seconds());

        telemetry.update();
    }

    private VisionObservation readPollen() {

        VisionObservation observation = new VisionObservation();

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return observation;
        }

        List<LLResultTypes.DetectorResult> detections =
                result.getDetectorResults();

        observation.rawDetectionCount = detections.size();

        // Read EVERY detector result instead of only the first ball.
        for (LLResultTypes.DetectorResult detection : detections) {

            String className = detection.getClassName();

            if (className == null
                    || !className.equalsIgnoreCase("pollen")) {
                continue;
            }

            // Horizontal angle in degrees relative to the camera's
            // principal pixel, using the Limelight detector API.
            double targetX =
                    detection.getTargetXDegreesNoCrosshair();

            observation.pollenCount++;

            if (targetX < LEFT_BOUNDARY) {

                observation.leftCount++;
                observation.leftAngleSum += targetX;

            } else if (targetX > RIGHT_BOUNDARY) {

                observation.rightCount++;
                observation.rightAngleSum += targetX;

            } else {

                observation.centerCount++;
                observation.centerAngleSum += targetX;
            }
        }

        // Prefer center on ties, then left or right only if the count
        // is strictly greater.
        int bestCount = observation.centerCount;
        double bestAngleSum = observation.centerAngleSum;
        observation.bestSector = "CENTER";

        if (observation.leftCount > bestCount) {

            bestCount = observation.leftCount;
            bestAngleSum = observation.leftAngleSum;
            observation.bestSector = "LEFT";
        }

        if (observation.rightCount > bestCount) {

            bestCount = observation.rightCount;
            bestAngleSum = observation.rightAngleSum;
            observation.bestSector = "RIGHT";
        }

        observation.bestSectorCount = bestCount;

        if (bestCount > 0) {
            observation.targetAngle =
                    bestAngleSum / bestCount;
        }

        return observation;
    }

    @Override
    public void stop() {

        // Always stop movement when the OpMode is stopped.
        if (follower != null) {
            follower.setTeleOpDrive(0.0, 0.0, 0.0, false);
        }

        if (limelight != null) {
            limelight.stop();
        }
    }
}
