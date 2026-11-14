
package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.List;
import java.util.Locale;

@TeleOp(name = "Vision Limited Intake", group = "Testing")
public class Limelight extends LinearOpMode {

    // HARDWARE
    private DcMotor intakeFront;
    private Limelight3A limelight;

    // CAPACITY
    private static final int MAX_POLLEN = 2;
    private static final int MAX_NECTAR = 1;

    private int pollenCount = 0;
    private int nectarCount = 0;

    // INTAKE POWER
    private static final double INTAKE_POWER = 0.75;

    // LIMELIGHT SETTINGS
    private static final int PIPELINE = 0;
    private static final double MIN_CONFIDENCE = 60.0;

    // Virtual intake counting line.
    // MUST be calibrated using the actual camera stream.
    private static final double INTAKE_LINE_Y = 120.0;

    // Prevents the same ball being counted repeatedly.
    private static final long CLEAR_TIME_MS = 350;

    private boolean countArmed = true;
    private long clearSince = 0;

    @Override
    public void runOpMode() {

        // Initialize motor
        intakeFront = hardwareMap.get(
                DcMotor.class, "IntakeFront");

        intakeFront.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize Limelight
        limelight = hardwareMap.get(
                Limelight3A.class, "limelight");

        limelight.pipelineSwitch(PIPELINE);
        limelight.start();

        telemetry.addLine("Vision Limited Intake Ready");
        telemetry.addLine("Hold RIGHT BUMPER to intake");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            long now = System.currentTimeMillis();

            boolean intakeRequested = gamepad1.right_bumper;

            boolean full =
                    pollenCount >= MAX_POLLEN &&
                            nectarCount >= MAX_NECTAR;

            // Read Limelight
            LLResult result = limelight.getLatestResult();

            boolean nearIntakeLine = false;

            String detectedType = "None";
            double bestY = Double.NEGATIVE_INFINITY;
            double bestConfidence = 0;

            if (result != null && result.isValid()) {

                List<LLResultTypes.DetectorResult> detections =
                        result.getDetectorResults();

                for (LLResultTypes.DetectorResult detection : detections) {

                    String className =
                            detection.getClassName()
                                    .toLowerCase(Locale.US);

                    double confidence = detection.getConfidence();
                    double y = detection.getTargetYPixels();

                    if (confidence < MIN_CONFIDENCE) {
                        continue;
                    }

                    boolean isPollen =
                            className.equals("pollen");

                    boolean isNectar =
                            className.contains("nectar");

                    if (!isPollen && !isNectar) {
                        continue;
                    }

                    telemetry.addData(
                            "Visible object",
                            "%s | %.1f%% | Y: %.1f",
                            className, confidence, y);

                    if (y >= INTAKE_LINE_Y - 50) {
                        nearIntakeLine = true;
                    }

                    // Select the object closest to the intake.
                    if (y >= INTAKE_LINE_Y &&
                            y > bestY) {

                        bestY = y;
                        bestConfidence = confidence;
                        detectedType = isPollen
                                ? "Pollen" : "Nectar";
                    }
                }
            }

            // Count one object when it crosses the line.
            if (intakeRequested &&
                    !full &&
                    countArmed &&
                    !detectedType.equals("None")) {

                if (detectedType.equals("Pollen") &&
                        pollenCount < MAX_POLLEN) {

                    pollenCount++;
                    countArmed = false;
                    clearSince = 0;
                }

                else if (detectedType.equals("Nectar") &&
                        nectarCount < MAX_NECTAR) {

                    nectarCount++;
                    countArmed = false;
                    clearSince = 0;
                }
            }

            // Re-arm only after the intake area clears.
            if (!nearIntakeLine) {

                if (clearSince == 0) {
                    clearSince = now;
                }

                if (now - clearSince >= CLEAR_TIME_MS) {
                    countArmed = true;
                }

            } else {
                clearSince = 0;
            }

            // Check capacity
            full = pollenCount >= MAX_POLLEN &&
                    nectarCount >= MAX_NECTAR;

            // Run or stop intake
            if (intakeRequested && !full) {
                intakeFront.setPower(INTAKE_POWER);
            } else {
                intakeFront.setPower(0);
            }

            // DRIVER STATION TELEMETRY
            telemetry.addData("Pollen", "%d / %d",
                    pollenCount, MAX_POLLEN);

            telemetry.addData("Nectar", "%d / %d",
                    nectarCount, MAX_NECTAR);

            telemetry.addData("Total",
                    pollenCount + nectarCount);

            telemetry.addData("Current Detection",
                    detectedType);

            telemetry.addData("Detection Confidence",
                    "%.1f%%", bestConfidence);

            telemetry.addData("Intake Requested",
                    intakeRequested);

            telemetry.addData("Intake Motor",
                    intakeRequested && !full
                            ? "RUNNING" : "STOPPED");

            telemetry.addData("Capacity",
                    full ? "FULL" : "AVAILABLE");

            telemetry.update();
        }

        // Safety shutdown
        intakeFront.setPower(0);
        limelight.stop();
    }
}
