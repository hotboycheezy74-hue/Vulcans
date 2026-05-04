package edu.desu.cis.robot.control;

public class SampleRetriever extends RobotController {
    private static final String SAMPLE_COLOR = "RED";
    private static final String INSERTION_POINT_COLOR = "YELLOW";
    private static final double OBJECT_STOP_THRESHOLD_CM = 14.0;
    private static final double IDENTIFY_CREEP_CM = 1.5;
    private static final double MIN_IDENTIFY_DISTANCE_CM = 6.0;
    private static final int IDENTIFY_APPROACH_STEPS = 4;
    private static final int CAMERA_READ_ATTEMPTS = 5;
    private static final long CAMERA_READ_DELAY_MS = 200;
    private static final long POLL_DELAY_MS = 75;
    private static final double SAMPLE_APPROACH_BUFFER_CM = 2.0;

    private boolean lineFollowActive = false;

    public SampleRetriever(String robotName) {
        super(robotName);
    }

    private void startLineFollowIfNeeded() {
        if (!lineFollowActive) {
            mbot.followLine();
            lineFollowActive = true;
        }
    }

    private void stopLineFollowIfNeeded() {
        if (lineFollowActive) {
            mbot.stopBehavior("LINE_FOLLOW");
            lineFollowActive = false;
        }
    }

    private String readStableCameraColor() {
        for (int attempt = 0; attempt < CAMERA_READ_ATTEMPTS; attempt++) {
            String color = mbot.getColorObjectFromCamera(true);
            if (SAMPLE_COLOR.equals(color) || INSERTION_POINT_COLOR.equals(color)) {
                return color;
            }

            try {
                Thread.sleep(CAMERA_READ_DELAY_MS);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return "";
            }
        }
        return "";
    }

    private void pauseLoop() {
        try {
            Thread.sleep(POLL_DELAY_MS);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    private double safeApproachDistance(double distance) {
        if (Double.isNaN(distance) || distance <= SAMPLE_APPROACH_BUFFER_CM) {
            return 0;
        }
        return distance - SAMPLE_APPROACH_BUFFER_CM;
    }

    private boolean isObjectAhead(double distance) {
        return !Double.isNaN(distance) && distance > 0 && distance <= OBJECT_STOP_THRESHOLD_CM;
    }

    private String identifyObjectCautiously() {
        for (int step = 0; step < IDENTIFY_APPROACH_STEPS; step++) {
            String color = readStableCameraColor();
            if (SAMPLE_COLOR.equals(color) || INSERTION_POINT_COLOR.equals(color)) {
                return color;
            }

            double distance = mbot.readUltrasonic();
            if (Double.isNaN(distance) || distance <= MIN_IDENTIFY_DISTANCE_CM) {
                break;
            }

            mbot.straight(IDENTIFY_CREEP_CM);
            mbot.stop();
        }

        return "";
    }

    @Override
    public void run() {
        boolean sampleCollected = false;
        boolean missionComplete = false;

        startLineFollowIfNeeded();

        while (!missionComplete && !Thread.currentThread().isInterrupted()) {
            double distance = mbot.readUltrasonic();

            if (!sampleCollected && isObjectAhead(distance)) {
                stopLineFollowIfNeeded();
                mbot.stop();

                String color = identifyObjectCautiously();
                if (SAMPLE_COLOR.equals(color)) {
                    double updatedDistance = mbot.readUltrasonic();
                    double approachDistance = safeApproachDistance(updatedDistance);
                    if (approachDistance > 0) {
                        mbot.straight(approachDistance);
                    }

                    // Visual cue for sample detection before returning.
                    mbot.flashLed(5, 255, 0, 0, 0.3);

                    // Turn around and head back to the insertion point.
                    mbot.turnLeft(180);
                    sampleCollected = true;
                }

                startLineFollowIfNeeded();
            } else if (sampleCollected) {
                String color = readStableCameraColor();
                if (INSERTION_POINT_COLOR.equals(color)) {
                    stopLineFollowIfNeeded();
                    mbot.stopAllBehaviors();
                    mbot.stop();
                    mbot.flashLed(5, 255, 255, 0, 0.3);
                    missionComplete = true;
                }
            }

            pauseLoop();
        }
    }

    public static void main(String[] args) {
        try (SampleRetriever robot = new SampleRetriever("Vulcans")) {
            robot.run();
        }
    }
}
