package edu.desu.cis.robot.control;

import edu.desu.cis.robot.service.SensorSnapshot;

public class MazeRobot extends RobotController {
    private static final double OBSTACLE_DISTANCE_CM = 15.0;
    private static final double STEER_AROUND_THRESHOLD_CM = 25.0;
    private static final double STEER_AROUND_SPEED = 40.0;
    private static final double STEER_AROUND_DIFF = 20.0;
    private static final long STEER_AROUND_DURATION_MS = 3000;
    private static final long PUSH_OBJECT_DURATION_MS = 5000;
    private static final double SAMPLE_APPROACH_BUFFER_CM = 2.0;
    private static final String SAMPLE_COLOR = "RED";
    private static final String MOVABLE_COLOR = "GREEN";
    private static final String IMMOVABLE_COLOR = "BLUE";
    private static final String INSERTION_POINT_COLOR = "YELLOW";
    private static final double IDENTIFY_CREEP_CM = 1.5;
    private static final double MIN_IDENTIFY_DISTANCE_CM = 6.0;
    private static final int IDENTIFY_APPROACH_STEPS = 4;
    private static final int CAMERA_READ_ATTEMPTS = 5;
    private static final long CAMERA_READ_DELAY_MS = 200;

    private enum RobotState {
        CRUISE,
        IDENTIFY_OBJECT,
        MOVE_OBJECT,
        AVOID_OBJECT,
        COLLECT_SAMPLE,
        RETURN_TO_BASE,
        MISSION_COMPLETE
    }

    private RobotState currentState = RobotState.CRUISE;
    private boolean lineFollowActive = false;
    private boolean carryingSample = false;
    private boolean returnHeadingEstablished = false;

    public MazeRobot(String robotName) {
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

    private void resetCruiseBehaviors() {
        lineFollowActive = false;
    }

    private boolean isObstacleAhead(double distance) {
        return !Double.isNaN(distance) && distance > 0 && distance <= OBSTACLE_DISTANCE_CM;
    }

    private double safeApproachDistance(double distance) {
        if (Double.isNaN(distance) || distance <= SAMPLE_APPROACH_BUFFER_CM) {
            return 0;
        }
        return distance - SAMPLE_APPROACH_BUFFER_CM;
    }

    private RobotState resumeStateAfterObject() {
        return carryingSample ? RobotState.RETURN_TO_BASE : RobotState.CRUISE;
    }

    private void collectSample(double distance) {
        mbot.straight(safeApproachDistance(distance));
        mbot.flashLed(5, 255, 0, 0, 0.3);
        carryingSample = true;
        mbot.turnLeft(180);
        returnHeadingEstablished = true;
    }

    private boolean insertionPointReached(String color) {
        return INSERTION_POINT_COLOR.equals(color);
    }

    private String readStableCameraColor() {
        for (int attempt = 0; attempt < CAMERA_READ_ATTEMPTS; attempt++) {
            String color = mbot.getColorObjectFromCamera(true);
            if (MOVABLE_COLOR.equals(color)
                    || IMMOVABLE_COLOR.equals(color)
                    || SAMPLE_COLOR.equals(color)
                    || INSERTION_POINT_COLOR.equals(color)) {
                return color;
            }

            try {
                Thread.sleep(CAMERA_READ_DELAY_MS);
            } catch (InterruptedException ignored) {
                Thread.currentThread().interrupt();
                break;
            }
        }
        return "";
    }

    private String identifyObjectCautiously() {
        for (int step = 0; step < IDENTIFY_APPROACH_STEPS; step++) {
            String color = readStableCameraColor();
            if (MOVABLE_COLOR.equals(color)
                    || IMMOVABLE_COLOR.equals(color)
                    || SAMPLE_COLOR.equals(color)
                    || INSERTION_POINT_COLOR.equals(color)) {
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

    private void pauseLoop() {
        try {
            Thread.sleep(50);
        } catch (InterruptedException ignored) {
            Thread.currentThread().interrupt();
        }
    }

    @Override
    public void run() {
        mbot.avoidCrashing(OBSTACLE_DISTANCE_CM);
        mbot.followLine();

        while (currentState != RobotState.MISSION_COMPLETE) {
            SensorSnapshot sensor = awaitNewData();

            switch (currentState) {
                case CRUISE:
                    double distance = sensor.distance();
                    if (isObstacleAhead(distance)) {
                        mbot.stopAllBehaviors();
                        currentState = RobotState.IDENTIFY_OBJECT;
                    }
                    break;

                case IDENTIFY_OBJECT:

                    String color = mbot.getColorObjectFromCamera();
                    System.out.println("Color = "+color);
                    if (MOVABLE_COLOR.equals(color)) {
                        mbot.flashLed(1, 0, 255, 0, 0.3);
                        currentState = RobotState.MOVE_OBJECT;
                    } else if (IMMOVABLE_COLOR.equals(color)) {
                        mbot.flashLed(1, 0, 0, 255, 0.3);
                        currentState = RobotState.AVOID_OBJECT;
                    } else if (SAMPLE_COLOR.equals(color) && !carryingSample) {
                        mbot.flashLed(3, 255, 0, 0, 0.3);
                        currentState = RobotState.COLLECT_SAMPLE;
                    } else if (INSERTION_POINT_COLOR.equals(color) && carryingSample) {
                        currentState = RobotState.MISSION_COMPLETE;
                    } else {
                        mbot.avoidCrashing(OBSTACLE_DISTANCE_CM);
                        mbot.followLine();
                        currentState = RobotState.CRUISE;
                    }
                    break;

                case MOVE_OBJECT:
                    mbot.pushObject();

                    double pushdistance = sensor.distance();

                    try {
                        Thread.sleep(PUSH_OBJECT_DURATION_MS);
                    } catch (InterruptedException ignored) {

                    }




                    mbot.stopBehavior("PUSH_OBJECT");
                    mbot.avoidCrashing(15);
                    mbot.followLine();
                    currentState = RobotState.CRUISE;
                    break;

                case AVOID_OBJECT:
                    mbot.steerAround(
                            STEER_AROUND_THRESHOLD_CM,
                            STEER_AROUND_SPEED,
                            STEER_AROUND_DIFF
                    );
                    try {
                        Thread.sleep(STEER_AROUND_DURATION_MS);
                    } catch (InterruptedException ignored) {
                        Thread.currentThread().interrupt();
                    }
                    

                    mbot.stopAllBehaviors();


                    mbot.avoidCrashing(15);
                    mbot.followLine();
                    currentState = RobotState.CRUISE;
                    break;

                case COLLECT_SAMPLE:
                    // collectSample(distance);
                    mbot.avoidCrashing(15);
                    mbot.followLine();
                    currentState = RobotState.CRUISE;
                    break;

                case RETURN_TO_BASE:
                    if (!returnHeadingEstablished) {
                        mbot.turnLeft(180);
                        returnHeadingEstablished = true;
                    }
                    mbot.avoidCrashing(15);
                    mbot.followLine();
                    currentState = RobotState.CRUISE;

                    while (currentState == RobotState.RETURN_TO_BASE) {
                        double returnDistance = mbot.readUltrasonic();
                        String returnColor = readStableCameraColor();
                        if (insertionPointReached(returnColor)) {
                            currentState = RobotState.MISSION_COMPLETE;
                            break;
                        }

                        if (isObstacleAhead(returnDistance)) {
                            mbot.stop();
                            currentState = RobotState.IDENTIFY_OBJECT;
                        }
                        pauseLoop();
                    }
                    break;

                case MISSION_COMPLETE:
                    break;
            }
        }

        mbot.stopAllBehaviors();
        mbot.flashLed(5, 0, 255, 0, 0.3);
        mbot.stop();
    }

    public static void main(String[] args) {
        try (MazeRobot robot = new MazeRobot("Stingbot")) {
            robot.run();
        }
    }
}