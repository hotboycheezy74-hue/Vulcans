package edu.desu.cis.robot.control;

import edu.desu.cis.robot.service.SensorSnapshot;

public class MazeRobot extends RobotController {
    private static final double OBSTACLE_DISTANCE_CM = 15.0;
    private static final double STEER_AROUND_THRESHOLD_CM = 25.0;
    private static final double STEER_AROUND_SPEED = 40.0;
    private static final double STEER_AROUND_DIFF = 20.0;
    private boolean lineFollowActive = false;

    // Robot states
    private enum RobotState {
        FIND_LINE,
        CRUISE,
        IDENTIFY_OBJECT,
        PUSH_OBJECT,
        AVOID_OBJECT,
        FIND_SAMPLE,
        RETURN_TO_BASE,
        STOP
    }

    private RobotState currentState = RobotState.FIND_LINE;

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
        mbot.avoidCrashing(OBSTACLE_DISTANCE_CM);
        lineFollowActive = false;
    }

    public void run() {

        resetCruiseBehaviors();
        currentState = RobotState.CRUISE;

        while (currentState != RobotState.STOP) {

            SensorSnapshot s = awaitNewData();

            switch (currentState) {

                case CRUISE:
                    startLineFollowIfNeeded();
                    if (s.distance() <= OBSTACLE_DISTANCE_CM) {
                        stopLineFollowIfNeeded();
                        mbot.stop();
                        currentState = RobotState.IDENTIFY_OBJECT;
                    }
                    break;

                case IDENTIFY_OBJECT:
                    String color = mbot.getColorObjectFromCamera(false);
                    if (color.equals("GREEN")) {
                        currentState = RobotState.PUSH_OBJECT;
                    } else if (color.equals("BLUE")) {
                        currentState = RobotState.AVOID_OBJECT;
                    } else if (color.equals("RED")) {
                        mbot.flashLed(3, 255, 0, 0, 0.3);
                        currentState = RobotState.FIND_SAMPLE;
                    } else if (color.equals("YELLOW")) {
                        mbot.stopAllBehaviors();
                        currentState = RobotState.STOP;
                    } else {
                        currentState = RobotState.AVOID_OBJECT;
                    }
                    break;

                case PUSH_OBJECT:
                    mbot.pushObject();
                    resetCruiseBehaviors();
                    currentState = RobotState.CRUISE;
                    break;

                case AVOID_OBJECT:
                    mbot.steerAround(STEER_AROUND_THRESHOLD_CM,
                            STEER_AROUND_SPEED,
                            STEER_AROUND_DIFF);
                    resetCruiseBehaviors();
                    currentState = RobotState.CRUISE;
                    break;

                case FIND_SAMPLE:
                    // Move to sample and signal found (rubric: audible or visual cue)
                    mbot.straight(s.distance() - 5);
                    mbot.flashLed(5, 0, 255, 0, 0.3);
                    currentState = RobotState.RETURN_TO_BASE;
                    break;

                case RETURN_TO_BASE:
                    mbot.turnLeft(180);
                    resetCruiseBehaviors();

                    // Keep scanning until yellow insertion point found
                    String returnColor = mbot.getColorObjectFromCamera(false);
                    while (!returnColor.equals("YELLOW")) {
                        SensorSnapshot rs = awaitNewData();
                        startLineFollowIfNeeded();

                        if (rs.distance() <= OBSTACLE_DISTANCE_CM) {
                            stopLineFollowIfNeeded();
                            mbot.stop();
                            String blockColor = mbot.getColorObjectFromCamera(false);
                            if (blockColor.equals("GREEN")) {
                                mbot.pushObject();
                            } else {
                                mbot.steerAround(STEER_AROUND_THRESHOLD_CM,
                                        STEER_AROUND_SPEED,
                                        STEER_AROUND_DIFF);
                            }
                            resetCruiseBehaviors();
                        }
                        returnColor = mbot.getColorObjectFromCamera(false);
                    }

                    // Mission complete!
                    mbot.stopAllBehaviors();
                    mbot.flashLed(5, 0, 255, 0, 0.3);
                    currentState = RobotState.STOP;
                    break;

                case STOP:
                    mbot.stopAllBehaviors();
                    break;
            }
        }
    }

    public static void main(String[] args) {
        try (MazeRobot amazin = new MazeRobot("Vulcans")) {
            amazin.run();
        }
    }
}
