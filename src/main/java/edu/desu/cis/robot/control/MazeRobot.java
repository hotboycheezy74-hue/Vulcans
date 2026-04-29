package edu.desu.cis.robot.control;

import edu.desu.cis.robot.service.SensorSnapshot;

public class MazeRobot extends RobotController {

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

    // ============================================================
    // Helper: push a movable obstacle using existing MBot2 methods
    // Backs up, arcs around, then realigns
    // ============================================================
    private void pushMovableObstacle(double distanceToCm) {
        mbot.straight(-(distanceToCm * 1.3));       // back up
        mbot.moveAndTurnLeft(40, 2.0, 20);           // arc left beside object
        mbot.straight(distanceToCm * 1.3);           // push forward past it
        mbot.moveAndTurnRight(40, 2.0, 20);          // arc right to realign
    }

    // ============================================================
    // Helper: steer around an immovable obstacle
    // ============================================================
    private void steerAroundObstacle() {
        mbot.turnRight(45);          // turn away from obstacle
        mbot.forward(40, 1.5);       // move past it
        mbot.turnLeft(45);           // realign with path
    }

    // ============================================================
    // Helper: one step of line following using motor power + offset
    // Call this repeatedly in the CRUISE loop
    // ============================================================
    private void followLineStep() {
        int offset = mbot.readLineOffsetTrack();
        int status = mbot.readLineStatus();

        if (status == 0) {
            mbot.stop();
            return;
        }

        double speed = 40;
        double kp = 0.8;
        double correction = kp * offset;
        mbot.setMotorPower(speed + correction, speed - correction);
    }

    public void run() {

        mbot.avoidCrashing(15);
        currentState = RobotState.CRUISE;

        while (currentState != RobotState.STOP) {

            SensorSnapshot s = awaitNewData();

            switch (currentState) {

                case CRUISE:
                    followLineStep();
                    if (s.distance() <= 15) {
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
                    pushMovableObstacle(s.distance());
                    mbot.avoidCrashing(15);
                    currentState = RobotState.CRUISE;
                    break;

                case AVOID_OBJECT:
                    steerAroundObstacle();
                    mbot.avoidCrashing(15);
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
                    mbot.avoidCrashing(15);

                    // Keep scanning until yellow insertion point found
                    String returnColor = mbot.getColorObjectFromCamera(false);
                    while (!returnColor.equals("YELLOW")) {
                        SensorSnapshot rs = awaitNewData();
                        followLineStep();

                        if (rs.distance() <= 15) {
                            mbot.stop();
                            String blockColor = mbot.getColorObjectFromCamera(false);
                            if (blockColor.equals("GREEN")) {
                                pushMovableObstacle(rs.distance());
                            } else {
                                steerAroundObstacle();
                            }
                            mbot.avoidCrashing(15);
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