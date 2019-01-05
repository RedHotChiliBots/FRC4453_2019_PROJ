package org.usfirst.frc.team4453.robot.commands.autonomous;

import edu.wpi.first.wpilibj.DriverStation;

public final class FieldConstants {
    public final static double AUTO_LINE_DIST = 120; // 10ft
    public final static double RUNG_HEIGHT = 7*12 - 3.5; // 7ft - 3.5in platform
    public final static double ROBOT_LENGTH = 34.5;
    public final static double FENCE_DEPTH = 4*12 + 8;
    public final static double WALL_TO_FENCE = 140;
    public final static double CUBE_WIDTH = 13;

    public enum SwitchPosition {
        LEFT,
        RIGHT
    }
    
    public enum Alliance {
        US,
        THEM
    }
    
    public static SwitchPosition getSwitchPosition(Alliance a){
        return DriverStation.getInstance().getGameSpecificMessage().charAt(a == Alliance.US ? 0 : 2) == 'L' ? SwitchPosition.LEFT : SwitchPosition.RIGHT;
    }
    
    public static SwitchPosition getScalePosition() {
        return DriverStation.getInstance().getGameSpecificMessage().charAt(1) == 'L' ? SwitchPosition.LEFT : SwitchPosition.RIGHT;
    }
}
