package org.usfirst.frc.team4453.robot.library;

public class Functions {

    public static float clamp(float val, float min, float max) {
        return Math.max(min, Math.min(max, val));
    }
}
