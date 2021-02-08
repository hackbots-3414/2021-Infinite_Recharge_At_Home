/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.HashMap;
import java.util.Map;

/**
 * Add your docs here.
 */
public class ColorSystem {

    static Map<String, Color> colorMap = new HashMap<String, Color>();
    public static final String COLOR_BLUE = "blue";
    public static final String COLOR_RED = "red";
    public static final String COLOR_YELLOW = "yellow";
    public static final String COLOR_GREEN = "green";
    public static final String COLOR_PURPLE = "purple";
    public static final String COLOR_ORANGE = "orange";
    public static final String COLOR_LIGHT_BLUE = "light_blue";
    public static final String COLOR_PINK = "pink";

    public static final String COLOR_PATTERN_SOLID = "solid";
    public static final String COLOR_PATTERN_CHASE = "chase";
    public static final String COLOR_PATTERN_PULSE = "pulse";

    static {
        init();
    }

    public static void init() {

        colorMap.put(COLOR_RED, new Color(COLOR_RED, 0, 255, 255));
        colorMap.put(COLOR_YELLOW, new Color(COLOR_YELLOW, 47, 255, 255));
        colorMap.put(COLOR_GREEN, new Color(COLOR_GREEN, 61, 255, 255));
        colorMap.put(COLOR_BLUE, new Color(COLOR_BLUE, 120, 255, 255));
        colorMap.put(COLOR_PURPLE, new Color(COLOR_PURPLE, 137, 255, 255));
        colorMap.put(COLOR_ORANGE, new Color(COLOR_ORANGE, 12, 255, 255));
        colorMap.put(COLOR_LIGHT_BLUE, new Color(COLOR_LIGHT_BLUE, 85, 255, 255));
        colorMap.put(COLOR_PINK, new Color(COLOR_PINK, 162, 255, 255));
    }

    public static Color getColor(String name) {
        return colorMap.get(name);
    }
}
