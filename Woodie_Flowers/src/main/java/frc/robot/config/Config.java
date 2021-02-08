/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.config;

import edu.wpi.first.wpilibj.Preferences;

import java.util.HashMap;

/**
 * Add your docs here.
 */
public class Config {
    private static Config me = null; 

    private String robotName = null;

    private HashMap< String, HashMap< String, Double>> allConfig = null;
    

    private Config() {
        robotName = Preferences.getInstance().getString("robotName", "null");
        allConfig = new HashMap<>();
        loadRobotConfig();
    }

    public synchronized static Config getInstance() {
        if (me == null){ 
            me = new Config();
        } 
        return me;
    }

    private void loadRobotConfig(){ 
        allConfig.put("woodie", WoodieConfig.getConfig());
        allConfig.put("flowers", FlowersConfig.getConfig());
        allConfig.put("testChassis1", TestChassis1Config.getConfig());
    }

    public double getDouble(String key) {
        if (robotName.equals ("null")){
            throw new IllegalArgumentException("you failed; this happened because robotName was not in preferences; rerun preferences and add a robotName in shuffleBoard");
        }
        if (allConfig.get (robotName).containsKey(key) == false){
            throw new IllegalArgumentException("you failed; this happened because " +key+ " does not exsists for " +robotName+ " in your config class you have to add it to the HashMap");
        }
        return allConfig.get(robotName).get(key); 
    }
}

