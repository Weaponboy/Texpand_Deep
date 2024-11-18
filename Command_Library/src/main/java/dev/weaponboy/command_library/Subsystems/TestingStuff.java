package dev.weaponboy.command_library.Subsystems;

import dev.weaponboy.command_library.Hardware.motionProfile;

public class TestingStuff {

    static motionProfile profile = new motionProfile(1700, 210, 75, 1100, 0.35);

    public static void main(String[] args) {
        profile.generateMotionProfile(50, 0);
        int value = 0;
        profile.followProfile(value);
        while (profile.isSlideRunning()){
            System.out.println(profile.followProfile(value));
            value+=3;
        }

    }
}
