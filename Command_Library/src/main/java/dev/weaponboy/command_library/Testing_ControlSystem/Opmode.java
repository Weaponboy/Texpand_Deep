package dev.weaponboy.command_library.Testing_ControlSystem;

import dev.weaponboy.command_library.OpmodeEX.OpModeEX;

public class Opmode {

    static OpModeEX opmode = new OpModeEX() {
        @Override
        public void initEX() {

        }

        @Override
        public void loopEX() {

        }
    };

    static SchedulerTest schedulerTest = new SchedulerTest(opmode);

    public static void main(String[] args) {

        schedulerTest.subsystem.queueCommand(schedulerTest.subsystem.defaultCommandSecond);
        schedulerTest.subsystem.queueCommand(schedulerTest.subsystem.defaultCommandThird);

        for(int i = 0; i < 50; i++){
            schedulerTest.execute();
        }

    }

}
