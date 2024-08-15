package dev.weaponboy.command_library.Commands;

public interface Command {

    void execute();

    void init();

    boolean isFinished();

}
