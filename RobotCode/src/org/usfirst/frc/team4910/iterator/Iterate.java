package org.usfirst.frc.team4910.iterator;

/**
 * This interface is used by every subsystem.
 * As it extendes the Runnable interface a
 * class that implements it is a thread.
 */
public interface Iterate extends Runnable {
	public void init();
	public void run();
	public void end();
}
