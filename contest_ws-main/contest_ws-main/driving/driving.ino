/*
 * Contest workspace entry point for the CURRENT vehicle firmware.
 *
 * Keep a single source of truth for wiring, serial protocol, calibration and
 * watchdog behavior.  From this directory the canonical firmware is located
 * at the relative path below.
 *
 * Current wiring in that firmware:
 *   Steering IN1=D3, IN2=D2
 *   Right    IN1=D4, IN2=D5
 *   Left     IN1=D7, IN2=D6
 *   Potentiometer OUT=A2
 *   Serial 115200 baud
 *
 * Open/upload this sketch from the repository in its present directory layout,
 * or upload Autonomous_Capstone-integrated/src/control/driving_user_pins/
 * driving_user_pins.ino directly.
 */

#include "../../../Autonomous_Capstone-integrated/src/control/driving_user_pins/driving_user_pins.ino"
