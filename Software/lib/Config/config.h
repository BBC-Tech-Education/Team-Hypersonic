#ifndef CONFIG_H
#define CONFIG_H

/* COMPETITION MODE */
#define COMP true

/* ROBOT */
#define ROBOT_1 true
#define ROBOT_2 (!ROBOT_1)
#define ATTACK false

/* ATTACK DIRECTION */
#define BLUE_GOAL_ATTACK false
// True: Attacking Blue Goal (Defending Yellow Goal)
// False: Attacking Yellow Goal (Defending Blue Goal)

/* DEBUG */
#define DEBUG_MODE 0
#define DEBUG          (DEBUG_MODE != 0)
#define DEBUG_BATTERY  (DEBUG_MODE == 1)
#define DEBUG_IMU      (DEBUG_MODE == 2)
#define DEBUG_CAMERA   (DEBUG_MODE == 3)
#define DEBUG_LS       (DEBUG_MODE == 4)

#endif
