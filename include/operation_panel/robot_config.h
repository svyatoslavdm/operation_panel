#ifndef ROBOTCONFIG_H_
#define ROBOTCONFIG_H_

#define KAWASAKI
// #define KUKA

#ifdef KAWASAKI

#define ROBOTNAME "kawasaki"
#define TOOLNAME "r/link_6"
#define GROUPNAME "right_arm"
#define JOINT1NAME "r/JT1"
#define JOINT2NAME "r/JT2"
#define JOINT3NAME "r/JT3"
#define JOINT4NAME "r/JT4"
#define JOINT5NAME "r/JT5"
#define JOINT6NAME "r/JT6"

#define SUPJOINT1NAME "l/JT1"
#define SUPJOINT2NAME "l/JT2"
#define SUPJOINT3NAME "l/JT3"
#define SUPJOINT4NAME "l/JT4"
#define SUPJOINT5NAME "l/JT5"
#define SUPJOINT6NAME "l/JT6"

#endif

#ifdef KUKA

#define ROBOTNAME "kuka"
#define TOOLNAME "kuka/tool"
#define GROUPNAME "kuka"
#define JOINT1NAME "kuka/A1"
#define JOINT2NAME "kuka/A2"
#define JOINT3NAME "kuka/A3"
#define JOINT4NAME "kuka/A4"
#define JOINT5NAME "kuka/A5"
#define JOINT6NAME "kuka/A6"

#define SUPJOINT1NAME "kaw/JT1"
#define SUPJOINT2NAME "kaw/JT2"
#define SUPJOINT3NAME "kaw/JT3"
#define SUPJOINT4NAME "kaw/JT4"
#define SUPJOINT5NAME "kaw/JT5"
#define SUPJOINT6NAME "kaw/JT6"

#endif

#endif
