#ifndef JOINTS_H_
#define JOINTS_H_

// Standard table of Atlas joint names
const std::vector<std::string> ATLAS_JOINT_NAMES = {
  "back_lbz", "back_mby", "back_ubx", "neck_ay",
  "l_leg_uhz", "l_leg_mhx", "l_leg_lhy", "l_leg_kny", "l_leg_uay", "l_leg_lax",
  "r_leg_uhz", "r_leg_mhx", "r_leg_lhy", "r_leg_kny", "r_leg_uay", "r_leg_lax",
  "l_arm_usy", "l_arm_shx", "l_arm_ely", "l_arm_elx", "l_arm_uwy", "l_arm_mwx",
  "r_arm_usy", "r_arm_shx", "r_arm_ely", "r_arm_elx", "r_arm_uwy", "r_arm_mwx"
};

const std::vector<std::string> AUGMENTED_ATLAS_JOINT_NAMES = {
  "back_lbz", "back_mby", "back_ubx", "neck_ay",
  "l_leg_uhz", "l_leg_mhx", "l_leg_lhy", "l_leg_kny", "l_leg_uay", "l_leg_lax",
  "r_leg_uhz", "r_leg_mhx", "r_leg_lhy", "r_leg_kny", "r_leg_uay", "r_leg_lax",
  "l_arm_usy", "l_arm_shx", "l_arm_ely", "l_arm_elx", "l_arm_uwy", "l_arm_mwx",
  "r_arm_usy", "r_arm_shx", "r_arm_ely", "r_arm_elx", "r_arm_uwy", "r_arm_mwx",
  "left_f3_j0", "right_f3_j0"
};

const std::vector<int> TORSO_JOINTS     = { 0, 1, 2, 3};
const std::vector<int> LEFT_LEG_JOINTS  = { 4, 5, 6, 7, 8, 9};
const std::vector<int> RIGHT_LEG_JOINTS = {10,11,12,13,14,15};
const std::vector<int> LEFT_ARM_JOINTS  = {16,17,18,19,20,21,28};
const std::vector<int> RIGHT_ARM_JOINTS = {22,23,24,25,26,27,29};

const std::vector<std::vector<int>> LIMBS = { TORSO_JOINTS,
                                              LEFT_LEG_JOINTS,
                                              RIGHT_LEG_JOINTS,
                                              LEFT_ARM_JOINTS,
                                              RIGHT_ARM_JOINTS };

#endif /* JOINTS_H_ */
