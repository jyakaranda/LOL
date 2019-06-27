/**
 * @file main.cpp
 * @author heng zhang (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2019-05-11
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#include "lol/lolLocalization.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lol");
  ros::NodeHandle nh, pnh("~");

  localization::LolLocalization *lol = new localization::LolLocalization(nh, pnh);
  std::thread opt_thread(&localization::LolLocalization::optimizeThread, lol);
  lol->run();

  ros::spin();
  return 0;
}