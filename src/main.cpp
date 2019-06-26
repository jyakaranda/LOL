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
#include "lego_lol/lolLocalization.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lol");
  ros::NodeHandle nh, pnh("~");

  localization::LolLocalization *lol = new localization::LolLocalization(std::shared_ptr<ros::NodeHandle>(&nh), std::shared_ptr<ros::NodeHandle>(&pnh));
  std::thread opt_thread(&localization::LolLocalization::optimizeThread, lol);
  lol->run();

  ros::spin();
  return 0;
}