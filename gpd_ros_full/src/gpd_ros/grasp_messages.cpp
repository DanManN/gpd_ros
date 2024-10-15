#include <gpd_ros/grasp_messages.h>

gpd_ros::GraspConfigList GraspMessages::createGraspListMsg(const std::vector<std::unique_ptr<gpd::candidate::Hand>>& hands, const std_msgs::Header& header)
{
  gpd_ros::GraspConfigList msg;

  for (int i = 0; i < hands.size(); i++) {
    msg.grasps.push_back(convertToGraspMsg(*hands[i]));
  }

  msg.header = header;

  return msg;
}

gpd_ros::GraspConfig GraspMessages::convertToGraspMsg(const gpd::candidate::Hand& hand)
{
  gpd_ros::GraspConfig msg;
  tf::pointEigenToMsg(hand.getPosition(), msg.position);
  tf::vectorEigenToMsg(hand.getApproach(), msg.approach);
  tf::vectorEigenToMsg(hand.getBinormal(), msg.binormal);
  tf::vectorEigenToMsg(hand.getAxis(), msg.axis);
  msg.width.data = hand.getGraspWidth();
  msg.score.data = hand.getScore();
  tf::pointEigenToMsg(hand.getSample(), msg.sample);

  return msg;
}

std::vector<std::unique_ptr<gpd::candidate::Hand>> GraspMessages::createHands(const gpd_ros::GraspConfigList& grasps)
{
  std::vector<std::unique_ptr<gpd::candidate::Hand>> hands;

  for (int i = 0; i < grasps.grasps.size(); i++) {
    hands.push_back(convertToHand(grasps.grasps[i]));
  }

  return hands;
}

std::unique_ptr<gpd::candidate::Hand> GraspMessages::convertToHand(const gpd_ros::GraspConfig& grasp)
{
  std::unique_ptr<gpd::candidate::Hand> hand(new gpd::candidate::Hand());
  tf::pointMsgToEigen(grasp.position, hand->position_);
  Eigen::Vector3d temp;
  tf::vectorMsgToEigen(grasp.approach, temp);
  hand->orientation_.col(0) = temp;
  tf::vectorMsgToEigen(grasp.binormal, temp);
  hand->orientation_.col(1) = temp;
  tf::vectorMsgToEigen(grasp.axis, temp);
  hand->orientation_.col(2) = temp;
  tf::pointMsgToEigen(grasp.sample, temp);
  hand->sample_ = temp;
  hand->setGraspWidth(grasp.width.data);
  // hand->setScore(grasp.score.data);

  return hand;
}
