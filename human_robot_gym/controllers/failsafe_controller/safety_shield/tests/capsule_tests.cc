#include <gtest/gtest.h>

#include "verify_iso_fixture.h"
#include "geometry_msgs/Point.h"
#include "custom_robot_msgs/Segment.h"

namespace safety_shield {

TEST_F(VerifyIsoTest, pointPointTest){
  geometry_msgs::Point p1; 
  geometry_msgs::Point p2; 
  p1.x =0; p1.y =0; p1.z=0;
  p2.x =0; p2.y =0; p2.z=0;
  EXPECT_EQ(verify_iso_.pointPointDistance(p1, p2), 0.0) << 
      "Distance between (0,0,0) and (0,0,0) is not 0.";
  p1.x =1; p1.y =1; p1.z=1;
  p2.x =0; p2.y =0; p2.z=0;
  EXPECT_EQ(verify_iso_.pointPointDistance(p1, p2), sqrt(3)) << 
      "Distance between (1,1,1) and (0,0,0) is not sqrt(3).";
  p1.x =-1; p1.y =-1; p1.z=-1;
  p2.x =0; p2.y =0; p2.z=0;
  EXPECT_EQ(verify_iso_.pointPointDistance(p1, p2), sqrt(3)) << 
      "Distance between (-1,-1,-1) and (0,0,0) is not sqrt(3)";
}

TEST_F(VerifyIsoTest, pointSegmentTest){
  geometry_msgs::Point s; 
  geometry_msgs::Point e; 
  s.x =0; s.y =0; s.z=0;
  e.x =1; e.y =0; e.z=0;
  custom_robot_msgs::Segment seg;
  seg.p = s; seg.q = e;
  geometry_msgs::Point p;
  p.x =0; p.y =0; p.z=0;
  EXPECT_EQ(verify_iso_.pointSegmentDistance(p, seg), 0.0) << 
      "Distance between point (0,0,0) and segment x 0-1 is not 0.";
  p.x =0; p.y =1; p.z=0;
  EXPECT_EQ(verify_iso_.pointSegmentDistance(p, seg), 1.0) << 
      "Distance between point (0,1,0) and segment x 0-1 is not 1.";
  p.x =-1; p.y =0; p.z=0;
  EXPECT_EQ(verify_iso_.pointSegmentDistance(p, seg), 1.0) << 
      "Distance between point (-1,0,0) and segment x 0-1 is not 1.";
  p.x =0.5; p.y =1; p.z=0;
  EXPECT_EQ(verify_iso_.pointSegmentDistance(p, seg), 1.0) << 
      "Distance between point (0.5,1,0) and segment x 0-1 is not 1.";
  p.x =1; p.y =1; p.z=0;
  EXPECT_EQ(verify_iso_.pointSegmentDistance(p, seg), 1.0) << 
      "Distance between point (1,1,0) and segment x 0-1 is not 1.";
  p.x =2; p.y =1; p.z=0;
  EXPECT_EQ(verify_iso_.pointSegmentDistance(p, seg), sqrt(2)) << 
      "Distance between point (2,1,0) and segment x 0-1 is not sqrt(2).";
  p.x =0.3; p.y =-1; p.z=-1;
  EXPECT_EQ(verify_iso_.pointSegmentDistance(p, seg), sqrt(2)) << 
      "Distance between point (0.3,-1,-1) and segment x 0-1 is not sqrt(2).";
}

TEST_F(VerifyIsoTest, segmentSegmentTest){
  geometry_msgs::Point s; 
  geometry_msgs::Point e; 
  s.x =0; s.y =0; s.z=0;
  e.x =1; e.y =0; e.z=0;
  custom_robot_msgs::Segment seg;
  seg.p = s; seg.q = e;
  geometry_msgs::Point s1; 
  geometry_msgs::Point e1; 
  custom_robot_msgs::Segment seg1;
  s1.x =0; s1.y =0; s1.z=0;
  e1.x =1; e1.y =0; e1.z=0;
  seg1.p = s1; seg1.q = e1;
  EXPECT_EQ(verify_iso_.segmentsDistance(seg, seg1), 0.0) 
      << "Distance between segment [(0,0,0)-(1,0,0)] and segment [(0,0,0)-(1,0,0)] is not 0.";
  s1.x =1; s1.y =0; s1.z=0;
  e1.x =0; e1.y =0; e1.z=0;
  seg1.p = s1; seg1.q = e1;
  EXPECT_EQ(verify_iso_.segmentsDistance(seg, seg1), 0.0) 
      << "Distance between segment [(0,0,0)-(1,0,0)] and segment [(1,0,0)-(0,0,0)] is not 0.";
  s1.x =0; s1.y =1; s1.z=0;
  e1.x =1; e1.y =1; e1.z=0;
  seg1.p = s1; seg1.q = e1;
  EXPECT_EQ(verify_iso_.segmentsDistance(seg, seg1), 1.0) 
      << "Distance between segment [(0,0,0)-(1,0,0)] and segment [(0,1,0)-(1,1,0)] is not 1.";
  s1.x =-2; s1.y =1; s1.z=0;
  e1.x =3; e1.y =1; e1.z=0;
  seg1.p = s1; seg1.q = e1;
  EXPECT_EQ(verify_iso_.segmentsDistance(seg, seg1), 1.0) 
      << "Distance between segment [(0,0,0)-(1,0,0)] and segment [(-2,1,0)-(3,1,0)] is not 1.";
  s1.x =-2; s1.y =0; s1.z=0;
  e1.x =-1; e1.y =0; e1.z=0;
  seg1.p = s1; seg1.q = e1;
  EXPECT_EQ(verify_iso_.segmentsDistance(seg, seg1), 1.0)
      << "Distance between segment [(0,0,0)-(1,0,0)] and segment [(-2,0,0)-(-1,0,0)] is not 1.";
  s1.x =3; s1.y =0; s1.z=0;
  e1.x =2; e1.y =0; e1.z=0;
  seg1.p = s1; seg1.q = e1;
  EXPECT_EQ(verify_iso_.segmentsDistance(seg, seg1), 1.0) 
      << "Distance between segment [(0,0,0)-(1,0,0)] and segment [(2,0,0)-(3,0,0)] is not 1.";
  s1.x =0.5; s1.y =-1; s1.z=0;
  e1.x =0.5; e1.y =1; e1.z=0;
  seg1.p = s1; seg1.q = e1;
  EXPECT_EQ(verify_iso_.segmentsDistance(seg, seg1), 0.0) 
      << "Distance between segment [(0,0,0)-(1,0,0)] and segment [(0.5,-1,0)-(0.5,1,0)] is not 0.";
  s1.x =0.9; s1.y =0.1; s1.z=0.1;
  e1.x =3; e1.y =3; e1.z=0;
  seg1.p = s1; seg1.q = e1;
  EXPECT_NEAR(verify_iso_.segmentsDistance(seg, seg1), sqrt(0.02), 1E-6) 
      << "Distance between segment [(0,0,0)-(1,0,0)] and segment [(0.9,0.1,0.1)-(3,3,0)] is not sqrt(0.02).";
  s1.x =0.2; s1.y =-0.1; s1.z=0.1;
  e1.x =3; e1.y =3; e1.z=0;
  seg1.p = s1; seg1.q = e1;
  EXPECT_NEAR(verify_iso_.segmentsDistance(seg, seg1), 0.0967, 1E-3) 
      << "Distance between segment [(0,0,0)-(1,0,0)] and segment [(0.2,-0.1,0.1)-(3,3,0)] is not 0.0967.";
}
} // namespace safety_shield

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    // ros::init(argc, argv, "verify iso tester");
    // ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}