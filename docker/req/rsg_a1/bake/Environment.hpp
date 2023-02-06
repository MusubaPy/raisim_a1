//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//

#pragma once

#include <stdlib.h>
#include <set>
#include "../../RaisimGymEnv.hpp"
#include "raisim/OgreVis.hpp"
#include "visSetupCallback.hpp"

#include "visualizer/raisimKeyboardCallback.hpp"
#include "visualizer/helper.hpp"
#include "visualizer/guiState.hpp"
#include "visualizer/raisimBasicImguiPanel.hpp"

#include "visualizer/a1/a1_imgui_render_callback.hpp"
#include "visualizer/a1/gaitLogger.hpp"
#include "visualizer/a1/jointSpeedTorqueLogger.hpp"
#include "visualizer/a1/RewardLogger.hpp"
#include "visualizer/a1/videoLogger.hpp"
#include "visualizer/a1/frameVisualizer.hpp"

namespace raisim {

class ENVIRONMENT : public RaisimGymEnv {

 public:

  explicit ENVIRONMENT(const std::string& resourceDir, const Yaml::Node& cfg, bool visualizable) :
      RaisimGymEnv(resourceDir, cfg), visualizable_(visualizable) {
    //std::cout<<"Dirs ="<<resourceDir<<std::endl;
    std::default_random_engine generator;
    std::normal_distribution<double> distribution(0.0, 0.3);
    /// add objects
    a1_ = world_->addArticulatedSystem(resourceDir_+"/a1/urdf/a1.urdf");
    a1_->setName("Unitree A1");
    a1_->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
    auto ground = world_->addGround();

    /// get robot data
    
    gcDim_ = a1_->getGeneralizedCoordinateDim();
    gvDim_ = a1_->getDOF();
    nJoints_ = gvDim_ - 6; // 12
   
    std::cout<<"nJoints ="<<gcDim_<<std::endl;
    /// initialize containers
    gc_.setZero(gcDim_); gc_init_.setZero(gcDim_); gc_wish_.setZero(gcDim_);
    gv_.setZero(gvDim_); gv_init_.setZero(gvDim_);
    pTarget_.setZero(gcDim_); vTarget_.setZero(gvDim_); pTarget12_.setZero(nJoints_);

    normalForce.setZero(3);
    sumNormalForce.setZero(4);

    jointTorque.setZero(12); jointSpeed.setZero(12);
    obTorque_.setZero(12); deltaTorque_.setZero(12);
    obVelocity_.setZero(12); deltaVelocity_.setZero(12);

    /// this is nominal configuration of uniA1
    gc_init_ << 0.0, 0.0, 0.12, 1.0, 0.00, 0.0, 0.0, -0.36, 1.14, -2.69, 0.36, 1.14, -2.69, -0.36, 1.14, -2.69, 0.36, 1.14, -2.69;
    gc_wish_ << 0.0, 0.0, 0.39, 1.0, 0.0, 0.0, 0.0, 0.06, 0.6, -1.2, -0.06, 0.6, -1.2, 0.06, 0.6, -1.2, -0.06, 0.6, -1.2;
    //gc_init_ << 0.0, 0.0, 0.39, 1.0, 0.0, 0.0, 0.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    //P_x, P_y, P_z,!!!, A_x, A_y, A_z, FR_hip, FR_thigh, FR_calf, FL_hip, FL_thigh, FL_calf, RR_hip, RR_thigh, RR_calf, RL_hip, RL_thigh, RL_calf.
    /// set pd gains

    //raisim::a1_gui::reward::clear();

    Eigen::VectorXd jointPgain(gvDim_), jointDgain(gvDim_);
    jointPgain.setZero(); //jointPgain.tail(nJoints_).setConstant(300.0);
    jointDgain.setZero(); //jointDgain.tail(nJoints_).setConstant(8);
    
    jointPgain(0+6) = 100.0; jointPgain(1+6) = 300.0; jointPgain(2+6) = 300.0;
    jointPgain(3+6) = 100.0; jointPgain(4+6) = 300.0; jointPgain(5+6) = 300.0;
    jointPgain(6+6) = 100.0; jointPgain(7+6) = 300.0; jointPgain(8+6) = 300.0;
    jointPgain(9+6) = 100.0; jointPgain(10+6) = 300.0; jointPgain(11+6) = 300.0;

    jointDgain(0+6) = 5.0; jointDgain(1+6) = 8.0; jointDgain(2+6) = 8.0;
    jointDgain(3+6) = 5.0; jointDgain(4+6) = 8.0; jointDgain(5+6) = 8.0;
    jointDgain(6+6) = 5.0; jointDgain(7+6) = 8.0; jointDgain(8+6) = 8.0;
    jointDgain(9+6) = 5.0; jointDgain(10+6) = 8.0; jointDgain(11+6) = 8.0;

    a1_->setPdGains(jointPgain, jointDgain);
    a1_->setGeneralizedForce(Eigen::VectorXd::Zero(gvDim_));

    /// MUST BE DONE FOR ALL ENVIRONMENTS
    obDim_ = 39;
    actionDim_ = nJoints_; actionMean_.setZero(actionDim_); actionStd_.setZero(actionDim_);
    obDouble_.setZero(obDim_);

    /// action scaling
    actionMean_ = gc_init_.tail(nJoints_);
    actionStd_.setConstant(0.3);

    /// Reward coefficients
    rewards_.initializeFromConfigurationFile (cfg["reward"]);

    /// indices of links that should NOT make contact with ground
    footIndices_.insert(a1_->getBodyIdx("RR_hip"));
    footIndices_.insert(a1_->getBodyIdx("RL_hip"));
    footIndices_.insert(a1_->getBodyIdx("FR_hip"));
    footIndices_.insert(a1_->getBodyIdx("FL_hip"));
    //footIndices_.insert(a1_->getBodyIdx("base"));
    //footIndices_.insert(a1_->getBodyIdx("RR_thigh"));
    //footIndices_.insert(a1_->getBodyIdx("RL_thigh"));
    //footIndices_.insert(a1_->getBodyIdx("FR_thigh"));
    //footIndices_.insert(a1_->getBodyIdx("FL_thigh"));



    /// indices of links that should make contact with ground
    contactIndices_.insert(a1_->getBodyIdx("RR_calf"));
    contactIndices_.insert(a1_->getBodyIdx("RL_calf"));
    contactIndices_.insert(a1_->getBodyIdx("FR_calf"));
    contactIndices_.insert(a1_->getBodyIdx("FL_calf"));
    
//    std::cout<<"Idx contact1 = "<<a1_->getBodyIdx("RR_calf")<<std::endl;
//    std::cout<<"Idx contact1 = "<<a1_->getBodyIdx("RL_calf")<<std::endl;
//    std::cout<<"Idx contact1 = "<<a1_->getBodyIdx("FR_calf")<<std::endl;
//    std::cout<<"Idx contact1 = "<<a1_->getBodyIdx("FL_calf")<<std::endl;


    /// visualize if it is the first environment
    if (visualizable_) 
    {
      auto vis = raisim::OgreVis::get();
      
      a1_gui::init({a1_gui::video::init(vis->getResourceDir()),
                    a1_gui::joint_speed_and_torque::init(100),
                    a1_gui::gait::init(100),
                    a1_gui::reward::init({"difference", "GeneralHipsAngles", "BodyHeight", "GeneralBodyOrientation", "axisPos", "GenVelocity", "GenTorque"}),
                    a1_gui::frame::init()});

      /// these method must be called before initApp
      vis->setWorld(world_.get());
      vis->setWindowSize(1920, 1080);
      vis->setImguiSetupCallback(imguiSetupCallback);
      vis->setImguiRenderCallback(raisim::a1_gui::a1ImguiRenderCallBack);
      vis->setKeyboardCallback(raisimKeyboardCallback);
      vis->setSetUpCallback(setupCallback);
      vis->setAntiAliasing(2);

      /// starts visualizer thread
      vis->initApp();
      a1Visual_ = vis->createGraphicalObject(a1_, "Uni A1");
      vis->createGraphicalObject(ground, 20, "floor", "checkerboard_green");
      desired_fps_ = 60.;
      vis->setDesiredFPS(desired_fps_);
      vis->select(a1Visual_->at(0), false);
      vis->getCameraMan()->setYawPitchDist(Ogre::Radian(-1.0), Ogre::Radian(-1.0), 3);
    }

    

  }



  void init() final { }

  void reset() final {
    a1_->setState(gc_init_, gv_init_);
    updateObservation(); 
    /*
    raisim::a1_gui::reward::clear();
    raisim::a1_gui::gait::clear();
    raisim::a1_gui::joint_speed_and_torque::clear();
    */
    if(visualizable_)
      raisim::a1_gui::reward::clear();
  }

  float step(const Eigen::Ref<EigenVec>& action) final {
    /// action scaling
    pTarget12_ = action.cast<double>();
    pTarget12_ = pTarget12_.cwiseProduct(actionStd_);
    pTarget12_ += actionMean_;
    pTarget_.tail(nJoints_) = pTarget12_;

    a1_->setPdTarget(pTarget_, vTarget_);

    auto visDecimation = int(1. / (desired_fps_ * simulation_dt_) + 1e-10);

    for(int i=0; i< int(control_dt_ / simulation_dt_ + 1e-10); i++){
      world_->integrate();
      if (visualizable_ && visualizeThisStep_ && visualizationCounter_ % visDecimation == 0)
        raisim::OgreVis::get()->renderOneFrame();
      visualizationCounter_++;
    }

    updateObservation();

    ClearAndSlip = calcFootClearanceAndSlipCost(footClearanceCost_, footSlipCost_);
    //std::cout<<"ClearAndSlip ="<<ClearAndSlip<<std::endl;
    //std::cout<<"footClearanceCost_ ="<<footClearanceCost_<<std::endl;
    //std::cout<<"footSlipCost_ ="<<footSlipCost_<<std::endl;

    rewards_.record("GenTorque", a1_->getGeneralizedForce().squaredNorm());
    rewards_.record("GenVelocity", a1_->getGeneralizedVelocity().squaredNorm());
    rewards_.record("axisPos", abs(gc_[1])+abs(gc_[0])); 
    rewards_.record("GeneralHipsAngles", generalHipsAngles);
    rewards_.record("GeneralBodyOrientation", generalBodyOrientation);
    rewards_.record("BodyHeight", BodyHeight);
    rewards_.record("difference", difference);
    //rewards_.record("rewardSmoothTorqVel", rewardSmoothTorqVel);
    //rewards_.record("fullContact", numContacts_);
    

    if(visualizeThisStep_) 
    {

    jointTorque = a1_->getGeneralizedForce().e().tail(12);

    jointSpeed = a1_->getGeneralizedVelocity().e().tail(12);
    /// torque, speed and contact state
    a1_gui::joint_speed_and_torque::push_back(world_->getWorldTime(), jointSpeed, jointTorque);
    a1_gui::gait::push_back(footContactState);
    //std::cout<<"jointSpeed ="<<jointSpeed<<std::endl;
    
    a1_gui::reward::log("GenTorque", a1_->getGeneralizedForce().squaredNorm() * rewards_.getCoef("GenTorque"));
    a1_gui::reward::log("GenVelocity",a1_->getGeneralizedVelocity().squaredNorm() * rewards_.getCoef("GenVelocity"));
    a1_gui::reward::log("axisPos", (abs(gc_[1])+abs(gc_[0])) * rewards_.getCoef("axisPos"));
    a1_gui::reward::log("GeneralHipsAngles", generalHipsAngles * rewards_.getCoef("GeneralHipsAngles"));
    a1_gui::reward::log("GeneralBodyOrientation", generalBodyOrientation * rewards_.getCoef("GeneralBodyOrientation"));
    a1_gui::reward::log("BodyHeight",  BodyHeight * rewards_.getCoef("BodyHeight"));
    a1_gui::reward::log("difference",  difference * rewards_.getCoef("difference"));
    //a1_gui::reward::log("rewardSmoothTorqVel",  rewardSmoothTorqVel * rewards_.getCoef("rewardSmoothTorqVel"));
    //a1_gui::reward::log("fullContact",  numContacts_ * rewards_.getCoef("fullContact"));

      /// reset camera
      auto vis = raisim::OgreVis::get();

      vis->select(a1Visual_->at(0), false);
      vis->getCameraMan()->setYawPitchDist(Ogre::Radian(3.14), Ogre::Radian(-1.3), 3, true);
      // vis->getCameraMan()->setYawPitchDist(Ogre::Radian(1.3), Ogre::Radian(-1.3), 5, true);
    }



    return rewards_.sum();
  }

double calcFootClearanceAndSlipCost(double& footClearanceCost, double& footSlipCost) {
    double c_f = 0.1 * world_->getTimeStep();
    double c_fv = 2.0 * world_->getTimeStep();
    double p_f_hat = 0.07;
    double k_c_= 0.1;

    std::set<long> contactedFoot;
    for(auto& contact: a1_->getContacts()) {
      contactedFoot.insert(contact.getlocalBodyIndex());
      if (footIndices_.find(contact.getlocalBodyIndex()) == footIndices_.end()) {
        return true;
      }
    }

    footClearanceCost = 0.0;
    footSlipCost = 0.0;
    for (auto footIndex : footIndices_) {
      raisim::Vec<3> pos, vel;
      a1_->getPosition(footIndex, pos);
      a1_->getVelocity(footIndex, vel);
      if (contactedFoot.find(footIndex) == contactedFoot.end())
        footClearanceCost += k_c_ * c_f * (p_f_hat - pos[2]) * (p_f_hat - pos[2]) * vel.norm();
      else
        footSlipCost += k_c_ * c_fv * vel.norm();
    }

    return footClearanceCost + footSlipCost;
  }





  void updateObservation() {
    a1_->getState(gc_, gv_);
    raisim::Vec<4> quat;
    raisim::Mat<3,3> rot;
    quat[0] = gc_[3]; quat[1] = gc_[4]; quat[2] = gc_[5]; quat[3] = gc_[6];
    raisim::quatToRotMat(quat, rot);
    bodyLinearVel_ = rot.e().transpose() * gv_.segment(0, 3);
    bodyAngularVel_ = rot.e().transpose() * gv_.segment(3, 3);
    numContacts_ = a1_->getContacts().size();
    //std::cout<<"Num contacts ="<<numContacts_<<std::endl;
    airRotation_ = rot.e().row(2).transpose();
    //std::cout<<"air Rotation ="<<airRotation_<<std::endl;
    //generalHipsAngles = abs(gc_[7]) + abs(gc_[10]) + abs(gc_[13]) + abs(gc_[16]) + abs(gc_[9]) + abs(gc_[12]) + abs(gc_[15]) + abs(gc_[18]);
    generalHipsAngles = abs(abs(gc_wish_[9]) - abs(gc_[9])) + abs(abs(gc_wish_[12]) - abs(gc_[12])) + abs(abs(gc_wish_[15]) - abs(gc_[15])) + abs(abs(gc_wish_[18]) - abs(gc_[18]));
    generalHipsAngles = generalHipsAngles + abs(abs(gc_wish_[10]) - abs(gc_[10])) + abs(abs(gc_wish_[13]) - abs(gc_[13])) + abs(abs(gc_wish_[16]) - abs(gc_[16])) + abs(abs(gc_wish_[19]) - abs(gc_[19]));
    generalHipsAngles = generalHipsAngles + abs(abs(gc_wish_[8]) - abs(gc_[8])) + abs(abs(gc_wish_[11]) - abs(gc_[11])) + abs(abs(gc_wish_[14]) - abs(gc_[14])) + abs(abs(gc_wish_[17]) - abs(gc_[17]));;

    generalBodyOrientation = abs(airRotation_[0]) + abs(airRotation_[1]) + abs(airRotation_[2]);

    difference = abs(abs(gc_[9]) - abs(gc_[12])) + abs(abs(gc_[15]) - abs(gc_[18]));
    difference = difference + abs(abs(gc_[10]) - abs(gc_[13])) + abs(abs(gc_[16]) - abs(gc_[19]));
    //BodyHeight = abs(gc_init_[2] - gc_[2]);
    BodyHeight = gc_[2];

    if (BodyHeight < 0.35)
      {
        //BodyHeight = (1 - BodyHeight) * 10;
        BodyHeight = BodyHeight;
      }
    else
      {
        BodyHeight = -BodyHeight*10;
      }
    //deltaVelocity_ << abs(obVelocity_.tail(12) - jointSpeed.tail(12));
    //deltaTorque_ << abs(obTorque_.tail(12) - jointTorque.tail(12));

    for (int i = 0; i<=12; i++)
    {
      rewardSmoothVel = rewardSmoothVel + abs(abs(obVelocity_(i)) - abs(jointSpeed(i)));
      //rewardSmoothTorq = rewardSmoothTorq + abs(abs(obTorque_(i)) - abs(jointTorque(i)));
      //rewardSmoothTorqVel = rewardSmoothTorqVel + deltaVelocity_[i] + deltaTorque_[i];
      //std::cout<<"Data: i/vel/torq/ -- ="<<i<<"/"<<deltaVelocity_[i]<<"/"<<deltaTorque_[i]<<std::endl;
    }
    //rewardSmoothTorqVel = std::min(360.00, round(rewardSmoothVel*10e5)/10e5) + std::min(1920.00, round(rewardSmoothVel*10e5)/10e5);
    rewardSmoothTorqVel = std::min(3000.00, round(rewardSmoothVel*10e3)/10e3);
    //std::cout<<"obVelocity_ ="<<rewardSmoothTorqVel<<std::endl;
    //rewardSmoothTorqVel = 1;
    
    obVelocity_ << jointSpeed.tail(12);
    obTorque_ << jointTorque.tail(12);
    
    obDouble_ << gc_[2], /// body height 1
        airRotation_, /// body orientation 3
        gc_.tail(12), /// joint angles 12
        bodyLinearVel_, bodyAngularVel_, /// body linear&angular velocity 6
        gv_.tail(12),/// joint velocity 12
        numContacts_,/// num contacts 1
        sumNormalForce.tail(4);/// Contact force 4
//    std::cout<<"body height ="<<gc_[2]<<std::endl; // 
//    std::cout<<"body orientation ="<<rot.e().row(2).transpose()<<std::endl; // 3
//    std::cout<<"joint angles ="<<gc_.tail(12)<<std::endl; // 12
//    std::cout<<"body Lin.vel. ="<<bodyLinearVel_<<std::endl; // 3
//    std::cout<<"body Ang.vel. ="<<bodyAngularVel_<<std::endl; // 3
//    std::cout<<"joint vel. ="<<gv_.tail(12)<<std::endl; //12
//    std::cout<<"Sum Normal Force ="<<sumNormalForce.tail(4)<<std::endl; //4
//    normalno = contact.normal().e().transpose();
  }

  void observe(Eigen::Ref<EigenVec> ob) final {
    /// convert it to float
    ob = obDouble_.cast<float>();
  }



  bool isTerminalState(float& terminalReward) final 
  {
    terminalReward = float(terminalRewardCoeff_); 


    for(auto& fs: footContactState) fs = false;
    /// if the contact body is not feet
    for(auto& contact: a1_->getContacts())
    {
      if(footIndices_.find(contact.getlocalBodyIndex()) != footIndices_.end()) return true;

      auto normalForce = contact.getContactFrame().e() * contact.getImpulse()->e() / world_->getTimeStep();
      netForce_ = sqrt(pow(normalForce[0],2) + pow(normalForce[1],2) + pow(normalForce[2],2));
      
      if(contactIndices_.find(contact.getlocalBodyIndex()) != contactIndices_.end())
      {
        if(contact.getlocalBodyIndex() == 3)
        {
          //std::cout<<"Leg ID = "<<contact.getlocalBodyIndex()<< "; Normal Force = "<<netForce_<<std::endl;
          sumNormalForce[0] = netForce_;
          footContactState[1] = true;
        }
        if(contact.getlocalBodyIndex() == 6)
        {
          sumNormalForce[1] = netForce_;
          footContactState[0] = true;

        }
        if(contact.getlocalBodyIndex() == 9)
        {
          sumNormalForce[2] = netForce_;
          footContactState[3] = true;
        }
        if(contact.getlocalBodyIndex() == 12)
        {
          sumNormalForce[3] = netForce_;
          footContactState[2] = true;
        }
      }    
    }


    terminalReward = 0.f;
    return false;
  }

  





 private:
  int gcDim_, gvDim_, nJoints_, numContacts_, visualizationCounter_ = 0;
  bool visualizable_ = false;
  raisim::ArticulatedSystem* a1_;
  Eigen::VectorXd gc_init_, gc_wish_, gv_init_, gc_, gv_, pTarget_, pTarget12_, vTarget_, normalForce, sumNormalForce;
  double terminalRewardCoeff_ = -5.00, netForce_ = 0.00, generalHipsAngles = 0.00, desired_fps_ = 50.00, generalBodyOrientation = 0.00, difference = 0.00;
  Eigen::VectorXd actionMean_, actionStd_, obDouble_, jointTorque, jointSpeed, obTorque_, obVelocity_, deltaTorque_, deltaVelocity_;
  Eigen::Vector3d bodyLinearVel_, bodyAngularVel_, airRotation_;
  double footClearanceCost_ = 0.00, footSlipCost_ = 0.00, ClearAndSlip = 0.00, BodyHeight = 0.00;
  double rewardSmoothTorqVel = 0.00, rewardSmoothVel = 0.00, rewardSmoothTorq = 0.00;
  std::set<size_t> footIndices_;
  std::set<size_t> contactIndices_;
  std::array<bool, 4> footContactState, allur;
  std::vector<GraphicObject> * a1Visual_;

};
}

