/*
    author:gui xiangyu
    location:xi'an jiaotong university
*/

#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_io_interface.h>

#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <cassert>
#include <cmath>
#include <mutex>
#include <condition_variable>

using namespace ur_rtde;
using namespace std::chrono;
using namespace std;


bool isStage1Begin = false;   // isStage1Begin,isStage1End are used to keep right_arm: home_right to install_ready_right segment 
bool isStage1End = false;                                              //and left_arm: install_mid1 to install_ready segment  synchronously

bool isStage2Begin = false;   // isStage2Begin,isStage2End are used to keep right_arm: install_ready_right to install_ready1_right 
bool isStage2End = false;                                             //and left_arm: silent

bool isStage3Begin = false;  // isStage3Begin, isStage3End are used to keep right_arm: install_ready1_right to install_right 
bool isStage3End = false;                                             //and left_arm: install_ready to install segment synchronously

bool isStage4Begin = false;  // isStage4Begin , isStage4End are used to keep right_arm: setStandardDigitalOut(0,false);
bool isStage4End = false;                                             //and left_arm: setStandardDigitalOut(0,false); synchronously

bool isStage5Begin = false;  // isStage4Begin , isStage5End are used to keep right_arm:  install_right to home_right
bool isStage5End = false;                                             //and left_arm:  install  to home synchronously


//if the path is relatively short,you can use sparse shape.
//s is the start vector,e is the end vector,dt is the time interval between two interpolation point.
//the whole path is 2s.
vector<vector<double>> threetimePlan_sparse(vector<double> & s,vector<double> & e, double dt){
  
  vector<vector<double>> invT(4,vector<double>(4,0));
  invT[0][0] = 1.0;
  invT[1][2] = 1.0;
  invT[2][0] = -3.0/(2*2);
  invT[2][1] = 3.0/(2*2);
  invT[2][2] = -2.0/2;
  invT[2][3] = -1.0/2;
  invT[3][0] = 2.0/(2*2*2);
  invT[3][1] = -2.0/(2*2*2);
  invT[3][2] = 1.0/(2*2);
  invT[3][3] = 1.0/(2*2); 

  vector<vector<double>> matFor46(4,vector<double>(6,0));
  for(int i=0;i<6;++i)  matFor46[0][i] = s[i];
  for(int i=0;i<6;++i)  matFor46[1][i] = e[i];
  
  vector<vector<double>> coefficient(4,vector<double>(6,0));
  
  for(int i=0;i<4;++i){
    for(int j=0;j<6;++j){
       for(int k=0;k<4;++k){
           coefficient[i][j] += invT[i][k]* matFor46[k][j];
       }
    }
  }

  vector<vector<double>> inter(250,vector<double>(6,0));

  for(int i=0;i<200;++i){
    for(int j=0;j<6;++j){
        inter[i][j] = coefficient[0][j] + coefficient[1][j]*(dt*i)+ 
                   coefficient[2][j]*(dt*i*dt*i)+coefficient[3][j]*(dt*i*dt*i*dt*i);
     }
  }

  for(int i=200;i<250;++i){
    for(int j=0;j<6;++j){
        inter[i][j] = e[j];
     }
  }

  return inter;
}

////if the path is relatively long,you can use dense shape.
vector<vector<double>> threetimePlan_dense(vector<double> & s,vector<double> & e, double dt){
  
  vector<vector<double>> invT(4,vector<double>(4,0));
  invT[0][0] = 1.0;
  invT[1][2] = 1.0;
  invT[2][0] = -3.0/(2*2);
  invT[2][1] = 3.0/(2*2);
  invT[2][2] = -2.0/2;
  invT[2][3] = -1.0/2;
  invT[3][0] = 2.0/(2*2*2);
  invT[3][1] = -2.0/(2*2*2);
  invT[3][2] = 1.0/(2*2);
  invT[3][3] = 1.0/(2*2); 

  vector<vector<double>> matFor46(4,vector<double>(6,0));
  for(int i=0;i<6;++i)  matFor46[0][i] = s[i];
  for(int i=0;i<6;++i)  matFor46[1][i] = e[i];
  
  vector<vector<double>> coefficient(4,vector<double>(6,0));
  
  for(int i=0;i<4;++i){
    for(int j=0;j<6;++j){
       for(int k=0;k<4;++k){
           coefficient[i][j] += invT[i][k]* matFor46[k][j];
       }
    }
  }

  vector<vector<double>> inter(600,vector<double>(6,0));

  for(int i=0;i<500;++i){
    for(int j=0;j<6;++j){
        inter[i][j] = coefficient[0][j] + coefficient[1][j]*(dt*i)+ 
                   coefficient[2][j]*(dt*i*dt*i)+coefficient[3][j]*(dt*i*dt*i*dt*i);
     }
     
  }

  for(int i=500;i<600;++i){
    for(int j=0;j<6;++j){
        inter[i][j] = e[j];
     }
  }

  return inter;
}

//Func1 is the son Funcion which is used as the parameter for thread
void Func1(){
  // Parameters
  double velocity = 0.5;
  double acceleration = 0.2;
  double dt = 2.0/200; // 2ms
  double lookahead_time = 0.08;
  double gain = 800;


  std::string hostname_right = "192.168.1.40";  //right_arm's ip
  
  RTDEReceiveInterface rtde_receive_right(hostname_right);
  RTDEControlInterface rtde_control_right(hostname_right);
  RTDEIOInterface rtde_io_right(hostname_right);

  while(isStage1Begin == false){
      this_thread::sleep_for(chrono::milliseconds(2));
  }
  //home_right到install_ready_right段
  vector<double> currentJoint_right = rtde_receive_right.getActualQ();
  vector<double> home_right{62.81*0.0174533 ,-139.95*0.0174533 ,127.70*0.0174533 ,-33.87*0.0174533 ,100.12*0.0174533, 30.01*0.0174533};
  assert( abs(currentJoint_right[0] - home_right[0]) < 0.01 || abs(currentJoint_right[1] - home_right[1]) < 0.01 || abs(currentJoint_right[2] - home_right[2]) < 0.01 || abs(currentJoint_right[3] - home_right[3]) < 0.01);
  vector<double> install_ready_right{ 0.972229,-1.76563,1.37847,-0.41572,1.68283,0.635854};
  vector<vector<double>> inter_homeright_installreadyright = threetimePlan_sparse(home_right,install_ready_right,dt);
  vector<double> joint_homeright_installreadyright = {inter_homeright_installreadyright[0][0] ,
                                           inter_homeright_installreadyright[0][1] , 
                                           inter_homeright_installreadyright[0][2] ,
                                           inter_homeright_installreadyright[0][3] ,
                                           inter_homeright_installreadyright[0][4] ,
                                           inter_homeright_installreadyright[0][5]};
  // Execute 500Hz control loop for 2 seconds, each cycle is ~2ms
  for (unsigned int i=1; i<250; i++)
  {
    auto t_start_right = high_resolution_clock::now();
    rtde_control_right.servoJ(joint_homeright_installreadyright, velocity, acceleration, dt, lookahead_time, gain);
    //rtde_control.speedJ(joint_q,  acceleration, dt);
    joint_homeright_installreadyright[0] = inter_homeright_installreadyright[i][0];
    joint_homeright_installreadyright[1] = inter_homeright_installreadyright[i][1];
    joint_homeright_installreadyright[2] = inter_homeright_installreadyright[i][2];
    joint_homeright_installreadyright[3] = inter_homeright_installreadyright[i][3];
    joint_homeright_installreadyright[4] = inter_homeright_installreadyright[i][4];
    joint_homeright_installreadyright[5] = inter_homeright_installreadyright[i][5];
    auto t_stop_right = high_resolution_clock::now();
    auto t_duration_right = std::chrono::duration<double>(t_stop_right - t_start_right);
    if (t_duration_right.count() < dt)
    {
      std::this_thread::sleep_for(std::chrono::duration<double>(dt - t_duration_right.count()));
    }
  }
  currentJoint_right = rtde_receive_right.getActualQ();
  assert( abs(currentJoint_right[0] - install_ready_right[0]) < 0.01 || abs(currentJoint_right[1] - install_ready_right[1]) < 0.01 || abs(currentJoint_right[2] - install_ready_right[2]) < 0.01 || abs(currentJoint_right[3] - install_ready_right[3]) < 0.01);
  rtde_control_right.servoStop();
  isStage1End = true;
  

  while(isStage2Begin == false){
    this_thread::sleep_for(chrono::milliseconds(2));
  }

  //install_ready_right到install_ready1_right段
  //vector<double> install_ready1_right{ 0.973415,-1.75449,1.36313,-0.417254,1.68239,0.622209};  //the point is abondoned.
  vector<double> install_ready1_right{ 55.87*0.01745 , -100.12*0.01745  , 76.92*0.01745 , -20.11*0.01745 , 96.71*0.01745 , 35.97*0.01745 };
  rtde_control_right.moveL_FK(install_ready1_right,0.1,0.1);
  currentJoint_right = rtde_receive_right.getActualQ();
  assert( abs(currentJoint_right[0] - install_ready1_right[0]) < 0.01 || abs(currentJoint_right[1] - install_ready1_right[1]) < 0.01 || abs(currentJoint_right[2] - install_ready1_right[2]) < 0.01 || abs(currentJoint_right[3] - install_ready1_right[3]) < 0.01);
  this_thread::sleep_for(chrono::milliseconds(500));
  rtde_io_right.setStandardDigitalOut(0,true);
  isStage2End = true;


  while(isStage3Begin == false){
    this_thread::sleep_for(chrono::milliseconds(2));
  }
  
  //install_ready1_right到install_right段
  vector<double> install_right{63.45*0.0174533, -96.76*0.0174533, 74.28*0.0174533, -21.59*0.0174533, 102.20*0.0174533, 30.64*0.0174533};
  rtde_control_right.moveL_FK(install_right,0.1,0.1);
  currentJoint_right = rtde_receive_right.getActualQ();
  assert( abs(currentJoint_right[0] - install_right[0]) < 0.01 || abs(currentJoint_right[1] - install_right[1]) < 0.01 || abs(currentJoint_right[2] - install_right[2]) < 0.01 || abs(currentJoint_right[3] - install_right[3]) < 0.01);
  isStage3End = true;


  while(isStage4Begin == false){
    this_thread::sleep_for(chrono::milliseconds(2));
  }
  rtde_io_right.setStandardDigitalOut(0,false);
  isStage4End = true;



  while(isStage5Begin == false){
    this_thread::sleep_for(chrono::milliseconds(2));
  }
  
  vector<vector<double>> inter_installright_homeright = threetimePlan_sparse(install_right , home_right,dt);
  vector<double> joint_installright_homeright = {inter_installright_homeright[0][0] ,
                                          inter_installright_homeright[0][1] , 
                                          inter_installright_homeright[0][2] ,
                                          inter_installright_homeright[0][3] ,
                                          inter_installright_homeright[0][4] , 
                                          inter_installright_homeright[0][5]};
  // Execute 500Hz control loop for 2 seconds, each cycle is ~2ms
  for (unsigned int i=1; i<250; i++)
  {
    auto t_start_right = high_resolution_clock::now();
    rtde_control_right.servoJ(joint_installright_homeright, velocity, acceleration, dt, lookahead_time, gain);
    //rtde_control.speedJ(joint_q,  acceleration, dt);
    joint_installright_homeright[0] = inter_installright_homeright[i][0];
    joint_installright_homeright[1] = inter_installright_homeright[i][1];
    joint_installright_homeright[2] = inter_installright_homeright[i][2];
    joint_installright_homeright[3] = inter_installright_homeright[i][3];
    joint_installright_homeright[4] = inter_installright_homeright[i][4];
    joint_installright_homeright[5] = inter_installright_homeright[i][5];
    auto t_stop_right = high_resolution_clock::now();
    auto t_duration_right = std::chrono::duration<double>(t_stop_right - t_start_right);

    if (t_duration_right.count() < dt)
    {
      std::this_thread::sleep_for(std::chrono::duration<double>(dt - t_duration_right.count()));
    }
  }
  currentJoint_right = rtde_receive_right.getActualQ();
  assert( abs(currentJoint_right[0] - home_right[0]) < 0.01 || abs(currentJoint_right[1] - home_right[1]) < 0.01 || abs(currentJoint_right[2] - home_right[2]) < 0.01 || abs(currentJoint_right[3] - home_right[3]) < 0.01);
  rtde_control_right.servoStop();
  isStage5End = true;
  
  rtde_control_right.stopScript();
}


int main(int argc, char* argv[])
{
  std::string hostname = "192.168.1.41";  //left arm's ip
  
  RTDEReceiveInterface rtde_receive(hostname);
  RTDEControlInterface rtde_control(hostname);
  RTDEIOInterface rtde_io(hostname);   

  vector<double> currentJoint = rtde_receive.getActualQ();
  vector<double> home{15.36*0.0174533 ,-86.34*0.0174533 ,-125.38*0.0174533 ,248.13*0.0174533 ,-84.74*0.0174533 ,-74.12*0.0174533};
  assert( abs(currentJoint[0] - home[0]) < 0.01 || abs(currentJoint[1] - home[1]) < 0.01 || abs(currentJoint[2] - home[2]) < 0.01 || abs(currentJoint[3] - home[3]) < 0.01);
 
  thread myThread(Func1);  //create a new thread for right arm
  myThread.detach();  //detach the right arm thread for synchronously instead of "join" fuction which would blocking the main thread (the left arm thread) 

  // Parameters
  double velocity = 0.5;
  double acceleration = 0.2;
  double dt = 2.0/200; // 2ms
  double lookahead_time = 0.08;
  double gain = 800;
  



  double dt_dense = 2.0/500;
  //home to grasp_ready segment
  vector<double> grasp_ready{ 37.03* 0.0174533 , -24.55*0.0174533, -75.53*0.0174533, 144.63*0.0174533,-62.26*0.0174533,-44.73*0.0174533};
  vector<vector<double>> inter_home_graspready = threetimePlan_dense(home,grasp_ready,dt_dense);
  vector<double> joint_home_graspready = {inter_home_graspready[0][0] ,
                                          inter_home_graspready[0][1] , 
                                          inter_home_graspready[0][2] ,
                                          inter_home_graspready[0][3] ,
                                          inter_home_graspready[0][4] , 
                                          inter_home_graspready[0][5]};
  // Execute 500Hz control loop for 2 seconds, each cycle is ~2ms
  for (unsigned int i=1; i<600; i++)
  {
    auto t_start = high_resolution_clock::now();
    rtde_control.servoJ(joint_home_graspready, velocity, acceleration, dt_dense, lookahead_time, gain);
    //rtde_control.speedJ(joint_q,  acceleration, dt);
    joint_home_graspready[0] = inter_home_graspready[i][0];
    joint_home_graspready[1] = inter_home_graspready[i][1];
    joint_home_graspready[2] = inter_home_graspready[i][2];
    joint_home_graspready[3] = inter_home_graspready[i][3];
    joint_home_graspready[4] = inter_home_graspready[i][4];
    joint_home_graspready[5] = inter_home_graspready[i][5];
    auto t_stop = high_resolution_clock::now();
    auto t_duration = std::chrono::duration<double>(t_stop - t_start);

    if (t_duration.count() < dt_dense)
    {
      std::this_thread::sleep_for(std::chrono::duration<double>(dt_dense - t_duration.count()));
    }
  }


  currentJoint = rtde_receive.getActualQ();
  assert( abs(currentJoint[0] - grasp_ready[0]) < 0.01 || abs(currentJoint[1] - grasp_ready[1]) < 0.01 || abs(currentJoint[2] - grasp_ready[2]) < 0.01 || abs(currentJoint[3] - grasp_ready[3]) < 0.01);
 


  //grasp_ready to grasp segment
  vector<double> grasp{80.66*0.0174533 , -52.45*0.0174533 , -16.38*0.0174533 , 142.85*0.0174533 , -41.39*0.0174533
                                , -87.64*0.0174533};
  vector<vector<double>> inter_graspready_grasp = threetimePlan_sparse(grasp_ready,grasp,dt);
  vector<double> joint_graspready_grasp = {inter_graspready_grasp[0][0] ,
                                           inter_graspready_grasp[0][1] , 
                                           inter_graspready_grasp[0][2] ,
                                           inter_graspready_grasp[0][3] ,
                                           inter_graspready_grasp[0][4] ,
                                           inter_graspready_grasp[0][5]};
  // Execute 500Hz control loop for 2 seconds, each cycle is ~2ms
  for (unsigned int i=1; i<250; i++)
  {
    auto t_start = high_resolution_clock::now();
    rtde_control.servoJ(joint_graspready_grasp, velocity, acceleration, dt, lookahead_time, gain);
    //rtde_control.speedJ(joint_q,  acceleration, dt);
    joint_graspready_grasp[0] = inter_graspready_grasp[i][0];
    joint_graspready_grasp[1] = inter_graspready_grasp[i][1];
    joint_graspready_grasp[2] = inter_graspready_grasp[i][2];
    joint_graspready_grasp[3] = inter_graspready_grasp[i][3];
    joint_graspready_grasp[4] = inter_graspready_grasp[i][4];
    joint_graspready_grasp[5] = inter_graspready_grasp[i][5];
    auto t_stop = high_resolution_clock::now();
    auto t_duration = std::chrono::duration<double>(t_stop - t_start);

    if (t_duration.count() < dt)
    {
      std::this_thread::sleep_for(std::chrono::duration<double>(dt - t_duration.count()));
    }
  }

  currentJoint = rtde_receive.getActualQ();
  assert( abs(currentJoint[0] - grasp[0]) < 0.01 || abs(currentJoint[1] - grasp[1]) < 0.01 || abs(currentJoint[2] - grasp[2]) < 0.01 || abs(currentJoint[3] - grasp[3]) < 0.01);
  std::this_thread::sleep_for(std::chrono::duration<double>(0.5));
  rtde_io.setStandardDigitalOut(0,true);

  //grasp到grasp_ready段
  vector<vector<double>> inter_grasp_graspready = threetimePlan_sparse(grasp,grasp_ready,dt);

  vector<double> joint_grasp_graspready = {inter_grasp_graspready[0][0] ,
                                           inter_grasp_graspready[0][1] , 
                                           inter_grasp_graspready[0][2] ,
                                           inter_grasp_graspready[0][3] ,
                                           inter_grasp_graspready[0][4] ,
                                           inter_grasp_graspready[0][5]};
  // Execute 500Hz control loop for 2 seconds, each cycle is ~2ms
  for (unsigned int i=1; i<250; i++)
  {
    auto t_start = high_resolution_clock::now();
    rtde_control.servoJ(joint_grasp_graspready, velocity, acceleration, dt, lookahead_time, gain);
    //rtde_control.speedJ(joint_q,  acceleration, dt);
    joint_grasp_graspready[0] = inter_grasp_graspready[i][0];
    joint_grasp_graspready[1] = inter_grasp_graspready[i][1];
    joint_grasp_graspready[2] = inter_grasp_graspready[i][2];
    joint_grasp_graspready[3] = inter_grasp_graspready[i][3];
    joint_grasp_graspready[4] = inter_grasp_graspready[i][4];
    joint_grasp_graspready[5] = inter_grasp_graspready[i][5];
    auto t_stop = high_resolution_clock::now();
    auto t_duration = std::chrono::duration<double>(t_stop - t_start);

    if (t_duration.count() < dt)
    {
      std::this_thread::sleep_for(std::chrono::duration<double>(dt - t_duration.count()));
    }
  }
  
  currentJoint = rtde_receive.getActualQ();
  assert( abs(currentJoint[0] - grasp_ready[0]) < 0.01 || abs(currentJoint[1] - grasp_ready[1]) < 0.01 || abs(currentJoint[2] - grasp_ready[2]) < 0.01 || abs(currentJoint[3] - grasp_ready[3]) < 0.01);


  //grasp_ready到install_mid1段
  vector<double> install_mid1{ 44.73*0.0174533 , -75.34*0.0174533 , -127.07*0.0174533 , 246.35*0.0174533 , -90.36*0.0174533 , -44.88*0.0174533 };
  vector<vector<double>> inter_graspready_installmid1 = threetimePlan_dense(grasp_ready,install_mid1,dt_dense);
  vector<double> joint_graspready_installmid1 = {inter_graspready_installmid1[0][0] ,
                                          inter_graspready_installmid1[0][1] , 
                                          inter_graspready_installmid1[0][2] ,
                                          inter_graspready_installmid1[0][3] ,
                                          inter_graspready_installmid1[0][4] , 
                                          inter_graspready_installmid1[0][5]};
  // Execute 500Hz control loop for 2 seconds, each cycle is ~2ms
  for (unsigned int i=1; i<600; i++)
  {
    auto t_start = high_resolution_clock::now();
    rtde_control.servoJ(joint_graspready_installmid1, velocity, acceleration, dt_dense, lookahead_time, gain);
    //rtde_control.speedJ(joint_q,  acceleration, dt);
    joint_graspready_installmid1[0] = inter_graspready_installmid1[i][0];
    joint_graspready_installmid1[1] = inter_graspready_installmid1[i][1];
    joint_graspready_installmid1[2] = inter_graspready_installmid1[i][2];
    joint_graspready_installmid1[3] = inter_graspready_installmid1[i][3];
    joint_graspready_installmid1[4] = inter_graspready_installmid1[i][4];
    joint_graspready_installmid1[5] = inter_graspready_installmid1[i][5];
    auto t_stop = high_resolution_clock::now();
    auto t_duration = std::chrono::duration<double>(t_stop - t_start);

    if (t_duration.count() < dt_dense)
    {
      std::this_thread::sleep_for(std::chrono::duration<double>(dt_dense - t_duration.count()));
    }
  }

  currentJoint = rtde_receive.getActualQ();
  assert( abs(currentJoint[0] - install_mid1[0]) < 0.01 || abs(currentJoint[1] - install_mid1[1]) < 0.01 || abs(currentJoint[2] - install_mid1[2]) < 0.01 || abs(currentJoint[3] - install_mid1[3]) < 0.01);

  
  isStage1Begin = true;
  //install_mid1 to install_ready segment 
  vector<double> install_ready{0.707947,-2.50307,-0.981553,4.28216,-1.59688,-0.751079};
  vector<vector<double>> inter_installmid1_installready = threetimePlan_sparse(install_mid1,install_ready,dt);
  vector<double> joint_installmid1_installready = {inter_installmid1_installready[0][0] ,
                                           inter_installmid1_installready[0][1] , 
                                           inter_installmid1_installready[0][2] ,
                                           inter_installmid1_installready[0][3] ,
                                           inter_installmid1_installready[0][4] ,
                                           inter_installmid1_installready[0][5]};
  for (unsigned int i=1; i<250; i++)
  {
    auto t_start = high_resolution_clock::now();
    rtde_control.servoJ(joint_installmid1_installready, velocity, acceleration, dt, lookahead_time, gain);
    //rtde_control.speedJ(joint_q,  acceleration, dt);
    joint_installmid1_installready[0] = inter_installmid1_installready[i][0];
    joint_installmid1_installready[1] = inter_installmid1_installready[i][1];
    joint_installmid1_installready[2] = inter_installmid1_installready[i][2];
    joint_installmid1_installready[3] = inter_installmid1_installready[i][3];
    joint_installmid1_installready[4] = inter_installmid1_installready[i][4];
    joint_installmid1_installready[5] = inter_installmid1_installready[i][5];
    auto t_stop = high_resolution_clock::now();
    auto t_duration = std::chrono::duration<double>(t_stop - t_start);

    if (t_duration.count() < dt)
    {
      std::this_thread::sleep_for(std::chrono::duration<double>(dt - t_duration.count()));
    }
  }
  currentJoint = rtde_receive.getActualQ();
  assert( abs(currentJoint[0] - install_ready[0]) < 0.01 || abs(currentJoint[1] - install_ready[1]) < 0.01 || abs(currentJoint[2] - install_ready[2]) < 0.01 || abs(currentJoint[3] - install_ready[3]) < 0.01);

  
  rtde_control.servoStop();
  while(isStage1End == false){
      this_thread::sleep_for(chrono::milliseconds(2));
  }

  isStage2Begin = true;
  while(isStage2End == false){
      this_thread::sleep_for(chrono::milliseconds(2));
  }


  isStage3Begin = true;
  //install_ready to install segment
  vector<double> install{36.93*0.0174533,-145.36*0.0174533,-52.51*0.0174533,243.74*0.0174533,-94.04*0.0174533,-40.42*0.0174533};
  rtde_control.moveL_FK(install,0.1,0.1);
  currentJoint = rtde_receive.getActualQ();
  assert( abs(currentJoint[0] - install[0]) < 0.01 || abs(currentJoint[1] - install[1]) < 0.01 || abs(currentJoint[2] - install[2]) < 0.01 || abs(currentJoint[3] - install[3]) < 0.01);
  while(isStage3End == false){
      this_thread::sleep_for(chrono::milliseconds(2));
  }


  isStage4Begin = true;
  rtde_io.setStandardDigitalOut(0,false);
  while(isStage4End == false){
      this_thread::sleep_for(chrono::milliseconds(2));
  }


  isStage5Begin = true;
  vector<vector<double>> inter_install_home = threetimePlan_dense(install,home,dt_dense);
  vector<double> joint_install_home = {   inter_install_home[0][0] ,
                                          inter_install_home[0][1] , 
                                          inter_install_home[0][2] ,
                                          inter_install_home[0][3] ,
                                          inter_install_home[0][4] , 
                                          inter_install_home[0][5]};
  
  // Execute 500Hz control loop for 2 seconds, each cycle is ~2ms
  for (unsigned int i=1; i<600; i++)
  {
    auto t_start = high_resolution_clock::now();
    rtde_control.servoJ(joint_install_home, velocity, acceleration, dt_dense, lookahead_time, gain);
    //rtde_control.speedJ(joint_q,  acceleration, dt);
    joint_install_home[0] = inter_install_home[i][0];
    joint_install_home[1] = inter_install_home[i][1];
    joint_install_home[2] = inter_install_home[i][2];
    joint_install_home[3] = inter_install_home[i][3];
    joint_install_home[4] = inter_install_home[i][4];
    joint_install_home[5] = inter_install_home[i][5];
    auto t_stop = high_resolution_clock::now();
    auto t_duration = std::chrono::duration<double>(t_stop - t_start);

    if (t_duration.count() < dt_dense)
    {
      std::this_thread::sleep_for(std::chrono::duration<double>(dt_dense - t_duration.count()));
    }
    
  }
  currentJoint = rtde_receive.getActualQ();
  assert( abs(currentJoint[0] - home[0]) < 0.01 || abs(currentJoint[1] - home[1]) < 0.01 || abs(currentJoint[2] - home[2]) < 0.01 || abs(currentJoint[3] - home[3]) < 0.01);
  rtde_control.servoStop();
  while( isStage5End == false){
      this_thread::sleep_for(chrono::milliseconds(2));
  }

  rtde_control.stopScript();

  return 0;
}
