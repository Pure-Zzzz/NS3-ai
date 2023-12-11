/*
 * Copyright (c) 2023 Huazhong University of Science and Technology
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author:  Muyuan Shen <muyuan_shen@hust.edu.cn>
 */

#include <ns3/ai-module.h>
#include <ns3/core-module.h>
#include <random>
#include <chrono>
#include <iostream>


#define NUM_ENV 3

namespace ns3
{

class ApbEnv : public OpenGymEnv
{
  public:
    ApbEnv();
    ~ApbEnv() override;
    static TypeId GetTypeId();
    void DoDispose() override;
    void GetAPlusB();

    // OpenGym interfaces:
    //创造自定义环境不需要的
    Ptr<OpenGymSpace> GetActionSpace() override;
    Ptr<OpenGymSpace> GetObservationSpace() override;
    bool GetGameOver() override;
    Ptr<OpenGymDataContainer> GetObservation() override;
    float GetReward() override;
    std::string GetExtraInfo() override;
    bool ExecuteActions(Ptr<OpenGymDataContainer> action) override;

    double snr;
    double dist;
    double rate;

};

ApbEnv::ApbEnv()
{
    // std::cout << "ApbEnv()" << std::endl;
    SetOpenGymInterface(OpenGymInterface::Get());
}

ApbEnv::~ApbEnv()
{
}

TypeId
ApbEnv::GetTypeId()
{
    // std::cout << "ApbEnv::GetTypeId()" << std::endl;
    static TypeId tid = TypeId("ns3::ApbEnv").SetParent<OpenGymEnv>().SetGroupName("OpenGym");
    return tid;
}

void
ApbEnv::DoDispose()
{
    // std::cout << "ApbEnv::DoDispose()" << std::endl;
}

void
ApbEnv::GetAPlusB()
{
    // std::cout << "ApbEnv::GetAPlusB()" << std::endl;
    // std::cout << "执行Notify 正在等待" << std::endl;

    Notify();
    // std::cout << "Notify执行完毕" << std::endl;
}
/**
 * 设置动作空间，大小为2，动作内容[0,1]
*/
Ptr<OpenGymSpace>
ApbEnv::GetActionSpace()
{
    // std::cout << "执行ApbEnv::GetActionSpace()" << std::endl;
    std::vector<uint32_t> shape = {2};
    std::string dtype = TypeNameGet<uint32_t>();
    Ptr<OpenGymBoxSpace> box = CreateObject<OpenGymBoxSpace>(0, 1, shape, dtype);
    return box;
}
/***
 * 设置状态空间
 * 大小为3
 * snr
 * dist
 * rate
*/
Ptr<OpenGymSpace>
ApbEnv::GetObservationSpace()
{
    // std::cout << "ApbEnv::GetObservationSpace()" << std::endl;
    std::vector<uint32_t> shape = {3};
    std::string dtype = TypeNameGet<double>();
    Ptr<OpenGymBoxSpace> box = CreateObject<OpenGymBoxSpace>(0, 200, shape, dtype);
    return box;
}

bool
ApbEnv::GetGameOver()
{
    // std::cout << "ApbEnv::GetGameOver()" << std::endl;
    return false;
}
/**
 * 获取状态
 * snr
 * dist
 * rate
*/
Ptr<OpenGymDataContainer>
ApbEnv::GetObservation()
{
    // std::cout << "ApbEnv::GetObservation()" << std::endl;
    std::vector<uint32_t> shape = {3};
    Ptr<OpenGymBoxContainer<double>> box = CreateObject<OpenGymBoxContainer<double>>(shape);

    box->AddValue(snr);
    box->AddValue(dist);
    box->AddValue(rate);

    return box;
}
/**
 * 计算reward
 * 0.5 x rate
 * 0.3 x snr
 * 0.2 x 1/dist
*/
float
ApbEnv::GetReward()
{
    // std::cout << "ApbEnv::GetReward()" << std::endl;
    double reward;
    reward = 0.5 * rate + 0.3 * snr + 0.2 * (1 / dist);
    return reward;
}

std::string
ApbEnv::GetExtraInfo()
{
    // std::cout << "ApbEnv::GetExtraInfo()" << std::endl;
    return "this is ExtraInfo";
}

bool
ApbEnv::ExecuteActions(Ptr<OpenGymDataContainer> action)
{
    // std::cout << "执行action动作 ApbEnv::ExecuteActions(Ptr<OpenGymDataContainer> action)" << std::endl;
    Ptr<OpenGymBoxContainer<double>> box = DynamicCast<OpenGymBoxContainer<double>>(action);
    if(action == 0){
        rate -= 20;
        dist += 5;
        snr += 10;
    }else
    {
        rate += 20;
        dist -= 5;
        snr -= 10;
    }
    return true;
}

} // namespace ns3

int
main(int argc, char* argv[])
{
    using namespace ns3;
    std::random_device rd;
    std::mt19937 gen(rd());
    Ptr<ApbEnv> apb = CreateObject<ApbEnv>();
    apb->snr = std::uniform_real_distribution<double>(0.0, 40.0)(gen);
    apb->dist = std::uniform_real_distribution<double>(1.0, 100.0)(gen);
    apb->rate = std::uniform_real_distribution<double>(1.0, 1000.0)(gen);
    for (int i = 0; i < NUM_ENV; ++i)
    {
        apb->GetAPlusB();
        std::cout << "CPP循环执行" << std::endl << i << std::endl;
    }
    std::cout << "主函数 提示等待结束 NotifySimulationEnd" << std::endl;
    // 通过NotifySimulationEnd()返回done=True
    apb->NotifySimulationEnd();
    return 0;
}
