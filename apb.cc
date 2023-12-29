/*
 * Copyright (c) 2020-2023 Huazhong University of Science and Technology
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
 * Authors: Pengyu Liu <eic_lpy@hust.edu.cn>
 *          Hao Yin <haoyin@uw.edu>
 *          Muyuan Shen <muyuan_shen@hust.edu.cn>
 */

#include "apb.h"

#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/mobility-helper.h"
#include "ns3/multi-model-spectrum-channel.h"
#include "ns3/non-communicating-net-device.h"
#include "ns3/on-off-helper.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/packet-sink.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/spectrum-wifi-helper.h"
#include "ns3/ssid.h"
#include "ns3/string.h"
#include "ns3/udp-client-server-helper.h"
#include "ns3/waveform-generator-helper.h"
#include "ns3/waveform-generator.h"
#include "ns3/wifi-net-device.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/yans-wifi-helper.h"
#include <iomanip>
#include <random>
#include <ns3/ai-module.h>
#include <ns3/core-module.h>
#include "ns3/log.h"
#include <chrono>
#include <iostream>
#include <random>

#define NUM_ENV 10000

using namespace ns3;
// Global variables for use in callbacks.
double g_signalDbmAvg; //!< Average signal power [dBm]
double g_noiseDbmAvg;  //!< Average noise power [dBm]
uint32_t g_samples;    //!< Number of samples
double SNR = 0;
uint32_t current_disturb_channel;
uint32_t next_channel;
uint32_t next_power;
uint32_t action;
/**
 * Monitor sniffer Rx trace
 *
 * \param packet The sensed packet.
 * \param channelFreqMhz The channel frequency [MHz].
 * \param txVector The Tx vector.
 * \param aMpdu The aMPDU.
 * \param signalNoise The signal and noise dBm.
 * \param staId The STA ID.
 */
void
MonitorSniffRx(Ptr<const Packet> packet,
               uint16_t channelFreqMhz,
               WifiTxVector txVector,
               MpduInfo aMpdu,
               SignalNoiseDbm signalNoise,
               uint16_t staId)

{
    g_samples++;
    g_signalDbmAvg += ((signalNoise.signal - g_signalDbmAvg) / g_samples);
    g_noiseDbmAvg += ((signalNoise.noise - g_noiseDbmAvg) / g_samples);
}

NS_LOG_COMPONENT_DEFINE("WifiSpectrumPerInterference");

Ptr<SpectrumModel> SpectrumModelMultiBand; //!< Spectrum model at 2412 MHz.

int
main()
{
    // bool udp = true;
    double distance = 50;
    double simulationTime = 5; // seconds
    std::string wifiType = "ns3::SpectrumWifiPhy";
    std::string errorModelType = "ns3::NistErrorRateModel";
    // bool enablePcap = false;
    double waveformPower = 0.015;
    /**
     * 创建干扰频谱
    */
    std::vector<Bands> allBands;    // 2.4GHz WiFi频段的起始频率
    double startFrequency = 2412e6;
    // 2.4GHz WiFi频段每个信道的间隔
    double channelSpacing = 5e6;
    // 2.4GHz WiFi标准信道宽度
    double channelWidth = 20e6;
    for (int i = 0; i < 13; ++i) {
        Bands band;
        BandInfo bandInfo;
        bandInfo.fc = startFrequency + i * channelSpacing;
        bandInfo.fl = bandInfo.fc - channelWidth / 2.0;
        bandInfo.fh = bandInfo.fc + channelWidth / 2.0;
        band.push_back(bandInfo);
        // 将每个 Band 添加到 allBands 中
        allBands.push_back(band);
    }
    uint32_t current_channel = 9;
    DoubleValue power[] = {16,20,24};
    int pw_index = 1;
    //创建gym实例
    auto interface = Ns3AiMsgInterface::Get();
    interface->SetIsMemoryCreator(false);
    interface->SetUseVector(false);
    interface->SetHandleFinish(true);
    Ns3AiMsgInterfaceImpl<EnvStruct, ActStruct>* msgInterface =
        interface->GetInterface<EnvStruct, ActStruct>();
    for (uint32_t i = 0; i < 1000; i++)
    {   
        
        msgInterface->CppSendBegin();
        std::cout << "第一次开始cppsend" << std::endl;
        msgInterface->GetCpp2PyStruct()->envtmp1;
        msgInterface->GetCpp2PyStruct()->envtmp2;
        msgInterface->GetCpp2PyStruct()->envtmp3;
        msgInterface->GetCpp2PyStruct()->envtmp4;
        msgInterface->GetCpp2PyStruct()->cpp_action = 1;
        msgInterface->CppSendEnd();
        std::cout << "第一次结束cppsend" << std::endl;

        std::cout << "当前通信信道: " << current_channel; 
        if (i%13==0){
             std::random_device rd;
            // 使用 Mersenne Twister 引擎
            std::mt19937 gen(rd());
            // 定义分布范围为 [1, 100]
            std::uniform_int_distribution<> dis(1, 10000);
            // 生成伪随机数
            int random_number = dis(gen) % 13;
            SpectrumModelMultiBand = Create<SpectrumModel>(allBands[random_number]);
            std::cout << "现在正在干扰的信道为" << random_number+1 << std::endl;
            current_disturb_channel = random_number+1;
        }
        
        
        uint32_t payloadSize;
        payloadSize = 972; // 1000 bytes IPv4
        NodeContainer wifiStaNode;
        wifiStaNode.Create(1);
        NodeContainer wifiApNode;
        wifiApNode.Create(1);
        NodeContainer interferingNode;
        interferingNode.Create(1);
        SpectrumWifiPhyHelper spectrumPhy;
        Ptr<MultiModelSpectrumChannel> spectrumChannel;
        uint16_t frequency = 2412;
        spectrumChannel = CreateObject<MultiModelSpectrumChannel>();
        Ptr<FriisPropagationLossModel> lossModel = CreateObject<FriisPropagationLossModel>();
        lossModel->SetFrequency(frequency * 1e6);
        spectrumChannel->AddPropagationLossModel(lossModel);
        Ptr<ConstantSpeedPropagationDelayModel> delayModel = CreateObject<ConstantSpeedPropagationDelayModel>();
        spectrumChannel->SetPropagationDelayModel(delayModel);
        spectrumPhy.SetChannel(spectrumChannel);
        spectrumPhy.SetErrorRateModel(errorModelType);

        //设置频谱通信信道
        std::string setting = "{" + std::to_string(current_channel) +", 20, BAND_2_4GHZ, 0}";
        spectrumPhy.Set("ChannelSettings",StringValue(setting));
        DoubleValue txPower = power[pw_index];
        spectrumPhy.Set("TxPowerStart", txPower);
        spectrumPhy.Set("TxPowerEnd", txPower);

        WifiHelper wifi;
        // wifi.SetStandard(WIFI_STANDARD_80211ax);
        wifi.SetStandard(WIFI_STANDARD_80211n);
        WifiMacHelper mac;
        Ssid ssid = Ssid("ns380211n");
        double datarate = 0;
        StringValue DataRate;
        DataRate = StringValue("HtMcs0");
        datarate = 6.5;
        wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                     "DataMode",
                                     DataRate,
                                     "ControlMode",
                                     DataRate);

        NetDeviceContainer staDevice;
        NetDeviceContainer apDevice;
        mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid));
        staDevice = wifi.Install(spectrumPhy, mac, wifiStaNode);
        mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
        apDevice = wifi.Install(spectrumPhy, mac, wifiApNode);
        Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HtConfiguration/"
                        "ShortGuardIntervalSupported",
                        BooleanValue(false));
        // mobility.
        MobilityHelper mobility;
        Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
        positionAlloc->Add(Vector(0.0, 0.0, 0.0));
        positionAlloc->Add(Vector(distance, 0.0, 0.0));
        positionAlloc->Add(Vector(distance, distance, 0.0));
        mobility.SetPositionAllocator(positionAlloc);
        mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
        mobility.Install(wifiApNode);
        mobility.Install(wifiStaNode);
        mobility.Install(interferingNode);
        /* Internet stack*/
        InternetStackHelper stack;
        stack.Install(wifiApNode);
        stack.Install(wifiStaNode);
        Ipv4AddressHelper address;
        address.SetBase("192.168.1.0", "255.255.255.0");
        Ipv4InterfaceContainer staNodeInterface;
        Ipv4InterfaceContainer apNodeInterface;
        staNodeInterface = address.Assign(staDevice);
        apNodeInterface = address.Assign(apDevice);
        /* Setting applications */
        ApplicationContainer serverApp;
        uint16_t port = 9;
        UdpServerHelper server(port);
        serverApp = server.Install(wifiStaNode.Get(0));
        serverApp.Start(Seconds(0.0));
        serverApp.Stop(Seconds(simulationTime + 1));
        UdpClientHelper client(staNodeInterface.GetAddress(0), port);
        client.SetAttribute("MaxPackets", UintegerValue(4294967295U));
        client.SetAttribute("Interval", TimeValue(Time("0.0001"))); // packets/s
        client.SetAttribute("PacketSize", UintegerValue(payloadSize));
        ApplicationContainer clientApp = client.Install(wifiApNode.Get(0));
        clientApp.Start(Seconds(1.0));
        clientApp.Stop(Seconds(simulationTime + 1));
        Ptr<SpectrumValue> wgPsd =
            Create<SpectrumValue>(SpectrumModelMultiBand);
        *wgPsd = waveformPower / 20e6; // PSD spread across 20 MHz
        WaveformGeneratorHelper waveformGeneratorHelper;
        waveformGeneratorHelper.SetChannel(spectrumChannel);
        waveformGeneratorHelper.SetTxPowerSpectralDensity(wgPsd);
        waveformGeneratorHelper.SetPhyAttribute("Period", TimeValue(Seconds(0.0007)));
        waveformGeneratorHelper.SetPhyAttribute("DutyCycle", DoubleValue(1));
        NetDeviceContainer waveformGeneratorDevices =
        waveformGeneratorHelper.Install(interferingNode);
        Simulator::Schedule(Seconds(0.002),
                            &WaveformGenerator::Start,
                            waveformGeneratorDevices.Get(0)
                                ->GetObject<NonCommunicatingNetDevice>()
                                ->GetPhy()
                                ->GetObject<WaveformGenerator>());

        Config::ConnectWithoutContext("/NodeList/0/DeviceList/*/Phy/MonitorSnifferRx",
                                      MakeCallback(&MonitorSniffRx));
        g_signalDbmAvg = 0;
        g_noiseDbmAvg = 0;
        g_samples = 0;



        Simulator::Stop(Seconds(simulationTime + 1));
        Simulator::Run();

        double throughput = 0;
        uint64_t totalPacketsThrough = 0;
            totalPacketsThrough = DynamicCast<UdpServer>(serverApp.Get(0))->GetReceived();
            throughput =
                totalPacketsThrough * payloadSize * 8 / (simulationTime * 1000000.0); // Mbit/s


        std::cout << std::setprecision(2)
                  << std::fixed << std::setw(10) << datarate << std::setw(12) << throughput
                  << std::setw(8) << totalPacketsThrough;

        if (totalPacketsThrough > 0)
        {   
            SNR = g_signalDbmAvg - g_noiseDbmAvg;
            std::cout << std::setw(12) << g_signalDbmAvg << std::setw(12) << g_noiseDbmAvg
                      << std::setw(12) << (g_signalDbmAvg - g_noiseDbmAvg) << std::endl;
        }
        else
        {   
            SNR = 0;
            std::cout << std::setw(12) << "N/A" << std::setw(12) << "N/A" << std::setw(12) << "N/A"
                      << std::endl;
        }
        
        if(SNR <= 15){
            std::cout << "当前信道受干扰，正在切换信道......." << std::endl;

            //创建传输通道------------------------------------------------------------------
            // auto interface = Ns3AiMsgInterface::Get();
            // interface->SetIsMemoryCreator(false);
            // interface->SetUseVector(false);
            // interface->SetHandleFinish(true);
            // Ns3AiMsgInterfaceImpl<EnvStruct, ActStruct>* msgInterface =
            //     interface->GetInterface<EnvStruct, ActStruct>();

            msgInterface->CppSendBegin();
            std::cout << "第二次开始cppsend" << std::endl;
            msgInterface->GetCpp2PyStruct()->current_channel = current_channel;
            msgInterface->GetCpp2PyStruct()->current_power = static_cast<u_int32_t>(power[pw_index].Get());
            msgInterface->GetCpp2PyStruct()->current_disturbed_channel = current_disturb_channel;
            msgInterface->GetCpp2PyStruct()->current_snr = SNR;
            msgInterface->CppSendEnd();
            std::cout << "第二次结束cppsend" << std::endl;
            
            // msgInterface->CppRecvBegin();
            // std::cout << " 开始cppRecv " << std::endl;
            // next_channel = msgInterface->GetPy2CppStruct()->next_channel;
            // next_power = msgInterface->GetPy2CppStruct()->next_power;
            // msgInterface->CppRecvEnd();
            // std::cout << " 结束cppRecv " << std::endl;

            //-----------------------------------------------------------------------------
            if(next_power == 16){
                pw_index = 0;
            }else
            if(next_power == 20){
                pw_index = 1;
            }else
            if(next_power == 24){
                pw_index = 2;
            }
            current_channel = next_channel;
            std::cout << "Next Channel:  " << next_channel << "     Next Power:  " << next_power << std::endl;
        }
        // std::cout << "当前仿真结束" << std::endl;
        Simulator::Destroy();
     }
    return 0;

}
