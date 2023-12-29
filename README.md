### C++添加频谱干扰
```cpp
/**
    * 创建干扰频谱
*/
Ptr<SpectrumModel> SpectrumModelMultiBand; 
std::vector<Bands> allBands;    
// 2.4GHz WiFi频段的起始频率
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
/**
    *随机干扰信道
*/
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
/**
    *产生干扰信息
*/
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
