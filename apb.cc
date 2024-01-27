#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/internet-module.h"
#include "ns3/udp-client-server-helper.h"
#include "ns3/applications-module.h"
#include "ns3/aodv-module.h"
#include "ns3/interference-helper.h"
#include "ns3/csma-module.h"
#include "ns3/tap-bridge-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/names.h"
#include "ns3/node.h"
#include "ns3/waveform-generator-helper.h"
#include "ns3/waveform-generator.h"
#include "ns3/non-communicating-net-device.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/flow-monitor.h"
#include "ns3/ptr.h"
#include <iostream>
#include <nlohmann/json.hpp>
#include <map>
#include "ns3/dsr-module.h"
#include <math.h>
#include <regex>
#include <string>
#include <iomanip>
#include "ns3/multi-model-spectrum-channel.h"
#include <chrono>
#include <thread>
#include "apb.h"
#include <ns3/ai-module.h>
using namespace ns3;
using namespace std;
Ns3AiMsgInterfaceImpl<EnvStruct, ActStruct>* msgInterface;
using namespace ns3;
using namespace std;
using json = nlohmann::json;

//节点数据
struct NodeData {
    string nodeID;
    double throughput;
    double signalToNoise;
    int packetSize;
    long long lastUpdateTime;
    double delay;
    int activity;

    //构造函数
    NodeData(string id, double tpt, double snr, int size, double dly, double act, long long lastUpdateTime)
        :nodeID(id), throughput(tpt), signalToNoise(snr), packetSize(size), 
         delay(dly), activity(act), lastUpdateTime(lastUpdateTime){
        
    }

    // 更新吞吐量
    void updateThroughput(double throughput) {
        this->throughput = throughput;
    }

    // 更新信噪比
    void updateSNR(double SNR) {
        this-> signalToNoise = SNR;
    }

    // 更新时间
    void updateTime(long long lastUpdateTime) {
        this -> lastUpdateTime = lastUpdateTime;
    }

    // 更新包大小
    void updatePacketSize(int packetSize) {
        this->packetSize = packetSize;
    }

    // 更新时延
    void updateDelay(double delay) {
        this->delay = delay;
    }

    // 更新活跃度
    void updateAcitity(double activity){
        this->activity = activity;
    }
};

vector<NodeData> nodeVector;

// 输出节点数据的函数
void PrintNodeVector() {
    cout << "Node Vector at " << Simulator::Now().GetSeconds() << " seconds:" << endl;

    for (const auto &node : nodeVector) {
        cout << "Node ID: " << node.nodeID << ", Throughput: " << node.throughput
             << ", SNR: " << node.signalToNoise << ", Packet Size: " << node.packetSize
             << ", Last Update Time: " << node.lastUpdateTime << ", Delay: " << node.delay
             << ", Activity: " << node.activity << endl;
        cout << "--------------------------------------------------------------------------" << endl;
    }

    // 重新安排下一个输出
    Simulator::Schedule(Seconds(1.0), &PrintNodeVector);
}

struct DataForThpt{
    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor;
    uint32_t totalRxPackets; // Total number of received packets in all flows
    uint64_t totalRxBytes;   // Total bytes received in all flows
    double totalDelaySum;    // Total delay sum in all flows

    // average delay (ms)
    double averageDelay()
    {
        return totalRxPackets ? totalDelaySum / totalRxPackets / 1000000 : 0;
    }
} flowdata;

double duration = 5.0;
double statInterval = 0.2; 

static void Throughput(){
    flowdata.monitor->CheckForLostPackets();
    const FlowMonitor::FlowStatsContainer stats = flowdata.monitor->GetFlowStats();

    uint64_t totalRxBytes = 0;
    uint32_t totalRxPackets = 0;
    double totalDelaySum = 0;

    for (FlowMonitor::FlowStatsContainerCI iter = stats.begin(); iter != stats.end(); iter++){
        totalRxBytes += iter->second.rxBytes;
        totalDelaySum += iter->second.delaySum.GetDouble();
        totalRxPackets += iter->second.rxPackets;
    }
    uint64_t rxBytesDiff = totalRxBytes - flowdata.totalRxBytes;
    uint32_t rxPacketsDiff = totalRxPackets - flowdata.totalRxPackets;
    double delayDiff = totalDelaySum - flowdata.totalDelaySum;

    flowdata.totalRxBytes = totalRxBytes;
    flowdata.totalRxPackets = totalRxPackets;
    flowdata.totalDelaySum = totalDelaySum;

    double delay = 0.0; // ms
    if (rxPacketsDiff != 0 && delayDiff != 0){
        delay = delayDiff / rxPacketsDiff / 1000000;
    }
    double tpt = 8.0 * rxBytesDiff / statInterval / (1024 * 1024); // Mbps
    
    //cout << ": Delay: " << delay << "ms, Throughput: " << tpt << "Mbps" << endl;
    Simulator::Schedule(Seconds(statInterval), &Throughput);
}

//socket
Ptr<Socket> srcSocket;
queue<string> stringQueue;

//默认节点速度
double soldierSpeed=5.0;
double tankerSpeed=16.0;
double constructionVehicleSpeed=11.0;
double transportVehicleSpeed=19.0;
double ambulanceSpeed=19.0;
double commandTentSpeed=0.0;
double radarVehicleSpeed=14.0;
double radarSpeed=17.0;

//默认损失函数参数设置
double frequencyGHz = 100e6;       // 频率, 单位GHz
double rxSensitivity = -110.0;          // 接收灵敏度, 单位dBm
uint32_t antennas = 1;                  // 天线数量
uint32_t maxTxSpatialStreams = 1;       // 最大传输空间流数量
uint32_t maxRxSpatialStreams = 1;       // 最大接收空间流数量
double txGain = 12.5;                   // 发射天线增益, 单位dB
double rxGain = 12.5;                   // 接收天线增益, 单位dB
double rxNoiseFigure = 40;              // 接收噪声指数
double txPowerStart = 35;               // 发射功率起始值, 单位dBm
double txPowerEnd = 35;                 // 发射功率结束值, 单位dBm
double m0=1.2;
double m1=1.2;
double m2=1.2;
double referenceLoss=50;
double referenceDistance=2.95;
double pathLossExponent=2.95;

// 存储所有频带的向量
vector<Bands> allBands;
// 2.4GHz WiFi频段的起始频率
double startFrequency = 2412e6;
// 2.4GHz WiFi频段每个信道的间隔
double channelSpacing = 5e6;
// 2.4GHz WiFi标准信道宽度
double channelWidth = 20e6;

NodeContainer interferingNode;
Ptr<MultiModelSpectrumChannel> spectrumChannel;

//创建损失函数信道助手,并设置信道
Ptr<FriisPropagationLossModel> FriisPlainlossModel = CreateObject<FriisPropagationLossModel>();
Ptr<FriisPropagationLossModel> FriisMountainlossModel = CreateObject<FriisPropagationLossModel>();
Ptr<NakagamiPropagationLossModel> NakagamiMountainlossModel = CreateObject<NakagamiPropagationLossModel>(); 
Ptr<NakagamiPropagationLossModel> NakagamiCitylossModel = CreateObject<NakagamiPropagationLossModel>(); 
Ptr<LogDistancePropagationLossModel> LogDistanceCitylossModel= CreateObject<LogDistancePropagationLossModel>();
Ptr<LogDistancePropagationLossModel> LogDistanceForestlossModel= CreateObject<LogDistancePropagationLossModel>();
Ptr<NakagamiPropagationLossModel> NakagamiForestlossModel = CreateObject<NakagamiPropagationLossModel>(); 

//默认节点数量
uint16_t soldierNum=4;
uint16_t tankerNum=6;
uint16_t constructionVehicleNum=1;
uint16_t transportVehicleNum=2;
uint16_t ambulanceNum=2;
uint16_t commandTentNum=1;
uint16_t radarVehicleNum=3;
uint16_t radarNum=9;
uint16_t totalNodes=28;

vector<uint16_t> myVector = {soldierNum, tankerNum, constructionVehicleNum, transportVehicleNum, ambulanceNum, commandTentNum, radarVehicleNum, radarNum};
vector<uint16_t> InitialVector(){
    vector<uint16_t> transvector(myVector.size());
    int sum = 0;
    for(int i=0;i<myVector.size();i++){
        transvector[i] = sum + myVector[i];
        sum += myVector[i];
    }
    return transvector;
}

// 发送数据的函数
void SendData(Ptr<Socket> socket) {
    // 检查队列是否不为空
    if(!stringQueue.empty()){
        string toSend = stringQueue.front(); // 获取队列的第一个元素
        stringQueue.pop(); // 将该元素从队列中移除

        // 创建一个数据包并通过socket发送
        Ptr<Packet> packet = Create<Packet>((uint8_t*)toSend.c_str(), toSend.length());
        socket->Send(packet);
    }
    Simulator::Schedule(Seconds(0.01),&SendData,socket);
}

// 创建一个新的类，该类继承自 Object
class ComplexData : public Object {
public:
    static TypeId GetTypeId (void) {
        static TypeId tid = TypeId("ComplexData")
            .SetParent<Object>()
            .SetGroupName("Tutorial")
            .AddConstructor<ComplexData>();
        return tid;
    }
    void SetStringData(string data) {
        m_stringData = data;
    }
    string GetStringData(void) const {
        return m_stringData;
    }
    
    void SetVectorData(vector<string> data) {
        m_vectorData = data;
    }
    vector<string> GetVectorData(void) const {
        return m_vectorData;
    }

    bool HasStringData() const {
        return !m_stringData.empty();
    }

    bool HasVectorData() const {
        return !m_vectorData.empty();
    }
    
    // ... 你可以添加更多的数据成员和对应的设置/获取方法

private:
    string m_stringData;
    vector<string> m_vectorData;
    // ... 其他数据成员
};

// 设置日志组件，方便调试
NS_LOG_COMPONENT_DEFINE ("PositionTacticalInternetExample");

//记录时延
struct DataRecord {
    string startNode;
    string endNode;
    long long startTime;  
    long long durationTime;

    // Updated constructor to include startTime
    DataRecord(const std::string& start, const std::string& end, long long startTime, long long t)
        : startNode(start), endNode(end), startTime(startTime), durationTime(t) {}
};
vector<DataRecord> dataRecords;
uint64_t totalReceivedBytes = 0;
uint64_t accumulatedTime = 15000;
//记录吞吐量
static map<string, uint64_t> throughoutputByModelId;

NodeContainer nodes;//初始化节点

uint32_t packetId = 0;
Time totalDelay = Seconds(0);
double totalPacketsReceived = 0;
double totalPacketsSent = 0;

static map<Ipv4Address, string> ipToIdMap;
static map<string, string> idToNameMap;
static map<string, string> idToGroupMap;
static map<string, string> nameToIdMap;
static map<string, string> nameToTypeMap;//根据Nodename查询节点的类型
static map<string, int> idToActivityMap;//根据id查询节点的活跃度
int num=1;

Time startTransmissionTime ;
Time transmissionDuration ;

//地理坐标系和笛卡尔坐标系的转换
constexpr double DEG_TO_RAD_LOCAL = 3.1415926535897932 / 180.0;
constexpr double RAD_TO_DEG_LOCAL = 180.0 /3.1415926535897932;
ofstream outputFile;
ofstream dataputFile;
ofstream dataActiviaty;
ofstream dataAi;
Ptr<Node> tempNode;//临时记录节点信息的全局变量,用于输出snr等信息
int first = 1;//判断输出的json是否为第一行从而决定是否输出
int second = 1;//判断数据包信息输出文件是否为第一行
int third = 1;//判断活跃度文件信息输出是否为第一行
int activiaty ;//判断是否给用户活跃度文件添加信息，每25s会添加5s的数据
int busy = 0;//判读当前是否正在进行数据包的发送，如果没有为0，有为1
string continuetime = "1";//onoffhelper持续发送数据包时间
string stoptime = "2";//onoffhelper停止发送数据包时间
int jishu = 0;//用来判断当前进行信道修改的节点

double totalSnr = 0;//统计多少秒内的平均信噪比
double totalPackets = 0;//总包量

int sendtimeinterval = 20;//数据包发送时间间隔

int tempnum =1;
double NodePower(Ptr<Node> node);
int isBusy(double nowtime);

struct Data {
    string col1;
    string col2;
    string col10;
    string col11;
    string col12;
};
//存放临时要输出的信噪比等参数
struct Temp{
    int status = 0;//用于标记当前temp状态，若为1表明需要进行输出，否则不需要输出
    NodeContainer nodes;
    long long timestamp;
    uint16_t frequency;
    double snr;
    WifiMode mcs;
};
Temp temp, datatemp;

enum MediaType {
    VIDEO = 1,
    TEXT = 2,
    AUDIO = 3
};

string MediaTypeToString(MediaType mediaType) {
    switch (mediaType) {
        case VIDEO: return "视频";
        case TEXT:  return "文本";
        case AUDIO: return "录音";
        default:    return "未知类型";
    }
}

//指数分布
MediaType GenerateRandomMediaType() {
    Ptr<ExponentialRandomVariable> x = CreateObject<ExponentialRandomVariable>();
    x->SetAttribute("Mean", DoubleValue(2.0)); // 设置均值

    double randomValue = x->GetValue();

    // 按照指数分布的特性调整返回值，使得视频和音频更频繁
    if (randomValue < 2.0) {
        return VIDEO; // 视频
    } else if (randomValue < 4.0) {
        return AUDIO; // 音频
    } else {
        return TEXT; // 文本
    }
}

//读取节点位置信息
vector<Data> readData(const string &filePath, int soldier, int tanke, int engineer, int trans, int ambulace, int command, int radiovech, int radio) {
    vector<Data> dataArray;
    ifstream file(filePath);
    string line;
    int count = 1;
    vector<uint16_t> A = InitialVector();
    while (getline(file, line)) {
        istringstream iss(line);
        vector<string> columns;
        string item;
        while (getline(iss, item, '\t')) {
            columns.push_back(item);
        }
        if (columns.size() >= 12) {
            if(radiovech == 3){
                if((count>0 && count<=soldier) || (count>A[0] && count<=tanke+A[0]) || (count>A[1] && count<=engineer+A[1]) || (count>A[2] && count<=trans+A[2]) || (count>A[3] && count<=ambulace+A[3]) || (count>A[4] && count<=command+A[4]) ||(count>A[5] && count<=radiovech+A[5]-1) || (count>A[6]-1 && count<=radio+A[6]-1)){
                    Data data = {columns[0], columns[1], columns[6], columns[7], columns[8]};//获取原始转换的笛卡尔坐标系的数据
                    dataArray.push_back(data);
                }
                if(count == 28){
                    Data data = {columns[0], columns[1], columns[6], columns[7], columns[8]};//获取原始转换的笛卡尔坐标系的数据
                    dataArray.push_back(data);
                }
            }else{
                if((count>0 && count<=soldier) || (count>A[0] && count<=tanke+A[0]) || (count>A[1] && count<=engineer+A[1]) || (count>A[2] && count<=trans+A[2]) || (count>A[3] && count<=ambulace+A[3]) || (count>A[4] && count<=command+A[4]) ||(count>A[5] && count<=radiovech+A[5]) || (count>A[6]-1 && count<=radio+A[6]-1)){
                    Data data = {columns[0], columns[1], columns[6], columns[7], columns[8]};//获取原始转换的笛卡尔坐标系的数据
                    dataArray.push_back(data);
                }
            }
        }
        count++;
    }
    return dataArray;
}


//添加string类型数据
void AddStringData(Ptr<Node> node,const string& data){
    Ptr<ComplexData> complexData = node->GetObject<ComplexData>();
    // 将自定义的 StringData 对象附加到节点上
    if (complexData == nullptr) {
        complexData = CreateObject<ComplexData>();
        node->AggregateObject(complexData);
    }
    complexData->SetStringData (data);
}


//添加Vector类型数据
void AddVectorData(Ptr<Node> node, const vector<string>& data){
    Ptr<ComplexData> complexData = node->GetObject<ComplexData>();
    // 将自定义的 StringData 对象附加到节点上
    if (complexData == nullptr) {
        complexData = CreateObject<ComplexData>();
        node->AggregateObject(complexData);
    }
    complexData->SetVectorData (data);
}

//获取string类型数据
string SearchStringData(Ptr<Node> node){
    Ptr<ComplexData> retrieved = node->GetObject<ComplexData>();
    if (retrieved != nullptr && retrieved->HasStringData()) {
        return retrieved->GetStringData();
    } else {
        cout << "String data not exists for this node!" << endl;
        return "";
    }
}

//获取Vector类型数据
vector<string> SearchVectorData(Ptr<Node> node){
    Ptr<ComplexData> retrieved = node->GetObject<ComplexData>();
    if (retrieved != nullptr && retrieved->HasVectorData()) {
        return retrieved->GetVectorData();
    } else {
        cout << "Vector data not exists for this node!" << endl;
        return vector<string>();
    }
}

// 假设您的文件每行有三列：id（字符串）、terrain（字符串）和weather（字符串）
struct DataRow {
    string id;
    string terrain;
    string weather;
};

vector<DataRow> ReadCsvFile(const string& filename) {
    vector<DataRow> data;
    ifstream file(filename);

    if (!file.is_open()) {
        cerr << "无法打开文件: " << filename << endl;
        return data;
    }

    string line;
    while (getline(file, line)) {
        stringstream ss(line);
        string item;
        vector<string> row;

        while (getline(ss, item, ',')) {
            row.push_back(item);
        }

        if (row.size() == 3) {
            DataRow dataRow;
            dataRow.id = row[0];
            dataRow.terrain = row[1];
            dataRow.weather = row[2];
            data.push_back(dataRow);
        }
    }

    file.close();
    return data;
}

//加入map中实现一对一关系，其中Node为节点，nodeName为节点名字，nodeId为外部获取的节点的唯一ID属性
void AddToMap(Ptr<Node> Node, string nodeName, string nodeId, string nodeType){
    Names::Add(nodeName, Node);//将Node的名字作为Node的一个别名
    idToNameMap[nodeId] = nodeName; //将Node的名字和外部的nodeId作为一对键值对放入idToNameMap(通过nodeId查询nodeName)
    nameToIdMap[nodeName] = nodeId;//将Node的名字和外部的nodeId作为一对键值对放入nameToIdMap(通过nodeName查询nodeId)
    nameToTypeMap[nodeName] = nodeType;
    idToActivityMap[nodeId] = 1;
}

//将节点id和阵营对应关系加入map中
void AddidToGroup(string nodeId, string group){
    idToGroupMap[nodeId] = group;
}

//从map里面通过id查找阵营
string FindFromGroupMap(string searchId){
    if(idToGroupMap.find(searchId)!=idToGroupMap.end()){
        //通过ID获取节点阵营
        string foundNodeGroup = idToGroupMap[searchId];
        string foundNodeName = idToNameMap[searchId];

        //使用节点名称从Names空间中找到对应的节点
        Ptr<Node> foundNode = Names::Find<Node>(foundNodeName);
        return foundNodeGroup;
    }else{
        cout<<"Group with ID"<<searchId<<" not found."<<endl;
    }
    return 0;
}

//根据节点查询ID
string FindIdFromMap(Ptr<Node> node){
    if (Names::FindName(node) != "") {
        string foundName = Names::FindName(node);
        if(nameToIdMap.find(foundName)!=nameToIdMap.end()){
            //通过NodeName来获取Id
            string foundNodeId = nameToIdMap[foundName];
            return foundNodeId;
        }else{
            cout<<"No name is associated with the node." << endl;
        }
    } else {
        cout << "No name is associated with the node. " << endl;
    }
    return 0;
}

Ptr<Node> FindFromMap(string searchId){
    if(idToNameMap.find(searchId)!=idToNameMap.end()){
        //通过ID获取节点名称
        string foundNodeName = idToNameMap[searchId];

        //使用节点名称从Names空间中找到对应的节点
        Ptr<Node> foundNode = Names::Find<Node>(foundNodeName);
        return foundNode;
    }else{
        // cout<<"Node with ID"<<searchId<<" not found."<<endl;
        return 0;
    }
    return 0;
}

void AddPosition(int &i, Data d, Ptr<ListPositionAllocator> positionAlloc,NodeContainer Nodes, string Nodename){
    double x = stod(d.col10);
    double y = stod(d.col11);
    double z = stod(d.col12);
    positionAlloc->Add(Vector(x/20.0, y/20.0, z/20.0));
    AddToMap(Nodes.Get(i),Nodename + ".Get(" + to_string(i) +")", d.col2, d.col1);
    i++;
}

//通过节点的ip查询节点的id
void FindIdFromIpMap(NodeContainer& nodes) {
    for (NodeContainer::Iterator i = nodes.Begin(); i != nodes.End(); ++i) {
        Ptr<Node> node = *i;
        Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>(); // 假设节点已安装了 IPv4
        std::vector<Ipv4Address> addresses;

        // 获取每个接口上的 IP 地址
        for (uint32_t j = 0; j < ipv4->GetNInterfaces(); j++) {
            for (uint32_t k = 0; k < ipv4->GetNAddresses(j); k++) {
                Ipv4InterfaceAddress iface = ipv4->GetAddress(j, k);
                addresses.push_back(iface.GetLocal());
            }
        }
        if(addresses.size()>2){
            // for(uint16_t i=1 ; i<addresses.size() ; ++i){
            string id = nameToIdMap[Names::FindName(node)];
            cout<<"节点id为: "<<id<<"  ";
            ipToIdMap[addresses[3]] = id;
            cout << "ip为: " << addresses[3] << endl;
            // }
        }else{
            string id = nameToIdMap[Names::FindName(node)];
            cout<<"节点id为: "<<id<<"  ";
            ipToIdMap[addresses[1]] = id;
            cout << "ip为: " << addresses[1] << endl;
        }
        num++;
    }
}

//将笛卡尔坐标系数据转换成地理坐标系数据
void XYZ2LLA(  double X, double Y, double Z,double& longitude, double& latitude,double& altitude)
{
		double a = 6378137.0;
		double b = 6356752.31424518;
		double c = sqrt(((a * a) - (b * b)) / (a * a));
		double d = sqrt(((a * a) - (b * b)) / (b * b));
		double p = sqrt((X * X) + (Y * Y));
		double q = atan2((Z * a), (p * b));
		
		longitude = atan2(Y, X);
        latitude = atan2((Z + (d * d) * b * pow(sin(q), 3)), (p - (c * c) * a * pow(cos(q), 3)));
		double N = a / sqrt(1 - ((c * c) * sin(latitude)* sin(latitude)));
		altitude = (p / cos(latitude)) - N;
		
		longitude = longitude * RAD_TO_DEG_LOCAL;
		latitude = latitude * RAD_TO_DEG_LOCAL;
}

//打印节点接收灵敏度
double NodeSensitivity(Ptr<Node> onenode){
    // 获取第一个节点的第一个设备
    Ptr<Node> node = onenode;
    Ptr<NetDevice> dev = node->GetDevice (0);
    Ptr<WifiNetDevice> wifiDev = dev->GetObject<WifiNetDevice> ();
    Ptr<SpectrumWifiPhy> phy = DynamicCast<SpectrumWifiPhy>(wifiDev->GetPhy ());
    

    // 打印接收器灵敏度
    double rxSensitivity = phy->GetRxSensitivity ();

    return rxSensitivity;
}

//根据节点找传输功率
double NodePower(Ptr<Node> node){
    Ptr<NetDevice> dev = node->GetDevice(0); 
    Ptr<WifiNetDevice> wifiDev = dev->GetObject<WifiNetDevice>();
    Ptr<SpectrumWifiPhy> phy = DynamicCast<SpectrumWifiPhy>(wifiDev->GetPhy());
    double txPower = phy->GetTxPowerStart();
    return txPower;
}    

//将内部数据输出到json文件中
void LogJsonPosition(NodeContainer Nodes, ofstream& outputFile)
{
//处理非收到数据包数据 
    for(uint16_t i = 0; i < Nodes.GetN(); ++i) 
    {
        Ptr<MobilityModel> mobility = Nodes.Get(i)->GetObject<MobilityModel>();
        Vector pos = mobility->GetPosition();
        Vector speed = mobility->GetVelocity();
        double speedMagnitude = 20*sqrt(speed.x * speed.x + speed.y * speed.y + speed.z * speed.z);//节点的当前速度
        string foundName = Names::FindName(Nodes.Get(i));
        vector<string> retrievedVector = SearchVectorData(Nodes.Get(i));
        // 获取当前时间点
        auto now = chrono::system_clock::now();

        // 转换为时间戳
        auto timestamp = chrono::duration_cast<chrono::milliseconds>(now.time_since_epoch()).count();

        double currentTime = Simulator::Now ().GetSeconds ();
        double throughout;
            if(currentTime == 0)
                throughout = 0;
            else    
                throughout = throughoutputByModelId[nameToIdMap[foundName]]/currentTime ;
        

        double X,Y,Z;
        XYZ2LLA(pos.x*20,pos.y*20,pos.z*20,X,Y,Z);//直接采用原始未经处理的地理坐标转换的笛卡尔坐标系坐标
        if(first == 1)
            outputFile << "[" <<endl;
        outputFile << "{";  // 开始一个 JSON 对象
        // 添加各种数据到 JSON 对象
        outputFile << "\"Timestamp\": \"" << timestamp << "ms\", ";
        outputFile << "\"PkgType\": \"001\",";
        outputFile << "\"NodeId\": \"" << FindIdFromMap(Nodes.Get(i)) << "\", ";
        outputFile << "\"NodeActivity\": \"" << idToActivityMap[FindIdFromMap(Nodes.Get(i))] << "\", ";
        outputFile << "\"NodeSensitivity\": \"" << NodeSensitivity(Nodes.Get(i)) << "dBm\", ";
        outputFile << "\"NodeGroup\":\""<< FindFromGroupMap(FindIdFromMap(Nodes.Get(i))) <<"\",";
        outputFile << "\"NodeType\": \"" << nameToTypeMap[foundName] << "\", ";
        outputFile << "\"NodeName\": \"" << foundName << "\", ";  
        outputFile << "\"NodeTxPower\": \"" << NodePower(Nodes.Get(i)) << "dBm\", ";
        outputFile << "\"NodeTP\": \"" << throughout << "Bps\", ";
        outputFile << "\"Position\": {\"x\": "  << fixed << setprecision(6)<< X << ", \"y\": " << Y << ", \"z\": " << Z << "}, ";//输出地理坐标
        outputFile.unsetf(ios_base::fixed);
        outputFile.precision(streamsize(-1));
        outputFile << "\"Terrain\": \"" << retrievedVector[0] << "\", ";
        outputFile << "\"Weather\": \"" << retrievedVector[1] << "\"";
        outputFile << "}";  // 结束 JSON 对象
        if(first == 1)
            first = 0;
        outputFile <<","<<endl;  // 换行

        stringstream ss;
        ss << "{";  // 开始一个 JSON 对象
        // 添加各种数据到 JSON 对象
        ss << "\"Timestamp\": \"" << timestamp << "ms\", ";
        ss << "\"PkgType\": \"001\",";
        ss << "\"NodeId\": \"" << FindIdFromMap(Nodes.Get(i)) << "\", ";
        ss << "\"NodeActivity\": \"" << idToActivityMap[FindIdFromMap(Nodes.Get(i))] << "\", ";
        ss << "\"NodeSensitivity\": \"" << NodeSensitivity(Nodes.Get(i)) << "dBm\", ";
        ss << "\"NodeGroup\":\""<< FindFromGroupMap(FindIdFromMap(Nodes.Get(i))) <<"\",";
        ss << "\"NodeType\": \"" << nameToTypeMap[foundName] << "\", ";
        ss << "\"NodeName\": \"" << foundName << "\", ";  
        ss << "\"NodeTxPower\": \"" << NodePower(Nodes.Get(i)) << "dBm\", ";
        ss << "\"NodeTP\": \"" << throughout << "Bps\", ";
        ss << std::fixed << std::setprecision(6);
        ss << "\"Position\": {\"x\": "  << X << ", \"y\": " << Y << ", \"z\": " << Z << "}, "; // 输出地理坐标
        ss.unsetf(std::ios_base::fixed);
        ss.precision(std::streamsize(-1));
        ss << "\"Terrain\": \"" << retrievedVector[0] << "\", ";
        ss << "\"Weather\": \"" << retrievedVector[1] << "\"";
        ss << "}";  // 结束 JSON 对象        

        string jsonString = ss.str();  // 将 stringstream 转换为 string
        stringQueue.push(jsonString);
    }
    // 每秒记录
    Simulator::Schedule(Seconds(2), &LogJsonPosition, Nodes, ref(outputFile));
}

void Closefile(ofstream& outputFile){
    outputFile.close();
}

//处理json文件最后一行
void ModifyJsonFile(std::fstream& file) {
    if (!file.is_open()) {
        std::cerr << "文件流未打开。" << std::endl;
        return;
    }

    // 定位到文件倒数第二个字符（考虑逗号后的换行符）
    file.seekg(-2, std::ios::end);

    char second_last_char, last_char;
    file.get(second_last_char);
    file.get(last_char);

    // 检查倒数第二个字符是否是逗号，且最后一个字符是换行符
    if (second_last_char == ',' && last_char == '\n') {
        // 回退两个字符
        file.seekp(-2, std::ios::end);
        // 替换为 ']'
        file << "\n]\n";
    } else {
        std::cerr << "文件末尾格式不符合预期，不需要修改。" << std::endl;
    }
}

void ClearActivityFile(ofstream& outputFile, ofstream& inputFile) {
    outputFile.open("data-activiaty-log.json", ofstream::out | ofstream::trunc); // 以截断方式打开文件
    if (outputFile.is_open()) {
        outputFile.close(); // 关闭文件，这将清空文件内容
        inputFile.close();
        fstream file("data-activiaty-log.json", ios::in | ios::out | ios::ate);

        ModifyJsonFile(file);
    }

    // fstream file("data-activiaty-log.json", ios::in | ios::out | ios::ate);

    // ModifyJsonFile(file);

    std::ifstream src("data-activiaty-log.json", std::ios::binary);

    std::ofstream dest("activity-data-log.json", std::ios::binary);

    dest << src.rdbuf();



    src.close();
    dest.close();

    outputFile.open("data-activiaty-log.json");
    inputFile.open("activity-data-log.json", std::ios::out | std::ios::app);
    outputFile << "[" << endl;

}

void ClearFile(ofstream& outputFile, string filename) {
    outputFile.open(filename, ofstream::out | ofstream::trunc); // 以截断方式打开文件
    if (outputFile.is_open()) {
        outputFile.close(); // 关闭文件，这将清空文件内容
    }
    outputFile.open(filename);
    outputFile << "[" << endl;
    // 重新安排下一次清空文件的时间
    if(filename == "data-transmit-log.json"){
        Simulator::Schedule(Seconds(120), &ClearFile, ref(outputFile), filename);
    } else
        Simulator::Schedule(Seconds(30), &ClearFile, ref(outputFile), filename);
}

//解析一行文本并返回解析后的数据
DataRow parseLine(const string &line) {
    istringstream iss(line);
    vector<DataRow> result;
    string token;

    DataRow data;
    if (getline(iss, data.id, ',') &&
        getline(iss, data.terrain, ',') &&
        getline(iss, data.weather)) {
        result.push_back(data);
    }

    return data;
}

NodeContainer CreateNode(int number){
    NodeContainer nodes;
    nodes.Create(number);
    return nodes;
}

WifiHelper CreateWifiHelper(WifiStandard standard){
    WifiHelper wifi;
    wifi.SetStandard(standard);
    return wifi;
}

// 创建MAC层助手,并设置为AD-Hoc模式
WifiMacHelper CreateWifiMacHelper(string macModel)
{
    WifiMacHelper wifiMac;
    wifiMac.SetType(macModel);
    return wifiMac;
}


void NodesAddMovement(MobilityHelper &mobility){
    int time;
    for (size_t i=0;i<nodes.GetN();i++){
        string typeName=nameToTypeMap[Names::FindName(nodes.Get(i))];
        float nodeSpeed;//不在此类型内的节点初始速度为士兵速度
        if(typeName=="士兵"){
            nodeSpeed = soldierSpeed/20;
            time = 10;
        } else if (typeName=="坦克车"){
            nodeSpeed = tankerSpeed/20;
            time = 3;
        } else if (typeName=="工程车"){
            nodeSpeed = constructionVehicleSpeed/20;
            time = 4;
        } else if (typeName=="运输车"){
            nodeSpeed = transportVehicleSpeed/20;
            time = 2;
        } else if (typeName=="医疗车"){
            nodeSpeed = ambulanceSpeed/20;
            time = 2;
        } else if (typeName=="指挥中心"){
            nodeSpeed = commandTentSpeed/20;
            time = 10;
        } else if (typeName=="雷达车"){
            nodeSpeed = radarVehicleSpeed/20;
            time = 3;
        } else if (typeName=="电台"){
            nodeSpeed = radarSpeed/20;
            time = 2;
        } else {
            nodeSpeed = soldierSpeed/20;
            time = 10;
        }
        stringstream speedValue;
        speedValue << "ns3::ConstantRandomVariable[Constant=" << nodeSpeed << "]";
        mobility.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
                        "Bounds", RectangleValue(Rectangle(-70000,-55000,245000,260000)),
                        "Mode", StringValue("Time"),
                        "Time", TimeValue(Seconds(time)),
                        "Speed", StringValue(speedValue.str()));
        mobility.Install (nodes.Get(i));
    }   
}

void ConfigurePlainSpectrumWifiPhy(SpectrumWifiPhyHelper& spectrumWifiPhy, Ptr<FriisPropagationLossModel>& lossModel,double frequency, double rxSensitivity, 
                                    uint32_t antennas, uint32_t maxTxSpatialStreams, uint32_t maxRxSpatialStreams, double txGain, double rxGain, double rxNoiseFigure, 
                                    double txPowerStart, double txPowerEnd)
{
    lossModel->SetFrequency(frequency);
    spectrumWifiPhy.Set("RxSensitivity", DoubleValue(rxSensitivity)); // 设置接收器灵敏度
    spectrumWifiPhy.Set("Antennas", UintegerValue(antennas)); // 设置天线数量
    spectrumWifiPhy.Set("MaxSupportedTxSpatialStreams", UintegerValue(maxTxSpatialStreams)); //设备支持的最大传输空间流的数量。
    spectrumWifiPhy.Set("MaxSupportedRxSpatialStreams", UintegerValue(maxRxSpatialStreams));//设备支持的最大接收空间流的数量。
    spectrumWifiPhy.Set("TxGain", DoubleValue(txGain));// 设置发射天线增益
    spectrumWifiPhy.Set("RxGain", DoubleValue(rxGain));// 设置接收天线增益
    spectrumWifiPhy.Set("RxNoiseFigure", DoubleValue(rxNoiseFigure));
    spectrumWifiPhy.Set("TxPowerStart", DoubleValue(txPowerStart));// 设置功率
    spectrumWifiPhy.Set("TxPowerEnd", DoubleValue(txPowerEnd));
}

void ConfigureMountainSpectrumWifiPhy(SpectrumWifiPhyHelper& spectrumWifiPhy, Ptr<FriisPropagationLossModel>& FriislossModeldouble ,Ptr<NakagamiPropagationLossModel>& NakagamilossModeldouble,
                                                   double txPowerStart,double txPowerEnd, double m0, double m1, double m2,double Antennas,double txGain, double rxGain,double MaxSupportedTxSpatialStreams,
                                                   double MaxSupportedRxSpatialStreams,double RxSensitivity,double frequency ,double rxNoiseFigure) 
{
    spectrumWifiPhy.Set("RxSensitivity", DoubleValue(RxSensitivity));// 设置接收器灵敏度
    spectrumWifiPhy.Set("Antennas", UintegerValue(Antennas));// 设置天线数量
    spectrumWifiPhy.Set("MaxSupportedTxSpatialStreams", UintegerValue(MaxSupportedTxSpatialStreams)); //设备支持的最大传输空间流的数量。
    spectrumWifiPhy.Set("MaxSupportedRxSpatialStreams", UintegerValue(MaxSupportedRxSpatialStreams));//设备支持的最大接收空间流的数量。
    spectrumWifiPhy.Set("TxGain", DoubleValue(txGain));  // 设置发射天线增益
    spectrumWifiPhy.Set("RxGain", DoubleValue(rxGain));  // 设置接收天线增益
    spectrumWifiPhy.Set("RxNoiseFigure", DoubleValue(rxNoiseFigure));
    NakagamilossModeldouble->SetAttribute("m0", DoubleValue(m0));
    NakagamilossModeldouble->SetAttribute("m1", DoubleValue(m1));
    NakagamilossModeldouble->SetAttribute("m2", DoubleValue(m2));
    NakagamilossModeldouble->SetAttribute("Distance1" ,ns3::DoubleValue(501.0));
    NakagamilossModeldouble->SetAttribute("Distance2", ns3:: DoubleValue(600.0));
    spectrumWifiPhy.Set("TxPowerStart", DoubleValue(txPowerStart));// 设置功率
    spectrumWifiPhy.Set("TxPowerEnd", DoubleValue(txPowerEnd));  // 设置功率
}

    
void ConfigureCitySpectrumWifiPhy(SpectrumWifiPhyHelper& spectrumWifiPhy,Ptr<LogDistancePropagationLossModel>& LogDistancelossModeldouble,Ptr<NakagamiPropagationLossModel>&NakagamilossModeldouble,
                                    double txPowerStart,double txPowerEnd, double m0, double m1, double m2,double Antennas,double txGain, double rxGain,
                                    double MaxSupportedTxSpatialStreams,double MaxSupportedRxSpatialStreams,double RxSensitivity,double referenceLoss,double referenceDistance,double pathLossExponent) 
{
    // 使用 SpectrumWifiPhyHelper
    Ptr<MultiModelSpectrumChannel> spectrumChannel = CreateObject<MultiModelSpectrumChannel>();
    LogDistancelossModeldouble->SetAttribute("ReferenceLoss",ns3::DoubleValue(referenceLoss));
    LogDistancelossModeldouble->SetAttribute("ReferenceDistance",ns3 ::DoubleValue(referenceDistance));
    LogDistancelossModeldouble->SetPathLossExponent(pathLossExponent);
    Ptr<ConstantSpeedPropagationDelayModel> delayModel = CreateObject<ConstantSpeedPropagationDelayModel>();
    spectrumChannel->SetPropagationDelayModel(delayModel);
    spectrumWifiPhy.Set("RxSensitivity", DoubleValue(RxSensitivity)); // 设置接收器灵敏度
    spectrumWifiPhy.Set("Antennas", UintegerValue(Antennas));// 设置天线数量
    spectrumWifiPhy.Set("MaxSupportedTxSpatialStreams", UintegerValue(MaxSupportedTxSpatialStreams));  //设备支持的最大传输空间流的数量。
    spectrumWifiPhy.Set("MaxSupportedRxSpatialStreams", UintegerValue(MaxSupportedRxSpatialStreams));//设备支持的最大接收空间流的数量。
    spectrumWifiPhy.Set("TxGain", DoubleValue(txGain));  // 设置发射天线增益
    spectrumWifiPhy.Set("RxGain", DoubleValue(rxGain));  // 设置接收天线增益
    //   spectrumWifiPhy.Set("RxNoiseFigure",DoubleValue(5));//设置噪声指数
    NakagamilossModeldouble->SetAttribute("m0", DoubleValue(m0));
    NakagamilossModeldouble->SetAttribute("m1", DoubleValue(m1));
    NakagamilossModeldouble->SetAttribute("m2", DoubleValue(m2));
    NakagamilossModeldouble->SetAttribute("Distance1" ,ns3::DoubleValue(501.0));
    NakagamilossModeldouble->SetAttribute("Distance2", ns3:: DoubleValue(600.0));
    spectrumWifiPhy.Set("TxPowerStart", DoubleValue(txPowerStart)); // dBm// 设置功率
    spectrumWifiPhy.Set("TxPowerEnd", DoubleValue(txPowerEnd)); // 设置功率
    
}


void ConfigureForestSpectrumWifiPhy(SpectrumWifiPhyHelper& spectrumWifiPhy,Ptr<LogDistancePropagationLossModel>& LogDistancelossModeldouble,Ptr<NakagamiPropagationLossModel>&NakagamilossModeldouble,
                                    double txPowerStart,double txPowerEnd, double m0, double m1, double m2,double Antennas,double txGain, double rxGain,
                                    double MaxSupportedTxSpatialStreams,double MaxSupportedRxSpatialStreams,double RxSensitivity,double referenceLoss,double referenceDistance,double pathLossExponent) 
                                    {
    // 使用 SpectrumWifiPhyHelper
    Ptr<MultiModelSpectrumChannel> spectrumChannel = CreateObject<MultiModelSpectrumChannel>();
    LogDistancelossModeldouble->SetAttribute("ReferenceLoss",ns3::DoubleValue(referenceLoss));
    LogDistancelossModeldouble->SetAttribute("ReferenceDistance",ns3 ::DoubleValue(referenceDistance));
    LogDistancelossModeldouble->SetPathLossExponent(pathLossExponent);
    Ptr<ConstantSpeedPropagationDelayModel> delayModel = CreateObject<ConstantSpeedPropagationDelayModel>();
    spectrumChannel->SetPropagationDelayModel(delayModel);
    spectrumWifiPhy.Set("RxSensitivity", DoubleValue(RxSensitivity)); // 设置接收器灵敏度
    spectrumWifiPhy.Set("Antennas", UintegerValue(Antennas));// 设置天线数量
    spectrumWifiPhy.Set("MaxSupportedTxSpatialStreams", UintegerValue(MaxSupportedTxSpatialStreams)); //设备支持的最大传输空间流的数量。
    spectrumWifiPhy.Set("MaxSupportedRxSpatialStreams", UintegerValue(MaxSupportedRxSpatialStreams));//设备支持的最大接收空间流的数量。
    spectrumWifiPhy.Set("TxGain", DoubleValue(txGain));  // 设置发射天线增益
    spectrumWifiPhy.Set("RxGain", DoubleValue(rxGain));  // 设置接收天线增益
    //   spectrumWifiPhy.Set("RxNoiseFigure",DoubleValue(5));//设置噪声指数
    NakagamilossModeldouble->SetAttribute("m0", DoubleValue(m0));
    NakagamilossModeldouble->SetAttribute("m1", DoubleValue(m1));
    NakagamilossModeldouble->SetAttribute("m2", DoubleValue(m2));
    NakagamilossModeldouble->SetAttribute("Distance1" ,ns3::DoubleValue(501.0));
    NakagamilossModeldouble->SetAttribute("Distance2", ns3:: DoubleValue(600.0));
    spectrumWifiPhy.Set("TxPowerStart", DoubleValue(txPowerStart)); // dBm// 设置功率
    spectrumWifiPhy.Set("TxPowerEnd", DoubleValue(txPowerEnd));// 设置功率
}

void SetTxPower(Ptr<Node> node, double txPower) {
    Ptr<WifiNetDevice> wifiDevice = DynamicCast<WifiNetDevice>(node->GetDevice(0)); // 假设wifi设备是第一个设备
    Ptr<SpectrumWifiPhy> phy = DynamicCast<SpectrumWifiPhy>(wifiDevice->GetPhy());
    phy->SetTxPowerStart(txPower);
    phy->SetTxPowerEnd(txPower);
}

void ChangeChannel(Ptr<Node> node, uint16_t channelId, int number) {
    double nowtime = Simulator::Now().GetSeconds();
    if(isBusy(nowtime)){
        cout << nowtime <<",当前信道正忙,ConfigureWifiPhyAttributes除channel以外修改成功"<<endl;
    } else {
        Ptr<WifiNetDevice> wifiDevice = DynamicCast<WifiNetDevice>(node->GetDevice(0)); // 获取节点的 WiFi 设备
        Ptr<SpectrumWifiPhy> spectrumPhy = DynamicCast<SpectrumWifiPhy>(wifiDevice->GetPhy());
        if(!spectrumPhy -> IsStateSwitching()){//判断当前节点的物理设备是否被选中
            spectrumPhy->SetAttribute("ChannelSettings",StringValue(string("{" + std::to_string(channelId) +", 20, BAND_2_4GHZ, 0}"))); // 设置新的频道号
            cout << nowtime << " 节点: " << FindIdFromMap(node) <<"的ConfigureWifiPhyAttributes修改成功"<<endl;
            number++;
        } else {
            cout << nowtime <<",当前信道正忙,ConfigureWifiPhyAttributes除channel以外修改成功"<<endl;
        }
    } 
}

void ConfigureNode(int nodeId, double factor, SpectrumWifiPhyHelper& spectrumWifiPhy, WifiMacHelper& nodeswifiMac, NodeContainer& nodes,Ptr<FriisPropagationLossModel>& FriislossModeldouble,
                    Ptr<NakagamiPropagationLossModel>&NakagamilossModeldouble,WifiHelper& wifi, string terrain) 
{
    Ptr<Node> node = nodes.Get(nodeId);
    string nodetype = nameToTypeMap[Names::FindName(node)];

    if(nodetype == "士兵"){//士兵采用手持电台设备
        txPowerStart = 35;
        txPowerEnd = 35;
        txGain = 3;
        rxGain = 3;
        rxSensitivity = -101.0;
    }else if(nodetype == "坦克车"){
        txPowerStart = 45;
        txPowerEnd = 45;
        txGain = 12;
        rxGain = 12;
        rxSensitivity = -101.0;
    }else if(nodetype == "工程车"){
        txPowerStart = 45;
        txPowerEnd = 45;
        txGain = 9;
        rxGain = 9;
        rxSensitivity = -101.0;        
    }else if(nodetype == "运输车"){
        txPowerStart = 45;
        txPowerEnd = 45;
        txGain = 12;
        rxGain = 12;
        rxSensitivity = -101.0;    
    }else if(nodetype == "医疗车"){
        txPowerStart = 44.8;
        txPowerEnd = 44.8;
        txGain = 6;
        rxGain = 6;
        rxSensitivity = -101.0;        
    }else if(nodetype == "指挥中心"){
        txPowerStart = 55;
        txPowerEnd = 55;
        txGain = 20;
        rxGain = 20;
        rxSensitivity = -111.0;        
    }else if(nodetype == "雷达车"){
        txPowerStart = 60;
        txPowerEnd = 60;
        txGain = 30;
        rxGain = 30;
        rxSensitivity = -101.0;        
    }else if(nodetype == "电台"){
        txPowerStart = 50;
        txPowerEnd = 50;
        txGain = 15;
        rxGain = 15;
        rxSensitivity = -101.0;        
    }else {
        txPowerStart = 35;
        txPowerEnd = 35;
        txGain = 3;
        rxGain = 3;
        rxSensitivity = -101.0;
    }
    if( terrain == "plain"){
        ConfigurePlainSpectrumWifiPhy (spectrumWifiPhy,FriisPlainlossModel,frequencyGHz*factor,rxSensitivity, antennas, maxTxSpatialStreams, maxRxSpatialStreams, txGain, rxGain,
                                    rxNoiseFigure,txPowerStart,txPowerEnd); 
    }else if( terrain == "city"){
        ConfigureCitySpectrumWifiPhy (spectrumWifiPhy,LogDistanceCitylossModel,NakagamiCitylossModel,txPowerStart,txPowerEnd,m0, m1, m2, antennas, txGain, rxGain,
                                    maxTxSpatialStreams,maxRxSpatialStreams,rxSensitivity,referenceLoss+factor,referenceDistance,pathLossExponent);
    }else if( terrain == "forest"){
        ConfigureForestSpectrumWifiPhy (spectrumWifiPhy,LogDistanceForestlossModel,NakagamiForestlossModel,txPowerStart,txPowerEnd,m0, m1, m2, antennas, txGain, rxGain,
                                    maxTxSpatialStreams,maxRxSpatialStreams,rxSensitivity,referenceLoss+factor,referenceDistance,pathLossExponent); 
    }else if( terrain == "mountain"){
        ConfigureMountainSpectrumWifiPhy(spectrumWifiPhy, FriislossModeldouble, NakagamilossModeldouble, txPowerStart, txPowerEnd, m0-factor, m1-factor, m2-factor, antennas, 
                                    txGain, rxGain, maxTxSpatialStreams, maxRxSpatialStreams, rxSensitivity, frequencyGHz, rxNoiseFigure);
    }else{
        ConfigureMountainSpectrumWifiPhy(spectrumWifiPhy, FriislossModeldouble, NakagamilossModeldouble, txPowerStart, txPowerEnd, m0-factor, m1-factor, m2-factor, antennas, 
                                    txGain, rxGain, maxTxSpatialStreams, maxRxSpatialStreams, rxSensitivity, frequencyGHz, rxNoiseFigure);
    }
} 

Ptr<ListPositionAllocator> ReadToPositionAlloc(vector<Data> &data, NodeContainer nodes){
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    int i = 0;
    // string name;
    for(const auto& d : data)
    {
        AddidToGroup(d.col2, "Red");
        AddPosition(i, d, positionAlloc, nodes, "nodes");
    }  
    return positionAlloc;
}

//返回起始时间，否则返回0
long long getStartTimeIfDuplicate(const std::string& start, const std::string& end) {
    for (const auto& record : dataRecords) {
        if (record.startNode == start && record.endNode == end) {
            return record.startTime;
        }
    }
    return 0;
}

//判断是否有存在的记录
bool isDuplicate(const vector<DataRecord>& records, const string& start, const string& end) {
    for (const auto& record : records) {
        if (record.startNode == start && record.endNode == end) {
            return true;  // 已经存在相同的记录
        }
    }
    return false;  // 没有相同的记录
}

//返回数据在数组中的具体位置
int findIndexIfDuplicate(const std::string& start, const std::string& end) {
    for (size_t i = 0; i < dataRecords.size(); ++i) {
        if (dataRecords[i].startNode == start && dataRecords[i].endNode == end) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

//15s输出延迟时间
void printDataRecords() {
    double totaldelatTime = 0;
    for (const auto& record : dataRecords) {
    //    cout << "Start Node: " << record.startNode
    //              << ", End Node: " << record.endNode
    //              << ", Delay Time: " << record.durationTime/accumulatedTime << " seconds\n";
    //     totaldelatTime += record.durationTime/(accumulatedTime*1000);
    }
    cout<< "前" << accumulatedTime/1000 << "s的整个网络的平均时延为: " << totaldelatTime << " ms " << endl;
    accumulatedTime+=15000;
    Simulator::Schedule(Seconds(15.0), &printDataRecords);
}

//输出吞吐量
void CalculateAndPrintThroughput(){
    // 获取当前时间点
    auto now = chrono::system_clock::now();
    // 转换为时间戳
    auto timeStamp = chrono::duration_cast<chrono::milliseconds>(now.time_since_epoch()).count();

    double currentTime = Simulator::Now ().GetSeconds ();
    double throughput = totalReceivedBytes / currentTime;
    double totalThroughout = 0;
    for (const auto& entry:throughoutputByModelId)
    {
        // cout<<"currentTime:"<<timeStamp<<",Model ID:" << entry.first << ",throughput:"<<entry.second/currentTime << "Bps" << endl;
        totalThroughout += entry.second/currentTime;
    }
    cout << "前" << currentTime << "s的整个网络的平均吞吐量为: " << totalThroughout/1024.0 << "KBps" << endl;
    Simulator::Schedule (Seconds (15.0), &CalculateAndPrintThroughput);
}

void DataInfoFile(Ptr<Node> start, Ptr<Node> &target, uint32_t size,long long time, const string &mediaType, ofstream& outputFile){
    if(second == 1){
        outputFile << "[" << endl;
        second = 0;
    }

    string startModelId = FindIdFromMap(start);
    string targetModelId = FindIdFromMap(target);
    //返回对应节点的时延
    long long startTime = getStartTimeIfDuplicate(startModelId, targetModelId);
    //判断对应节点是否有相同的记录
    if(!isDuplicate(dataRecords, startModelId, targetModelId)){
        dataRecords.push_back(DataRecord(startModelId, targetModelId, time ,0));
    }
    //返回对应节点数组中的位置
    int index = findIndexIfDuplicate(startModelId, targetModelId);
    //更新时间
    if (index != -1){
        dataRecords[index].durationTime += (time -startTime);
        dataRecords[index].startTime = time;
    }

    double delay = time -startTime;

    string datarate;
    if(size == 2048){
        datarate = "500kbps";
    }else if(size == 1024){
        datarate = "100kbps";
    }else{
        datarate = "50kbps";
    }


    for (auto& vectorNode : nodeVector){
        if (vectorNode.nodeID == targetModelId) {     
            vectorNode.updateSNR(datatemp.snr);
            vectorNode.updateTime(datatemp.timestamp);
            vectorNode.updateDelay(delay);
            vectorNode.updatePacketSize(size);
        }
    }

    outputFile << "{";//开始第一个json对象
    outputFile << "\"StartTimestamp\": \"" << startTime << "ms\", ";
    outputFile << "\"ArriveTimestamp\": \"" << time << "ms\", ";
    outputFile << "\"PkgType\": \"002\",";
    outputFile << "\"StartNodeid\": \"" << startModelId << "\", ";
    outputFile << "\"StartNodeGroup\":\""<< FindFromGroupMap(FindIdFromMap(start)) <<"\",";
    outputFile << "\"StartNodeType\": \"" << nameToTypeMap[Names::FindName(start)] << "\", ";
    outputFile << "\"TargetNodeid\":\""<< targetModelId <<"\",";
    outputFile << "\"TargetNodeGroup\":\""<< FindFromGroupMap(FindIdFromMap(target)) <<"\",";
    outputFile << "\"TargetNodeType\": \"" << nameToTypeMap[Names::FindName(target)] << "\", ";  
    outputFile << "\"DataSize\": \"" << size << "\",";
    outputFile << "\"MediaType\": \"" << mediaType << "\",";  // 添加 MediaType
    outputFile << "\"frequency\": \"" << datatemp.frequency << "MHz" << "\",";
    outputFile << "\"SNR\": \"" << datatemp.snr << "dB" << "\",";
    outputFile << "\"MCS\": \"" << datatemp.mcs << "\",";
    outputFile << "\"Delay\": \"" << delay <<"ms\",";
    outputFile << "\"DataRate\": \"" << datarate << "\"";
    outputFile << "}";  // 结束 JSON 对象
    outputFile << "," <<endl;    

    stringstream ss;  // 创建 stringstream 对象
    ss << "{";  // 开始第一个json对象
    ss << "\"StartTimestamp\": \"" << startTime << "ms\", ";
    ss << "\"ArriveTimestamp\": \"" << time << "ms\", ";
    ss << "\"PkgType\": \"002\",";
    ss << "\"StartNodeid\": \"" << startModelId << "\", ";
    ss << "\"StartNodeGroup\":\""<< FindFromGroupMap(FindIdFromMap(start)) <<"\",";
    ss << "\"StartNodeType\": \"" << nameToTypeMap[Names::FindName(start)] << "\", ";
    ss << "\"TargetNodeid\":\""<< targetModelId <<"\",";
    ss << "\"TargetNodeGroup\":\""<< FindFromGroupMap(FindIdFromMap(target)) <<"\",";
    ss << "\"TargetNodeType\": \"" << nameToTypeMap[Names::FindName(target)] << "\", ";  
    ss << "\"DataSize\": \"" << size << "\",";
    ss << "\"MediaType\": \"" << mediaType << "\",";  // 添加 MediaType
    ss << "\"frequency\": \"" << datatemp.frequency << "MHz" << "\",";
    ss << "\"SNR\": \"" << datatemp.snr << "dB" << "\",";
    ss << "\"MCS\": \"" << datatemp.mcs << "\",";
    ss << "\"Delay\": \"" << delay <<"ms\",";
    ss << "\"DataRate\": \"" << datarate << "\"";
    ss << "}";  // 结束 JSON 对象

    string jsonString = ss.str();  // 将 stringstream 转换为 string
    stringQueue.push(jsonString);
}


void startDataActiviatyInfo(){
    activiaty = 1;
    Simulator::Schedule(Seconds(30.0), &startDataActiviatyInfo);
}

//
void stopDataActiviatyInfo(fstream& file, ofstream& outputfile, ofstream& inputfile){
    activiaty = 0;

    // Simulator::Schedule(Seconds(1.0), &Closefile, ref(inputfile));

    // Simulator::Schedule(Seconds(2), &ModifyJsonFile, ref(file));//修改json文件格式

    Simulator::Schedule(Seconds(0.5), &ClearActivityFile, ref(outputfile), ref(inputfile));

    Simulator::Schedule(Seconds(30.0), &stopDataActiviatyInfo, ref(file), ref(outputfile), ref(inputfile));
}

int stepcount=0;

void dataActiviatyInfoFile(NodeContainer Nodes, ofstream& outputFile)
{
//处理收到数据包数据 
    double nowtime = Simulator::Now().GetSeconds();
    if(fmod(nowtime,30.0) == 0){
        stepcount++;
        for(uint16_t i = 0; i < Nodes.GetN(); ++i) {
            Ptr<MobilityModel> mobility = Nodes.Get(i)->GetObject<MobilityModel>();
            Vector pos = mobility->GetPosition();
            Vector speed = mobility->GetVelocity();
            double speedMagnitude = 20*sqrt(speed.x * speed.x + speed.y * speed.y + speed.z * speed.z);//节点的当前速度
            string foundName = Names::FindName(Nodes.Get(i));
            vector<string> retrievedVector = SearchVectorData(Nodes.Get(i));
            // 获取当前时间点
            auto now = chrono::system_clock::now();
            //节点吞吐量信息
            double currentTime = Simulator::Now ().GetSeconds ();
            double throughout;
            if(currentTime == 0)
                throughout = 0;
            else    
                throughout = throughoutputByModelId[nameToIdMap[foundName]]/currentTime ;

            // 转换为时间戳
            auto timestamp = chrono::duration_cast<chrono::milliseconds>(now.time_since_epoch()).count();
            double X,Y,Z;
            if(third == 1)
                outputFile << "[" <<endl;
            outputFile << "{";  // 开始一个 JSON 对象

            // 添加各种数据到 JSON 对象
            outputFile << "\"Step\": \"" << stepcount << "\", ";
            outputFile << "\"Timestamp\": \"" << timestamp << "ms\", ";
            outputFile << "\"PkgType\": \"003\",";
            outputFile << "\"NodeId\": \"" << FindIdFromMap(Nodes.Get(i)) << "\", ";
            outputFile << "\"NodeGroup\":\""<< FindFromGroupMap(FindIdFromMap(Nodes.Get(i))) <<"\",";
            outputFile << "\"NodeType\": \"" << nameToTypeMap[foundName] << "\", ";
            outputFile << "\"NodeName\": \"" << foundName << "\", ";
            outputFile << "\"NodeSpeed\": \""  << speedMagnitude << "m/s\", ";

            outputFile << "\"NodeTxPower\": \"" << NodePower(Nodes.Get(i)) << "dBm\", ";
            outputFile << "\"NodeTP\": \"" << throughout << "Bps\", ";

            XYZ2LLA(pos.x*20,pos.y*20,pos.z*20,X,Y,Z);//直接采用原始未经处理的地理坐标转换的笛卡尔坐标系坐标
            outputFile << "\"Position\": {\"x\": "  << fixed << setprecision(6)<< X << ", \"y\": " << Y << ", \"z\": " << Z << "}, ";//输出地理坐标
            outputFile.unsetf(ios_base::fixed);
            outputFile.precision(streamsize(-1));
            outputFile << "\"Terrain\": \"" << retrievedVector[0] << "\", ";
            outputFile << "\"Weather\": \"" << retrievedVector[1] << "\"";

            outputFile << ",\"frequency\": \"" << "0" << "\",";
            outputFile << "\"SNR\": \"" << "0" << "\",";
            outputFile << "\"MCS\": \"" << "0" << "\"";

            outputFile << "}";  // 结束 JSON 对象
            if(third == 1) {
                third = 0;
                outputFile << "," <<endl;
            }else{
                outputFile <<","<<endl;  // 换行
            }
        }
        // 每30秒记录
        Simulator::Schedule(Seconds(30), &dataActiviatyInfoFile, Nodes, ref(outputFile));
    }

    if(activiaty == 1){
        if(datatemp.status == 1){
            datatemp.status = 0;
            Ptr<Node> node = datatemp.nodes.Get(0);
            Ptr<MobilityModel> mobility = node->GetObject<MobilityModel>();
            Vector pos = mobility->GetPosition();
            Vector speed = mobility->GetVelocity();
            double speedMagnitude = 20*sqrt(speed.x * speed.x + speed.y * speed.y + speed.z * speed.z);//节点的当前速度
            string foundName = Names::FindName(node);
            vector<string> retrievedVector = SearchVectorData(node);
            double X,Y,Z;

            //节点吞吐量信息
            double currentTime = Simulator::Now ().GetSeconds ();
            double throughout;
            if(currentTime == 0)
                throughout = 0;
            else    
                throughout = throughoutputByModelId[nameToIdMap[foundName]]/currentTime ;

            if(third == 1)
                outputFile << "[" <<endl;
            outputFile << "{";  // 开始一个 JSON 对象

            // 添加各种数据到 JSON 对象
            outputFile << "\"Step\": \"" << stepcount << "\", ";
            outputFile << "\"Timestamp\": \"" << datatemp.timestamp << "ms\", ";
            outputFile << "\"PkgType\": \"003\",";
            outputFile << "\"NodeId\": \"" << FindIdFromMap(node) << "\", ";
            outputFile << "\"NodeGroup\":\""<< FindFromGroupMap(FindIdFromMap(node)) <<"\",";
            outputFile << "\"NodeType\": \"" << nameToTypeMap[foundName] << "\", ";
            outputFile << "\"NodeName\": \"" << foundName << "\", ";
            outputFile << "\"NodeSpeed\": \""  << speedMagnitude << "m/s\", ";
            
            outputFile << "\"NodeTxPower\": \"" << NodePower(node) << "dBm\", ";
            outputFile << "\"NodeTP\": \"" << throughout << "Bps\", ";

            XYZ2LLA(pos.x*20,pos.y*20,pos.z*20,X,Y,Z);//直接采用原始未经处理的地理坐标转换的笛卡尔坐标系坐标
            outputFile << "\"Position\": {\"x\": "  << fixed << setprecision(6)<< X << ", \"y\": " << Y << ", \"z\": " << Z << "}, ";//输出地理坐标
            outputFile.unsetf(ios_base::fixed);
            outputFile.precision(streamsize(-1));
            outputFile << "\"Terrain\": \"" << retrievedVector[0] << "\", ";
            outputFile << "\"Weather\": \"" << retrievedVector[1] << "\"";
            outputFile << ",\"frequency\": \"" << datatemp.frequency << "MHz" << "\",";
            outputFile << "\"SNR\": \"" << datatemp.snr << "dB" << "\",";
            outputFile << "\"MCS\": \"" << datatemp.mcs.GetUniqueName()<< "\"";
            outputFile << "}";  // 结束 JSON 对象
            if(third == 1) {
                third = 0;
                outputFile << "," <<endl;
            }else{
                outputFile <<","<<endl;  // 换行
            }
        }
    }
}

void ReceivePacket (string context, Ptr<const Packet> packet, const Address &from){
    regex rgx("/NodeList/(\\d+)/"); // 正则表达式匹配节点 ID
    smatch match;

    if (regex_search(context, match, rgx) && match.size() > 1)
    {
        int nodeId = stoi(match.str(1)); // 获取节点 ID
        // cout << nodeId <<endl;
        // 获取节点并从中检索 IP 地址
        Ptr<Node> node = NodeList::GetNode(nodeId);
        Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
        Ipv4Address localAddr = ipv4->GetAddress(1, 0).GetLocal();

        // 获取当前时间点
        auto now = chrono::system_clock::now();

        // 转换为时间戳
        auto timestamp = chrono::duration_cast<chrono::milliseconds>(now.time_since_epoch()).count();

        string mediaTypeString;

        if(packet->GetSize() == 512){
            mediaTypeString = "文本";
        }else if(packet->GetSize() == 1024){
            mediaTypeString = "音频";
        }else{
            mediaTypeString = "视频";
        }

        //更新吞吐量
        throughoutputByModelId[FindIdFromMap(node)]+=packet->GetSize ();
        // 打印信息
        InetSocketAddress addr = InetSocketAddress::ConvertFrom (from);
        if(addr.GetIpv4 ()!=localAddr){
            DataInfoFile(FindFromMap(ipToIdMap[addr.GetIpv4()]), node, packet->GetSize(), timestamp, mediaTypeString, ref(dataputFile));
        }
    }else{
        cout << "Failed to extract node ID from context: " << context << std::endl;
    }
}

void StartSpecificTransmission(uint32_t sourceIndex, uint32_t targetIndex, NodeContainer &sourceNodes, NodeContainer &targetNodes, uint16_t port, Time duration) {
    Ptr<Node> sourceNode = sourceNodes.Get(sourceIndex);
    Ipv4Address targetAddress;
    Ptr<Node> node = targetNodes.Get(targetIndex);
    Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>(); // 假设节点已安装了 IPv4
    std::vector<Ipv4Address> addresses;

    // 获取每个接口上的 IP 地址
    for (uint32_t j = 0; j < ipv4->GetNInterfaces(); j++) {
        for (uint32_t k = 0; k < ipv4->GetNAddresses(j); k++) {
            Ipv4InterfaceAddress iface = ipv4->GetAddress(j, k);
            addresses.push_back(iface.GetLocal());
        }
    }
    if(addresses.size()>2){
        // for(uint16_t i=1 ; i<addresses.size() ; ++i){
        string id = FindIdFromMap(node);
        ipToIdMap[addresses[3]] = id;
        targetAddress = addresses[3];
        // }
    }else{
        string id = FindIdFromMap(node);
        ipToIdMap[addresses[1]] = id;
        targetAddress = addresses[1];
    }


    string datarate;//数据包发送速率
    int packetsize, maxPackets;//数据包大小以及最大数据包个数

    MediaType mediaType = GenerateRandomMediaType();
    string mediaTypeString = MediaTypeToString(mediaType);//视频，文本，录音，未知类型

    string endid = FindIdFromMap(targetNodes.Get(targetIndex));

    double videoPacketRate = 50;     // 每秒数据包数
    double audioPacketRate = 70;     // 每秒数据包数
    double textPacketRate = 30;       // 每秒数据包数

    if(mediaTypeString == "文本" || mediaTypeString == "未知类型"){
        datarate = "50kb/s";
        packetsize = 512;
        maxPackets = textPacketRate * duration.GetSeconds();
    }else if(mediaTypeString == "视频"){
        datarate = "500kb/s";
        packetsize = 2048;
        maxPackets = videoPacketRate * duration.GetSeconds();
    }else if(mediaTypeString == "录音"){
        packetsize = 1024;
        datarate = "100kb/s";
        maxPackets = audioPacketRate * duration.GetSeconds();
    }else{
        datarate = "50kb/s";
        packetsize = 512;
        maxPackets = textPacketRate * duration.GetSeconds();
    }
    OnOffHelper onOffHelper("ns3::UdpSocketFactory", Address(InetSocketAddress(targetAddress, port)));
    onOffHelper.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant="+continuetime+"]"));
    onOffHelper.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant="+stoptime+"]"));
    onOffHelper.SetAttribute("DataRate", DataRateValue(DataRate(datarate)));
    onOffHelper.SetAttribute("PacketSize", UintegerValue(packetsize));
    onOffHelper.SetAttribute("MaxBytes", UintegerValue(packetsize * maxPackets)); // 设置最大字节数
    // 记录发送开始时间
    auto now = chrono::system_clock::now();
    long long initialTime = chrono::duration_cast<chrono::milliseconds>(now.time_since_epoch()).count();

    string startModelId = FindIdFromMap(sourceNode);
    string targetModelId = FindIdFromMap(node);
    long long startTime = initialTime;
    if(!isDuplicate(dataRecords, startModelId, targetModelId)){
        dataRecords.push_back(DataRecord(startModelId, targetModelId, startTime ,0));
    }

    ApplicationContainer tempApp = onOffHelper.Install(sourceNode);
    tempApp.Start(Seconds(Simulator::Now().GetSeconds())); // 立即开始
    tempApp.Stop(Seconds(Simulator::Now().GetSeconds() + duration.GetSeconds())); // 持续时间后停止
}


void MonitorSnifferRx (Ptr<Node> node, Ptr<const Packet> packet, uint16_t channelFreqMhz, WifiTxVector txVector, 
                        MpduInfo mpduInfo, SignalNoiseDbm signalNoise, uint16_t frequency) {
    double snr = signalNoise.signal - signalNoise.noise;

    NodeContainer tempNodes;
    tempNodes.Add(node);
    auto now = chrono::system_clock::now();
    //给用户活跃度文件添加数据的结构体
    datatemp.timestamp = chrono::duration_cast<chrono::milliseconds>(now.time_since_epoch()).count();
    datatemp.frequency = channelFreqMhz;
    datatemp.nodes = tempNodes;
    datatemp.status = 1;
    datatemp.snr = snr;
    datatemp.mcs = txVector.GetMode();
    // 使用字符串流进行转换
    stringstream ss;
    ss << txVector.GetMode();
    
    string wifiModeAsString = ss.str(); // 字符串表示

    // if(wifiModeAsString != "DsssRate1Mbps"){
        totalSnr += snr;
        totalPackets += 1;
    // }

    throughoutputByModelId[FindIdFromMap(node)]+=packet->GetSize ();
    if(activiaty == 1)
        Simulator::Schedule(Seconds(0.0),&dataActiviatyInfoFile,tempNodes,ref(dataActiviaty));
}

void StartTransmit(NodeContainer Nodes, uint16_t port){
    startTransmissionTime += Seconds(sendtimeinterval);
    Simulator::Schedule(startTransmissionTime, &StartSpecificTransmission, 0, 1, Nodes, Nodes, port, transmissionDuration);
    for (uint32_t i = 0; i < Nodes.GetN(); i++)
    {
        for (uint32_t j = 0; j < Nodes.GetN(); j++)
        {
            if(i != j && !(i == 0 && j==1)){
                Simulator::Schedule(startTransmissionTime, &StartSpecificTransmission, i, j, Nodes, Nodes, port, transmissionDuration);            
            }
        }
    }
    Simulator::Schedule(Seconds(sendtimeinterval), &StartTransmit, Nodes, port);
}

int weatherType(string Weather){
    int weather;
    if(Weather == "sunny"){
        weather = 1;
    }else if(Weather == "rainy"){
        weather = 2;
    }else if(Weather == "snowy"){
        weather = 3;
    }else if(Weather == "thunder"){
        weather = 4;
    }else{
        weather = 1;
    }
    return weather;
}

int isBusy(double nowtime){//判断节点当前是否处于发送数据包的繁忙状态 1为忙，0为空闲
    double overtime =fmod(nowtime, 60.0);
    double inittime = fmod(overtime, (stod(continuetime) + stod(stoptime)));
    if(overtime <= transmissionDuration.GetSeconds()){
        if(inittime <= stod(continuetime)){
            busy = 1;
        } else {
            busy = 0;
        }
    } else {
        busy = 0;
    }
    return busy;
}

void ConfigureWifiPhyAttributes(vector<string> data, int intervaltime, int maxsize){
    cout << "ConfigureWifiPhyAttributes运行" << endl;
    double nowtime = Simulator::Now().GetSeconds();
    if(isBusy(nowtime)){
        cout << fmod(nowtime, 60.0) << ",当前时间不支持ConfigureWifiPhyAttributes修改,请稍后再试！" << endl;
    } else {
        string name=idToNameMap[data[0]];
        //使用节点名称从Names空间中找到对应的节点
        Ptr<Node> node = Names::Find<Node>(name);
        Ptr<WifiNetDevice> wifiDevice = DynamicCast<WifiNetDevice>(node->GetDevice(0));
        if(wifiDevice !=nullptr) {
            // 获取WifiPhy对象
            Ptr<WifiPhy> wifiPhy = wifiDevice->GetPhy();
            wifiPhy->SetAttribute("Antennas", UintegerValue(stoi(data[1])));
            wifiPhy->SetAttribute("TxGain", DoubleValue(stod(data[2])));  // 设置发射天线增益
            wifiPhy->SetAttribute("RxGain", DoubleValue(stod(data[3])));  // 设置发射天线增益
            wifiPhy->SetAttribute("MaxSupportedTxSpatialStreams", UintegerValue(stoi(data[4]))); //设备支持的最大传输空间流的数量。
            wifiPhy->SetAttribute("MaxSupportedRxSpatialStreams", UintegerValue(stoi(data[5]))); //设备支持的最大传输空间流的数量。
            Ptr<SpectrumWifiPhy> spectrumPhy = DynamicCast<SpectrumWifiPhy>(wifiDevice->GetPhy());
            spectrumPhy->SetTxPowerStart(stod(data[7]));//设置设备的发送功率
            spectrumPhy->SetTxPowerEnd(stod(data[7]));//设置设备的接收功率
            unsigned long number = stoul(data[6]);
            uint16_t convertedNumber = static_cast<uint16_t>(number);
            // cout << "修改的信道号为: " << convertedNumber << endl;
            // Simulator::Schedule(MilliSeconds(10), &ChangeChannel, nodes.Get(jishu), convertedNumber);  
            cout << "修改的信道号为: " << convertedNumber << endl;
            Simulator::Schedule(MilliSeconds(20+20*intervaltime), &ChangeChannel, node, convertedNumber, 0); 
        }
    }
}

void ChangeSingleNodeDataRate(string modelId, string dataRate) {
    string name=idToNameMap[modelId];
    //使用节点名称从Names空间中找到对应的节点
    Ptr<Node> node = Names::Find<Node>(name);
    // 获取指定节点上的 Wi-Fi 网络设备
    Ptr<WifiNetDevice> wifiDevice = DynamicCast<WifiNetDevice>(node->GetDevice(0));
    NS_ASSERT(wifiDevice != nullptr); // 确保设备确实是 Wi-Fi 设备

    // 获取 Remote Station Manager
    Ptr<WifiRemoteStationManager> stationManager = wifiDevice->GetRemoteStationManager();
    NS_ASSERT(stationManager->GetInstanceTypeId() == ConstantRateWifiManager::GetTypeId());

    // 将 Remote Station Manager 强制转换为 ConstantRateWifiManager
    Ptr<ConstantRateWifiManager> constantRateManager = DynamicCast<ConstantRateWifiManager>(stationManager);

    // 设置新的速率
    constantRateManager->SetAttribute("DataMode", StringValue(dataRate));
    constantRateManager->SetAttribute("ControlMode", StringValue("HtMcs0"));
    cout << Simulator::Now().GetSeconds() <<"ChangeSingleNodeDataRate修改成功"<<endl;
}


void ConfigureEncoding(string modelId,string encode,string encodeRate,string maxTransmissonRate){
    cout << "ConfigureEncoding运行" << endl;
    double nowtime = Simulator::Now().GetSeconds();
    if(isBusy(nowtime)){
        cout << fmod(nowtime, 60.0) << ",当前时间不支持ChangeSingleNodeDataRate修改,请稍后再试！" << endl;
    } else {
        if(encode=="BPSK"){
            ChangeSingleNodeDataRate(modelId,"HtMcs0");
        }else if (encode=="QPSK"){
            if(encodeRate=="1/2(a)"){
                ChangeSingleNodeDataRate(modelId,"HtMcs1");
            }else if (encodeRate=="3/4(b)"){
                ChangeSingleNodeDataRate(modelId,"HtMcs2");
            }
        }else if (encode=="16-QAM"){
            if(encodeRate=="1/2(a)"){
                ChangeSingleNodeDataRate(modelId,"HtMcs3");
            }else if (encodeRate=="3/4(b)"){
                ChangeSingleNodeDataRate(modelId,"HtMcs4");
            }
        }else if (encode=="64-QAM"){
            if(encodeRate=="2/3(a)"){
                ChangeSingleNodeDataRate(modelId,"HtMcs5");
            }else if (encodeRate=="3/4(b)"){
                ChangeSingleNodeDataRate(modelId,"HtMcs6");
            }else if (encodeRate=="5/6(c)"){
                ChangeSingleNodeDataRate(modelId,"HtMcs7");
            }
        }else if (encode=="OFDM"){
            ChangeSingleNodeDataRate(modelId,"ErpOfdmRate24Mbps");
        }else if (encode=="DSSS"){
            ChangeSingleNodeDataRate(modelId,"DsssRate1Mbps");
        }
    }    
}

//干扰相关代码
//干扰事件生成
void ScheduleWaveformGeneratorEvents(NetDeviceContainer& devices, Time startTime, Time stopTime) {
    Simulator::Schedule(Seconds(startTime.GetSeconds()), 
                        &WaveformGenerator::Start, 
                        devices.Get(0)->GetObject<NonCommunicatingNetDevice>()->GetPhy()->GetObject<WaveformGenerator>());

    Simulator::Schedule(Seconds(stopTime.GetSeconds()), 
                        &WaveformGenerator::Stop, 
                        devices.Get(0)->GetObject<NonCommunicatingNetDevice>()->GetPhy()->GetObject<WaveformGenerator>());
}

//单音干扰
void CreateSingleToneInterference(){
    // 创建单音干扰
    Ptr<SpectrumModel> DisturbBand = Create<SpectrumModel>(allBands[2]);
    interferingNode.Create(1);
    Ptr<SpectrumValue> wgPsd = Create<SpectrumValue>(DisturbBand);
    *wgPsd = 0.00000001 / 20e6;
    WaveformGeneratorHelper waveformGeneratorHelper;
    waveformGeneratorHelper.SetChannel(spectrumChannel);
    waveformGeneratorHelper.SetTxPowerSpectralDensity(wgPsd);
    //设置干扰波形的周期为 0.0007 秒，并设置占空比为 1（即连续发射，没有间断）。
    waveformGeneratorHelper.SetPhyAttribute("Period", TimeValue(Seconds(0.0007)));
    waveformGeneratorHelper.SetPhyAttribute("DutyCycle", DoubleValue(1));
    //在干扰节点上安装波形生成器设备。
    NetDeviceContainer waveformGeneratorDevices = waveformGeneratorHelper.Install(interferingNode);
    //调度一个仿真事件，在仿真时启动波形生成器，开始生成干扰。
        // 获取当前仿真时间
    Time currentTime = Simulator::Now();

    // 在当前仿真时间上加上15秒
    Time newTime = currentTime + Seconds(15.0);

    ScheduleWaveformGeneratorEvents(waveformGeneratorDevices, currentTime, newTime); 
}

//多音干扰
void CreateMultiToneInterference(){
    // 创建多个频率带
    Ptr<SpectrumModel> disturbBand2 = Create<SpectrumModel>(allBands[2]);
    Ptr<SpectrumModel> disturbBand4 = Create<SpectrumModel>(allBands[4]);

    interferingNode.Create(2); // 创建两个干扰节点

    // 对每个频率带设置功率谱密度
    Ptr<SpectrumValue> wgPsd2 = Create<SpectrumValue>(disturbBand2);
    *wgPsd2 = 0.000001 / 20e6;
    Ptr<SpectrumValue> wgPsd4 = Create<SpectrumValue>(disturbBand4);
    *wgPsd4 = 0.000002 / 20e6;

    WaveformGeneratorHelper waveformGeneratorHelper;

    // 设置通用属性
    waveformGeneratorHelper.SetChannel(spectrumChannel);
    waveformGeneratorHelper.SetPhyAttribute("Period", TimeValue(Seconds(0.0007)));
    waveformGeneratorHelper.SetPhyAttribute("DutyCycle", DoubleValue(1));

    // 在节点上安装波形生成器
    waveformGeneratorHelper.SetTxPowerSpectralDensity(wgPsd2);
    NetDeviceContainer waveformGeneratorDevices2 = waveformGeneratorHelper.Install(interferingNode.Get(0));
    waveformGeneratorHelper.SetTxPowerSpectralDensity(wgPsd4);
    NetDeviceContainer waveformGeneratorDevices4 = waveformGeneratorHelper.Install(interferingNode.Get(1));

    // 调度仿真事件来启动和停止波形生成器
    Time currentTime = Simulator::Now();
    Time newTime = currentTime + Seconds(15.0);

    ScheduleWaveformGeneratorEvents(waveformGeneratorDevices2, currentTime, newTime);
    ScheduleWaveformGeneratorEvents(waveformGeneratorDevices4, currentTime + Seconds(2), newTime + Seconds(2));
}

//梳状谱干扰
void CreateCombSpectrumInterference(){
    // 创建多个频率带
    Ptr<SpectrumModel> disturbBand3 = Create<SpectrumModel>(allBands[3]);
    Ptr<SpectrumModel> disturbBand4 = Create<SpectrumModel>(allBands[4]);
    Ptr<SpectrumModel> disturbBand5 = Create<SpectrumModel>(allBands[5]);
    Ptr<SpectrumModel> disturbBand6 = Create<SpectrumModel>(allBands[6]);
    Ptr<SpectrumModel> disturbBand7 = Create<SpectrumModel>(allBands[7]);
    Ptr<SpectrumModel> disturbBand8 = Create<SpectrumModel>(allBands[8]);
    Ptr<SpectrumModel> disturbBand9 = Create<SpectrumModel>(allBands[9]);
    Ptr<SpectrumModel> disturbBand10 = Create<SpectrumModel>(allBands[10]);
    
    // 创建干扰节点
    interferingNode.Create(8);

    double power = 0.000001 / 20e6;

    // 对每个频率带设置功率谱密度
    Ptr<SpectrumValue> wgPsd3 = Create<SpectrumValue>(disturbBand3);*wgPsd3 = power;
    Ptr<SpectrumValue> wgPsd4 = Create<SpectrumValue>(disturbBand4);*wgPsd4 = power;
    Ptr<SpectrumValue> wgPsd5 = Create<SpectrumValue>(disturbBand5);*wgPsd5 = power;
    Ptr<SpectrumValue> wgPsd6 = Create<SpectrumValue>(disturbBand6);*wgPsd6 = power;
    Ptr<SpectrumValue> wgPsd7 = Create<SpectrumValue>(disturbBand7);*wgPsd7 = power;
    Ptr<SpectrumValue> wgPsd8 = Create<SpectrumValue>(disturbBand8);*wgPsd8 = power;
    Ptr<SpectrumValue> wgPsd9 = Create<SpectrumValue>(disturbBand9);*wgPsd9 = power;
    Ptr<SpectrumValue> wgPsd10 = Create<SpectrumValue>(disturbBand10);*wgPsd10 = power;

    WaveformGeneratorHelper waveformGeneratorHelper;

    // 设置通用属性
    waveformGeneratorHelper.SetChannel(spectrumChannel);
    waveformGeneratorHelper.SetPhyAttribute("Period", TimeValue(Seconds(0.0007)));
    waveformGeneratorHelper.SetPhyAttribute("DutyCycle", DoubleValue(1));

    // 在节点上安装波形生成器
    waveformGeneratorHelper.SetTxPowerSpectralDensity(wgPsd3);
    NetDeviceContainer waveformGeneratorDevices3 = waveformGeneratorHelper.Install(interferingNode.Get(0));
    waveformGeneratorHelper.SetTxPowerSpectralDensity(wgPsd4);
    NetDeviceContainer waveformGeneratorDevices4 = waveformGeneratorHelper.Install(interferingNode.Get(1));
    waveformGeneratorHelper.SetTxPowerSpectralDensity(wgPsd5);
    NetDeviceContainer waveformGeneratorDevices5 = waveformGeneratorHelper.Install(interferingNode.Get(2));
    waveformGeneratorHelper.SetTxPowerSpectralDensity(wgPsd6);
    NetDeviceContainer waveformGeneratorDevices6 = waveformGeneratorHelper.Install(interferingNode.Get(3));
    waveformGeneratorHelper.SetTxPowerSpectralDensity(wgPsd7);
    NetDeviceContainer waveformGeneratorDevices7 = waveformGeneratorHelper.Install(interferingNode.Get(4));
    waveformGeneratorHelper.SetTxPowerSpectralDensity(wgPsd8);
    NetDeviceContainer waveformGeneratorDevices8 = waveformGeneratorHelper.Install(interferingNode.Get(5));
    waveformGeneratorHelper.SetTxPowerSpectralDensity(wgPsd9);
    NetDeviceContainer waveformGeneratorDevices9 = waveformGeneratorHelper.Install(interferingNode.Get(6));
    waveformGeneratorHelper.SetTxPowerSpectralDensity(wgPsd10);
    NetDeviceContainer waveformGeneratorDevices10 = waveformGeneratorHelper.Install(interferingNode.Get(7));

    // 调度仿真事件来启动和停止波形生成器
    Time currentTime = Simulator::Now();
    Time newTime = currentTime + Seconds(15.0);

    ScheduleWaveformGeneratorEvents(waveformGeneratorDevices3, currentTime, newTime);
    ScheduleWaveformGeneratorEvents(waveformGeneratorDevices4, currentTime + Seconds(1), newTime + Seconds(1));
    ScheduleWaveformGeneratorEvents(waveformGeneratorDevices5, currentTime + Seconds(2), newTime + Seconds(2));
    ScheduleWaveformGeneratorEvents(waveformGeneratorDevices6, currentTime + Seconds(3), newTime + Seconds(3));
    ScheduleWaveformGeneratorEvents(waveformGeneratorDevices7, currentTime + Seconds(4), newTime + Seconds(4));
    ScheduleWaveformGeneratorEvents(waveformGeneratorDevices8, currentTime + Seconds(5), newTime + Seconds(5));
    ScheduleWaveformGeneratorEvents(waveformGeneratorDevices9, currentTime + Seconds(6), newTime + Seconds(6));
    ScheduleWaveformGeneratorEvents(waveformGeneratorDevices10, currentTime + Seconds(7), newTime + Seconds(7));
}

// 部分频带噪声干扰
void CreatePartialBandNoiseJamming() {
    // 创建频带信息
    BandInfo band1;
    band1.fl = 2430e6;//频率下限
    band1.fh = 2450e6; // 频率上限
    band1.fc = (band1.fl + band1.fh) / 2; //频率中心

    Bands bands;
    bands.push_back(band1);
    Ptr<SpectrumModel> spectrumModel = Create<SpectrumModel>(bands);

    // 创建干扰节点
    interferingNode.Create(1);

    // 创建噪声调频干扰节点
    Ptr<SpectrumValue> wgPsd = Create<SpectrumValue>(spectrumModel);
    Ptr<UniformRandomVariable> randVar = CreateObject<UniformRandomVariable>();
    randVar->SetAttribute("Min", DoubleValue(0.000001 / 20e6));
    randVar->SetAttribute("Max", DoubleValue(0.00001 / 20e6)); // 调整最大值根据需要
    *wgPsd = randVar->GetValue();

    WaveformGeneratorHelper waveformGeneratorHelper;
    // 设置通用属性
    waveformGeneratorHelper.SetChannel(spectrumChannel);
    waveformGeneratorHelper.SetPhyAttribute("Period", TimeValue(Seconds(0.0007)));
    waveformGeneratorHelper.SetPhyAttribute("DutyCycle", DoubleValue(1));
    waveformGeneratorHelper.SetTxPowerSpectralDensity(wgPsd);
    NetDeviceContainer waveformGeneratorDevices = waveformGeneratorHelper.Install(interferingNode.Get(0));

    // 调度仿真事件来启动和停止波形生成器
    Time currentTime = Simulator::Now();
    Time newTime = currentTime + Seconds(15.0);

    ScheduleWaveformGeneratorEvents(waveformGeneratorDevices, currentTime, newTime);
}

// 噪声调频干扰
void CreateNoiseFrequencyJamming() {
    //创建频带信息
    BandInfo band1,band2,band3;
    band1.fl=2412e6;band1.fh=2432e6;band1.fc=(band1.fl+band1.fh)/2;
    band2.fl=2432e6;band2.fh=2452e6;band2.fc=(band2.fl+band2.fh)/2;
    band3.fl=2452e6;band3.fh=2472e6;band3.fc=(band3.fl+band3.fh)/2;
    Bands bands;
    bands.push_back(band1);
    bands.push_back(band2);
    bands.push_back(band3);
    Ptr<SpectrumModel> spectrumModel = Create<SpectrumModel>(bands);

    interferingNode.Create(3);

    // 创建噪声调频干扰节点
    Ptr<SpectrumValue> wgPsd1 = Create<SpectrumValue>(spectrumModel);
    Ptr<UniformRandomVariable> randVar1 = CreateObject<UniformRandomVariable>();
    randVar1->SetAttribute("Min", DoubleValue(0.000001 / 20e6));
    randVar1->SetAttribute("Max", DoubleValue(0.00001 / 20e6)); // 调整最大值根据需要
    *wgPsd1 = randVar1->GetValue();

    Ptr<SpectrumValue> wgPsd2 = Create<SpectrumValue>(spectrumModel);
    Ptr<UniformRandomVariable> randVar2 = CreateObject<UniformRandomVariable>();
    randVar2->SetAttribute("Min", DoubleValue(0.000004 / 20e6));
    randVar2->SetAttribute("Max", DoubleValue(0.00004 / 20e6)); // 调整最大值根据需要
    *wgPsd2 = randVar2->GetValue();

    Ptr<SpectrumValue> wgPsd3 = Create<SpectrumValue>(spectrumModel);
    Ptr<UniformRandomVariable> randVar3 = CreateObject<UniformRandomVariable>();
    randVar3->SetAttribute("Min", DoubleValue(0.000009 / 20e6));
    randVar3->SetAttribute("Max", DoubleValue(0.00009 / 20e6)); // 调整最大值根据需要
    *wgPsd3 = randVar3->GetValue();

    WaveformGeneratorHelper waveformGeneratorHelper;
    // 设置通用属性
    waveformGeneratorHelper.SetChannel(spectrumChannel);
    waveformGeneratorHelper.SetPhyAttribute("Period", TimeValue(Seconds(0.0007)));
    waveformGeneratorHelper.SetPhyAttribute("DutyCycle", DoubleValue(1));

    waveformGeneratorHelper.SetTxPowerSpectralDensity(wgPsd1);
    NetDeviceContainer waveformGeneratorDevices1 = waveformGeneratorHelper.Install(interferingNode.Get(0));
    waveformGeneratorHelper.SetTxPowerSpectralDensity(wgPsd2);
    NetDeviceContainer waveformGeneratorDevices2 = waveformGeneratorHelper.Install(interferingNode.Get(1));
    waveformGeneratorHelper.SetTxPowerSpectralDensity(wgPsd3);
    NetDeviceContainer waveformGeneratorDevices3 = waveformGeneratorHelper.Install(interferingNode.Get(2));
    
    // 调度仿真事件来启动和停止波形生成器
    Time currentTime = Simulator::Now();
    Time newTime = currentTime + Seconds(15.0);
    ScheduleWaveformGeneratorEvents(waveformGeneratorDevices1, currentTime, newTime);
    ScheduleWaveformGeneratorEvents(waveformGeneratorDevices2, currentTime, newTime);
    ScheduleWaveformGeneratorEvents(waveformGeneratorDevices3, currentTime, newTime);
}
//干扰相关代码

string RemoveOf(const string& input) {
    string output;
    for (char c : input) {
        if (c != '"') {
            output += c; 
        } 
    }
    return output; 
}

void ModifyNetwork(json jsonData) {
    int intervaltime = 0;
    for (const auto& param : jsonData["Params"]) {
        auto antennasCount = to_string(param["AntennasCount"]);
        auto channel = RemoveOf(to_string(param["Channel"]));
        auto encode = RemoveOf(to_string(param["Encode"]));
        auto encodeRate = RemoveOf(to_string(param["EncodeRate"]));
        auto maxReceivingSpace = to_string(param["MaxReceivingSpace"]);
        auto maxTransmissionRate = RemoveOf(to_string(param["MaxTransmissionRate"]));
        auto maxTransmissionSpace = to_string(param["MaxTransmissionSpace"]);
        auto nodeID = RemoveOf(to_string(param["NodeID"]));
        auto power = to_string(param["Power"]);
        auto rxGain = to_string(param["ReceivingGain"]);
        auto txGain = to_string(param["TransmissionGain"]);
        vector<string> data;
        data.push_back(nodeID);
        data.push_back(antennasCount);
        data.push_back(rxGain);
        data.push_back(txGain);
        data.push_back(maxTransmissionSpace);
        data.push_back(maxReceivingSpace);
        data.push_back(channel);
        data.push_back(power);
        jishu = 0;
        double nowtime = Simulator::Now().GetSeconds();//记录调用时间
        if (isBusy(nowtime)) {
            cout << fmod(nowtime, 60.0) << "当前时间不支持修改，请稍后再试！" << endl;
        } else {
            // ConfigureWifiPhyAttributes(data);
            // ConfigureEncoding(nodeID, encode, encodeRate, maxTransmissionRate);
            Simulator::Schedule(MilliSeconds(1+10*intervaltime), &ConfigureWifiPhyAttributes, data, intervaltime, static_cast<int>(jsonData["Params"].size()));
            intervaltime++;
            Simulator::Schedule(MilliSeconds(1+10*intervaltime), &ConfigureEncoding, nodeID, encode, encodeRate, maxTransmissionRate);
        }
    }
}

void ModifyElectromagnetism(json jsonData) {
    for (const auto& param : jsonData["Params"]){
        string interfereType = RemoveOf(to_string(param["InterfereType"]));
        double nowtime = Simulator::Now().GetSeconds();
        if (isBusy(nowtime)) {
            cout << fmod(nowtime, 60.0) << "当前时间不支持修改，请稍后再试！" << endl;
        } else {
            if (interfereType=="CSNJ") {
                CreateCombSpectrumInterference();
                cout << "梳状谱噪声干扰添加成功" << endl;
            } else if (interfereType=="CW") {
                CreateSingleToneInterference();
                cout << "单音干扰添加成功" << endl;
            } else if (interfereType=="MTJ") {
                CreateMultiToneInterference();
                cout << "多音干扰添加成功" << endl;
            } else if (interfereType=="PBNJ") {
                CreatePartialBandNoiseJamming();
                cout << "部分频带噪声干扰" << endl;
            } else if (interfereType=="NFM") {
                CreateNoiseFrequencyJamming();
                cout << "噪声调频类干扰添加成功" << endl;
            }
        }
    }
}

void ReceiveOutPacket (Ptr<Socket> socket){
    try{
        Ptr<Packet> packet = socket->Recv (1472,0);
        uint8_t *buffer = new uint8_t[packet->GetSize ()-1];
        packet->CopyData(buffer, packet->GetSize ());
        // string receiveData = string((char*)buffer);
        string receiveData(reinterpret_cast<char*>(buffer), packet->GetSize());
        delete[] buffer;
        json jsonData = json::parse(receiveData);
        auto arrangeType = jsonData["ArrangeType"];
        if (arrangeType == "network") {
            ModifyNetwork(jsonData);
        } else if (arrangeType == "electromagnetism"){
            ModifyElectromagnetism(jsonData);
        }
    }
    catch(const nlohmann::json::parse_error& e)
    {
        cerr << "JSON解析错误: " << e.what() << " 位置：" << e.byte << endl;
    }
}

//更新节点活跃度
void updataNodeActivity(string nodeId, int activity){
    for(auto nodevector : nodeVector){
        if(nodevector.nodeID == nodeId){
            nodevector.activity = activity;
            idToActivityMap[nodeId] = activity;
        }
    }
}

//计算当前时间的平均信噪比
void averageSnr(){
    double current_time = Simulator::Now().GetSeconds();
    double average = totalSnr/totalPackets;
    if(totalPackets > 0)
        cout << current_time-15.0 << "s到" << current_time << "s内的整个网络收到" << totalPackets << "个包，平均信噪比(snr): " << average << "dB" << endl;
    else
        cout << current_time-15.0 << "s到" << current_time << "s内的整个网络的平均信噪比(snr): " << "0dB" << endl;
    totalSnr = 0;
    totalPackets = 0;
    Simulator::Schedule(Seconds(15.0), &averageSnr);
}

int main (int argc, char *argv[])
{
    CommandLine cmd;
    cmd.AddValue("soldiernum", "the number of soldiers", soldierNum);
    cmd.AddValue("tankernum", "the number of tankers", tankerNum);
    cmd.AddValue("constructionvehiclenum","the number of constructionVehicle",constructionVehicleNum);
    cmd.AddValue("transportvehiclenum","the number of transportVehicle",transportVehicleNum);
    cmd.AddValue("ambulancenum","the number of ambulance",ambulanceNum);
    cmd.AddValue("commandtentnum","the number of commandTent",commandTentNum);
    cmd.AddValue("radarvehiclenum","the number of radarVehicle",radarVehicleNum);
    cmd.AddValue("radarnum","the number of radarNum",radarNum);

    cmd.AddValue("soldierspeed","the speed of soldiers",soldierSpeed);
    cmd.AddValue("tankerspeed","the speed of tankers",tankerSpeed);
    cmd.AddValue("constructionvehiclespeed","the speed of constructionVehicle",constructionVehicleSpeed);
    cmd.AddValue("transportvehiclespeed","the speed of transportVehicle",transportVehicleSpeed);
    cmd.AddValue("ambulancespeed","the speed of ambulance",ambulanceSpeed);
    cmd.AddValue("commandtentspeed","the speed of commandTent",commandTentSpeed);
    cmd.AddValue("radarvehiclespeed","the speed of radarVehicle",radarVehicleSpeed);
    cmd.AddValue("radarspeed","the speed of radar",radarSpeed);

    cmd.AddValue("sendtimeinterval","the sendtimeinterval of dataPacket",sendtimeinterval);
    cmd.Parse(argc, argv);


    //读取节点位置信息
    vector<Data> data = readData("/home/ns3/project/red_position.txt", soldierNum, tankerNum, constructionVehicleNum, transportVehicleNum,
                                     ambulanceNum, commandTentNum, radarVehicleNum, radarNum);

    //输出内容保存的json文件
    outputFile.open("node-movement-log.json");
    dataputFile.open("data-transmit-log.json");
    dataActiviaty.open("data-activiaty-log.json");
    dataAi.open("activity-data-log.json");
    fstream rewriteFile("node-movement-log.json", ios::in | ios::out | ios::ate);
    fstream rewriteDataFile("data-transmit-log.json", ios::in | ios::out | ios::ate);
    fstream rewriteDataActiviatyFile("data-activiaty-log.json", ios::in | ios::out | ios::ate);
    fstream rewriteDataActivityFile("activity-data-log.json", ios::in | ios::out | ios::ate);

    //读取位置，天气信息
    vector<DataRow> fileData = ReadCsvFile("/home/ns3/project/red_info.txt");

    // 设置全局变量以使用实时模拟器和启用校验和
    GlobalValue::Bind ("SimulatorImplementationType", StringValue ("ns3::RealtimeSimulatorImpl"));
    GlobalValue::Bind ("ChecksumEnabled", BooleanValue (true));



    uint16_t totalNodes = soldierNum+tankerNum+constructionVehicleNum+transportVehicleNum+
                            ambulanceNum+commandTentNum+radarVehicleNum+radarNum;
    //创建节点数量
    nodes = CreateNode(totalNodes);

    //设置wifi的标准
    WifiStandard standard = WIFI_STANDARD_80211n; 
    WifiHelper wifi = CreateWifiHelper(standard);


    //创建物理层助手
    SpectrumWifiPhyHelper spectrumWifiPhy = SpectrumWifiPhyHelper();
    //设置移动模型
    MobilityHelper mobility;

    //给节点添加对应的地理位置等属性
    Ptr<ListPositionAllocator> nodesPositionAlloc = ReadToPositionAlloc(data, nodes);

    mobility.SetPositionAllocator(nodesPositionAlloc);
    string terrain,Weather;
    int weather;
    //记录天气
    //给节点添加对应的地形以及天气属性
    for (const auto& row : fileData) {
        if(FindFromMap(row.id)){
            Ptr<Node> node = FindFromMap(row.id);
            vector<string> data = {row.terrain,row.weather};
            AddVectorData(node, data);
            terrain = row.terrain;
            Weather = row.weather;
        }
    }
    weather = weatherType(Weather);//获取天气类型 1.晴朗  2.雨  3.雪 4.雾


    NodesAddMovement(mobility);
    double factor=0;
    WifiMacHelper nodeswifiMac = CreateWifiMacHelper("ns3::AdhocWifiMac");
        // 设置远程站点管理器
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                          "DataMode", StringValue("HtMcs4"),
                                          "ControlMode", StringValue("HtMcs0"));
    const int NUM_NODES = totalNodes;
    NetDeviceContainer nodeDevices[NUM_NODES];

    if(terrain == "plain"){
        spectrumChannel = CreateObject<MultiModelSpectrumChannel>();
        spectrumChannel->AddPropagationLossModel(FriisPlainlossModel);
        Ptr<ConstantSpeedPropagationDelayModel> delayModel = CreateObject<ConstantSpeedPropagationDelayModel>();
        spectrumChannel->SetPropagationDelayModel(delayModel);
        spectrumWifiPhy.SetChannel(spectrumChannel);
        switch(weather){
        case 1:factor=5;break; 
        case 2:factor=6;break; 
        case 3:factor=7;break; 
        case 4:factor=8;break; 
        }
        for(uint8_t i =0; i < nodes.GetN(); i++){
            ConfigureNode(i, factor, spectrumWifiPhy, nodeswifiMac, nodes,FriisMountainlossModel,NakagamiMountainlossModel,wifi,terrain);
            nodeDevices[i] = wifi.Install(spectrumWifiPhy, nodeswifiMac, nodes.Get(i));
        }
    }
    if(terrain == "mountain"){
        frequencyGHz = 100e6*8;
        spectrumChannel = CreateObject<MultiModelSpectrumChannel>();
        spectrumChannel->AddPropagationLossModel(FriisMountainlossModel);
        spectrumChannel->AddPropagationLossModel(NakagamiMountainlossModel);
        Ptr<ConstantSpeedPropagationDelayModel> delayModel = CreateObject<ConstantSpeedPropagationDelayModel>();
        spectrumChannel->SetPropagationDelayModel(delayModel);
        spectrumWifiPhy.SetChannel(spectrumChannel);
        switch(weather){
        case 1:factor=0;break; 
        case 2:factor=0.1;break; 
        case 3:factor=0.12;break; 
        case 4:factor=0.14;break; 
        }
        for(uint8_t i =0; i < nodes.GetN(); i++){
            ConfigureNode(i, factor, spectrumWifiPhy, nodeswifiMac, nodes,FriisMountainlossModel,NakagamiMountainlossModel,wifi,terrain);
            nodeDevices[i] = wifi.Install(spectrumWifiPhy, nodeswifiMac, nodes.Get(i));
        }       
    }
    if(terrain == "city"){
        spectrumChannel = CreateObject<MultiModelSpectrumChannel>();
        spectrumChannel->AddPropagationLossModel(NakagamiCitylossModel);
        spectrumChannel->AddPropagationLossModel(LogDistanceCitylossModel);
        Ptr<ConstantSpeedPropagationDelayModel> delayModel = CreateObject<ConstantSpeedPropagationDelayModel>();
        spectrumChannel->SetPropagationDelayModel(delayModel);
        spectrumWifiPhy.SetChannel(spectrumChannel);
        switch(weather){
        case 1:factor=0;break; 
        case 2:factor=3;break; 
        case 3:factor=6;break; 
        case 4:factor=7;break; 
        }
        for(uint8_t i =0; i < nodes.GetN(); i++){
            ConfigureNode(i, factor, spectrumWifiPhy, nodeswifiMac, nodes,FriisMountainlossModel,NakagamiMountainlossModel,wifi,terrain);
            nodeDevices[i] = wifi.Install(spectrumWifiPhy, nodeswifiMac, nodes.Get(i));
        }           
    }
    if(terrain == "forest"){
        spectrumChannel = CreateObject<MultiModelSpectrumChannel>();
        spectrumChannel->AddPropagationLossModel(NakagamiForestlossModel);
        spectrumChannel->AddPropagationLossModel(LogDistanceForestlossModel);
        Ptr<ConstantSpeedPropagationDelayModel> delayModel = CreateObject<ConstantSpeedPropagationDelayModel>();
        spectrumChannel->SetPropagationDelayModel(delayModel);
        spectrumWifiPhy.SetChannel(spectrumChannel);
        switch(weather){
        case 1:factor=0;break; 
        case 2:factor=2;break; 
        case 3:factor=5;break; 
        case 4:factor=6;break; 
        }
        for(uint8_t i =0; i < nodes.GetN(); i++){
            ConfigureNode(i, factor, spectrumWifiPhy, nodeswifiMac, nodes,FriisMountainlossModel,NakagamiMountainlossModel,wifi, terrain);
            nodeDevices[i] = wifi.Install(spectrumWifiPhy, nodeswifiMac, nodes.Get(i));      
        }   
    }

    InternetStackHelper stack;
    AodvHelper aodv;
    stack.SetRoutingHelper(aodv);
    stack.Install(nodes);

    Ipv4AddressHelper address;
    address.SetBase("10.1.8.0","255.255.255.0");
    const int NUM_INTERFACES = NUM_NODES;
    Ipv4InterfaceContainer nodesInterfaces[NUM_INTERFACES];
    for(int i=0;i<NUM_INTERFACES;i++)
    {
        nodesInterfaces[i] = address.Assign(nodeDevices[i]);
    }

    //初始化吞吐量对应表
    for (size_t i = 0; i < nodes.GetN(); i++){
        Ptr<Node> node = nodes.Get(i);
        string modelId=FindIdFromMap(node);
        throughoutputByModelId[modelId]=0;
    }

    // 设置Packet Sink应用并连接回调
    uint16_t port = 9; // UDP端口
    PacketSinkHelper packetSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), port));
    ApplicationContainer sinkApps = packetSinkHelper.Install (nodes);
    sinkApps.Start (Seconds (0.0));

    // 添加13个信道
    for (int i = 0; i < 13; ++i) {
        Bands band;
        BandInfo bandInfo;
        // 计算每个信道的中心频率
        bandInfo.fc = startFrequency + i * channelSpacing;
        // 计算频率下限
        bandInfo.fl = bandInfo.fc - channelWidth / 2.0;
        // 计算频率上限
        bandInfo.fh = bandInfo.fc + channelWidth / 2.0;
        // 将频带信息添加到 band
        band.push_back(bandInfo);
        // 将每个 Band 添加到 allBands 中
        allBands.push_back(band);
    }

    //连接回调
    for (uint32_t i = 0; i < sinkApps.GetN (); ++i) {
        Ptr<PacketSink> sink = DynamicCast<PacketSink> (sinkApps.Get (i));
        std::ostringstream oss;
        oss << "/NodeList/" << sink->GetNode ()->GetId () << "/ApplicationList/" << i << "/$ns3::PacketSink/Rx";
        sink->TraceConnect ("Rx", oss.str (), MakeCallback (&ReceivePacket));
    }

    // 安排特定时间点的事件
    uint32_t sourceIndex = 0;
    uint32_t targetIndex = 1; 
    FindIdFromIpMap(nodes);
    startTransmissionTime = Seconds(0.0);
    transmissionDuration = Seconds(3.0);
    Simulator::Schedule(startTransmissionTime, &StartSpecificTransmission, sourceIndex, targetIndex, nodes, nodes, port, transmissionDuration);            
    for (uint32_t i = 0; i < nodes.GetN(); i++)
    {
        for (uint32_t j = 0; j < nodes.GetN(); j++)
        {
            sourceIndex = i;
            targetIndex = j;
            if(i != j && !(i == 0 && j==1)){
                Simulator::Schedule(startTransmissionTime, &StartSpecificTransmission, i, j, nodes, nodes, port, transmissionDuration);            
            }
        }
    }

    //绑定MonitorSnifferRx生成snr等数据
    for (uint32_t i = 0; i < nodes.GetN(); ++i) { 
        Ptr<NetDevice> device = nodes.Get(i)->GetDevice(0);
        Ptr<WifiNetDevice> wifiDevice = DynamicCast<WifiNetDevice>(device);
        Ptr<SpectrumWifiPhy> phy = DynamicCast<SpectrumWifiPhy>(wifiDevice->GetPhy());
        tempNode = nodes.Get(i);
        phy->TraceConnectWithoutContext("MonitorSnifferRx", MakeBoundCallback(&MonitorSnifferRx, tempNode));
    } 
    
    flowdata.monitor = flowdata.flowmon.InstallAll();
    flowdata.totalDelaySum = 0;
    flowdata.totalRxBytes = 0;
    flowdata.totalRxPackets = 0;

    Simulator::Schedule(Seconds(2.0 - 1.0), &Throughput);

    // 初始化节点
    for (int i = 0; i < nodes.GetN(); ++i) {
        double throughput = 0.0;     // 设置初始值
        double signalToNoise = 0.0;  // 设置初始值
        int packetSize = 0;          // 设置初始值
        double delay = 0.0;          // 设置初始值
        int activity = 0.0;       // 设置初始值
        string typeName=nameToIdMap[Names::FindName(nodes.Get(i))];
        auto now = chrono::system_clock::now();
        long long lastUpdateTime = chrono::duration_cast<chrono::milliseconds>(now.time_since_epoch()).count();
        // 创建一个新的节点并添加到vector中
        nodeVector.emplace_back(typeName, throughput, signalToNoise, packetSize, delay, activity, lastUpdateTime);
    }

    // 安排第一个输出
    // Simulator::Schedule(Seconds(1.0), &PrintNodeVector);

/*
    // 示例用法：遍历所有节点并输出信息
    for (const auto& node : nodeVector) {
        std::cout << "Node ID: " << node.nodeID << std::endl;
        std::cout << "Throughput: " << node.throughput << " Mbps" << std::endl;
        std::cout << "Signal-to-Noise Ratio: " << node.signalToNoise << " dB" << std::endl;
        std::cout << "Packet Size: " << node.packetSize << " bytes" << std::endl;
        std::cout << "Last Update Time (timestamp): " << chrono::duration_cast<std::chrono::seconds>(
                                                            node.lastUpdateTime.time_since_epoch()).count()
                  << " seconds since epoch" << std::endl;
        cout << "delay：" << node.delay << "ms" << endl;
        // 其他信息的输出...

        std::cout << "------------------------" << std::endl;
    }
*/


    Simulator::Schedule(Seconds(0.0), &startDataActiviatyInfo);
    Simulator::Schedule(Seconds(25.0), &stopDataActiviatyInfo, ref(rewriteDataActivityFile), ref(dataActiviaty), ref(dataAi));

    Simulator::Schedule(Seconds(0.0), &dataActiviatyInfoFile, nodes, ref(dataActiviaty));
    Simulator::Schedule(Seconds(0.0), &LogJsonPosition, nodes, ref(outputFile));

    Simulator::Schedule(Seconds(30), &ClearFile, ref(outputFile), "node-movement-log.json");//设置刷新文件的时间
    // Simulator::Schedule(Seconds(30), &ClearFile, ref(dataActiviaty), "data-activiaty-log.json");//设置刷新文件的时间
    Simulator::Schedule(Seconds(120), &ClearFile, ref(dataputFile), "data-transmit-log.json");//设置刷新文件的时间


    Simulator::Schedule(Seconds(15.0), &printDataRecords);
    Simulator::Schedule(Seconds(15.0), &CalculateAndPrintThroughput);
    Simulator::Schedule(Seconds(15.0), &averageSnr);
/*
    //关闭json文件
    Simulator::Schedule(Seconds(60.0*5), &Closefile,ref(outputFile));
    Simulator::Schedule(Seconds(60.0*5), &Closefile,ref(dataputFile));
    Simulator::Schedule(Seconds(60.0*5), &Closefile,ref(dataActiviaty));
*/
    Simulator::Schedule(Seconds(sendtimeinterval), &StartTransmit, nodes, port);


    if (!rewriteFile.is_open()) {
        std::cerr << "无法打开文件1。" << std::endl;
        return -1;
    }

    if (!rewriteDataFile.is_open()) {
        std::cerr << "无法打开文件2。" << std::endl;
        return -1;
    }

    if (!rewriteDataActiviatyFile.is_open()) {
        std::cerr << "无法打开文件3。" << std::endl;
        return -1;        
    }

    if (!rewriteDataActivityFile.is_open()) {
        std::cerr << "无法打开文件4。" << std::endl;
        return -1;        
    }



    NodeContainer nodesLeft;
    nodesLeft.Create (2);    
    CsmaHelper csmaSN0;
    NetDeviceContainer devicesLeft = csmaSN0.Install (nodesLeft);
    InternetStackHelper internetLeft;
    internetLeft.Install (nodesLeft);
    Ipv4AddressHelper ipv4Left;
    ipv4Left.SetBase ("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer interfacesLeft = ipv4Left.Assign (devicesLeft);
    TapBridgeHelper tapBridge (interfacesLeft.GetAddress (0));
    tapBridge.SetAttribute ("Mode", StringValue ("UseBridge"));
    tapBridge.SetAttribute ("DeviceName", StringValue ("tap-test1"));
    tapBridge.Install (nodesLeft.Get (0), devicesLeft.Get (0));
    NodeContainer nodesRight;
    nodesRight.Create (2);
    CsmaHelper csmaRight;
    NetDeviceContainer devicesRight = csmaRight.Install (nodesRight);
    InternetStackHelper internetRight;
    internetRight.Install (nodesRight);
    Ipv4AddressHelper ipv4Right;
    ipv4Right.SetBase ("10.1.3.0", "255.255.255.0");
    Ipv4InterfaceContainer interfacesRight = ipv4Right.Assign (devicesRight);
    PointToPointHelper p2p;
    NodeContainer connectNodes = NodeContainer (nodesLeft.Get (1), nodesRight.Get (0));
    NetDeviceContainer devices = p2p.Install (connectNodes);
    Ipv4AddressHelper ipv4;
    ipv4.SetBase ("10.1.2.0", "255.255.255.192");
    Ipv4InterfaceContainer interfaces = ipv4.Assign (devices);    
    Ipv4GlobalRoutingHelper::PopulateRoutingTables ();
    TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
    srcSocket = Socket::CreateSocket(nodesRight.Get(0), tid);
    InetSocketAddress remote = InetSocketAddress (Ipv4Address ("10.1.1.5"), 12345);
    srcSocket->Connect(remote);

    // 从外部接收参数
    Ptr<Socket> recvSink = Socket::CreateSocket (nodesRight.Get (1), tid);
    InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (),2399);
    recvSink->Bind (local);
    recvSink->SetRecvCallback (MakeCallback (&ReceiveOutPacket));

    Simulator::Schedule(Seconds(0.001),&SendData,srcSocket);

    Simulator::Run();
    Simulator::Destroy();

    rewriteFile.close();
    rewriteDataFile.close();
    rewriteDataActiviatyFile.close();

    return 0;
}