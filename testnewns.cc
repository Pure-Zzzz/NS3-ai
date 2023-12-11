 // 引入必要的头文件
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/yans-wifi-phy.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/udp-client-server-helper.h"
#include "ns3/udp-echo-client.h"
#include "ns3/udp-echo-helper.h"
#include "ns3/udp-echo-server.h"
#include "ns3/interference-helper.h"
#include "ns3/netanim-module.h"
#include "ns3/csma-module.h"
#include "ns3/tap-bridge-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/names.h"
#include "ns3/node.h"
#include "ns3/ptr.h"
#include <iostream>
#include <map>
#include <string>
#include <iomanip>
#include <ns3/ai-module.h>
#include <ns3/core-module.h>
#include <random>
using namespace std;
using namespace ns3;
// 设置日志组件，方便调试
#define NUM_ENV 3

NS_LOG_COMPONENT_DEFINE ("PositionTacticalInternetExample");
ofstream file("result.txt");

queue<Ptr<Packet>> blueQue;
queue<Ptr<Packet>> redQue;

double NodePower(Ptr<Node> node);

struct Data {
    std::string col1;
    std::string col2;
    std::string col10;
    std::string col11;
    std::string col12;
};

static map<string, string> idToNameMap;
static map<string, string> idToGroupMap;
static map<string, string> nameToIdMap;

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
void AddToMap(Ptr<Node> Node, string nodeName, string nodeId){
    Names::Add(nodeName, Node);//将Node的名字作为Node的一个别名
    idToNameMap[nodeId] = nodeName; //将Node的名字和外部的nodeId作为一对键值对放入idToNameMap(通过nodeId查询nodeName)
    nameToIdMap[nodeName] = nodeId;//将Node的名字和外部的nodeId作为一对键值对放入nameToIdMap(通过nodeName查询nodeId)
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

        // cout<<"Found node group: "<<foundNodeGroup<<endl;
        return foundNodeGroup;
    }else{
        cout<<"Group with ID"<<searchId<<" not found."<<endl;
    }
    return 0;
}

//根据节点查询ID
string FindIdFromMap(Ptr<Node> node){
    if (Names::FindName(node) != "") {
        std::string foundName = Names::FindName(node);
        if(nameToIdMap.find(foundName)!=nameToIdMap.end()){
            //通过NodeName来获取Id
            string foundNodeId = nameToIdMap[foundName];

            // cout<<"Found node Id: "<<foundNodeId<<endl;
            return foundNodeId;
        }else{
            cout<<"No name is associated with the node." << std::endl;
        }
    } else {
        std::cout << "No name is associated with the node. " << std::endl;
    }
    return 0;
}

Ptr<Node> FindFromMap(string searchId){
    if(idToNameMap.find(searchId)!=idToNameMap.end()){
        //通过ID获取节点名称
        string foundNodeName = idToNameMap[searchId];

        //使用节点名称从Names空间中找到对应的节点
        Ptr<Node> foundNode = Names::Find<Node>(foundNodeName);

        // cout<<"Found node name: "<<foundNodeName<<endl;
        return foundNode;
    }else{
        cout<<"Node with ID"<<searchId<<" not found."<<endl;
    }
    return 0;
}

std::vector<Data> readData(const std::string &filePath) {
    std::vector<Data> dataArray;
    std::ifstream file(filePath);
    std::string line;
    while (getline(file, line)) {
        std::istringstream iss(line);
        std::vector<std::string> columns;
        std::string item;
        while (getline(iss, item, '\t')) {
            columns.push_back(item);
        }
        if (columns.size() >= 12) {
            Data data = {columns[0], columns[1], columns[9], columns[10], columns[11]};
            dataArray.push_back(data);
        }
    }
    return dataArray;
}

//添加节点位置信息，i为节点的编号，从0开始，positionAlloc为位置容器的名字，Nodes为节点的容器名字，Nodename为节点容器名称（例如red_soldiernodes)
void AddPosition(int &i, Data d, Ptr<ListPositionAllocator> positionAlloc,NodeContainer Nodes,string Nodename){
    double x = std::stod(d.col10);
    double y = std::stod(d.col11);
    double z = std::stod(d.col12);
    positionAlloc->Add(Vector(x/100, y/100, z/100));
    AddToMap(Nodes.Get(i),Nodename + ".Get(" + to_string(i) +")", d.col2);
    i++;
}

//将内部数据输出到txt文件中
void LogPosition(NodeContainer Nodes, std::ofstream& outputFile)
{
    for(uint16_t i = 0;i < Nodes.GetN();++i)
    {
        Ptr<MobilityModel> mobility = Nodes.Get(i)->GetObject<MobilityModel>();
        Vector pos = mobility->GetPosition();
        std::string foundName = Names::FindName(Nodes.Get(i));
        vector<string> retrievedVector = SearchVectorData(Nodes.Get(i));
        //时间、节点ID和位置写入输出文件
        outputFile << "Time: " << Simulator::Now().GetSeconds() << "s\t";
        // outputFile << "Node ID: " << Nodes.Get(i)->GetId() << ", ";
        outputFile << "Node ID: " << FindIdFromMap(Nodes.Get(i))<< "\t";
        outputFile << "NodeName: " << foundName<< "\t";
        if((foundName != "red_commandPostNodes.Get(0)")&&(foundName != "blue_commandPostNodes.Get(0)"))
        {
            // cout<<foundName<<endl;
            outputFile << "NodeTxPower: " << NodePower(Nodes.Get(i)) << "\t"; 
        }
        outputFile << "Position: " << pos.x << " " << pos.y << " " << pos.z << "\t";
        outputFile << "Terrain: " << retrievedVector[0] << " Weather: " << retrievedVector[1] << endl;
    }
  // 每秒记录
  Simulator::Schedule(Seconds(1.0), &LogPosition, Nodes, std::ref(outputFile));
}

//根据节点找传输功率
double NodePower(Ptr<Node> node){
    Ptr<NetDevice> dev = node->GetDevice(0); 
    Ptr<WifiNetDevice> wifiDev = dev->GetObject<WifiNetDevice>();
    Ptr<YansWifiPhy> phy = DynamicCast<YansWifiPhy>(wifiDev->GetPhy());
    double txPower = phy->GetTxPowerStart();
    return txPower;
}

void ClearFile(std::ofstream& outputFile) {
    outputFile.open("node-movement-log.txt", std::ofstream::out | std::ofstream::trunc); // 以截断方式打开文件
    if (outputFile.is_open()) {
        outputFile.close(); // 关闭文件，这将清空文件内容
    }
    outputFile.open("node-movement-log.txt");
    // 重新安排下一次清空文件的时间
    Simulator::Schedule(Seconds(30), &ClearFile, std::ref(outputFile));
}

//设置传输功率


//将字符串转换成数据包
Ptr<Packet> RetPacket(std::string receiveData)
{
    Ptr<Packet> retPacket = Create<Packet> ((uint8_t*)receiveData.c_str(), receiveData.length());
    std::cout<<"retPacket="<<retPacket<<endl;
    return retPacket;
}

//解码红数据包
string DeRedPacket()
{
    if (redQue.empty() != 0)
    {
        Ptr<Packet> retPacket=redQue.front();
        redQue.pop();
        uint32_t retPacketSize = retPacket->GetSize();
        uint8_t *retBuffer = new uint8_t[retPacketSize];
        retPacket->CopyData(retBuffer, retPacketSize);
        string extractedData((char*)retBuffer, retPacketSize);
        delete[] retBuffer;
        cout<<"提取的字符串数据:"<< extractedData<<endl;
        cout << "redQue的数量:" << redQue.size() << endl;
        return extractedData;
    }
}

//解码蓝数据包
string DeBluePacket()
{
    if (redQue.empty() != 0)
    {
        Ptr<Packet> retPacket=blueQue.front();
        blueQue.pop();
        uint32_t retPacketSize = retPacket->GetSize();
        uint8_t *retBuffer = new uint8_t[retPacketSize];
        retPacket->CopyData(retBuffer, retPacketSize);
        string extractedData((char*)retBuffer, retPacketSize);
        delete[] retBuffer;
        cout<<"提取的字符串数据:"<< extractedData<<endl;
        cout << "redQue的数量:" << blueQue.size() << endl;
        return extractedData;
    }
}

DataRow parseLine(const std::string &line) {
    std::istringstream iss(line);
    std::vector<DataRow> result;
    std::string token;

    DataRow data;
    if (std::getline(iss, data.id, ',') &&
        std::getline(iss, data.terrain, ',') &&
        std::getline(iss, data.weather)) {
        result.push_back(data);
    }

    return data;
}


//收取红色数据包
void ReceiveRedPacket(Ptr<Socket> socket)
{
    Ptr<Packet> packet = socket->Recv(1472, 0);
    uint8_t *buffer = new uint8_t[packet->GetSize()-1];
    packet -> CopyData(buffer, packet->GetSize());
    string receivedData = std::string((char*)buffer);
    cout<<"接受的数据:"<<receivedData<<endl;
    delete[] buffer;
    DataRow dataRow = parseLine(receivedData);

    Ptr<Node> node = FindFromMap(dataRow.id);
    cout<<node<<endl;
    vector<string> data = {dataRow.terrain,dataRow.weather};
    AddVectorData(node, data);
    // cout << "ID: " << dataRow.id << ", Terrain: " << dataRow.terrain << ", Weather: " << dataRow.weather << endl;
    // vector<string> retrievedVector = SearchVectorData(node);
    // if(!retrievedVector.empty()){
    //     cout<<"成功给节点添加数据" <<endl;
    //     for (string value : retrievedVector) {
    //         cout << value << " ";
    //     }
    //     cout << endl;
    // }
}

//收取蓝色数据包
void ReceiveBluePacket(Ptr<Socket> socket)
{
    Ptr<Packet> packet = socket->Recv(1472, 0);
    uint8_t *buffer = new uint8_t[packet->GetSize()-1];
    packet -> CopyData(buffer, packet->GetSize());
    std::string receivedData = std::string((char*)buffer);
    cout<<"接受的数据:"<<receivedData<<endl;
    delete[] buffer;
    Ptr<Packet> retPacket = RetPacket(receivedData);
    blueQue.push(retPacket);
    cout<<"蓝色"<<DeBluePacket()<<endl;
}

void PrintTxPower(const NetDeviceContainer& devices,const string& devicesname) {
    cout<<devicesname<<endl;
    for (uint32_t i = 0; i < devices.GetN(); ++i) {
        Ptr<NetDevice> device = devices.Get(i);
        Ptr<WifiNetDevice> wifiDevice = DynamicCast<WifiNetDevice>(device);
        Ptr<YansWifiPhy> phy = DynamicCast<YansWifiPhy>(wifiDevice->GetPhy());
        double txPowerStart = phy->GetTxPowerStart();
        double txPowerEnd = phy->GetTxPowerEnd();

        std::cout << "\tNode " << i << " Tx Power: " << txPowerStart << " to " << txPowerEnd << " dBm" << std::endl;
    }
}
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

int main(int argc, char *argv[]) 
{  
    CommandLine cmd;
    cmd.Parse(argc, argv);
    std::cout << "CPP START" << std::endl ;
    std::string filePath1 = "./red_position.txt";  // 红色方位置文件
    std::string filePath2 = "./blue_position.txt";  // 蓝色方位置文件
    std::vector<Data> red_data = readData(filePath1);
    std::vector<Data> blue_data = readData(filePath2);

    //ns3-ai START
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
    // apb->NotifySimulationEnd();
    //ns3-ai END


    //创建并打开输出文件
    ofstream outputFile;
    outputFile.open("node-movement-log.txt");
    vector<DataRow> fileData = ReadCsvFile("./scratch/info.txt");

    // 启用日志组件
    LogComponentEnable ("PositionTacticalInternetExample", LOG_LEVEL_INFO);

    // 设置全局变量以使用实时模拟器和启用校验和
    GlobalValue::Bind ("SimulatorImplementationType", StringValue ("ns3::RealtimeSimulatorImpl"));
    GlobalValue::Bind ("ChecksumEnabled", BooleanValue (true));

    // 创建节点：士兵、无人机、装甲车、指挥部和通信车
    NodeContainer red_soldierNodes, red_droneNodes, red_armoredVehicleNodes, red_commandPostNodes, red_communicationVehicleNodes,red_engineeringVehicleNodes,red_ambulanceNodes,red_radioNodes;
    NodeContainer blue_soldierNodes,blue_droneNodes,blue_armoredVehicleNodes,blue_commandPostNodes,blue_communicationVehicleNodes,blue_ambulanceNodes,blue_radioNodes;
    //红色方节点初始化
    red_soldierNodes.Create(4);       // 创建4个士兵节点
    red_droneNodes.Create(2);          // 创建2个运输车节点
    red_armoredVehicleNodes.Create(6); // 创建6个坦克车节点
    red_commandPostNodes.Create(1);    // 创建1个指挥部节点
    red_engineeringVehicleNodes.Create(1);  ///创建1个工程车节点
    red_ambulanceNodes.Create(2);   //创建两个医疗车节点
    red_radioNodes.Create(9);   //创建9个电台节点
    red_communicationVehicleNodes.Create(3); // 创建3个雷达车节点

    //蓝色方节点初始化
    blue_soldierNodes.Create(3);    //创建蓝色方3个士兵节点
    blue_droneNodes.Create(2);  //创建蓝色方2个运输车节点
    blue_armoredVehicleNodes.Create(4); //创建蓝色方4个装甲车节点
    blue_ambulanceNodes.Create(1);  //创建蓝色方1个医疗车节点
    blue_commandPostNodes.Create(1);    //创建蓝色方1个指挥部节点
    blue_radioNodes.Create(5);  //创建蓝色方5个电台节点
    blue_communicationVehicleNodes.Create(3);   //创建蓝色方3个雷达车节点


// //从外部传参
//     //蓝色指挥部
//     NodeContainer nodesLeftBlue;
//     nodesLeftBlue.Create (2);
//     CsmaHelper csmaSN1;
//     NetDeviceContainer devicesLeftBlue = csmaSN1.Install (nodesLeftBlue);
//     InternetStackHelper internetLeftBlue;
//     internetLeftBlue.Install (nodesLeftBlue);
//     Ipv4AddressHelper ipv4LeftBlue;
//     ipv4LeftBlue.SetBase ("10.1.4.0", "255.255.255.0");
//     Ipv4InterfaceContainer interfacesLeftBlue = ipv4LeftBlue.Assign (devicesLeftBlue);
//     TapBridgeHelper tapBridgeBlue (interfacesLeftBlue.GetAddress (0));
//     tapBridgeBlue.SetAttribute ("Mode", StringValue ("UseBridge"));
//     tapBridgeBlue.SetAttribute ("DeviceName", StringValue ("tap0-test1"));
//     tapBridgeBlue.Install (nodesLeftBlue.Get (0), devicesLeftBlue.Get (0));
//     //红色指挥部
//     NodeContainer nodesLeftRed;
//     nodesLeftRed.Create (2);
//     CsmaHelper csmaSN0;
//     NetDeviceContainer devicesLeftRed = csmaSN0.Install (nodesLeftRed);
//     InternetStackHelper internetLeftRed;
//     internetLeftRed.Install (nodesLeftRed);
//     Ipv4AddressHelper ipv4LeftRed;
//     ipv4LeftRed.SetBase ("10.1.1.0", "255.255.255.0");
//     Ipv4InterfaceContainer interfacesLeftRed = ipv4LeftRed.Assign (devicesLeftRed);
//     TapBridgeHelper tapBridgeRed (interfacesLeftRed.GetAddress (0));
//     tapBridgeRed.SetAttribute ("Mode", StringValue ("UseBridge"));
//     tapBridgeRed.SetAttribute ("DeviceName", StringValue ("tap-test1"));
//     tapBridgeRed.Install (nodesLeftRed.Get (0), devicesLeftRed.Get (0));        
//     //蓝色指挥部
//     CsmaHelper csmaRightBlue;
//     NetDeviceContainer devicesRightBlue = csmaRightBlue.Install (blue_commandPostNodes);
//     InternetStackHelper internetRightBlue;
//     internetRightBlue.Install (blue_commandPostNodes);
//     Ipv4AddressHelper ipv4RightBlue;
//     ipv4RightBlue.SetBase ("10.1.7.0", "255.255.255.0");
//     Ipv4InterfaceContainer interfacesRightBlue = ipv4RightBlue.Assign (devicesRightBlue);
//     PointToPointHelper p2pBlue;
//     NodeContainer nodesBlue = NodeContainer (nodesLeftBlue.Get (1), blue_commandPostNodes.Get (0));
//     NetDeviceContainer devicesBlue = p2pBlue.Install (nodesBlue);
//     Ipv4AddressHelper ipv4Blue;
//     ipv4Blue.SetBase ("10.1.5.0", "255.255.255.192");
//     Ipv4InterfaceContainer interfaces = ipv4Blue.Assign (devicesBlue);
//     //红色指挥部
//     CsmaHelper csmaRightRed;
//     NetDeviceContainer devicesRightRed = csmaRightRed.Install (red_commandPostNodes);
//     InternetStackHelper internetRight;
//     internetRight.Install (red_commandPostNodes);
//     Ipv4AddressHelper ipv4RightRed;
//     ipv4RightRed.SetBase ("10.1.3.0", "255.255.255.0");
//     Ipv4InterfaceContainer interfacesRightRed = ipv4RightRed.Assign (devicesRightRed);
//     PointToPointHelper p2pRed;
//     NodeContainer nodesRed = NodeContainer (nodesLeftRed.Get (1), red_commandPostNodes.Get (0));
//     NetDeviceContainer devicesRed = p2pRed.Install (nodesRed);
//     Ipv4AddressHelper ipv4Red;
//     ipv4Red.SetBase ("10.1.2.0", "255.255.255.192");
//     Ipv4InterfaceContainer interfacesRed = ipv4Red.Assign (devicesRed);
//     //启用全局路由
//     Ipv4GlobalRoutingHelper::PopulateRoutingTables ();
//     TypeId tidBlue = TypeId::LookupByName("ns3::UdpSocketFactory");
//     Ptr<Socket> recvSinkBlue = Socket::CreateSocket(blue_commandPostNodes.Get(0), tidBlue);
//     InetSocketAddress localBlue = InetSocketAddress(Ipv4Address::GetAny(), 2399);
//     recvSinkBlue->Bind(localBlue);
//     TypeId tidRed = TypeId::LookupByName("ns3::UdpSocketFactory");
//     Ptr<Socket> recvSinkRed = Socket::CreateSocket(red_commandPostNodes.Get(0), tidRed);
//     InetSocketAddress locaRed = InetSocketAddress(Ipv4Address::GetAny(), 2399);
//     recvSinkRed->Bind(locaRed);
//     recvSinkBlue->SetRecvCallback(MakeCallback(&ReceiveBluePacket));
//     recvSinkRed->SetRecvCallback(MakeCallback(&ReceiveRedPacket)); 
// //外部传参代码结束


    // 配置Wi-Fi自组织网络（Ad-Hoc模式）
    WifiHelper red_soldierwifi;
    red_soldierwifi.SetStandard(WIFI_STANDARD_80211b);
    WifiHelper red_dronewifi;
    red_dronewifi.SetStandard(WIFI_STANDARD_80211b);
    WifiHelper red_armoredVehiclewifi;
    red_armoredVehiclewifi.SetStandard(WIFI_STANDARD_80211b);
    WifiHelper red_commandPostwifi;
    red_commandPostwifi.SetStandard(WIFI_STANDARD_80211b);
    WifiHelper  red_engineeringVehiclewifi;
    red_engineeringVehiclewifi.SetStandard(WIFI_STANDARD_80211b);
    WifiHelper red_ambulancewifi;
    red_ambulancewifi.SetStandard(WIFI_STANDARD_80211b);
    WifiHelper red_radiowifi;
    red_radiowifi.SetStandard(WIFI_STANDARD_80211b);
    WifiHelper red_communicationVehiclewifi;
    red_communicationVehiclewifi.SetStandard(WIFI_STANDARD_80211b);

    WifiHelper blue_soldierwifi;
    blue_soldierwifi.SetStandard(WIFI_STANDARD_80211b);
    WifiHelper blue_dronewifi;
    blue_dronewifi.SetStandard(WIFI_STANDARD_80211b);
    WifiHelper blue_armoredVehiclewifi;
    blue_armoredVehiclewifi.SetStandard(WIFI_STANDARD_80211b);
    WifiHelper  blue_ambulancewifi;
    blue_ambulancewifi.SetStandard(WIFI_STANDARD_80211b);
    WifiHelper blue_commandPostwifi;
    blue_commandPostwifi.SetStandard(WIFI_STANDARD_80211b);
    WifiHelper blue_radiowifi;
    blue_radiowifi.SetStandard(WIFI_STANDARD_80211b);
    WifiHelper blue_communicationVehiclewifi;
    blue_communicationVehiclewifi.SetStandard(WIFI_STANDARD_80211b);

    // 创建物理层助手
    YansWifiPhyHelper red_soldierwifiPhy = YansWifiPhyHelper(); 
    red_soldierwifiPhy.Set("TxPowerStart", DoubleValue(80)); // 设置传输功率
    red_soldierwifiPhy.Set("TxPowerEnd", DoubleValue(80)); // 设置传输功率
    YansWifiPhyHelper red_dronewifiPhy = YansWifiPhyHelper(); 
    red_dronewifiPhy.Set("TxPowerStart", DoubleValue(80)); // 设置传输功率
    red_dronewifiPhy.Set("TxPowerEnd", DoubleValue(80)); // 设置传输功率
    YansWifiPhyHelper red_armoredVehiclewifiPhy = YansWifiPhyHelper(); 
    red_armoredVehiclewifiPhy.Set("TxPowerStart", DoubleValue(80)); // 设置传输功率
    red_armoredVehiclewifiPhy.Set("TxPowerEnd", DoubleValue(80)); // 设置传输功率
    YansWifiPhyHelper red_commandPostwifiPhy = YansWifiPhyHelper();
    red_commandPostwifiPhy.Set("TxPowerStart", DoubleValue(80)); // 设置传输功率
    red_commandPostwifiPhy.Set("TxPowerEnd", DoubleValue(80)); // 设置传输功率 
    YansWifiPhyHelper red_engineeringVehiclewifiPhy = YansWifiPhyHelper();
    red_engineeringVehiclewifiPhy.Set("TxPowerStart", DoubleValue(80)); // 设置传输功率
    red_engineeringVehiclewifiPhy.Set("TxPowerEnd", DoubleValue(80)); // 设置传输功率 
    YansWifiPhyHelper red_ambulancewifiPhy = YansWifiPhyHelper();
    red_ambulancewifiPhy.Set("TxPowerStart", DoubleValue(80)); // 设置传输功率
    red_ambulancewifiPhy.Set("TxPowerEnd", DoubleValue(80)); // 设置传输功率 
    YansWifiPhyHelper red_radiowifiPhy = YansWifiPhyHelper(); 
    red_radiowifiPhy.Set("TxPowerStart", DoubleValue(80)); // 设置传输功率
    red_radiowifiPhy.Set("TxPowerEnd", DoubleValue(80)); // 设置传输功率
    YansWifiPhyHelper red_communicationVehiclewifiPhy = YansWifiPhyHelper();
    red_communicationVehiclewifiPhy.Set("TxPowerStart", DoubleValue(80)); // 设置传输功率
    red_communicationVehiclewifiPhy.Set("TxPowerEnd", DoubleValue(80)); // 设置传输功率 

    YansWifiPhyHelper blue_soldierwifiPhy = YansWifiPhyHelper();
    blue_soldierwifiPhy.Set("TxPowerStart", DoubleValue(80)); // 设置传输功率
    blue_soldierwifiPhy.Set("TxPowerEnd", DoubleValue(80)); // 设置传输功率 
    YansWifiPhyHelper blue_dronewifiPhy = YansWifiPhyHelper(); 
    blue_dronewifiPhy.Set("TxPowerStart", DoubleValue(80)); // 设置传输功率
    blue_dronewifiPhy.Set("TxPowerEnd", DoubleValue(80)); // 设置传输功率
    YansWifiPhyHelper blue_armoredVehiclewifiPhy = YansWifiPhyHelper();
    blue_armoredVehiclewifiPhy.Set("TxPowerStart", DoubleValue(80)); // 设置传输功率
    blue_armoredVehiclewifiPhy.Set("TxPowerEnd", DoubleValue(80)); // 设置传输功率 
    YansWifiPhyHelper blue_ambulancewifiPhy = YansWifiPhyHelper();
    blue_ambulancewifiPhy.Set("TxPowerStart", DoubleValue(80)); // 设置传输功率
    blue_ambulancewifiPhy.Set("TxPowerEnd", DoubleValue(80)); // 设置传输功率 
    YansWifiPhyHelper blue_commandPostwifiPhy = YansWifiPhyHelper();
    blue_commandPostwifiPhy.Set("TxPowerStart", DoubleValue(80)); // 设置传输功率
    blue_commandPostwifiPhy.Set("TxPowerEnd", DoubleValue(80)); // 设置传输功率 
    YansWifiPhyHelper blue_radiowifiPhy = YansWifiPhyHelper(); 
    blue_radiowifiPhy.Set("TxPowerStart", DoubleValue(80)); // 设置传输功率
    blue_radiowifiPhy.Set("TxPowerEnd", DoubleValue(80)); // 设置传输功率
    YansWifiPhyHelper blue_communicationVehiclewifiPhy = YansWifiPhyHelper();
    blue_communicationVehiclewifiPhy.Set("TxPowerStart", DoubleValue(80)); // 设置传输功率
    blue_communicationVehiclewifiPhy.Set("TxPowerEnd", DoubleValue(80)); // 设置传输功率 

    // 创建信道助手,并设置信道
    YansWifiChannelHelper wifiChannelred_soldier = YansWifiChannelHelper::Default();
    red_soldierwifiPhy.SetChannel(wifiChannelred_soldier.Create()); 
    YansWifiChannelHelper wifiChannelred_drone = YansWifiChannelHelper::Default();
    red_dronewifiPhy.SetChannel(wifiChannelred_drone.Create()); 
    YansWifiChannelHelper wifiChannelred_armoredVehicle = YansWifiChannelHelper::Default();
    red_armoredVehiclewifiPhy.SetChannel(wifiChannelred_armoredVehicle.Create()); 
    YansWifiChannelHelper wifiChannelred_commandPost = YansWifiChannelHelper::Default();
    red_commandPostwifiPhy.SetChannel(wifiChannelred_commandPost.Create()); 
    YansWifiChannelHelper wifiChannelred_engineeringVehicle = YansWifiChannelHelper::Default();
    red_engineeringVehiclewifiPhy.SetChannel(wifiChannelred_engineeringVehicle.Create()); 
    YansWifiChannelHelper wifiChannelred_ambulance = YansWifiChannelHelper::Default();
    red_ambulancewifiPhy.SetChannel(wifiChannelred_ambulance.Create()); 
    YansWifiChannelHelper wifiChannelred_radio = YansWifiChannelHelper::Default();
    red_radiowifiPhy.SetChannel(wifiChannelred_radio.Create()); 
    YansWifiChannelHelper wifiChannelred_communicationVehicle = YansWifiChannelHelper::Default();
    red_communicationVehiclewifiPhy.SetChannel(wifiChannelred_communicationVehicle.Create()); 

    YansWifiChannelHelper wifiChannelblue_soldier = YansWifiChannelHelper::Default();
    blue_soldierwifiPhy.SetChannel(wifiChannelblue_soldier.Create()); 
    YansWifiChannelHelper wifiChannelblue_drone = YansWifiChannelHelper::Default();
    blue_dronewifiPhy.SetChannel(wifiChannelblue_drone.Create()); 
    YansWifiChannelHelper wifiChannelblue_armoredVehicle = YansWifiChannelHelper::Default();
    blue_armoredVehiclewifiPhy.SetChannel(wifiChannelblue_armoredVehicle.Create()); 
    YansWifiChannelHelper wifiChannelblue_ambulance = YansWifiChannelHelper::Default();
    blue_ambulancewifiPhy.SetChannel(wifiChannelblue_ambulance.Create()); 
    YansWifiChannelHelper wifiChannelblue_commandPost = YansWifiChannelHelper::Default();
    blue_commandPostwifiPhy.SetChannel(wifiChannelblue_commandPost.Create()); 
    YansWifiChannelHelper wifiChannelblue_radio = YansWifiChannelHelper::Default();
    blue_radiowifiPhy.SetChannel(wifiChannelblue_radio.Create()); 
    YansWifiChannelHelper wifiChannelblue_communicationVehicle = YansWifiChannelHelper::Default();
    blue_communicationVehiclewifiPhy.SetChannel(wifiChannelblue_communicationVehicle.Create()); 



    // 创建MAC层助手,并设置为AD-Hoc模式
    WifiMacHelper red_soldierwifiMac; 
    red_soldierwifiMac.SetType("ns3::AdhocWifiMac");
    WifiMacHelper red_dronewifiMac; 
    red_dronewifiMac.SetType("ns3::AdhocWifiMac");
    WifiMacHelper red_armoredVehiclewifiMac; 
    red_armoredVehiclewifiMac.SetType("ns3::AdhocWifiMac");
    WifiMacHelper red_commandPostwifiMac; 
    red_commandPostwifiMac.SetType("ns3::AdhocWifiMac");
    WifiMacHelper red_engineeringVehiclewifiMac; 
    red_engineeringVehiclewifiMac.SetType("ns3::AdhocWifiMac");
    WifiMacHelper red_ambulancewifiMac;
    red_ambulancewifiMac.SetType("ns3::AdhocWifiMac");
    WifiMacHelper red_radiowifiMac; 
    red_radiowifiMac.SetType("ns3::AdhocWifiMac");
    WifiMacHelper red_communicationVehiclewifiMac; 
    red_communicationVehiclewifiMac.SetType("ns3::AdhocWifiMac");
    
    WifiMacHelper blue_soldierwifiMac; 
    blue_soldierwifiMac.SetType("ns3::AdhocWifiMac");
    WifiMacHelper blue_dronewifiMac;
    blue_dronewifiMac.SetType("ns3::AdhocWifiMac");
    WifiMacHelper blue_armoredVehiclewifiMac;
    blue_armoredVehiclewifiMac.SetType("ns3::AdhocWifiMac");
    WifiMacHelper blue_ambulancewifiMac;
    blue_ambulancewifiMac.SetType("ns3::AdhocWifiMac");
    WifiMacHelper blue_commandPostwifiMac;
    blue_commandPostwifiMac.SetType("ns3::AdhocWifiMac");
    WifiMacHelper blue_radiowifiMac;
    blue_radiowifiMac.SetType("ns3::AdhocWifiMac");
    WifiMacHelper blue_communicationVehiclewifiMac;
    blue_communicationVehiclewifiMac.SetType("ns3::AdhocWifiMac");

    
    // 安装Wi-Fi设备到节点
    NetDeviceContainer red_soldierDevices = red_soldierwifi.Install(red_soldierwifiPhy, red_soldierwifiMac, red_soldierNodes);
    NetDeviceContainer red_droneDevices = red_dronewifi.Install(red_dronewifiPhy, red_dronewifiMac, red_droneNodes);
    NetDeviceContainer red_commandPostDevices = red_commandPostwifi.Install(red_commandPostwifiPhy, red_commandPostwifiMac, red_commandPostNodes);
    NetDeviceContainer red_communicationVehicleDevices = red_communicationVehiclewifi.Install(red_communicationVehiclewifiPhy, red_communicationVehiclewifiMac, red_communicationVehicleNodes);
    NetDeviceContainer red_radioDevices = red_radiowifi.Install(red_radiowifiPhy,red_radiowifiMac,red_radioNodes);
    NetDeviceContainer red_ambulanceDevices = red_ambulancewifi.Install(red_ambulancewifiPhy,red_ambulancewifiMac,red_ambulanceNodes);
    NetDeviceContainer red_engineeringVehicleDevices = red_engineeringVehiclewifi.Install(red_engineeringVehiclewifiPhy,red_engineeringVehiclewifiMac,red_engineeringVehicleNodes);
    NetDeviceContainer red_armoredVehicleDevices = red_engineeringVehiclewifi.Install(red_armoredVehiclewifiPhy,red_armoredVehiclewifiMac,red_armoredVehicleNodes);

    NetDeviceContainer blue_ambulanceDevices = blue_ambulancewifi.Install(blue_ambulancewifiPhy,blue_ambulancewifiMac,blue_ambulanceNodes);
    NetDeviceContainer blue_armoredVehicleDevices = blue_armoredVehiclewifi.Install(blue_armoredVehiclewifiPhy,blue_armoredVehiclewifiMac,blue_armoredVehicleNodes);
    NetDeviceContainer blue_commandPostDevices = blue_commandPostwifi.Install(blue_commandPostwifiPhy,blue_commandPostwifiMac,blue_commandPostNodes);
    NetDeviceContainer blue_communicationVehicleDevices = blue_communicationVehiclewifi.Install(blue_communicationVehiclewifiPhy,blue_communicationVehiclewifiMac,blue_communicationVehicleNodes);
    NetDeviceContainer blue_droneDevices = blue_dronewifi.Install(blue_dronewifiPhy,blue_dronewifiMac,blue_droneNodes);
    NetDeviceContainer blue_radioDevices = blue_radiowifi.Install(blue_radiowifiPhy,blue_radiowifiMac,blue_radioNodes);
    NetDeviceContainer blue_soldierDevices = blue_soldierwifi.Install(blue_soldierwifiPhy,blue_soldierwifiMac,blue_soldierNodes);


    // 配置节点的位置
    MobilityHelper red_soldiermobility;
    MobilityHelper red_dronemobility;
    MobilityHelper red_armoredmobility;
    MobilityHelper red_commandPostmobility;
    MobilityHelper red_communicationVehiclemobility;
    MobilityHelper red_radiomobility;
    MobilityHelper red_ambulancemobility;
    MobilityHelper red_engineeringVehiclemobility;

    MobilityHelper blue_soldiermobility;
    MobilityHelper blue_dronemobility;
    MobilityHelper blue_armoredmobility;
    MobilityHelper blue_commandPostmobility;
    MobilityHelper blue_communicationVehiclemobility;
    MobilityHelper blue_radiomobility;
    MobilityHelper blue_ambulancemobility;
    MobilityHelper blue_engineeringVehiclemobility;

    //创建位置分配器
    Ptr<ListPositionAllocator> red_soldierpositionAlloc = CreateObject<ListPositionAllocator>();
    Ptr<ListPositionAllocator> red_dronepositionAlloc = CreateObject<ListPositionAllocator>();
    Ptr<ListPositionAllocator> red_armoredpositionAlloc = CreateObject<ListPositionAllocator>();
    Ptr<ListPositionAllocator> red_commandPostpositionAlloc = CreateObject<ListPositionAllocator>();
    Ptr<ListPositionAllocator> red_communicationVehiclepositionAlloc = CreateObject<ListPositionAllocator>();
    Ptr<ListPositionAllocator> red_radiopositionAlloc = CreateObject<ListPositionAllocator>();
    Ptr<ListPositionAllocator> red_ambulancepositionAlloc = CreateObject<ListPositionAllocator>();
    Ptr<ListPositionAllocator> red_engineeringVehiclepositionAlloc = CreateObject<ListPositionAllocator>();

    Ptr<ListPositionAllocator> blue_soldierpositionAlloc = CreateObject<ListPositionAllocator>();
    Ptr<ListPositionAllocator> blue_dronepositionAlloc = CreateObject<ListPositionAllocator>();
    Ptr<ListPositionAllocator> blue_armoredpositionAlloc = CreateObject<ListPositionAllocator>();
    Ptr<ListPositionAllocator> blue_commandPostpositionAlloc = CreateObject<ListPositionAllocator>();
    Ptr<ListPositionAllocator> blue_communicationVehiclepositionAlloc = CreateObject<ListPositionAllocator>();
    Ptr<ListPositionAllocator> blue_radiopositionAlloc = CreateObject<ListPositionAllocator>();
    Ptr<ListPositionAllocator> blue_ambulancepositionAlloc = CreateObject<ListPositionAllocator>();
    Ptr<ListPositionAllocator> blue_engineeringVehiclepositionAlloc = CreateObject<ListPositionAllocator>();
    
    //从文件中添加节点的位置信息为其分配坐标
    int a[8] = {0,0,0,0,0,0,0,0};
    int b[7] = {0,0,0,0,0,0,0};

// 添加红色方节点
    for (const auto &d : red_data) {
        AddidToGroup(d.col2, "Red");
        if(d.col1 == "士兵"){
            AddPosition(a[0],d,red_soldierpositionAlloc,red_soldierNodes,"red_soldierNodes");
        }else if(d.col1 == "坦克车"){
            AddPosition(a[1],d,red_armoredpositionAlloc,red_armoredVehicleNodes,"red_armoredVehicleNodes");
        }else if(d.col1 == "工程车"){ 
            AddPosition(a[2],d,red_engineeringVehiclepositionAlloc,red_engineeringVehicleNodes,"red_engineeringVehicleNodes");
        }else if(d.col1 == "运输车"){ 
            AddPosition(a[3],d,red_dronepositionAlloc,red_droneNodes,"red_droneNodes");    
        }else if(d.col1 == "医疗车"){
            AddPosition(a[4],d,red_ambulancepositionAlloc,red_ambulanceNodes,"red_ambulanceNodes");  
        }else if(d.col1 == "指挥帐篷"){
            AddPosition(a[5],d,red_commandPostpositionAlloc,red_commandPostNodes,"red_commandPostNodes");
        }else if(d.col1 == "雷达车"){
            AddPosition(a[6],d,red_communicationVehiclepositionAlloc,red_communicationVehicleNodes,"red_communicationVehicleNodes");   
        }else if(d.col1 == "电台"){
            AddPosition(a[7],d,red_radiopositionAlloc,red_radioNodes,"red_radioNodes");    
        }
    }

//添加蓝色方节点
    for (const auto &d : blue_data) {
        AddidToGroup(d.col2, "Blue");
        if(d.col1 == "士兵"){ 
            AddPosition(b[0],d,blue_soldierpositionAlloc,blue_soldierNodes,"blue_soldierNodes");
        }else if(d.col1 == "装甲车"){
            AddPosition(b[1],d,blue_armoredpositionAlloc,blue_armoredVehicleNodes,"blue_armoredVehicleNodes");
        }else if(d.col1 == "运输车"){
            AddPosition(b[2],d,blue_dronepositionAlloc,blue_droneNodes,"blue_droneNodes");
        }else if(d.col1 == "医疗车"){
            AddPosition(b[3],d,blue_ambulancepositionAlloc,blue_ambulanceNodes,"blue_ambulanceNodes");
        }else if(d.col1 == "指挥帐篷"){
            AddPosition(b[4],d,blue_commandPostpositionAlloc,blue_commandPostNodes,"blue_commandPostNodes");
        }else if(d.col1 == "雷达车"){
            AddPosition(b[5],d,blue_communicationVehiclepositionAlloc,blue_communicationVehicleNodes,"blue_communicationVehicleNodes");
        }else if(d.col1 == "电台"){
            AddPosition(b[6],d,blue_radiopositionAlloc,blue_radioNodes,"blue_radioNodes");
        }
    }


    //给节点添加对应的地形以及天气属性
    for (const auto& row : fileData) {
        Ptr<Node> node = FindFromMap(row.id);
        // cout<<node<<endl;
        vector<string> data = {row.terrain,row.weather};
        AddVectorData(node, data);
        // cout << "ID: " << row.id << ", Terrain: " << row.terrain << ", Weather: " << row.weather << endl;
    }

    // vector<string> retrievedVector = SearchVectorData(red_soldierNodes.Get(0));
    // if(!retrievedVector.empty()){
    //     cout<<"成功给节点red_soldierNodes.Get(0)添加数据" <<endl;
    //     for (string value : retrievedVector) {
    //         cout << value << " ";
    //     }
    //     cout << endl;
    // }



    //设置位置分配器
    red_soldiermobility.SetPositionAllocator(red_soldierpositionAlloc); 
    red_dronemobility.SetPositionAllocator(red_dronepositionAlloc);
    red_commandPostmobility.SetPositionAllocator(red_commandPostpositionAlloc);
    red_engineeringVehiclemobility.SetPositionAllocator(red_engineeringVehiclepositionAlloc);
    red_ambulancemobility.SetPositionAllocator(red_ambulancepositionAlloc);
    red_radiomobility.SetPositionAllocator(red_radiopositionAlloc);
    red_communicationVehiclemobility.SetPositionAllocator(red_communicationVehiclepositionAlloc);
    red_armoredmobility.SetPositionAllocator(red_armoredpositionAlloc);

    blue_soldiermobility.SetPositionAllocator(blue_soldierpositionAlloc);
    blue_dronemobility.SetPositionAllocator(blue_dronepositionAlloc);
    blue_armoredmobility.SetPositionAllocator(blue_armoredpositionAlloc);
    blue_commandPostmobility.SetPositionAllocator(blue_commandPostpositionAlloc);
    blue_engineeringVehiclemobility.SetPositionAllocator(blue_engineeringVehiclepositionAlloc);
    blue_ambulancemobility.SetPositionAllocator(blue_ambulancepositionAlloc);
    blue_radiomobility.SetPositionAllocator(blue_radiopositionAlloc);
    blue_communicationVehiclemobility.SetPositionAllocator(blue_communicationVehiclepositionAlloc);
 
    // 对其他节点应用ConstantPositionMobilityModel或其他模型
    // red_soldiermobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    red_soldiermobility.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
                              "Bounds", RectangleValue(Rectangle(-5000, 5000, -5000, 5000)), // 移动范围
                              "Distance", DoubleValue(10),  // 每次移动的距离
                              "Time", TimeValue(Seconds(1))); // 每次移动的时间间隔
    blue_soldiermobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    red_dronemobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    blue_dronemobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    red_armoredmobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    blue_armoredmobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    red_commandPostmobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    blue_commandPostmobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    red_engineeringVehiclemobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    blue_engineeringVehiclemobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    red_ambulancemobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    blue_ambulancemobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    red_radiomobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    blue_radiomobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    red_communicationVehiclemobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    blue_communicationVehiclemobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");

    //安装移动性模型
    red_dronemobility.Install(red_droneNodes); 
    red_armoredmobility.Install(red_armoredVehicleNodes);
    red_commandPostmobility.Install(red_commandPostNodes);
    red_engineeringVehiclemobility.Install(red_engineeringVehicleNodes);
    red_ambulancemobility.Install(red_ambulanceNodes);
    red_radiomobility.Install(red_radioNodes);
    red_communicationVehiclemobility.Install(red_communicationVehicleNodes);
    red_soldiermobility.Install(red_soldierNodes);

    blue_ambulancemobility.Install(blue_ambulanceNodes);
    blue_armoredmobility.Install(blue_armoredVehicleNodes);
    blue_commandPostmobility.Install(blue_commandPostNodes);
    blue_communicationVehiclemobility.Install(blue_communicationVehicleNodes);
    blue_dronemobility.Install(blue_droneNodes);
    blue_radiomobility.Install(blue_radioNodes);
    blue_soldiermobility.Install(blue_soldierNodes);

    // 安装互联网协议栈
    InternetStackHelper stack;
    stack.Install(red_soldierNodes);
    stack.Install(red_droneNodes);
    stack.Install(red_armoredVehicleNodes);
    stack.Install(red_commandPostNodes);
    stack.Install(red_engineeringVehicleNodes);
    stack.Install(red_ambulanceNodes);
    stack.Install(red_radioNodes);
    stack.Install(red_communicationVehicleNodes);

    stack.Install(blue_ambulanceNodes);
    stack.Install(blue_armoredVehicleNodes);
    stack.Install(blue_commandPostNodes);
    stack.Install(blue_communicationVehicleNodes);
    stack.Install(blue_droneNodes);
    stack.Install(blue_radioNodes);
    stack.Install(blue_soldierNodes);

    // 分配IP地址
    Ipv4AddressHelper red_address; 
    Ipv4AddressHelper blue_address; 
    red_address.SetBase("10.1.8.0", "255.255.255.0"); 
    blue_address.SetBase("10.1.9.0","255.255.255.0");
    // 将IP地址分配给节点
    Ipv4InterfaceContainer red_soldierInterfaces = red_address.Assign(red_soldierDevices);
    Ipv4InterfaceContainer red_droneInterfaces = red_address.Assign(red_droneDevices);
    Ipv4InterfaceContainer red_armoredVehicleInterfaces = red_address.Assign(red_armoredVehicleDevices);
    Ipv4InterfaceContainer red_commandPostInterfaces = red_address.Assign(red_commandPostDevices);
    Ipv4InterfaceContainer red_engineeringVehicleInterfaces = red_address.Assign(red_engineeringVehicleDevices);
    Ipv4InterfaceContainer red_ambulanceInterfaces = red_address.Assign(red_ambulanceDevices);
    Ipv4InterfaceContainer red_radioInterfaces = red_address.Assign(red_radioDevices);
    Ipv4InterfaceContainer red_communicationVehicleInterfaces = red_address.Assign(red_communicationVehicleDevices);

    Ipv4InterfaceContainer blue_soldierInterfaces = blue_address.Assign(blue_soldierDevices);
    Ipv4InterfaceContainer blue_ambulanceIntefaces = blue_address.Assign(blue_ambulanceDevices);
    Ipv4InterfaceContainer blue_armoredVehicleInterfaces = blue_address.Assign(blue_armoredVehicleDevices);
    Ipv4InterfaceContainer blue_commandPostInterfaces = blue_address.Assign(blue_commandPostDevices);
    Ipv4InterfaceContainer blue_communicationVehicleInterfaces = blue_address.Assign(blue_commandPostDevices);
    Ipv4InterfaceContainer blue_droneInterfaces = blue_address.Assign(blue_droneDevices);
    Ipv4InterfaceContainer blue_radioInterfaces = blue_address.Assign(blue_radioDevices);

    // 设置UDP回声应用程序
    uint16_t red_port = 9, blue_port = 10, red_soldierport=13, blue_soldierport=14;
    UdpEchoServerHelper red_echoServer(red_port), blue_echoServer(blue_port);
    UdpEchoServerHelper red_echoServer13(red_soldierport), blue_echoServer14(blue_soldierport);

    // 在指挥部节点上安装回声服务器,给第二个士兵节点设置了单独的端口号，定义了监听该端口号的服务器
    ApplicationContainer red_serverApps = red_echoServer.Install(red_commandPostNodes.Get(0));
    ApplicationContainer red_serverApps13 = red_echoServer13.Install(red_commandPostNodes.Get(0));
    red_serverApps13.Start(Seconds(1.0));
    red_serverApps13.Stop(Seconds(10.0));
    red_serverApps.Start(Seconds(1.0));
    red_serverApps.Stop(Seconds(50.0));

    ApplicationContainer blue_serverApps = blue_echoServer.Install(blue_commandPostNodes.Get(0));
    ApplicationContainer blue_serverApps14 = blue_echoServer14.Install(blue_commandPostNodes.Get(0));
    blue_serverApps14.Start(Seconds(1.0));
    blue_serverApps14.Stop(Seconds(10.0));
    blue_serverApps.Start(Seconds(1.0));
    blue_serverApps.Stop(Seconds(50.0));

    // 在士兵、装甲车和通信车、电台等节点上安装UDP回声客户端
    UdpEchoClientHelper red_echoClient(red_commandPostInterfaces.GetAddress(0), red_port);
    UdpEchoClientHelper blue_echoClient(blue_commandPostInterfaces.GetAddress(0),blue_port);
    UdpEchoClientHelper red_echoSingnalSoldierClient(red_commandPostInterfaces.GetAddress(0),red_soldierport);
    UdpEchoClientHelper blue_echoSingnalSoldierClient(blue_commandPostInterfaces.GetAddress(0),blue_soldierport);
    red_echoClient.SetAttribute("MaxPackets", UintegerValue(1));
    red_echoClient.SetAttribute("Interval", TimeValue(Seconds(1.0)));
    red_echoClient.SetAttribute("PacketSize", UintegerValue(1024));

    blue_echoClient.SetAttribute("MaxPackets", UintegerValue(1));
    blue_echoClient.SetAttribute("Interval", TimeValue(Seconds(1.0)));
    blue_echoClient.SetAttribute("PacketSize", UintegerValue(1024));

    red_echoSingnalSoldierClient.SetAttribute("MaxPackets", UintegerValue(1));
    red_echoSingnalSoldierClient.SetAttribute("Interval", TimeValue(Seconds(1.0)));
    red_echoSingnalSoldierClient.SetAttribute("PacketSize", UintegerValue(1024));

    blue_echoSingnalSoldierClient.SetAttribute("MaxPackets", UintegerValue(1));
    blue_echoSingnalSoldierClient.SetAttribute("Interval", TimeValue(Seconds(1.0)));
    blue_echoSingnalSoldierClient.SetAttribute("PacketSize", UintegerValue(1024));

    ApplicationContainer red_clientAppsSoldier = red_echoClient.Install(red_soldierNodes);
    ApplicationContainer red_clientAppsDrone = red_echoClient.Install(red_droneNodes);
    ApplicationContainer red_clientAppsCommunicationVehicle = red_echoClient.Install(red_communicationVehicleNodes);
    ApplicationContainer red_clientAppsengineering = red_echoClient.Install(red_engineeringVehicleNodes);
    ApplicationContainer red_clientAppsambulance = red_echoClient.Install(red_ambulanceNodes);
    ApplicationContainer red_clientAppsradio = red_echoClient.Install(red_radioNodes);
    ApplicationContainer red_clientAppsarmoredVehicle = red_echoClient.Install(red_armoredVehicleNodes);

    ApplicationContainer red_clientAppSingalSoldier = red_echoSingnalSoldierClient.Install(red_soldierNodes.Get(2));

    ApplicationContainer blue_clientAppsSoldier = blue_echoClient.Install(blue_soldierNodes);
    ApplicationContainer blue_clientAppsDrone = blue_echoClient.Install(blue_droneNodes);
    ApplicationContainer blue_clientAppsCommunicationVehicle = blue_echoClient.Install(blue_commandPostNodes);
    ApplicationContainer blue_clientAppsAmbulance = blue_echoClient.Install(blue_ambulanceNodes);
    ApplicationContainer blue_clientAppsRadio = blue_echoClient.Install(blue_radioNodes);
    ApplicationContainer blue_clientAppsArmoredVehicle = blue_echoClient.Install(blue_armoredVehicleNodes);

    ApplicationContainer blue_clientAppSingalSoldier = blue_echoSingnalSoldierClient.Install(blue_soldierNodes.Get(2));

    std::streambuf* coutBuffer = std::cout.rdbuf(); // 保存原始的 std::cout 缓冲区
    std::cout.rdbuf(file.rdbuf()); // 将输出重定向到文件 
    cout<<"红色方"<<endl;
    PrintTxPower(red_soldierDevices,"Soldier Devices");
    PrintTxPower(red_droneDevices,"Drone Devices");
    PrintTxPower(red_ambulanceDevices,"ambulanceDevices");
    PrintTxPower(red_armoredVehicleDevices,"armoredVehicleDevices");
    PrintTxPower(red_commandPostDevices,"commandPostDevices");
    PrintTxPower(red_engineeringVehicleDevices,"engineeringVehicleDevices");
    PrintTxPower(red_communicationVehicleDevices,"communicationVehicleDevices");
    PrintTxPower(red_radioDevices,"radioDevices");
    cout<<endl<<"蓝色方"<<endl;
    PrintTxPower(blue_soldierDevices,"blue_soldierDevices");
    PrintTxPower(blue_droneDevices,"blue_droneDevices");
    PrintTxPower(blue_ambulanceDevices,"blue_ambulanceDevices");
    PrintTxPower(blue_armoredVehicleDevices,"blue_armoredVehicleDevices");
    PrintTxPower(blue_commandPostDevices,"blue_commandPostDevices");
    PrintTxPower(blue_communicationVehicleDevices,"blue_communicationVehicleDevices");
    PrintTxPower(blue_radioDevices,"blue_radioDevices");
    std::cout.rdbuf(coutBuffer);

    Ptr<Node> foundNode = FindFromMap("Model.231025144318820-25");
    // cout<<"找到的节点信息为："<<foundNode<<endl;

    string group = FindFromGroupMap("Model.231025144318820-25");


    // cout<<"哈哈哈，我已经运行完啦！！！！！！"<<endl;


    red_clientAppsSoldier.Start(Seconds(2.0));
    red_clientAppsSoldier.Stop(Seconds(100.0));
    red_clientAppsDrone.Start(Seconds(2.5));
    red_clientAppsDrone.Stop(Seconds(100.0));
    red_clientAppsCommunicationVehicle.Start(Seconds(3.0));
    red_clientAppsCommunicationVehicle.Stop(Seconds(100.0));
    red_clientAppsengineering.Start(Seconds(3.5));
    red_clientAppsengineering.Stop(Seconds(100.0));
    red_clientAppsambulance.Start(Seconds(4.0));
    red_clientAppsambulance.Stop(Seconds(100.0));
    red_clientAppsradio.Start(Seconds(4.5));
    red_clientAppsradio.Stop(Seconds(100.0));
    red_clientAppsarmoredVehicle.Start(Seconds(5.0));
    red_clientAppsarmoredVehicle.Stop(Seconds(100.0));

    blue_clientAppsSoldier.Start(Seconds(2.1));
    blue_clientAppsSoldier.Stop(Seconds(100.0));
    blue_clientAppsAmbulance.Start(Seconds(2.3));
    blue_clientAppsAmbulance.Stop(Seconds(100.0));
    blue_clientAppsArmoredVehicle.Start(Seconds(2.7));
    blue_clientAppsArmoredVehicle.Stop(Seconds(100.0));
    blue_clientAppsCommunicationVehicle.Start(Seconds(3.3));
    blue_clientAppsCommunicationVehicle.Stop(Seconds(100.0));
    blue_clientAppsDrone.Start(Seconds(3.7));
    blue_clientAppsDrone.Stop(Seconds(100.0));
    blue_clientAppsRadio.Start(Seconds(4.3));
    blue_clientAppsRadio.Stop(Seconds(100.0));
    blue_clientAppsSoldier.Start(Seconds(4.7));
    blue_clientAppsSoldier.Stop(Seconds(100.0));

    Simulator::Schedule(Seconds(1.0), &LogPosition,red_soldierNodes, std::ref(outputFile));
    Simulator::Schedule(Seconds(1.1), &LogPosition,red_ambulanceNodes, std::ref(outputFile));
    Simulator::Schedule(Seconds(1.2), &LogPosition,red_armoredVehicleNodes, std::ref(outputFile));
    Simulator::Schedule(Seconds(1.3), &LogPosition,red_commandPostNodes, std::ref(outputFile));
    Simulator::Schedule(Seconds(1.4), &LogPosition,red_communicationVehicleNodes, std::ref(outputFile));
    Simulator::Schedule(Seconds(1.5), &LogPosition,red_droneNodes, std::ref(outputFile));
    Simulator::Schedule(Seconds(1.6), &LogPosition,red_radioNodes, std::ref(outputFile));
    Simulator::Schedule(Seconds(1.7), &LogPosition,red_engineeringVehicleNodes, std::ref(outputFile));
    Simulator::Schedule(Seconds(1.8), &LogPosition,blue_soldierNodes, std::ref(outputFile));
    Simulator::Schedule(Seconds(1.9), &LogPosition,blue_ambulanceNodes, std::ref(outputFile));
    Simulator::Schedule(Seconds(2.0), &LogPosition,blue_armoredVehicleNodes, std::ref(outputFile));
    Simulator::Schedule(Seconds(2.1), &LogPosition,blue_commandPostNodes, std::ref(outputFile));
    Simulator::Schedule(Seconds(2.2), &LogPosition,blue_communicationVehicleNodes, std::ref(outputFile));
    Simulator::Schedule(Seconds(2.3), &LogPosition,blue_droneNodes, std::ref(outputFile));
    Simulator::Schedule(Seconds(2.4), &LogPosition,blue_radioNodes, std::ref(outputFile));

    Simulator::Schedule(Seconds(30.0), &ClearFile, std::ref(outputFile));

    // 运行仿真         
    Simulator::Run();
    Simulator::Destroy();

    outputFile.close(); 

    return 0;
}
