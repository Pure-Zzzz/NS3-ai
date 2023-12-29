#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/yans-wifi-phy.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/ipv4-flow-classifier.h"
#include "ns3/internet-module.h"
#include "ns3/udp-client-server-helper.h"
#include "ns3/applications-module.h"
#include "ns3/aodv-module.h"
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
#include <math.h>
#include <regex>
#include <string>
#include <iomanip>
#include "apb.h"
#include <ns3/ai-module.h>
using namespace ns3;
using namespace std;


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
ofstream file("result.txt");


static map<Ipv4Address, string> ipToIdMap;
static map<string, string> idToNameMap;
static map<string, string> idToGroupMap;
static map<string, string> nameToIdMap;
static map<string, string> nameToTypeMap;//根据Nodename查询节点的类型
int num=1;

//地理坐标系和笛卡尔坐标系的转换
constexpr double DEG_TO_RAD_LOCAL = 3.1415926535897932 / 180.0;
constexpr double RAD_TO_DEG_LOCAL = 180.0 /3.1415926535897932;
ofstream outputFile;
ofstream dataputFile;
ofstream dataActiviaty;
ofstream tempFile;
Ptr<Node> tempNode;//临时记录节点信息的全局变量,用于输出snr等信息
int first = 1;//判断输出的json是否为第一行从而决定是否输出
int second = 1;//判断数据包信息输出文件是否为第一行
int third = 1;//判断活跃度文件信息输出是否为第一行
int activiaty ;//判断是否给用户活跃度文件添加信息，每25s会添加5s的数据
int tempnum =1;
double NodePower(Ptr<Node> node);

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

MediaType GenerateRandomMediaType() {
    Ptr<UniformRandomVariable> x = CreateObject<UniformRandomVariable>();
    x->SetAttribute("Min", DoubleValue(1));
    x->SetAttribute("Max", DoubleValue(3));
    int randomValue = std::round(x->GetValue());

    switch (randomValue) {
        case 1: return VIDEO;
        case 2: return TEXT;
        case 3: return AUDIO;
        default: return TEXT; // Default case, should not happen
    }
}

//读取节点位置信息
vector<Data> readData(const string &filePath) {
    vector<Data> dataArray;
    ifstream file(filePath);
    string line;
    while (getline(file, line)) {
        istringstream iss(line);
        vector<string> columns;
        string item;
        while (getline(iss, item, '\t')) {
            columns.push_back(item);
        }
        if (columns.size() >= 12) {
            // Data data = {columns[0], columns[1], columns[9], columns[10], columns[11]};//获取以第一个红色士兵节点为原点的坐标数据
            Data data = {columns[0], columns[1], columns[6], columns[7], columns[8]};//获取原始转换的笛卡尔坐标系的数据
            dataArray.push_back(data);
        }
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
        string foundName = Names::FindName(node);
        if(nameToIdMap.find(foundName)!=nameToIdMap.end()){
            //通过NodeName来获取Id
            string foundNodeId = nameToIdMap[foundName];

            // cout<<"Found node Id: "<<foundNodeId<<endl;
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

        // cout<<"Found node name: "<<foundNodeName<<endl;
        return foundNode;
    }else{
        cout<<"Node with ID"<<searchId<<" not found."<<endl;
    }
    return 0;
}

void AddPosition(int &i, Data d, Ptr<ListPositionAllocator> positionAlloc,NodeContainer Nodes, string Nodename){
    double x = stod(d.col10);
    double y = stod(d.col11);
    double z = stod(d.col12);
    positionAlloc->Add(Vector(x/10.0, y/10.0, z/10.0));
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

double NodeSensitivity(Ptr<Node> onenode){
    // 获取第一个节点的第一个设备
    Ptr<Node> node = onenode;
    Ptr<NetDevice> dev = node->GetDevice (0);
    Ptr<WifiNetDevice> wifiDev = dev->GetObject<WifiNetDevice> ();
    Ptr<YansWifiPhy> phy = DynamicCast<YansWifiPhy>(wifiDev->GetPhy ());
    

    // 打印接收器灵敏度
    double rxSensitivity = phy->GetRxSensitivity ();
    std::cout << "Rx Sensitivity of node " << FindIdFromMap(node) << " is " << rxSensitivity << " dBm" << std::endl;

    return rxSensitivity;
}

//将内部数据输出到json文件中
void LogJsonPosition(NodeContainer Nodes, ofstream& outputFile)
{
//处理非收到数据包数据 
    for(uint16_t i = 0; i < Nodes.GetN(); ++i) 
    {
        Ptr<MobilityModel> mobility = Nodes.Get(i)->GetObject<MobilityModel>();
        Vector pos = mobility->GetPosition();
        string foundName = Names::FindName(Nodes.Get(i));
        vector<string> retrievedVector = SearchVectorData(Nodes.Get(i));
        // 获取当前时间点
        auto now = chrono::system_clock::now();

        // 转换为时间戳
        auto timestamp = chrono::duration_cast<chrono::milliseconds>(now.time_since_epoch()).count();
        double X,Y,Z;
        if(first == 1)
            outputFile << "[" <<endl;
        outputFile << "{";  // 开始一个 JSON 对象

        // 添加各种数据到 JSON 对象
        // outputFile << "\"Time\": \"" << Simulator::Now().GetSeconds() << "s\", ";
        outputFile << "\"Timestamp\": \"" << timestamp << "s\", ";
        outputFile << "\"PkgType\": \"001\",";
        outputFile << "\"NodeId\": \"" << FindIdFromMap(Nodes.Get(i)) << "\", ";
        outputFile << "\"NodeActivity\": \"" << "1\", ";
        outputFile << "\"NodeSensitivity\": \"" << NodeSensitivity(Nodes.Get(i)) << "dBm\", ";
        outputFile << "\"NodeGroup\":\""<< FindFromGroupMap(FindIdFromMap(Nodes.Get(i))) <<"\",";
        outputFile << "\"NodeType\": \"" << nameToTypeMap[foundName] << "\", ";
        outputFile << "\"NodeName\": \"" << foundName << "\", ";
        
        // if((foundName != "red_commandPostNodes.Get(0)") && (foundName != "blue_commandPostNodes.Get(0)"))
        // {
            outputFile << "\"NodeTxPower\": \"" << NodePower(Nodes.Get(i)) << "dBm\", ";
        // }

        // outputFile << "\"Position\": {\"x\": " << pos.x << ", \"y\": " << pos.y << ", \"z\": " << pos.z << "}, ";
        // XYZ2LLA((pos.x*100)-1210135.1685510000,(pos.y*100)+5045472.0347960000,(pos.z*100)+3700382.7212280000,X,Y,Z);//以第一个红色士兵节点为原点并且将坐标等比缩小了100倍
        // XYZ2LLA((pos.x)-1210135.1685510000,(pos.y)+5045472.0347960000,(pos.z)+3700382.7212280000,X,Y,Z);//以第一个红色士兵节点为原点
        XYZ2LLA(pos.x*10,pos.y*10,pos.z*10,X,Y,Z);//直接采用原始未经处理的地理坐标转换的笛卡尔坐标系坐标
        outputFile << "\"Position\": {\"x\": "  << fixed << setprecision(6)<< X << ", \"y\": " << Y << ", \"z\": " << Z << "}, ";//输出地理坐标
        outputFile.unsetf(ios_base::fixed);
        outputFile.precision(streamsize(-1));
        outputFile << "\"Terrain\": \"" << retrievedVector[0] << "\", ";
        outputFile << "\"Weather\": \"" << retrievedVector[1] << "\"";
        outputFile << "}";  // 结束 JSON 对象
        if(first == 1) {
            first = 0;
            outputFile << "," <<endl;
        }else{
            outputFile <<","<<endl;  // 换行
        }
    }
    // 每秒记录
    Simulator::Schedule(Seconds(0.5), &LogJsonPosition, Nodes, ref(outputFile));
}

void Closefile(ofstream& outputFile){
    outputFile.close();
}

//处理json文件最后一行
void ModifyJsonFile(std::fstream& file) {
    /*
    修改已打开的 JSON 文件的函数。
    删除最后一行的最后一个字符（逗号）后的换行符，然后在下一行添加 ']'

    参数:
    file: 引用已打开的 JSON 文件流
    */

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



//根据节点找传输功率
double NodePower(Ptr<Node> node){
    Ptr<NetDevice> dev = node->GetDevice(0); 
    Ptr<WifiNetDevice> wifiDev = dev->GetObject<WifiNetDevice>();
    Ptr<YansWifiPhy> phy = DynamicCast<YansWifiPhy>(wifiDev->GetPhy());
    double txPower = phy->GetTxPowerStart();
    return txPower;
}

void ClearFile(ofstream& outputFile) {
    outputFile.open("node-movement-log.json", ofstream::out | ofstream::trunc); // 以截断方式打开文件
    if (outputFile.is_open()) {
        outputFile.close(); // 关闭文件，这将清空文件内容
    }
    outputFile.open("node-movement-log.json");
    outputFile << "[" << endl;
    // 重新安排下一次清空文件的时间
    Simulator::Schedule(Seconds(10), &ClearFile, ref(outputFile));
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

//提取并打印指定的网络设备集合中每个设备的传输功率范围
void PrintTxPower(const NetDeviceContainer& devices,const string& devicesname) {
    cout<<devicesname<<endl;
    for (uint32_t i = 0; i < devices.GetN(); ++i) {
        Ptr<NetDevice> device = devices.Get(i);
        Ptr<WifiNetDevice> wifiDevice = DynamicCast<WifiNetDevice>(device);
        Ptr<YansWifiPhy> phy = DynamicCast<YansWifiPhy>(wifiDevice->GetPhy());
        double txPowerStart = phy->GetTxPowerStart();
        double txPowerEnd = phy->GetTxPowerEnd();

        cout << "\tNode " << i << " Tx Power: " << txPowerStart << " to " << txPowerEnd << " dBm" << endl;
    }
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

YansWifiPhyHelper CreateYansWifiPhyHelper(double startPower,double endPower){
    YansWifiPhyHelper wifiPhy = YansWifiPhyHelper();
    wifiPhy.Set("TxPowerStart", DoubleValue(startPower));
    wifiPhy.Set("TxPowerEnd", DoubleValue(endPower));
    return wifiPhy;
}

//设置wifi信道
void SetWifiChannel(YansWifiPhyHelper &deviceName)
{
    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
    deviceName.SetChannel(wifiChannel.Create());
}

void SetMobilityModelRandomWalk2d(MobilityHelper& mobilityModel)
{
    mobilityModel.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
                                   "Bounds", RectangleValue(Rectangle(-2000000, 2000000, -6000000, 6000000)),
                                   "Distance", DoubleValue(10),
                                   "Time", TimeValue(Seconds(1)));
}

void SetMobilityModelConstantPosition(MobilityHelper& mobilityModel){
    mobilityModel.SetMobilityModel("ns3::ConstantPositionMobilityModel");
}

// 创建MAC层助手,并设置为AD-Hoc模式
WifiMacHelper CreateWifiMacHelper(string macModel)
{
    // WifiMacHelper red_soldierwifiMac; 
    // red_soldierwifiMac.SetType("ns3::AdhocWifiMac");
    WifiMacHelper wifiMac;
    wifiMac.SetType(macModel);
    return wifiMac;
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

void DataInfoFile(Ptr<Node> start, Ptr<Node> &target, uint32_t size,long long time, const string &mediaType, ofstream& outputFile){
    if(second == 1){
        outputFile << "[" << endl;
        second = 0;
    }
    outputFile << "{";//开始第一个json对象
    outputFile << "\"StartTimestamp\": \"" << "NULL\", ";
    outputFile << "\"ArriveTimestamp\": \"" << time << "s\", ";
    outputFile << "\"PkgType\": \"002\",";
    outputFile << "\"StartNodeid\": \"" << FindIdFromMap(start) << "\", ";
    outputFile << "\"StartNodeGroup\":\""<< FindFromGroupMap(FindIdFromMap(start)) <<"\",";
    outputFile << "\"StartNodeType\": \"" << nameToTypeMap[Names::FindName(start)] << "\", ";
    outputFile << "\"TargetNodeid\":\""<< FindIdFromMap(target) <<"\",";
    outputFile << "\"TargetNodeGroup\":\""<< FindFromGroupMap(FindIdFromMap(target)) <<"\",";
    outputFile << "\"TargetNodeType\": \"" << nameToTypeMap[Names::FindName(target)] << "\", ";  
    outputFile << "\"DataSize\": \"" << size << "\",";
    outputFile << "\"MediaType\": \"" << mediaType << "\",";  // 添加 MediaType
    outputFile << "\"frequency\": \"" << temp.frequency << "MHz" << "\",";
    outputFile << "\"SNR\": \"" << temp.snr << "dB" << "\",";
    outputFile << "\"MCS\": \"" << temp.mcs << "\",";
    outputFile << "\"PacketLossRate\": \"" << "NULL\",";
    outputFile << "\"Delay\": \"" << "NULL\",";
    outputFile << "\"DataRate\": \"" << "NULL\"";
    outputFile << "}";  // 结束 JSON 对象
    outputFile << "," <<endl;    
}

void startDataActiviatyInfo(){
    activiaty = 1;
    Simulator::Schedule(Seconds(25.0), &startDataActiviatyInfo);
}

void stopDataActiviatyInfo(){
    activiaty = 0;
    Simulator::Schedule(Seconds(25.0), &stopDataActiviatyInfo);
}


void dataActiviatyInfoFile(NodeContainer Nodes, ofstream& outputFile){
//处理收到数据包数据   
    if(activiaty == 1){
        if(datatemp.status == 1){
            datatemp.status = 0;
            Ptr<Node> node = datatemp.nodes.Get(0);
            Ptr<MobilityModel> mobility = node->GetObject<MobilityModel>();
            Vector pos = mobility->GetPosition();
            string foundName = Names::FindName(node);
            vector<string> retrievedVector = SearchVectorData(node);
            double X,Y,Z;
            if(third == 1)
                outputFile << "[" <<endl;
            outputFile << "{";  // 开始一个 JSON 对象

            // 添加各种数据到 JSON 对象
            // outputFile << "\"Time\": \"" << Simulator::Now().GetSeconds() << "s\", ";
            outputFile << "\"Timestamp\": \"" << datatemp.timestamp << "s\", ";
            outputFile << "\"PkgType\": \"003\",";
            outputFile << "\"NodeId\": \"" << FindIdFromMap(node) << "\", ";
            outputFile << "\"NodeGroup\":\""<< FindFromGroupMap(FindIdFromMap(node)) <<"\",";
            outputFile << "\"NodeType\": \"" << nameToTypeMap[foundName] << "\", ";
            outputFile << "\"NodeName\": \"" << foundName << "\", ";
            outputFile << "\"NodeSpeed\": \""  << "1m/s\", ";
            
            if((foundName != "red_commandPostNodes.Get(0)") && (foundName != "blue_commandPostNodes.Get(0)"))
            {
                outputFile << "\"NodeTxPower\": \"" << NodePower(node) << "dBm\", ";
            }

            // outputFile << "\"Position\": {\"x\": " << pos.x << ", \"y\": " << pos.y << ", \"z\": " << pos.z << "}, ";
            // XYZ2LLA((pos.x*100)-1210135.1685510000,(pos.y*100)+5045472.0347960000,(pos.z*100)+3700382.7212280000,X,Y,Z);//以第一个红色士兵节点为原点并且将坐标等比缩小了100倍
            // XYZ2LLA((pos.x)-1210135.1685510000,(pos.y)+5045472.0347960000,(pos.z)+3700382.7212280000,X,Y,Z);//以第一个红色士兵节点为原点
            XYZ2LLA(pos.x*10,pos.y*10,pos.z*10,X,Y,Z);//直接采用原始未经处理的地理坐标转换的笛卡尔坐标系坐标
            outputFile << "\"Position\": {\"x\": "  << fixed << setprecision(6)<< X << ", \"y\": " << Y << ", \"z\": " << Z << "}, ";//输出地理坐标
            outputFile.unsetf(ios_base::fixed);
            outputFile.precision(streamsize(-1));
            outputFile << "\"Terrain\": \"" << retrievedVector[0] << "\", ";
            outputFile << "\"Weather\": \"" << retrievedVector[1] << "\"";
            outputFile << ",\"frequency\": \"" << datatemp.frequency << "MHz" << "\",";
            outputFile << "\"SNR\": \"" << datatemp.snr << "dB" << "\",";
            // outputFile << "\"MCS\": \"" << datatemp.mcs.GetModulationClass() << "\"";
            outputFile << "\"MCS\": \"" << datatemp.mcs.GetUniqueName()<< "\"";
            outputFile << "}";  // 结束 JSON 对象
            if(third == 1) {
                third = 0;
                outputFile << "," <<endl;
            }else{
                outputFile <<","<<endl;  // 换行
            }
        }else{
    //处理非收到数据包数据 
            for(uint16_t i = 0; i < Nodes.GetN(); ++i) 
            {
                Ptr<MobilityModel> mobility = Nodes.Get(i)->GetObject<MobilityModel>();
                Vector pos = mobility->GetPosition();
                string foundName = Names::FindName(Nodes.Get(i));
                vector<string> retrievedVector = SearchVectorData(Nodes.Get(i));
                // 获取当前时间点
                auto now = chrono::system_clock::now();

                // 转换为时间戳
                auto timestamp = chrono::duration_cast<chrono::milliseconds>(now.time_since_epoch()).count();
                double X,Y,Z;
                if(third == 1)
                    outputFile << "[" <<endl;
                outputFile << "{";  // 开始一个 JSON 对象

                // 添加各种数据到 JSON 对象
                // outputFile << "\"Time\": \"" << Simulator::Now().GetSeconds() << "s\", ";
                // outputFile << "\"Timestamp\": \"" << timestamp << "s\", ";
                outputFile << "\"Timestamp\": \"" << timestamp << "s\", ";
                outputFile << "\"PkgType\": \"003\",";
                outputFile << "\"NodeId\": \"" << FindIdFromMap(Nodes.Get(i)) << "\", ";
                outputFile << "\"NodeGroup\":\""<< FindFromGroupMap(FindIdFromMap(Nodes.Get(i))) <<"\",";
                outputFile << "\"NodeType\": \"" << nameToTypeMap[foundName] << "\", ";
                outputFile << "\"NodeName\": \"" << foundName << "\", ";
                outputFile << "\"NodeSpeed\": \""  << "1m/s\", ";
                
                if((foundName != "red_commandPostNodes.Get(0)") && (foundName != "blue_commandPostNodes.Get(0)"))
                {
                    outputFile << "\"NodeTxPower\": \"" << NodePower(Nodes.Get(i)) << "dBm\", ";
                }

                // outputFile << "\"Position\": {\"x\": " << pos.x << ", \"y\": " << pos.y << ", \"z\": " << pos.z << "}, ";
                // XYZ2LLA((pos.x*100)-1210135.1685510000,(pos.y*100)+5045472.0347960000,(pos.z*100)+3700382.7212280000,X,Y,Z);//以第一个红色士兵节点为原点并且将坐标等比缩小了100倍
                // XYZ2LLA((pos.x)-1210135.1685510000,(pos.y)+5045472.0347960000,(pos.z)+3700382.7212280000,X,Y,Z);//以第一个红色士兵节点为原点
                XYZ2LLA(pos.x*10,pos.y*10,pos.z*10,X,Y,Z);//直接采用原始未经处理的地理坐标转换的笛卡尔坐标系坐标
                outputFile << "\"Position\": {\"x\": "  << fixed << setprecision(6)<< X << ", \"y\": " << Y << ", \"z\": " << Z << "}, ";//输出地理坐标
                outputFile.unsetf(ios_base::fixed);
                outputFile.precision(streamsize(-1));
                outputFile << "\"Terrain\": \"" << retrievedVector[0] << "\", ";
                outputFile << "\"Weather\": \"" << retrievedVector[1] << "\"";
                if(datatemp.snr&&datatemp.status==1){
                    outputFile << ",\"frequency\": \"" << datatemp.frequency << "\",";
                    outputFile << "\"SNR\": \"" << datatemp.snr << "\",";
                    outputFile << "\"MCS\": \"" << datatemp.mcs << "\"";
                }else{
                    outputFile << ",\"frequency\": \"" << "NULL" << "\",";
                    outputFile << "\"SNR\": \"" << "NULL" << "\",";
                    outputFile << "\"MCS\": \"" << "NULL" << "\"";
                }
                outputFile << "}";  // 结束 JSON 对象
                if(third == 1) {
                    third = 0;
                    outputFile << "," <<endl;
                }else{
                    outputFile <<","<<endl;  // 换行
                }
            }
            // 每秒记录
            Simulator::Schedule(Seconds(25), &dataActiviatyInfoFile, Nodes, ref(outputFile));
        }
    }
}

void ReceivePacket (std::string context, Ptr<const Packet> packet, const Address &from)
{
    std::regex rgx("/NodeList/(\\d+)/"); // 正则表达式匹配节点 ID
    std::smatch match;

    if (std::regex_search(context, match, rgx) && match.size() > 1)
    {
        int nodeId = std::stoi(match.str(1)); // 获取节点 ID

        // 获取节点并从中检索 IP 地址
        Ptr<Node> node = NodeList::GetNode(nodeId);
        Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
        Ipv4Address localAddr = ipv4->GetAddress(1, 0).GetLocal();

        // 获取当前时间点
        auto now = chrono::system_clock::now();

        // 转换为时间戳
        auto timestamp = chrono::duration_cast<chrono::milliseconds>(now.time_since_epoch()).count();

        MediaType mediaType = GenerateRandomMediaType();
        string mediaTypeString = MediaTypeToString(mediaType);

        // 打印信息
        InetSocketAddress addr = InetSocketAddress::ConvertFrom (from);
        if(addr.GetIpv4 ()!=localAddr){
            //DataInfoFile(FindFromMap(ipToIdMap[addr.GetIpv4 ()]), node, packet->GetSize(), timestamp, ref(dataputFile));
            DataInfoFile(FindFromMap(ipToIdMap[addr.GetIpv4()]), node, packet->GetSize(), timestamp, mediaTypeString, ref(dataputFile));
        }
    }else{
        cout << "Failed to extract node ID from context: " << context << std::endl;
    }
}

void StartSpecificTransmission(uint32_t sourceIndex, uint32_t targetIndex, NodeContainer &sourceNodes, Ipv4InterfaceContainer &targetInterfaces, uint16_t port, Time duration) {
    Ptr<Node> sourceNode = sourceNodes.Get(sourceIndex);
    Ipv4Address targetAddress = targetInterfaces.GetAddress(targetIndex);

    OnOffHelper onOffHelper("ns3::UdpSocketFactory", Address(InetSocketAddress(targetAddress, port)));
    onOffHelper.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    onOffHelper.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));//间隔1s发一次连续1s的信息
    // onOffHelper.SetAttribute("DataRate", DataRateValue(DataRate("500kb/s")));
    onOffHelper.SetAttribute("DataRate", StringValue("50kb/s"));
    onOffHelper.SetAttribute("PacketSize", UintegerValue(1024));

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
    temp.timestamp = chrono::duration_cast<chrono::milliseconds>(now.time_since_epoch()).count();
    temp.frequency = channelFreqMhz;
    temp.nodes = tempNodes;
    temp.status = 1;
    temp.snr = snr;
    temp.mcs = txVector.GetMode();
    
    //给用户活跃度文件添加数据的结构体
    datatemp.timestamp = chrono::duration_cast<chrono::milliseconds>(now.time_since_epoch()).count();
    datatemp.frequency = channelFreqMhz;
    datatemp.nodes = tempNodes;
    datatemp.status = 1;
    datatemp.snr = snr;
    datatemp.mcs = txVector.GetMode();
    // Simulator::Schedule(Seconds(0.0),&LogJsonPosition,tempNodes,ref(outputFile));
    Simulator::Schedule(Seconds(0.0),&dataActiviatyInfoFile,tempNodes,ref(dataActiviaty));
}

void PrintRoutingTable(std::string filePath, Time printInterval) {
    // 创建新的 OutputStreamWrapper 对象以覆盖原有文件
    Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper>(filePath, ios::app);

    // 打印当前时刻的路由表
    Ipv4RoutingHelper::PrintRoutingTableAllAt(Simulator::Now(), routingStream);

    // 安排下一次打印
    Simulator::Schedule(printInterval, &PrintRoutingTable, filePath, printInterval);
}

int main (int argc, char *argv[])
{
    CommandLine cmd;
    cmd.Parse(argc, argv);
    //创建interface实例
    auto interface = Ns3AiMsgInterface::Get();
    interface->SetIsMemoryCreator(false);
    interface->SetUseVector(false);
    interface->SetHandleFinish(true);
    Ns3AiMsgInterfaceImpl<EnvStruct, ActStruct>* msgInterface =
        interface->GetInterface<EnvStruct, ActStruct>();
    //读取节点位置信息
    vector<Data> data = readData("/home/ns3/ns-allinone-3.40/ns-3.40/contrib/ai/examples/a-plus-b/use-msg-stru/red_position.txt");

    //输出内容保存的json文件
    outputFile.open("node-movement-log.json");
    dataputFile.open("data-translate-log.json");
    dataActiviaty.open("data-activiaty-log.json");
    fstream rewriteFile("node-movement-log.json", ios::in | ios::out | ios::ate);
    fstream rewriteDataFile("data-translate-log.json", ios::in | ios::out | ios::ate);
    fstream rewriteDataActiviatyFile("data-activiaty-log.json", ios::in | ios::out | ios::ate);

    //读取位置，天气信息
    vector<DataRow> fileData = ReadCsvFile("/home/ns3/ns-allinone-3.40/ns-3.40/contrib/ai/examples/a-plus-b/use-msg-stru/red_info.txt");

    // 设置全局变量以使用实时模拟器和启用校验和
    GlobalValue::Bind ("SimulatorImplementationType", StringValue ("ns3::RealtimeSimulatorImpl"));
    GlobalValue::Bind ("ChecksumEnabled", BooleanValue (true));

    //创建节点数量
    NodeContainer nodes = CreateNode(28);
    
    //设置wifi的标准
    WifiStandard standard = WIFI_STANDARD_80211n;
    WifiHelper wifi = CreateWifiHelper(standard);

    //创建物理层助手
    double startPower, endPower;
    endPower = 50;
    startPower = endPower;
    YansWifiPhyHelper nodesWifiPhy = CreateYansWifiPhyHelper(startPower, endPower);

    //创建信道助手,并设置信道
    SetWifiChannel(nodesWifiPhy);

    //创建MAC层助手,并设置为AD-Hoc模式
    WifiMacHelper nodesWifiMac = CreateWifiMacHelper("ns3::AdhocWifiMac");

    NetDeviceContainer nodeDevices = wifi.Install(nodesWifiPhy, nodesWifiMac, nodes);

    //设置移动模型
    MobilityHelper mobility;

    //给节点添加对应的地理位置等属性
    Ptr<ListPositionAllocator> nodesPositionAlloc = ReadToPositionAlloc(data, nodes);

    mobility.SetPositionAllocator(nodesPositionAlloc);

    //给节点添加对应的地形以及天气属性
    for (const auto& row : fileData) {
        Ptr<Node> node = FindFromMap(row.id);
        vector<string> data = {row.terrain,row.weather};
        AddVectorData(node, data);
    }

    mobility.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
                                "Bounds", RectangleValue(Rectangle(-20000000, 20000000, -60000000, 60000000)),
                               "Distance", DoubleValue(10/10.0),
                               "Time", TimeValue(Seconds(1)));
    mobility.Install (nodes);

    InternetStackHelper stack;
    AodvHelper aodv;
    stack.SetRoutingHelper(aodv);
    stack.Install(nodes);

    Ipv4AddressHelper address;
    address.SetBase("10.1.8.0","255.255.255.0");
    Ipv4InterfaceContainer nodesInterfaces = address.Assign(nodeDevices);

/*
    Time printInterval = Seconds(30);
    string filePath = "./scratch/aodv.routes";

    // 在初次延迟后开始打印路由表
    Simulator::Schedule(printInterval, &PrintRoutingTable, filePath, printInterval);
*/

    // 设置Packet Sink应用并连接回调
    uint16_t port = 9; // UDP端口
    PacketSinkHelper packetSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), port));
    ApplicationContainer sinkApps = packetSinkHelper.Install (nodes);
    sinkApps.Start (Seconds (0.0));

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
    Time startTransmissionTime = Seconds(0.0);
    Time transmissionDuration = Seconds(3.0);
    Simulator::Schedule(startTransmissionTime, &StartSpecificTransmission, sourceIndex, targetIndex, nodes, nodesInterfaces, port, transmissionDuration);            
    startTransmissionTime = startTransmissionTime + transmissionDuration;
    for (uint32_t i = 0; i < nodes.GetN(); i++)
    {
        for (uint32_t j = 0; j < nodes.GetN(); j++)
        {
            sourceIndex = i;
            targetIndex = j;
            if(i != j && !(i == 0 && j==1)){
                Simulator::Schedule(startTransmissionTime, &StartSpecificTransmission, i, j, nodes, nodesInterfaces, port, transmissionDuration);            
                startTransmissionTime = startTransmissionTime + transmissionDuration;
            }
        }
    }

    //绑定MonitorSnifferRx生成snr等数据
    for (uint32_t i = 0; i < nodes.GetN(); ++i) { 
        Ptr<NetDevice> device = nodes.Get(i)->GetDevice(0);
        Ptr<WifiNetDevice> wifiDevice = DynamicCast<WifiNetDevice>(device);
        Ptr<YansWifiPhy> phy = DynamicCast<YansWifiPhy>(wifiDevice->GetPhy());
        tempNode = nodes.Get(i);
        phy->TraceConnectWithoutContext("MonitorSnifferRx", MakeBoundCallback(&MonitorSnifferRx, tempNode));
    } 
    startDataActiviatyInfo();
    Simulator::Schedule(Seconds(1.0), &dataActiviatyInfoFile, nodes, ref(dataActiviaty));
    Simulator::Schedule(Seconds(1.0), &LogJsonPosition, nodes, ref(outputFile));

    Simulator::Schedule(Seconds(10), &ClearFile, ref(outputFile));//设置刷新文件的时间
    //关闭json文件
    Simulator::Schedule(Seconds(60.0*5), &Closefile,ref(outputFile));
    Simulator::Schedule(Seconds(60.0*5), &Closefile,ref(dataputFile));
    Simulator::Schedule(Seconds(60.0*5), &Closefile,ref(dataActiviaty));

    if (!rewriteFile.is_open()) {
        std::cerr << "无法打开文件。" << std::endl;
        return -1;
    }
    if (!rewriteDataFile.is_open()) {
        std::cerr << "无法打开文件。" << std::endl;
        return -1;
    }
    if (!rewriteDataActiviatyFile.is_open()) {
        std::cerr << "无法打开文件。" << std::endl;
        return -1;        
    }

    // 调用函数修改文件
    Simulator::Schedule(Seconds(60.0*5+3),&ModifyJsonFile,ref(rewriteFile));
    Simulator::Schedule(Seconds(60.0*5+3),&ModifyJsonFile,ref(rewriteDataFile));
    Simulator::Schedule(Seconds(60.0*5+3),&ModifyJsonFile,ref(rewriteDataActiviatyFile));

    Simulator::Stop(Seconds(60.0*5+60));

    // 运行仿真
    Simulator::Run();
    Simulator::Destroy();
    
    rewriteFile.close();
    tempFile.close();
    rewriteDataFile.close();
    rewriteDataActiviatyFile.close();

    return 0;
}