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

#ifndef APB_H
#define APB_H

// #include <cstdint>
// #include <string>
// enum NodeType {
//     SOLDIER = 0,
//     TANK = 1,
//     ENGINEER = 2,
//     TRANSPORT = 3,
//     MEDICAL = 4,
//     COMMAND_TENT = 5,
//     RADAR = 6,
//     RADIO = 7
// };

// // 定义映射关系的函数
// int getNodeType(const std::string& nodetype) {
//     static std::unordered_map<std::string, NodeType> nameToTypeMap = {
//         {"士兵", SOLDIER},
//         {"坦克车", TANK},
//         {"工程车", ENGINEER},
//         {"运输车", TRANSPORT},
//         {"医疗车", MEDICAL},
//         {"指挥帐篷", COMMAND_TENT},
//         {"雷达车", RADAR},
//         {"电台", RADIO}
//     };
//     if (nameToTypeMap.find(nodetype) != nameToTypeMap.end()) {
//         return static_cast<int>(nameToTypeMap[nodetype]);
//     } else {
//         // 返回一个特殊值（例如 -1）表示未知的节点类型
//         return -1;
//     }
// }

// int getMcsValue(const std::string& mcsString) {
//         static std::unordered_map<std::string, int> indexToMcsMap = {
//         {"HtMcs0", 0},
//         {"HtMcs1", 1},
//         {"HtMcs2", 2},
//         {"HtMcs3", 3},
//         {"HtMcs4", 4},
//         {"HtMcs5", 5},
//         {"HtMcs6", 6},
//         {"HtMcs7", 7}
//     };
//     auto it = indexToMcsMap.find(mcsString);
//     if (it != indexToMcsMap.end()) {
//         return it->second;
//     } else {
//         // 返回一个特殊值（例如 -1）表示未知的 MCS 字符串
//         return -1;
//     }
// }

// std::string getMcsString(int mcsValue) {
//     static std::unordered_map<int, std::string> mcsToIndexMap = {
//         {0, "HtMcs0"},
//         {1, "HtMcs1"},
//         {2, "HtMcs2"},
//         {3, "HtMcs3"},
//         {4, "HtMcs4"},
//         {5, "HtMcs5"},
//         {6, "HtMcs6"},
//         {7, "HtMcs7"}
//     };

//     auto it = mcsToIndexMap.find(mcsValue);
//     if (it != mcsToIndexMap.end()) {
//         return it->second;
//     } else {
//         // 返回一个特殊值（例如空字符串）表示未知的 MCS 值
//         return "";
//     }
// }
//     string type=nameToTypeMap[Names::FindName(node)];//获取当前节点TYPE
//     /**
//      * CPP发送当前状态信息到PYTHON
//     */
//     msgInterface->CppSendBegin();
//     msgInterface->GetCpp2PyStruct()->snr = snr;
//     msgInterface->GetCpp2PyStruct()->delay = delay;
//     msgInterface->GetCpp2PyStruct()->tput = tput;
//     msgInterface.GetCpp2PyStruct()->current_power = power;
//     msgInterface.GetCpp2PyStruct()->nodetype = getNodeType(type);
//     msgInterface.GetCpp2PyStruct()->mcs = getMcsValue(mcsString);
//     msgInterface->CppSendEnd();

//     msgInterface->CppRecvBegin();
//     next_power = msgInterface->GetPy2CppStruct()->next_power;
//     next_mcs = msgInterface->GetPy2CppStruct()->next_mcs;
//     msgInterface->CppRecvEnd();
//     ChangeSingleNodeDataRate(node, getMcsString(next_mcs));
//     SetTxPower(node, next_power);


struct EnvStruct
{
    uint32_t current_channel;//当前信道
    uint32_t current_power;//当前功率
    uint32_t current_disturbed_channel;//当前干扰信道
    uint32_t id;//
    uint32_t delay;//总延时
    uint32_t tput;//总吞吐量
    float snr;//总信噪比
    uint8_t terrain;//当前场景地形
    uint8_t weather;//当前场景天气
    uint8_t nodetype;//该节点的类别
    uint8_t mcs;//当前MCS
};

struct ActStruct
{
    uint32_t next_channel;//下一信道
    uint32_t next_power;//下一功率
    uint32_t next_mcs=1;//下一mcs
};


#endif // APB_H
