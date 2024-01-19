#ifndef APB_H
#define APB_H

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
    uint32_t next_mcs;//下一mcs
};


#endif // APB_H
