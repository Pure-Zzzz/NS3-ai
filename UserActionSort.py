import os
import numpy as np
import skfuzzy as fuzz
import matplotlib.pyplot as plt
from sklearn.decomposition import PCA
import matplotlib
from sklearn.metrics.pairwise import euclidean_distances
import json
import re
from numpy import *

'''
NS-3生成的节点活跃度数据文件预处理
Input:.json文件
output:列表（节点数量,7） # 7表示最后聚类使用的特征
'''

# 指定 JSON 文件的路径
json_file_path = '/home/ns3/ns-allinone-3.40/ns-3.40/activity-data-log.json'
def extract_numbers_from_string(input_string):
    # 使用正则表达式提取字符串中的数字部分
    numbers = re.findall(r'\d+', input_string)
    # 将提取到的数字部分连接成一个字符串
    result = ''.join(numbers)
    return float(result)

def extract_numbers(input_string, index):
    str = input_string[:index]
    return float(str)

def convert_node_type(node_type):
    node_types = {
        '电台': 1,
        '士兵': 2,
        '坦克车': 3,
        '雷达车': 4,
        '运输车': 5,
        '医疗车': 6,
        '工程车': 7
    }
    return node_types.get(node_type, 8)

def convert_mcs(mcs):
    if 'Dsss' in mcs:
        return 1
    elif 'ERPOfdm' in mcs:
        return 2
    else:
        return 3

def pre_data():
    original_list = []
    try:
        with open(json_file_path, 'r', encoding='utf-8') as file:
            data = json.load(file)
            if isinstance(data, (list, dict)):
                for item in data:
                    item['dataNum'] = 1
                    del_keys = ['Timestamp', 'PkgType', 'NodeGroup', 'NodeName', 'Position', 'Terrain', 'Weather', 'frequency']
                    for key in del_keys:
                        item.pop(key, None)
                    item['NodeType'] = convert_node_type(item.get('NodeType', ''))
                    item['key'] = convert_mcs(item.get('MCS', ''))
                    item['MCS'] = extract_numbers_from_string(item.get('MCS', ''))
                    item['NodeSpeed'] = float(item['NodeSpeed'])
                    item['NodeTxPower'] = float(item['NodeTxPower'])
                    item['NodeTP'] = float(item['NodeTP'])
                    item['SNR'] = float(item['SNR'])
                    original_list.append(item)
            else:
                print("JSON 数据不是列表或字典，无法迭代处理。")
    except json.decoder.JSONDecodeError as e:
        print(f"JSONDecodeError: {e}")
    except FileNotFoundError:
        print(f"文件 '{json_file_path}' 未找到。")
    except Exception as e:
        print(f"发生错误: {e}")
    return original_list

def standardScaler():
    # 初步处理原始json文件
    origi_list = pre_data()
    unique_node_ids = []
    # print(len(origi_list))
    # 统计
    # 创建一个字典，用于存储相同'id'属性的行的合并结果
    merged_dict = {}
    # 循环遍历原始列表
    for item in origi_list:
        current_id = item['NodeId']
        if current_id not in unique_node_ids:
            unique_node_ids.append(current_id)
        # 如果'id'属性已经在merged_dict中，则合并其他属性的值
        if current_id in merged_dict:
            for key, value in item.items():
                # 跳过'id'属性，因为已经在之前处理过了
                if key == 'dataNum':
                    # 如果属性值是可加的（例如数字），则将其相加，否则连接字符串
                    merged_dict[current_id][key] = merged_dict[current_id].get(key, 0) + value
                if key == 'SNR':
                    # 如果属性值是可加的（例如数字），则将其相加，否则连接字符串
                    merged_dict[current_id][key] = merged_dict[current_id].get(key, 0) + value
                if key == 'NodeTxPower':
                    # 如果属性值是可加的（例如数字），则将其相加，否则连接字符串
                    merged_dict[current_id][key] = merged_dict[current_id].get(key, 0) + value
                if key == 'NodeTP':
                    # 如果属性值是可加的（例如数字），则将其相加，否则连接字符串
                    merged_dict[current_id][key] = value
                if key == 'MCS':
                    # 如果属性值是可加的（例如数字），则将其相加，否则连接字符串
                    merged_dict[current_id][key] = merged_dict[current_id].get(key, 0) + value
                if key == 'NodeSpeed':
                    # 如果属性值是可加的（例如数字），则将其相加，否则连接字符串
                    merged_dict[current_id][key] = merged_dict[current_id].get(key, 0) + value

        # 如果'id'属性不在merged_dict中，添加新的字典项
        else:
            merged_dict[current_id] = dict(item)
    # print('merged_dict=',merged_dict)

    # 将合并结果转换为列表
    merged_list = list(merged_dict.values())
    # print('长度为=',len(merged_list))
    key_number = len(merged_list)
    vector = zeros((key_number, 7))
    i = 0
    # vector=[id,Nodepower,SNR,MCS,DataNum]
    for key in merged_list:
        num = float(key['dataNum'])
        vector[i][0] = key['NodeType']
        vector[i][1] = round(key['NodeSpeed'] / num, 2)
        vector[i][2] = round(key['NodeTxPower'] / num, 2)
        vector[i][3] = round(key['NodeTP'], 2)
        vector[i][4] = round(key['SNR'] / num, 2)
        vector[i][5] = round(key['MCS'] / num, 2)
        vector[i][6] = num
        i = i + 1
    from sklearn.preprocessing import StandardScaler

    scaler = StandardScaler()
    normalized_data = scaler.fit_transform(vector)

    return normalized_data, unique_node_ids

# 设置matplotlib支持中文的字体
# matplotlib.rcParams['font.sans-serif'] = ['SimHei']  # 指定默认字体为黑体
# matplotlib.rcParams['axes.unicode_minus'] = False  # 解决负号'-'显示为方块的问题
plt.rcParams['font.sans-serif'] = ['WenQuanYi Zen Hei']
plt.rcParams['axes.unicode_minus'] = False  # 确保负号显示正常


def fuzzy_c_means_and_plot(data, cluster_number, max_iter=500, m=2.0, error=0.005):
    """
        Fuzzy C-Means Clustering Algorithm
        :param data: 数据集，形状为 N x M（N个样本，M个特征）
        :param cluster_number: 聚类数目
        :param max_iter: 最大迭代次数
        :param m: 模糊度指数，默认为2
        :param error: 收敛误差阈值
        :return: cntr: 聚类中心, u: 隶属度矩阵, u0: 初始隶属度矩阵, d: 距离矩阵, jm: 目标函数值, p: 迭代次数, fpc: Fuzzy Partition Coefficient
    """
    cntr, u, u0, d, jm, p, fpc = fuzz.cluster.cmeans(data.T, cluster_number, m, error, max_iter)
    fig, ax = plt.subplots()
    ax.set_title('节点活跃度聚类')
    # 绘制原始数据点
    for j in range(cluster_number):
        ax.plot(data[:, 0], data[:, 1], 'o', color='gray', markersize=5)
    # 绘制聚类中心
    for pt in cntr:
        ax.plot(pt[0], pt[1], 'rs')
    # 绘制隶属度等级
    for point_idx in range(len(data)):
        for cluster_idx in range(cluster_number):
            ax.plot([data[point_idx][0], cntr[cluster_idx][0]],
                    [data[point_idx][1], cntr[cluster_idx][1]],
                    color=f'blue',
                    lw=u[cluster_idx][point_idx] * 2)  # 线宽与隶属度成比例
    plt.savefig('/home/ns3/project/optimize/useractivate.png')
    plt.close()
    # plt.show()
    return cntr, u, u0, d, jm, p, fpc

def find_dominant_clusters(u):
    """
    根据隶属度矩阵确定每个样本最可能属于的类别。
    :param u: 隶属度矩阵
    :return: 最可能的类别列表
    """
    return np.argmax(u, axis=0)

def get_closest_samples(cluster_centers, labels, data):
    """
    查找每个簇的最接近中心节点的样本
    :cluster_centers : 中心节点
    :labels: 每个样本最可能的所属的簇
    :data: 所有参与聚类的样本
    """
    closest_samples = []
    for cluster_id in range(len(cluster_centers)):
        cluster_indices = np.where(labels == cluster_id)[0]
        cluster_center = cluster_centers[cluster_id]
        distances = euclidean_distances(data[cluster_indices], [cluster_center])
        closest_sample_index = cluster_indices[np.argmin(distances)]
        closest_samples.append(data[closest_sample_index])
    return closest_samples

def execute_cluster_operations():
    cluster_indices = {}
    
    # 数据预处理
    data, id_list = standardScaler()
    # 执行聚类并绘制结果
    cntr, u, _, _, _, _, _ = fuzzy_c_means_and_plot(data, 5)
    # 确定每个样本最可能属于的类别
    dominant_clusters = find_dominant_clusters(u)
    # 查找每个簇的中心节点
    closest_samples = get_closest_samples(cntr, dominant_clusters, data)
    print("每个簇最接近中心节点的样本：")

    # # 打印结果
    for i, cluster in enumerate(dominant_clusters):
        if cluster not in cluster_indices:
            cluster_indices[cluster] = []
        cluster_indices[cluster].append(i)
    # 按照 cluster 排序的键值对列表
    sorted_clusters = sorted(cluster_indices.items(), key=lambda x: x[0])


    with open('/home/ns3/project/optimize/output.txt', 'w', encoding='utf-8') as f:
        # 写入最近样本的结果
        clusterCenterNode = []
        clusterNodes = []
        tmp = []
        for i, sample in enumerate(closest_samples):
            center_nodes = np.where(np.all(data == sample, axis=1))[0]
            print("Cluster {} 的中心节点为：{}".format(i, id_list[center_nodes[0]]))
            f.write("Cluster {} 的中心节点为：{}\n".format(i, id_list[center_nodes[0]]))
            clusterCenterNode.append(center_nodes[0])
        f.write("\n")

        # 写入按 cluster 排序的结果
        sorted_clusters = sorted(cluster_indices.items(), key=lambda x: x[0])
        for cluster, indices in sorted_clusters:
            node_ids = [id_list[index] for index in indices]
            for index in indices:
                tmp.append(index)
            clusterNodes.append(tmp)
            tmp = []
            print(f"Cluster {cluster} 对应的节点有: {node_ids}")
            f.write(f"Cluster {cluster} 对应的节点有: \n{node_ids}\n")
    with open('/home/ns3/project/optimize/output_opt.txt', 'w', encoding='utf-8') as f:
        f.write('{}'.format(clusterCenterNode))
        f.write('\n')
        for a in clusterNodes:
            f.write('{}\n'.format(a))
    command = "sshpass -p 123 scp /home/ns3/project/optimize/output.txt alex@192.168.56.100:/C:/test/bin_x64/plugins/com_proj_networksimvisualization/data/useractivity/useractivate.txt"
    exit_code = os.system(command)
    command = "sshpass -p 123 scp /home/ns3/project/optimize/useractivate.png alex@192.168.56.100:/C:/test/bin_x64/plugins/com_proj_networksimvisualization/data/useractivity/useractivate.png"
    exit_code = os.system(command)
    # 检查命令执行结果
    if exit_code == 0:
        print("替换活跃度命令执行成功！")
    else:
        print("命令活跃度失败！")

if __name__=='__main__':
    execute_cluster_operations()