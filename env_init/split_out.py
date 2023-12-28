import os
import shutil
import random
import torch.nn.functional as F
from collections import Counter
import re
import torch
from torch import nn
import torchvision.models as models
import torchvision.transforms as transforms
from PIL import Image
from weather_pred import predict
from file_parser import parse_file
import numpy as np
from unet import Unet
x_transforms = transforms.Compose([
    transforms.ToTensor(),  # -> [0,1]
    transforms.Normalize([0.5, 0.5, 0.5], [0.5, 0.5, 0.5])
    ])
def test_single_image(model, image_path, transform):
    model.eval()
    
    # 加载并预处理图像
    img_raw = Image.open(image_path)
    img_x = img_raw.convert('RGB').resize((640, 640))
    img_x = transform(img_x)
    img_x = img_x.unsqueeze(0)  # 添加批处理维度

    with torch.no_grad():
        y = model(img_x)
        # 将输出转换为概率分布
        y = F.softmax(y, dim=1)
        # 获取最大值的索引
        y = torch.argmax(y, dim=1)
        # 去除批处理和通道维度并转换为 numpy 数组
        y = torch.squeeze(y).numpy()
    return y
def process():
    # 用于测试单个图像的示例用法
    model = Unet(3, 6)
    weight = 'weights_epoch293_loss0.546.pth'
    # weight = 'weights_epoch265_loss0.608.pth'
    # weight = 'best_weights_epoch147_loss0.848.pth'
    model.load_state_dict(torch.load(weight, map_location='cpu'))

    transform = x_transforms  # 使用与数据集相同的变换

    gray_img = test_single_image(model, '20231206-123436265.png', transform)
    print("--------------完成地形分割------------------")
    
    matches = parse_file()
    id_weather_dict = {}
    # 主文件夹路径
    weather_images_folder = './all'

    # ID列表
    id_list = []
    for match in matches:
        id, _, __ = match
        id_list.append(id)
    # 遍历每个ID
    for user_id in id_list:
        # 创建以ID命名的文件夹
        user_folder = f'./weather_split/{user_id}'
        os.makedirs(user_folder, exist_ok=True)

        # 随机选择一个天气文件夹
        weather_folder = random.choice(['rainy', 'snow', 'sunny', 'thunder'])

        # 复制十张图片到用户文件夹
        for i in range(1, 11):
            # 随机选择图片文件
            image_filename = random.choice(os.listdir(os.path.join(weather_images_folder, weather_folder)))
            image_source_path = os.path.join(weather_images_folder, weather_folder, image_filename)

            # 复制图片到用户文件夹
            image_destination_path = os.path.join(user_folder, f'{user_id}_{i}.jpg')
            shutil.copy(image_source_path, image_destination_path)
        # 分类天气并找到最常见的天气类别

        weather_counter = Counter()
        for filename in os.listdir(user_folder):
            if filename.endswith(".jpg"):
                image_path = os.path.join(user_folder, filename)
                # 这里调用你的分类模型，获取天气类别
                # 假设有一个名为 classify_weather 的函数
                weather_category = predict(image_path)
                weather_counter[weather_category] += 1

        # 获取出现频率最高的天气类别
        most_common_weather = weather_counter.most_common(1)[0][0]

        # 设置文件夹属性为最常见的天气类别
        print(f"文件夹 {user_id} 的主要天气类别是：{most_common_weather}")
        id_weather_dict[user_id] = most_common_weather
        
#-----------------------

    # 加载灰度图

    # 定义灰度值到地形的映射关系
    gray_to_terrain = {
        0: '_background_',
        1: 'Mountain',
        2: 'city',
        3: 'plain',
        4: 'rural',
        5: 'forest',
    }

    # 创建保存地形信息的列表
    terrain_info_list = []

    # 遍历每个匹配项，获取对应坐标的灰度值，并映射为地形
    for match in matches:
        id_, x, y = match
        x, y = int(x), int(y)  # 坐标可能是字符串，确保转为整数

        # 获取灰度值
        gray_value = gray_img[x, y]

        # 映射为地形
        terrain = gray_to_terrain.get(gray_value, 'Unknown')

        # 保存地形信息到列表
        terrain_info_list.append(f'{id_},{terrain},{id_weather_dict[id_]}')
        
    # 将地形信息保存到文件
    with open('terrain_info1.txt', 'w') as file:
        file.write('\n'.join(terrain_info_list))

    print("Terrain information saved to 'terrain_info.txt'")

    print("任务完成！")

