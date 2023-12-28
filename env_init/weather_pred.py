import torch
from torch import nn
from torchvision import models, transforms
from PIL import Image
path = "/home/ns3/ns-allinone-3.40/ns-3.40/contrib/ai/examples/a-plus-b/use-msg-stru/env_init"
net = models.resnet50()
net.load_state_dict(torch.load(path+"/resnet50-19c8e357.pth"))
class WeatherModel(nn.Module):
    def __init__(self, net):
        super(WeatherModel, self).__init__()
        # resnet50
        self.net = net
        self.relu = nn.ReLU()
        self.dropout = nn.Dropout(0.1)
        self.fc = nn.Linear(1000, 4)
        self.output = nn.Softmax(dim=1)

    def forward(self, x):
        x = self.net(x)
        x = self.relu(x)
        x = self.dropout(x)
        x = self.fc(x)
        x = self.output(x)
        return x

class Common:
    imageSize = (224, 224)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    labels = {0: "rainy", 1: "snow", 2: "sunny", 3: "thunder"}

def predict(imagePath):
    '''
    预测函数
    :param imagePath: 图片路径
    :param modelPath: 模型路径
    :return:
    '''
    model = WeatherModel(net)
    # 1. 读取图片
    image = Image.open(imagePath)
    # 如果图像只有一个通道（灰度图像），转换为RGB
    if image.mode == 'L':
        image = image.convert('RGB')
    # 2. 进行缩放
    image = image.resize(Common.imageSize)
    # image.show()
    # 3. 加载模型
    model = torch.load(path + '/weather_rec.pth')
    model = model.to(Common.device)
    # 4. 转为tensor张量
    transform = transforms.ToTensor()
    x = transform(image)
    x = torch.unsqueeze(x, 0)  # 升维
    x = x.to(Common.device)
    # 5. 传入模型
    output = model(x)
    # 6. 使用argmax选出最有可能的结果
    output = torch.argmax(output)
    # print("预测结果：", Common.labels[output.item()])
    return Common.labels[output.item()]
