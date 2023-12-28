import torch
from torch import nn
import torchvision.models as models

# 引入rest50模型
net = models.resnet50()
net.load_state_dict(torch.load("./resnet50-19c8e357.pth"))


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


model = WeatherModel(net)
