# file_parser.py

import re
path = "/home/ns3/ns-allinone-3.40/ns-3.40/contrib/ai/examples/a-plus-b/use-msg-stru/env_init/"

def parse_file(file_path=path + '20231206-123436265.txt'):
    with open(file_path, 'r') as file:
        text_data = file.read()

    # 定义正则表达式
    pattern = re.compile(r'(Model\.\d+-\d+),(\d+),(\d+)')
    # 查找匹配的模型信息
    matches = pattern.findall(text_data)

    return matches
