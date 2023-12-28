# file_parser.py

import re

def parse_file(file_path='20231206-123436265.txt'):
    with open(file_path, 'r') as file:
        text_data = file.read()

    # 定义正则表达式
    pattern = re.compile(r'(Model\.\d+-\d+),(\d+),(\d+)')
    # 查找匹配的模型信息
    matches = pattern.findall(text_data)

    return matches
