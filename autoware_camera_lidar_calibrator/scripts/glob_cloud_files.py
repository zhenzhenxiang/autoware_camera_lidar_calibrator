import os
from pathlib import Path

dir = '/home/zhenzhen/projects/YSG/data_parsing/sq_data_reader/data/lidar_data_reader'
output_file = os.path.join(dir, '../clouds.txt')

files = list()

for filename in Path(dir).rglob('*.ply'):
    files.append(filename)

total_num = len(files)
print('Total num: ', total_num)

files.sort()

with open(output_file, "w") as text_file:
    for i in range(total_num):
        text_file.write(files[i].as_posix() + '\n')

print('Finished!')
