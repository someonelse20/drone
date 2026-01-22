import json 

data = None

with open('../config.json', 'r', encoding='utf-8') as file:
    data = json.loads(file.read())

with open('../config.json', 'w', encoding='utf-8') as file:
    json.dump(data, file, indent=4)

