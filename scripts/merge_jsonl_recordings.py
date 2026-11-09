assert __name__ == '__main__'

import sys, json

data = []
universal_start_time = None
prev_time = None
current_time_diff = 0

for filename in sys.argv[1:]:
    with open(filename, 'r') as f:
        for line in f:
            frame = json.loads(line)
            
            if universal_start_time is None:
                universal_start_time = frame['time']
            else:
                tm_diff = frame['time'] - prev_time
                if abs(tm_diff) > 100_000:
                    current_time_diff += prev_time - frame['time']
            
            prev_time = frame['time']
            
            frame['time'] += current_time_diff
            
            data.append(frame)

for frame in data:
    print(json.dumps(frame))
