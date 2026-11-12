assert __name__ == '__main__'

import sys
import json
import math

data = []
universal_start_time = None
prev_time = None
current_time_diff = 0

# Track the last transformed pose from the previous recording
prev_transformed_pose = None
# Transformation parameters for the current recording
current_theta_diff = 0.0
current_x_offset = 0.0
current_y_offset = 0.0

n_switches = 0

class robotPose:
    def __init__(self, frame):
        self.pose = frame['robotPose']

    def __getitem__(self, key):
        match key:
            case 0:
                return self.pose['translation']['x']
            case 1:
                return self.pose['translation']['y']
            case 2:
                return self.pose['rotation']['radians']

    def __setitem__(self, key, value):
        match key:
            case 0:
                self.pose['translation']['x'] = value
            case 1:
                self.pose['translation']['y'] = value
            case 2:
                self.pose['rotation']['radians'] = value

for filename in sys.argv[1:]:
    with open(filename, 'r') as f:
        for line in f:
            frame = json.loads(line)
            
            # --- Time Transformation ---
            if universal_start_time is None:
                universal_start_time = frame['time']
            else:
                tm_diff = frame['time'] - prev_time
                if abs(tm_diff) > 10000000_000:
                    n_switches += 1
                    current_time_diff += prev_time - frame['time']
                    
                    # Calculate new pose transformation when a new recording starts
                    if prev_transformed_pose is not None:
                        p_first = robotPose(frame)
                        p_last = prev_transformed_pose
                        
                        current_theta_diff = p_last[2] - p_first[2]
                        
                        # Calculate offsets such that applying the rotation + offset to p_first yields p_last
                        cos_theta = math.cos(current_theta_diff)
                        sin_theta = math.sin(current_theta_diff)
                        
                        rotated_x = p_first[0] * cos_theta - p_first[1] * sin_theta
                        rotated_y = p_first[0] * sin_theta + p_first[1] * cos_theta
                        
                        current_x_offset = p_last[0] - rotated_x
                        current_y_offset = p_last[1] - rotated_y
            
            prev_time = frame['time']
            frame['time'] += current_time_diff
            
            # --- Pose Transformation ---
            if 'robotPose' in frame:
                orig_x = robotPose(frame)[0]
                orig_y = robotPose(frame)[1]
                orig_theta = robotPose(frame)[2]
                
                # Apply 2D rigid transform: rotate by theta_diff, then translate by offsets
                cos_theta = math.cos(current_theta_diff)
                sin_theta = math.sin(current_theta_diff)
                
                new_x = current_x_offset + orig_x * cos_theta - orig_y * sin_theta
                new_y = current_y_offset + orig_x * sin_theta + orig_y * cos_theta
                new_theta = orig_theta + current_theta_diff
                
                robotPose(frame)[0] = new_x
                robotPose(frame)[1] = new_y
                robotPose(frame)[2] = new_theta
                
                prev_transformed_pose = robotPose(frame)
            
            data.append(frame)

for frame in data:
    print(json.dumps(frame))

print("Num switches:", n_switches, file=sys.stderr)
if n_switches != len(sys.argv)-2:  # first argument is program name
    print("WARNING: WRONG NUMBER OF SWITCHES", file=sys.stderr)
