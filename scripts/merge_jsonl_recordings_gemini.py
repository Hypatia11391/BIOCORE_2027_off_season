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

for filename in sys.argv[1:]:
    with open(filename, 'r') as f:
        for line in f:
            frame = json.loads(line)
            
            # --- Time Transformation ---
            if universal_start_time is None:
                universal_start_time = frame['time']
            else:
                tm_diff = frame['time'] - prev_time
                if abs(tm_diff) > 100_000:
                    n_switches += 1
                    current_time_diff += prev_time - frame['time']
                    
                    # Calculate new pose transformation when a new recording starts
                    if prev_transformed_pose is not None:
                        p_first = frame['robotPose']
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
                orig_x = frame['robotPose'][0]
                orig_y = frame['robotPose'][1]
                orig_theta = frame['robotPose'][2]
                
                # Apply 2D rigid transform: rotate by theta_diff, then translate by offsets
                cos_theta = math.cos(current_theta_diff)
                sin_theta = math.sin(current_theta_diff)
                
                new_x = current_x_offset + orig_x * cos_theta - orig_y * sin_theta
                new_y = current_y_offset + orig_x * sin_theta + orig_y * cos_theta
                new_theta = orig_theta + current_theta_diff
                
                frame['robotPose'][0] = new_x
                frame['robotPose'][1] = new_y
                frame['robotPose'][2] = new_theta
                
                prev_transformed_pose = frame['robotPose']
            
            data.append(frame)

for frame in data:
    print(json.dumps(frame))

print("Num switches:", n_switches, file=sys.stderr)
if n_switches != len(sys.argv)-2:  # first argument is program name
    print("WARNING: WRONG NUMBER OF SWITCHES", file=sys.stderr)
