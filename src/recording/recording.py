import json


class Recording:
    def __init__(self, data: list[dict]):
        self.data = data
        self.i = -1
    
    @classmethod
    def from_json_file(cls, filename: str):
        with open(filename, "r") as f:
            data = json.load(f)
        return cls(data)
    
    @classmethod
    def from_jsonl_file(cls, filename: str):
        data = []
        with open(filename, "r") as f:
            for line in f:
                data.append(json.loads(line))
        return cls(data)
    
    def next_frame(self):
        if self.i < len(self.data) - 1:
            self.i += 1
        frame = self.data[self.i]
        return frame
    
    def jump_to_relative_time(self, time):
        rec_time = time + self.data[0]["time"]
        frame = self.current_frame()
        while frame["time"] < rec_time:
            frame = self.next_frame()
        if self.i >= 1 and abs(frame["time"] - rec_time) > abs(rec_time - self.prev_frame()["time"]):
            self.i -= 1
        return frame
    
    def current_frame(self):
        assert self.i >= 0
        return self.data[self.i]
    
    def prev_frame(self):
        assert self.i >= 1
        return self.data[self.i - 1]
    
    def reset(self):
        self.i = -1
    
    def is_finished(self):
        return self.i >= len(self.data) - 1
