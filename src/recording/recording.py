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
    
    def peek_next_frame(self):
        return self.data[min(self.i + 1, len(self.data) - 1)]
    
    def current_frame(self):
        return self.data[max(0, self.i)]
    
    def prev_frame(self):
        assert self.i >= 1
        self.i -= 1
        return self.data[self.i]
    
    def peek_prev_frame(self):
        assert self.i >= 1
        return self.data[self.i - 1]
    
    def jump_to_relative_time(self, time):
        rec_time = time * 1_000_000 + self.data[0]["time"]
        frame = self.current_frame() if self.i >= 0 else None
        while (not frame or frame["time"] < rec_time) and not self.is_finished():
            frame = self.next_frame()
        if self.i >= 1 and abs(frame["time"] - rec_time) > abs(rec_time - self.peek_prev_frame()["time"]):
            frame = self.prev_frame()
        return frame
    
    def reset(self):
        self.i = -1
    
    def is_finished(self):
        return self.i >= len(self.data) - 1
