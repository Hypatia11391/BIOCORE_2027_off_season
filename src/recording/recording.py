import json


class Recording:
    def __init__(self, data: list[dict]):
        self.data = data
        self.i = -1

    @classmethod
    def from_json_file(cls, filename: str):
        with open(filename, 'r') as f:
            data = json.load(f)
        return cls(data)

    def next_frame(self):
        if self.i<len(self.data)-1:
            self.i += 1
        frame = self.data[self.i]
        return frame

    def current_frame(self):
        assert self.i>=0
        return self.data[self.i]

    def is_finished(self):
        return self.i>=len(self.data)-1