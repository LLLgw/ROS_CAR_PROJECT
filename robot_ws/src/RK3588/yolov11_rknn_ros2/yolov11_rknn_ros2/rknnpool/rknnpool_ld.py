from concurrent.futures import ThreadPoolExecutor, as_completed
from queue import Queue

from rknnlite.api import RKNNLite


def initRKNN(rknnModel="./rknnModel/best.rknn", id=-1):
    rknn_lite = RKNNLite()
    ret = rknn_lite.load_rknn(rknnModel)
    if ret != 0:
        raise RuntimeError("Load RKNN model failed")

    if id == 0:
        ret = rknn_lite.init_runtime(core_mask=RKNNLite.NPU_CORE_0)
    elif id == 1:
        ret = rknn_lite.init_runtime(core_mask=RKNNLite.NPU_CORE_1)
    elif id == 2:
        ret = rknn_lite.init_runtime(core_mask=RKNNLite.NPU_CORE_2)
    elif id == -1:
        ret = rknn_lite.init_runtime(core_mask=RKNNLite.NPU_CORE_0_1_2)
    else:
        ret = rknn_lite.init_runtime()

    if ret != 0:
        raise RuntimeError("Init runtime environment failed")

    print(rknnModel, "\t\tdone")
    return rknn_lite


def initRKNNs(rknnModel="./rknnModel/best.rknn", TPEs=1):
    rknn_list = []
    for i in range(TPEs):
        rknn_list.append(initRKNN(rknnModel, i % 3))
    return rknn_list


class rknnPoolExecutor:
    def __init__(self, rknnModel, TPEs, func):
        self.TPEs = TPEs
        self.queue = Queue()
        self.rknnPool = initRKNNs(rknnModel, TPEs)
        self.pool = ThreadPoolExecutor(max_workers=TPEs)
        self.func = func
        self.num = 0

    def put(self, frame):
        self.queue.put(self.pool.submit(self.func, self.rknnPool[self.num % self.TPEs], frame))
        self.num += 1

    def get(self):
        if self.queue.empty():
            return None, False
        temp = [self.queue.get()]
        for frame in as_completed(temp):
            return frame.result(), True

    def release(self):
        self.pool.shutdown()
        for rknn_lite in self.rknnPool:
            rknn_lite.release()

    def get_num(self):
        return self.num
