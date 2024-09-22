import torch
import torch.multiprocessing as mp
import torch.nn as nn
from pygcm import CameraModel
import os

pic_root_path = "/mnt/i/project/slam/thirdparty/general_camera_model/apps/test_data/"

class NewCameraModel(CameraModel):
    def __init__(self) -> None:
        CameraModel.__init__(self)
        # 使用 torch.Tensor 使其支持共享内存
        self.other = torch.zeros(1, dtype=torch.float32)

    def __getstate__(self):
        state = CameraModel.__getstate__(self)
        state['other'] = self.other.cpu().numpy()
        return state

    def __setstate__(self, state):
        CameraModel.__setstate__(self, state)
        self.other = torch.tensor(state['other'], dtype=torch.float32)

def worker(shared_tensor):
    # 在进程中访问共享张量
    print(f"Worker process received tensor value: {shared_tensor.width()}")
    pass

if __name__ == "__main__":
    mp.set_start_method('spawn')

    idx = 0
    cam = NewCameraModel()
    cam.load_config_file(os.path.join(pic_root_path, "penn_c1_cam0.json"))

    print(cam.__getstate__())

    # # 创建子进程
    p1 = mp.Process(target=worker, args=(cam,))
    p1.start()

    print(cam.width())

    # 主进程中更新共享张量
    # while True:
    #     cam.set_width(idx)
    #     idx += 1

    p1.join()
