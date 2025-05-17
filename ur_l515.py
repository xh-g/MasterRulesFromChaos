import pyrealsense2 as rs
import numpy as np
import json
import math
import matplotlib.pyplot as plt
import cv2

# 170mm


class L515:
    def __init__(self):
        
        jsonDict = json.load(open("./model/l515_set.json"))
        jsonString= str(jsonDict).replace("'", '\"')
        try:
            dev = self.find_device_json_input_interface()
            ser_dev = rs.serializable_device(dev)
            ser_dev.load_json(jsonString)
            print("loaded json")
        except Exception as e:
            print(e)
            pass
        self.pipeline = rs.pipeline()
        config = rs.config()

        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)

        print("[INFO] start streaming...")
        profile = self.pipeline.start(config)
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        print("Depth Scale is: ", depth_scale)

        intr = (
            profile.get_stream(rs.stream.depth)
            .as_video_stream_profile()
            .get_intrinsics()
        )
        print("width is: ", intr.width)
        print("height is: ", intr.height)
        print("ppx is: ", intr.ppx)
        print("ppy is: ", intr.ppy)
        print("fx is: ", intr.fx)
        print("fy is: ", intr.fy)
        HFOV = math.degrees(2 * math.atan(intr.width / (intr.fx + intr.fy)))
        print("HFOV is", HFOV)
        VFOV = math.degrees(2 * math.atan(intr.height / (intr.fx + intr.fy)))
        print("VFOV is", VFOV)
        self.point_cloud = rs.pointcloud()

    def find_device_json_input_interface(self) :
        DS5_product_ids = ["0AD1", "0AD2", "0AD3", "0AD4", "0AD5", "0AF6", "0AFE", "0AFF", "0B00", "0B01", "0B03", "0B07", "0B3A", "0B5C", "0B64"]
        ctx = rs.context()
        ds5_dev = rs.device()
        devices = ctx.query_devices();
        for dev in devices:
            if dev.supports(rs.camera_info.product_id) and str(dev.get_info(rs.camera_info.product_id)) in DS5_product_ids:
                if dev.supports(rs.camera_info.name):
                    print("Found device", dev.get_info(rs.camera_info.name))
                return dev
        raise Exception("No product line device that has json input interface")
    def get_verts(self):

        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        
        decimation = rs.decimation_filter(4)
        depth_frame = decimation.process(depth_frame)
        points = self.point_cloud.calculate(depth_frame)
        verts = (
            np.asanyarray(points.get_vertices()).view(np.float32).reshape(120, 160, 3)
        )  # xyz
        
        color_image = np.asanyarray(color_frame.get_data())
        
        return verts, color_image

    def get_full_dpi_verts(self):

        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        points = self.point_cloud.calculate(depth_frame)
        full_dpi_verts = (
            np.asanyarray(points.get_vertices()).view(np.float32).reshape(480, 640, 3)
        )  # xyz
        color_image = np.asanyarray(color_frame.get_data())
        return full_dpi_verts, color_image


    def stop_streaming(self):
        self.pipeline.stop()


# %%
if __name__ == '__main__':
    L515 = L515()
    verts, _ = L515.get_verts()
    plt.imshow(verts)
    L515.stop_streaming()
    # %%
    cz = verts[:, :, 2]
    np.save('origin_cz.npy', cz)
    plt.imshow(cz)
    # %%
    cz[np.where(cz > 0.41)] = 1
    cp_w = 82
    cz = cz[:, cp_w - 60: cp_w + 60]
    plt.imshow(cz)
