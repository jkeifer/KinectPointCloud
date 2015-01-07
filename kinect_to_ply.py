import freenect
from PIL import Image
import numpy as np

focalLength = 525.0
centerX = 319.5
centerY = 239.5
scalingFactor = 5000.0


def generate_pointcloud(pil_rgb_img, pil_depth_img, ply_file):
    """
    Generate a colored point cloud in PLY format from a color and a depth image.
    
    Input:
    rgb_file -- filename of color image
    depth_file -- filename of depth image
    ply_file -- filename of ply file
    
    """

    if rgb.size != depth.size:
        raise Exception("Color and depth image do not have the same resolution.")
    if rgb.mode != "RGB":
        raise Exception("Color image is not in RGB format")
    if depth.mode != "I":
        raise Exception("Depth image is not in intensity format")

    points = []    
    for v in range(rgb.size[1]):
        for u in range(rgb.size[0]):
            color = rgb.getpixel((u,v))
            Z = depth.getpixel((u,v)) / scalingFactor
            if Z == 0: continue
            X = (u - centerX) * Z / focalLength
            Y = (v - centerY) * Z / focalLength
            points.append("%f %f %f %d %d %d 0\n"%(X,Y,Z,color[0],color[1],color[2]))

    with open(ply_file,"w") as ply:
        ply.write("ply\n" + \
                  "format ascii 1.0\n" + \
                  "element vertex {}\n".format(len(points)) + \
                  "property float x\n" + \
                  "property float y\n" + \
                  "property float z\n" + \
                  "property uchar red\n" + \
                  "property uchar green\n" + \
                  "property uchar blue\n" + \
                  "property uchar alpha\n" + \
                  "end_header\n" + \
                  "{}".format("".join(points)))


depth = Image.fromarray(freenect.sync_get_depth()[0].astype(np.uint8)).convert('I')
rgb = Image.fromarray(freenect.sync_get_video()[0])

print depth.mode

generate_pointcloud(rgb, depth, "pointcloud.ply")

