import numpy as np

def perspective(cam_coords, proj_mat, h, w):
    """
    P = proj_mat @ (x, y, z, 1)
    Project cam2pixel

    Args:
        cam_coords:         [4, npoints]
        proj_mat:           [4, 4]

    Returns:
        pix coords:         [h, w, 2]
    """
    eps = 1e-7
    pix_coords = proj_mat @ cam_coords

    pix_coords = pix_coords[:2, :] / (pix_coords[2, :] + eps)
    pix_coords = np.reshape(pix_coords, (2, h, w))
    pix_coords = np.transpose(pix_coords, (1, 2, 0))
    return pix_coords


def bilinear_sampler(imgs, pix_coords):
    """
    Construct a new image by bilinear sampling from the input image.
    Args:
        imgs:                   [H, W, C]
        pix_coords:             [h, w, 2]
    :return:
        sampled image           [h, w, c]
    """
    img_h, img_w, img_c = imgs.shape
    pix_h, pix_w, pix_c = pix_coords.shape
    out_shape = (pix_h, pix_w, img_c)

    pix_x, pix_y = np.split(pix_coords, [1], axis=-1)  # [pix_h, pix_w, 1]
    pix_x = pix_x.astype(np.float32)
    pix_y = pix_y.astype(np.float32)

    # Rounding
    pix_x0 = np.floor(pix_x)
    pix_x1 = pix_x0 + 1
    pix_y0 = np.floor(pix_y)
    pix_y1 = pix_y0 + 1

    # Clip within image boundary
    y_max = (img_h - 1)
    x_max = (img_w - 1)
    zero = np.zeros([1])

    pix_x0 = np.clip(pix_x0, zero, x_max)
    pix_y0 = np.clip(pix_y0, zero, y_max)
    pix_x1 = np.clip(pix_x1, zero, x_max)
    pix_y1 = np.clip(pix_y1, zero, y_max)

    # Weights [pix_h, pix_w, 1]
    wt_x0 = pix_x1 - pix_x
    wt_x1 = pix_x - pix_x0
    wt_y0 = pix_y1 - pix_y
    wt_y1 = pix_y - pix_y0

    # indices in the image to sample from
    dim = img_w

    # Apply the lower and upper bound pix coord
    base_y0 = pix_y0 * dim
    base_y1 = pix_y1 * dim

    # 4 corner vertices
    idx00 = (pix_x0 + base_y0).flatten().astype(np.int32)
    idx01 = (pix_x0 + base_y1).astype(np.int32)
    idx10 = (pix_x1 + base_y0).astype(np.int32)
    idx11 = (pix_x1 + base_y1).astype(np.int32)

    # Gather pixels from image using vertices
    imgs_flat = imgs.reshape([-1, img_c]).astype(np.float32)
    im00 = imgs_flat[idx00].reshape(out_shape)
    im01 = imgs_flat[idx01].reshape(out_shape)
    im10 = imgs_flat[idx10].reshape(out_shape)
    im11 = imgs_flat[idx11].reshape(out_shape)

    # Apply weights [pix_h, pix_w, 1]
    w00 = wt_x0 * wt_y0
    w01 = wt_x0 * wt_y1
    w10 = wt_x1 * wt_y0
    w11 = wt_x1 * wt_y1
    output = w00 * im00 + w01 * im01 + w10 * im10 + w11 * im11
    return output


def warped(src_image, pix_coords):
    """
    Warp source image using transformed points.
    """
    src_h, src_w, src_c = src_image.shape
    # dst_h, dst_w = dst_size
    dst_h, dst_w, pix_c = pix_coords.shape

    # Discretize & get points within image frame
    xpoints, ypoints = pix_coords[..., 0], pix_coords[..., 1]
    ind = np.where((xpoints >= 0) & (xpoints < src_w) & (ypoints >= 0) & (ypoints < src_h))
    ypoints, xpoints = ypoints[ind].astype(np.int), xpoints[ind].astype(np.int)

    # Get the corresponding point
    xpix, ypix = np.meshgrid(np.linspace(0, dst_w - 1, dst_w), np.linspace(0, dst_h - 1, dst_h))
    ypix, xpix = ypix[ind].astype(int), xpix[ind].astype(int)

    out = np.zeros((dst_h, dst_w, src_c), dtype=src_image.dtype)
    out[ypix, xpix, :] = src_image[ypoints, xpoints, :]
    return out


class Plane:
    """
    Defines a plane in the world
    """

    def __init__(self, x, y, z, roll, pitch, yaw,
                 col, row, scale):
        self.x, self.y, self.z = x, y, z
        self.roll, self.pitch, self.yaw = roll, pitch, yaw

        self.col, self.row = col, row
        self.scale = scale

        self.xyz = self.xyz_coord()

    def xyz_coord(self):
        """
        Returns:
            Grid coordinate: [b, 3/4, row*cols]
        """
        xmin = self.x
        xmax = self.x + self.col * self.scale
        ymin = self.y
        ymax = self.y + self.row * self.scale
        return meshgrid(xmin, xmax, self.col,
                        ymin, ymax, self.row)


def meshgrid(xmin, xmax, num_x, ymin, ymax, num_y, is_homogeneous=True):
    """
    Grid is parallel to z-axis

    Returns:
        array x,y,z,[1] coordinate   [3/4, num_x * num_y]
    """
    x = np.linspace(xmin, xmax, num_x)
    y = np.linspace(ymin, ymax, num_y)
    x, y = np.meshgrid(x, y)
    x = x.flatten()
    y = y.flatten()
    z = np.zeros_like(x)

    if is_homogeneous:
        coords = np.stack([x, y, z, np.ones_like(x)], axis=0)
    else:
        coords = np.stack([x, y, z], axis=0)
    return coords
