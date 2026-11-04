"""Edge-preserving spatial denoising for an owned preview depth map only."""
import numpy as np

_KERNEL=np.ones((3,3),np.uint8)


def smooth_depth(depth):
    if depth.dtype!=np.uint16 or depth.ndim!=2 or not depth.flags.c_contiguous:
        return None
    try:
        import cv2
    except ImportError:
        return None
    # A median reduces isolated depth fluctuations without temporal history.
    # Preserve missing pixels and every stencil crossing a >20 mm depth edge;
    # a thin foreground feature surrounded by background is not pulled into it.
    filtered=cv2.medianBlur(depth,3)
    span=cv2.morphologyEx(depth,cv2.MORPH_GRADIENT,_KERNEL,borderType=cv2.BORDER_REPLICATE)
    delta=cv2.absdiff(filtered,depth)
    keep_original=(depth==0)|(filtered==0)|(span>20)|(delta>12)
    np.copyto(filtered,depth,where=keep_original)
    return filtered
