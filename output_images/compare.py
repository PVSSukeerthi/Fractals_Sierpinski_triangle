from PIL import Image
import numpy as np
import glob

# Load reference image (float64)
ref = np.array(Image.open("sierpinski_double.png"),dtype=np.float64)

# Function to compute metrics
def compare_images(img_path, ref):
    img = np.array(Image.open(img_path), dtype=np.float64)
    mse = np.mean((img - ref) ** 2)
    psnr = 20 * np.log10(255.0 / np.sqrt(mse)) if mse != 0 else float('inf')
    ssim = structural_similarity(img, ref, channel_axis=2, data_range=255)
    return mse, psnr, ssim

# Need SSIM from skimage
from skimage.metrics import structural_similarity

# Compare all outputs
for img_path in glob.glob("sierpinski_*.png"):
    mse, psnr, ssim = compare_images(img_path, ref)
    print(f"{img_path}: MSE={mse:.4f}, PSNR={psnr:.2f} dB, SSIM={ssim:.4f}")
