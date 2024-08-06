import numpy as np
import math
from PIL import Image
from collections import defaultdict

def analyze_image(im_array, percent_pix):
        temp_array = im_array.copy()
        pixels = temp_array.reshape((-1, temp_array.shape[-1]))
        if(percent_pix == 100):
            unique_colors = set(map(tuple, pixels))
        else:
            num_rand_pixels = int(len(pixels) * (percent_pix/100))
            random_indices = np.random.choice(len(pixels), size=num_rand_pixels, replace=False)
            random_pixels = pixels[random_indices]
            unique_colors = set(map(tuple, random_pixels))
        return len(unique_colors)

#This function returns the dimensions of the segments within the image
def get_segments(im, dimension_split):

    width, height = im.size
    width_step = math.floor(width / dimension_split)
    height_step = math.floor(height / dimension_split)

    seg_dimensions = []

    # The crop method from the Image module takes four coordinates as input.
    # The right can also be represented as (left+width)
    # and lower can be represented as (upper+height).
    for i in range(dimension_split):
        for j in range(dimension_split):
            (left, upper, right, lower) = (j*width_step, i*height_step, (1+j)*width_step, (1+i)*height_step)
            dims = (left, upper, right, lower)
            # Here the image "im" is cropped and assigned to new variable im_crop
            # im_crop = im.crop((left, upper, right, lower))
            # #Saves new_image
            # im_crop = np.array(im_crop)
            seg_dimensions.append(dims)

    return seg_dimensions
    
def find_top_n(image, seg_dims, n, dimension_split, seg_subsample):

    #Find the image quality value for each segment
    colors_dict = defaultdict(int)
    for i in range(dimension_split):
        for j in range(dimension_split):
            seg = image.crop(seg_dims[i*dimension_split+j])
            seg = np.array(seg)
            segment_quality_num = analyze_image(seg, seg_subsample)
            name = str(i) + "_" + str(j)
            colors_dict[name] = segment_quality_num

    #Find top 5 images
    sorted_images = sorted(colors_dict.items(), key=lambda x: x[1], reverse=True)
    top_n_images = [image for image, _ in sorted_images[:n]]
    quality_nums = list(colors_dict.values())
    return top_n_images, quality_nums

def save_top_n(image, seg_dims, top_n, filename, dimension_split, img_quality):
    segment_paths = []
    filename = filename.split(".")[0]
    #Save each segment
    for n in top_n:
        n = n.split("_")
        i = int(n[0])
        j = int(n[1])
        index = i*dimension_split+j
        seg = image.crop(seg_dims[index])
        seg = np.array(seg)
        segment_img = Image.fromarray(seg)
        segment_name = filename + "_" + n[0] + "_" + n[1] + ".jpeg"
        save_path = r'Compressed_Segments/' + segment_name
        segment_img.save(save_path, "JPEG", optimize = True, quality=img_quality)
        segment_paths.append(segment_name)
    return segment_paths

def convert_seg_ids(top_n):
    seg_id = []
    for n in top_n:
        n = n.split("_")
        i = int(n[0])
        j = int(n[1])
        seg_id.append([i,j])
    return seg_id

def compress(input_path, output_path, img_quality):
    img = Image.open(input_path)
    img.save(output_path, "JPEG", optimize = True, quality=img_quality)

def process_captured_image():

    return

def selection():

    return

def segment_compress():

    return