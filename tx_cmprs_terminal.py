#!/usr/bin/env python3

import heapq
import os
from PIL import Image
import LoRaPi as lp  # Assuming lp is a custom module for serial communication
import iCrop  # Assuming iCrop is a custom module for image cropping and processing
import io

FORMAT = 'ASCII'
SRC_DIR = r'Crop_Images/' #Directory for the original whole image
SEL_SEG_DIR = r'Segments/' #Directory for the selected segments of an original image
CMPRS_SEG_DIR = r'Compressed_Segments/' #Directory for the compressed selected segments
SEG_DIM = 8 # (SEG_DIM x SEG_DIM) the number of segments that will be created from an image
SEG_SELECTION = 5 # the number of segments chosen for transmission to server
SUBSAMPLE = 5 #The subsample ratio used for the segment selection
CR_13 = 20 #The compression ratio used for the segments to be transmitted
FILE_TO_SEND = 3
current_filename = ""

def expand_labels(labels):

    new_labels = []
    for label in labels:
        if(label == "bb"):
            new_labels.append("bacterial_blight")
        elif(label == 'clb'):
            new_labels.append("cercospora_leaf_blight")
        elif(label == 'dm'):
            new_labels.append("downey_mildew")
        elif(label == 'f'):
            new_labels.append("frogeye")
        elif(label == 'h'):
            new_labels.append("healthy")
        elif(label == 'pd'):
            new_labels.append("potassium_deficiency")
        elif(label == 'sr'):
            new_labels.append("soybean_rust")
        elif(label == 'ts'):
            new_labels.append("target_spot")
        else:
            new_labels.append(label)

    return new_labels

#This code will grab the 5 most recently captured images
def grab_recent_images(path):

    files = os.listdir(path)
    paths = [os.path.join(path, basename) for basename in files]
    img_paths = heapq.nlargest(FILE_TO_SEND, paths, key=os.path.getctime)
    return img_paths

def mem_compress_image(image: Image):
    """
    Compresses the given image.
    :return: A compressed copy of the image
    """
    buffered = io.BytesIO()
    image.save(buffered, format="JPEG", optimize=True, quality=CR_13)
    return buffered

def main():

    # Start Serial Connection w/ LoRa Module
    ser = lp.SerialConnect()

    img_paths = grab_recent_images(SRC_DIR)

    for img_path in img_paths:
        current_filename = img_path.split('/')[1]


        with Image.open(img_path) as im:
            compressed_im = mem_compress_image(im)
            compress_name = 'cmprs_' + current_filename
            print('Tx Compress: ' + compress_name)
            lp.BytesSendSerial(ser, compress_name, compressed_im)

        #Wait on classification
        while True:
            prediction = lp.ReadPacketSerial(ser)
            if(prediction):
                prediction = lp.Sanitize_Serial(prediction)
                prediction = prediction.replace("\0","")
                labels = expand_labels([prediction])
                print('Final Label: \n' + labels[0])
                break

        #Segments transfer
        with Image.open(img_path) as im:

            # Selection
            seg_dims = iCrop.get_segments(im, 8)
            temp = iCrop.find_top_n(im, seg_dims, 5, 8, 5)
            top_segs = temp[0]

            # Save + Compress
            final_seg_paths = iCrop.save_top_n(im, seg_dims, top_segs, current_filename, 8, CR_13)

        # Transmit Segments
        for segment_name in final_seg_paths:
            print('Tx Seg: ' + segment_name)
            lp.FileSendSerial(ser, segment_name)


        #Wait on classification
        while True:
            prediction = lp.ReadPacketSerial(ser)
            if(prediction):
                prediction = lp.Sanitize_Serial(prediction)
                prediction = prediction.replace("\0","")
                labels = prediction.split("_")
                labels = expand_labels(labels)
                print('Seg 1: ' + labels[0] + '\nSeg 2: ' + labels[1] + '\nSeg 3: ' + labels[2] + '\nSeg 4: ' + labels[3] + '\nSeg 5: ' + labels[4] + '\nFinal Label: \n' + labels[5])
                break
        



if __name__ == "__main__":
    main()
