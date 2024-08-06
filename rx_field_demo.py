import LoRaPi as lp
import traceback
import numpy as np
import pandas as pd
import os
from statistics import mode
import random
import tensorflow as tf
from keras.preprocessing.image import ImageDataGenerator
import argparse

np.random.seed(70) 
random.seed(70) 
tf.random.set_seed(70)
img_height, img_width = (224,224)
save_dir = r'Compressed_Segments/'
PKT_LEN_NOCHECK = 66
class_labels = ['bacterial_blight', 'cercospora_leaf_blight','downey_mildew', 'frogeye', 'healthy', 'potassium_deficiency', 'soybean_rust', 'target_spot']

parser = argparse.ArgumentParser("simple_example")
parser.add_argument("sf", help="Spreading factor to be recorded in transmission stats file.", type=int)
parser.add_argument("bw", help="Bandwidth to be recorded in transmission stats file.", type=int)
parser.add_argument("distance", help="Distance to be recorded in transmission stats file.", type=int)
args = parser.parse_args()


#Shape of the list is column by colum from the excel spreadsheet
#So it has #1 Segments, #2, #3, etc. This means that the number of segments don't line up adjacently
def ensemble_majority_voting(predict_labels, num_segments):
    #Loop through for each imamge

    temp_list = []
    #Loop through each segment of this image
    for label in predict_labels:
        temp_list.append(label)

    new_predict = mode(temp_list)

    return new_predict

# ImageDataGenerator for data augmentation
datagen = ImageDataGenerator(
    width_shift_range=0.2,
    height_shift_range=0.2,
    fill_mode="nearest",
    brightness_range=[0.9, 1.1],
    rotation_range=30,
    vertical_flip=True,
    horizontal_flip=True,
    rescale=1./255
)


def simplify_labels(labels):

    new_labels = []
    for label in labels:
        if(label == "bacterial_blight"):
            new_labels.append("bb")
        elif(label == 'cercospora_leaf_blight'):
            new_labels.append("clb")
        elif(label == 'downey_mildew'):
            new_labels.append("dm")
        elif(label == 'frogeye'):
            new_labels.append("f")
        elif(label == 'healthy'):
            new_labels.append("h")
        elif(label == 'potassium_deficiency'):
            new_labels.append("pd")
        elif(label == 'soybean_rust'):
            new_labels.append("sr")
        elif(label == 'target_spot'):
            new_labels.append("ts")

    return new_labels


             
             
             
             
             
             



def process_image(segment_names):
    new_model = tf.keras.models.load_model('SoyNetFiveUnique.h5')

    # Create a DataFrame
    test_df = pd.DataFrame({'filename': [os.path.join(save_dir, segment) for segment in segment_names]})

    test_generator = datagen.flow_from_dataframe(
        test_df,
        target_size=(img_height, img_width),
        batch_size=1,
        shuffle=False,
        class_mode=None,  # Set to None for testing without labels
        x_col="filename",
        y_col="label"
    )

    predict_vals = new_model.predict(test_generator)
    predict_vals = tf.argmax(predict_vals, axis=1)

    predict_labels_str = [class_labels[val] for val in predict_vals]
    
    return predict_labels_str

#Basic instantions before main code can begin
ser = lp.SerialConnect()
segment_names = []
spreadfactor = args.sf
bandwidth = args.bw
distance = args.distance

#Main Infinite Loop
while(True):
    try:


        init_packet = lp.WaitForInitialPkt(ser)
        filename = lp.ParseHeaderPacket(ser, init_packet, spreadfactor, bandwidth, distance)

        #Check to see if received file is a compressed image or a segmented images
        compress_check = filename.split('_')[0]
        if(compress_check == 'cmprs'):
            predict = process_image([filename])
            simplified_predict = simplify_labels(predict)
            predict_packet = simplified_predict[0] + '\0' * (PKT_LEN_NOCHECK - len(simplified_predict[0]))
            lp.WritePacketSerial(ser, predict_packet)
            lp.SerialReadyAck(ser)
            segment_names = []
            #Code for compressed image handling
        else:    
            segment_names.append(filename)

        if(len(segment_names) % 5 == 0 and len(segment_names) != 0):
            predict = process_image(segment_names)
            simplified_predict = simplify_labels(predict)
            final_predict = ensemble_majority_voting(predict, 5)
            predict.append(final_predict)
            simplified_predict = simplify_labels(predict)
            predict_packet = "_".join(simplified_predict)
            predict_packet += '\0' * (PKT_LEN_NOCHECK - len(predict_packet))
            lp.WritePacketSerial(ser, predict_packet)
            lp.SerialReadyAck(ser)
            segment_names = []


    except KeyboardInterrupt:
        # Handle keyboard interrupt
        print("KeyboardInterrupt: Data reception interrupted by user.")
        traceback.print_exc()
        print("Closing serial port...")
        ser.close()
        quit()
    except Exception as e:
        # Handle other exceptions
        print("An error occurred during data reception and saving:")
        traceback.print_exc()
        # print("Closing serial port...")
        # ser.close()
        pass
