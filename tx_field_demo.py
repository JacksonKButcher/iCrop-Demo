import time
from PyQt5.QtWidgets import QApplication, QLabel, QPushButton, QWidget, QVBoxLayout, QHBoxLayout
from PyQt5.QtCore import Qt, QThreadPool, QRunnable, pyqtSlot
from PyQt5.QtGui import QPixmap
from picamera2 import Picamera2
from picamera2.previews.qt import QPicamera2
from PIL import Image
import LoRaPi as lp  # Assuming lp is a custom module for serial communication
import iCrop  # Assuming iCrop is a custom module for image cropping and processing

FORMAT = 'ASCII'
SRC_DIR = r'Crop_Images/' #Directory for the original whole image
SEL_SEG_DIR = r'Segments/' #Directory for the selected segments of an original image
CMPRS_SEG_DIR = r'Compressed_Segments/' #Directory for the compressed selected segments
SEG_DIM = 8 # (SEG_DIM x SEG_DIM) the number of segments that will be created from an image
SEG_SELECTION = 5 # the number of segments chosen for transmission to server
SUBSAMPLE = 5 #The subsample ratio used for the segment selection
CR_13 = 20 #The compression ratio used for the segments to be transmitted
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

class CaptureRunnable(QRunnable):
    def __init__(self, job, cam, ser, label, picture_label, qpicamera2, button):
        super().__init__()
        self.job = job
        self.cam = cam
        self.ser = ser
        self.label = label
        self.picture_label = picture_label
        self.qpicamera2 = qpicamera2
        self.button = button

    @pyqtSlot()
    def run(self):
        # Waits for capture to finish
        self.cam.wait(self.job)

        img_path = SRC_DIR + current_filename

        #Display Image on GUI
        pixmap = QPixmap(img_path)
        smaller_pixmap = pixmap.scaled(300, 300, Qt.KeepAspectRatio, Qt.FastTransformation)
        self.picture_label.setPixmap(smaller_pixmap)

        # Processing Captured Image
        with Image.open(img_path) as im:

            # Selection
            seg_dims = iCrop.get_segments(im, 8)
            temp = iCrop.find_top_n(im, seg_dims, 5, 8, 5)
            top_segs = temp[0]

            # Save + Compress
            final_seg_paths = iCrop.save_top_n(im, seg_dims, top_segs, current_filename, 8, CR_13)

        # Transmit
        for segment_name in final_seg_paths:
            self.label.setText('Tx Seg: ' + segment_name)
            #Display segment sending
            #
            lp.FileSendSerialGUI(self.ser, segment_name, self.label)

        #Wait on classification
        while True:
            prediction = lp.ReadPacketSerial(self.ser)
            if(prediction):
                prediction = lp.Sanitize_Serial(prediction)
                prediction = prediction.replace("\0","")
                labels = prediction.split("_")
                labels = expand_labels(labels)
                self.label.setText('Seg 1: ' + labels[0] + '\nSeg 2: ' + labels[1] + '\nSeg 3: ' + labels[2] + '\nSeg 4: ' + labels[3] + '\nSeg 5: ' + labels[4] + '\nFinal Label: \n' + labels[5])
                break

        # Transmission Finished
        # Restarts the preview
        self.picture_label.clear()
        self.qpicamera2.show()
        self.cam.start()
        self.button.setEnabled(True)

def main():

    global current_filename

    def on_button_clicked():
        global current_filename
        button.setEnabled(False)
        label.setText('Capturing Image...')
        qpicamera2.hide()
        cam.stop()
        cfg = cam.create_still_configuration(main={"size": (2080, 1520), "format": "RGB888"})
        current_filename = time.strftime("%m%d-%H%M%S") + ".jpg"
        cam.switch_mode_and_capture_file(cfg, SRC_DIR + current_filename, signal_function=qpicamera2.signal_done)

    def on_exit_clicked():
        quit()

    # Start Serial Connection w/ LoRa Module
    ser = lp.SerialConnect()

    # Camera Instantiation
    cam = Picamera2()
    cam.configure(cam.create_preview_configuration(main={"size": (300, 300)}))

    # GUI
    app = QApplication([])
    qpicamera2 = QPicamera2(cam, width=200, height=200, keep_ar=False)
    button = QPushButton("Capture")
    exit_button = QPushButton("Exit")
    label = QLabel()
    picture_label = QLabel()
    window = QWidget()
    
    threadpool = QThreadPool()

    def capture_done_wrapper(job):
        runnable = CaptureRunnable(job, cam, ser, label, picture_label, qpicamera2, button)
        threadpool.start(runnable)

    qpicamera2.done_signal.connect(capture_done_wrapper)
    button.clicked.connect(on_button_clicked)
    exit_button.clicked.connect(on_exit_clicked)

    label.setFixedWidth(175)
    label.setAlignment(Qt.AlignBottom)
    layout_h = QHBoxLayout()
    layout_v = QVBoxLayout()
    layout_v.addWidget(label)
    layout_v.addWidget(button)
    layout_v.addWidget(exit_button)
    layout_h.addWidget(qpicamera2, 80)
    layout_h.addWidget(picture_label)
    layout_h.addLayout(layout_v, 20)
    window.setWindowTitle("iCrop")
    window.resize(475, 250)
    window.setLayout(layout_h)

    cam.start()
    window.show()
    app.exec()

if __name__ == "__main__":
    main()
