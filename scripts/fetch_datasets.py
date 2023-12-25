import os
import dataset_tools as dtools


from source.settings import DATASETS_FOLDER


# FPIC-Component Dataset
# Labeled PCB dataset of 6260 images
dtools.download(dataset='FPIC-Component',
                dst_dir=f'{DATASETS_FOLDER}/fpic-component-dataset')
