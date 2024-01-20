import dataset_tools as dtools


# FPIC-Component Dataset
# Labeled PCB dataset of 6260 images
dtools.download(
    dataset="FPIC-Component",
    dst_dir=f"assets/datasets/fpic-component-dataset",
)
