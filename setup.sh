#!/bin/bash


# Configure conda env
conda create -n MRTA python=2.7

conda install -c conda-forge rospkg -n MRTA
conda install -c sotte empy -n MRTA
conda install -c conda-forge defusedxml -n MRTA
conda install -c miniconda numpy -n MRTA
conda install -c miniconda pyqtgraph -n MRTA
conda install -c miniconda yaml -n MRTA
conda install -c miniconda pycryptodomex -n MRTA
conda install -c conda-forge python-gnupg -n MRTA
conda install -c conda-forge matplotlib -n MRTA
# conda install -c conda-forge websockets -n MRTA

echo "conda activate MRTA" >> ~/.zshrc
echo "conda activate MRTA" >> ~/.bashrc


# Use rosdep to install all dependencies
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro noetic -y