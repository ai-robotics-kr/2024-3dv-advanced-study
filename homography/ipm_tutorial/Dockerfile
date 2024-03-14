FROM nvcr.io/nvidia/tensorrt:22.07-py3 

ARG DEBIAN_FRONTEND=noninteractive

ENV CONDA_DIR /opt/conda
RUN wget --quiet https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda.sh && \
    /bin/bash ~/miniconda.sh -b -p /opt/conda

ENV PATH=$CONDA_DIR/bin:$PATH

RUN conda update -n base conda &&\
    conda install -n base conda-libmamba-solver &&\
    conda config --set solver libmamba

RUN conda create -y -n ipmtutorial python numpy opencv