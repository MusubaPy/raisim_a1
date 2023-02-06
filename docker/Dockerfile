ARG from
FROM ${from}

ENV LANG C.UTF-8

RUN apt-get update && apt-get install -y git software-properties-common gcc-8 g++-8

RUN apt-get install -y cmake python3.6-dev python3-pip libpython3.6-dev libeigen3-dev

RUN mkdir .raisim

RUN pip3 install tensorflow-gpu setuptools

RUN apt-get install -y libgles2-mesa-dev libxt-dev libxaw7-dev libsdl2-dev libzzip-dev libfreeimage-dev libfreetype6-dev libpugixml-dev

ENV CXX=/usr/bin/g++-8
ENV CC=/usr/bin/gcc-8
ENV WORKSPACE=/raisim_workspace
ENV LOCAL_BUILD=/raisim_build
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${LOCAL_BUILD}/lib
ENV PYTHONPATH=$PYTHONPATH:${LOCAL_BUILD}/lib

RUN apt install -y gedit nano minizip ffmpeg mesa-utils


RUN pip3 install tensorboard==2.4.1 ruamel.yaml


RUN dpkg --add-architecture i386 && \
    apt-get update && apt-get install -y --no-install-recommends \
        libxau6 libxau6:i386 \
        libxdmcp6 libxdmcp6:i386 \
        libxcb1 libxcb1:i386 \
        libxext6 libxext6:i386 \
        libx11-6 libx11-6:i386 && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y firefox


#RUN add-apt-repository -y ppa:oibaf/graphics-drivers && apt update && apt #upgrade -y && apt install -y libvulkan1 mesa-vulkan-drivers vulkan-utils

# ==================================================================
# display
# ------------------------------------------------------------------
# For nvidia GUI issues
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

RUN echo "/usr/local/nvidia/lib" >> /etc/ld.so.conf.d/nvidia.conf && \
    echo "/usr/local/nvidia/lib64" >> /etc/ld.so.conf.d/nvidia.conf

ENV LD_LIBRARY_PATH /usr/lib/x86_64-linux-gnu:/usr/lib/i386-linux-gnu${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}:/usr/local/nvidia/lib:/usr/local/nvidia/lib64
EXPOSE 6006

RUN add-apt-repository -y -r ppa:graphics-drivers/ppa && \
	apt update && \
	apt upgrade -y
RUN apt-get install -y ubuntu-drivers-common
#RUN apt install -y nvidia-graphics-drivers-460
RUN pip3 install torch==1.8.1+cu111 -f https://download.pytorch.org/whl/torch_stable.html
