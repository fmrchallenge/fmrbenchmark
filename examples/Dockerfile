# docker build -f Dockerfile -t fmr:examples ..

FROM ros:melodic-ros-base-bionic

RUN apt-get update
RUN DEBIAN_FRONTEND=noninteractive apt-get -y install \
       libgmp-dev \
       python-pip \
       python-tk

COPY tools /root/tools

RUN cd /root/tools/fmrb-pkg && pip install --user -r requirements.txt && pip install --user .

COPY domains/integrator_chains/integrator_chains_msgs /root/integrator_chains_msgs

COPY examples/sci_concrete_examples /root/sci_concrete_examples

RUN . /opt/ros/melodic/setup.sh \
    && mkdir -p examples/src \
    && cd examples/src \
    && catkin_init_workspace \
    && ln -s /root/integrator_chains_msgs \
    && ln -s /root/sci_concrete_examples
RUN . /opt/ros/melodic/setup.sh && cd examples && catkin_make install

RUN DEBIAN_FRONTEND=noninteractive apt-get -y install curl

RUN curl -L -O https://repo.anaconda.com/miniconda/Miniconda2-latest-Linux-x86_64.sh && chmod +x Miniconda2-latest-Linux-x86_64.sh
# expected SHA256 of Miniconda2-latest-Linux-x86_64.sh: b820dde1a0ba868c4c948fe6ace7300a252b33b5befd078a15d4a017476b8979
RUN ./Miniconda2-latest-Linux-x86_64.sh -b
RUN bash -c "source /root/miniconda2/bin/activate && conda install -y -c conda-forge slycot control"

ENV PYTHONPATH="/root/miniconda2/lib/python2.7/site-packages:/opt/ros/melodic/lib/python2.7/dist-packages:/usr/lib/python2.7/dist-packages"


# e.g., docker run --net=x --name q -h q -e="ROS_MASTER_URI=http://p:11311" -it --rm fmr:examples bash -c "cd examples && source install/setup.bash && roslaunch sci_concrete_examples lqr.launch"