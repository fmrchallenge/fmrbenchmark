# docker build -t fmr:integrator_chains .

FROM ros:melodic-ros-base-bionic

RUN apt-get update
RUN DEBIAN_FRONTEND=noninteractive apt-get -y install \
       ros-melodic-tf \
       libeigen3-dev

COPY integrator_chains_msgs /root/integrator_chains_msgs
COPY dynamaestro /root/dynamaestro

RUN . /opt/ros/melodic/setup.sh \
    && mkdir -p integrators_workspace/src \
    && cd integrators_workspace/src \
    && catkin_init_workspace \
    && ln -s /root/integrator_chains_msgs \
    && ln -s /root/dynamaestro
RUN . /opt/ros/melodic/setup.sh && cd integrators_workspace && catkin_make install

COPY trial-runner.py /root/trial-runner.py

CMD ["bash", "-c", "cd integrators_workspace && source install/setup.bash && python /root/trial-runner.py -l -f /root/results/mydata.json /root/trialconf/tc.json"]
