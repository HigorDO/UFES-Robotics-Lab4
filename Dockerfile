FROM osrf/ros:humble-desktop-full

RUN curl https://bootstrap.pypa.io/get-pip.py > get-pip.py
RUN python3 get-pip.py

RUN echo ". /ros_entrypoint.sh" >> ~/.bashrc
RUN echo "source /root/ros_ws/install/setup.bash" >> ~/.bashrc
# Rospy2
RUN git clone https://github.com/dheera/rospy2.git
RUN cd rospy2 && python3 setup.py install

CMD ["tail", "-f", "/dev/null"]