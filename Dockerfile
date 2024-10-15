FROM osrf/ros:noetic-desktop-full

RUN \
  useradd user && \
  echo "user ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/user && \
  chmod 0440 /etc/sudoers.d/user && \
  mkdir -p /home/user && \
  chown user:user /home/user && \
  chsh -s /bin/bash user

RUN echo 'root:root' | chpasswd
RUN echo 'user:user' | chpasswd

# setup environment
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
RUN apt update && apt upgrade curl wget git -y

# add kitware repo to get latest cmake
RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null
RUN echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ focal main' | sudo tee /etc/apt/sources.list.d/kitware.list >/dev/null
RUN curl -sSL https://apt.kitware.com/keys/kitware-archive-latest.asc | apt-key add -
RUN apt update

# install packages
RUN apt update && apt install -y --no-install-recommends \
    cmake gdb python3-pip libeigen3-dev libpcl-dev libopencv-dev \
    && rm -rf /var/lib/apt/lists/*

RUN pip install transformations

RUN git clone https://github.com/atenpas/gpd.git
RUN cd gpd && \
    sed -i 's/samples_ = samples/samples_ = Eigen::Matrix3Xd(samples)/' src/gpd/util/cloud.cpp && \
    sed -i 's/private/public/' include/gpd/candidate/hand.h include/gpd/candidate/hand_set.h && \
    # sed -i 's/seed_ = (214013 \* seed_ + 2531011);/return 2531011;/' src/gpd/candidate/hand_set.cpp && \
    # sed -i 's/std::shared_ptr<net::Classifier> classifier_;//' include/gpd/grasp_detector.h && \
    # sed -i 's/std::unique_ptr<descriptor::ImageGenerator> image_generator_;//' include/gpd/grasp_detector.h && \
    # sed -i 's/ private:/  std::shared_ptr<net::Classifier> classifier_;\n  std::unique_ptr<descriptor::ImageGenerator> image_generator_;\n private:/' include/gpd/grasp_detector.h && \
    # sed -i 's/ private:/  std::vector<float> classify_images(std::vector<std::unique_ptr<cv::Mat>> \&images) {return  classifier_->classifyImages(images);}\n  std::unique_ptr<descriptor::ImageGenerator> image_generator_;\n private:/' include/gpd/grasp_detector.h && \
    mkdir build && \
    cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j && \
    make install

RUN mkdir -p /mnt/conf1 /mnt/conf2

WORKDIR /home/user

USER user
SHELL ["/bin/bash", "-c"]

########################################
########### WORKSPACE BUILD ############
########################################
# Installing catkin package
RUN mkdir -p ~/gpd_ws/src
COPY --chown=user . /home/user/gpd_ws/src/gpd_ros
RUN ls -al ~/gpd_ws/src/gpd_ros && \
    rm -r ~/gpd_ws/src/gpd_ros/gpd_ros && \
    rm ~/gpd_ws/src/gpd_ros/gpd_ros_full/CATKIN_IGNORE && \
    mv ~/gpd_ws/src/gpd_ros/gpd_ros_full ~/gpd_ws/src/gpd_ros/gpd_ros
RUN source /opt/ros/noetic/setup.bash && \
    cd ~/gpd_ws && catkin_make -j4 -DCMAKE_BUILD_TYPE=Release

########################################
########### ENV VARIABLE STUFF #########
########################################
RUN echo "source ~/gpd_ws/devel/setup.bash" >> ~/.bashrc
#RUN echo 'export PATH="$PATH:$HOME/.local/bin"' >> ~/.bashrc

CMD ["bash"]
