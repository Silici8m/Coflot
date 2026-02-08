# Image ROS Humble
FROM ros:humble-ros-base

# 1. Outils Système
RUN apt-get update && apt-get install -y \
    nginx \
    python3-pip \
    python3-scipy \ 
    ros-humble-rosbridge-suite \
    ros-humble-nav2-msgs \
    && rm -rf /var/lib/apt/lists/*

# 2. Config Web
RUN rm /etc/nginx/sites-enabled/default
COPY nginx.conf /etc/nginx/conf.d/default.conf

# 3. Préparation du Workspace
WORKDIR /app/coflot_ws

# 4. Copie la partie Décision
COPY decision_ws/src ./src
COPY IHM /app/IHM

# 5. Compilation
RUN . /opt/ros/humble/setup.sh && \
    apt-get update && \
    rosdep install --from-paths src -y --ignore-src && \
    colcon build --symlink-install

# 6. Setup Automatique
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /app/coflot_ws/install/setup.bash" >> ~/.bashrc

WORKDIR /app
CMD ["bash"]