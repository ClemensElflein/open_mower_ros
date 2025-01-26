FROM osrf/ros:noetic-desktop-full AS base
ARG MY_UID
ARG MY_GID
ARG MY_USER

# Create a new group with provided name and ID
RUN groupadd -g ${MY_GID} -o ${MY_USER}

# Create a new user, assign it to the group, and set its shell to bash
RUN useradd -m -u ${MY_UID} -g ${MY_GID} -o -s /bin/bash ${MY_USER} && yes ${MY_USER} | passwd ${MY_USER}

RUN echo $MY_USER ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$MY_USER \
    && chmod 0440 /etc/sudoers.d/$MY_USER

RUN apt-get update && apt-get install -y zsh git sudo ssh gdb rsync

RUN ( \
    echo 'LogLevel DEBUG2'; \
    echo 'PermitRootLogin yes'; \
    echo 'PasswordAuthentication yes'; \
    echo 'ListenAddress 0.0.0.0:2222'; \
    echo 'Subsystem sftp /usr/lib/openssh/sftp-server'; \
  ) > /etc/ssh/sshd_clion \
  && mkdir /run/sshd
RUN bash -c "ssh-keygen -A"

RUN echo "$MY_USER ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Update rosdep package manager for ROS
RUN rosdep update --rosdistro $ROS_DISTRO

# Create a new image layer based on the base image for storing dependencies
FROM base AS dependencies

# Generate a sorted list of apt-get install commands corresponding to the dependencies.
# No actual installation is performed in this stage.
RUN --mount=type=bind,source=./,target=/tmp/open_mower_ros \
    cd /tmp/open_mower_ros && \
    rosdep install --from-paths src --ignore-src --simulate | \
    sed --expression '1d' | sort | tr -d '\n' | sed --expression 's/  apt-get install//g' > /apt-install_list

# Create another image layer for the development environment
FROM base AS devenv

# Copy the list of packages into the new layer
COPY --link --from=dependencies /apt-install_list /apt-install_list

# Update package lists for upgrades or new package installation
RUN apt-get update && \
    apt-get install --no-install-recommends --yes $(cat /apt-install_list) && \
    rm -rf /var/lib/apt/lists/*


CMD ["sudo", "/usr/sbin/sshd", "-D", "-e", "-f", "/etc/ssh/sshd_clion"]
