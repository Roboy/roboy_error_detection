FROM ros:kinetic
RUN apt-get update && apt-get install -y openssh-server cmake gcc build-essential git python-rosdep
RUN apt-get install vim python tcpdump telnet -y
RUN apt-get install byacc flex -y
RUN apt-get install iproute2 gdbserver less bison valgrind -y

RUN mkdir /var/run/sshd
RUN echo 'root:root' | chpasswd
RUN sed -i 's/PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config

# SSH login fix. Otherwise user is kicked off after login
RUN sed 's@session\s*required\s*pam_loginuid.so@session optional pam_loginuid.so@g' -i /etc/pam.d/sshd

ENV NOTVISIBLE "in users profile"
RUN echo "export VISIBLE=now" >> /etc/profile

RUN echo "source /opt/ros/kinetic/setup.bash" >> /root/.bashrc
RUN echo "source /var/workspace/devel/setup.bash" >> /root/.bashrc

ADD . /var/workspace/src/roboy_error_detection

WORKDIR /var/workspace/src

RUN git clone -b master https://github.com/Roboy/roboy_system_notification
RUN git clone -b feature/error-detection-msgs https://github.com/Roboy/roboy_communication

RUN chmod u+x ./roboy_error_detection/bin/run-docker.sh

WORKDIR /var/workspace

EXPOSE 22 9999 7777
CMD /var/workspace/src/roboy_error_detection/bin/run-docker.sh