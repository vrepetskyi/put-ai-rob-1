FROM put/rob1:base

# Take existing base image, update mirrors and upgrade packages
RUN apt update -y && \
    apt upgrade -y
