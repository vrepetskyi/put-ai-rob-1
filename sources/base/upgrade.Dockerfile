FROM put/ai-rob-1:base

# Take existing base image, update mirrors and upgrade packages
RUN apt update -y && \
    apt upgrade -y
