# Use an official Python runtime as a parent image
FROM ubuntu:focal
ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update && apt-get install  -y \
    wget \
    curl \
    tree \
    python3-pip \
    git \
    apache2-dev \
    apache2 \
    util-linux \
    openssh-server

RUN pip3 install gitpython
RUN pip3 install psutil
RUN pip3 install paramiko
RUN apt-get install -y gunicorn


RUN curl -sL https://deb.nodesource.com/setup_14.x | bash - \
&& apt-get install -y nodejs

RUN mkdir -p ~/extractor-interface
WORKDIR /root/extractor-interface


# Python requirements
COPY extractor-interface/requirements.txt .
RUN pip install -Ir requirements.txt

COPY extractor-interface/package*.json ./
RUN npm install -g parcel
RUN npm install materialize-css@next
RUN npx browserslist@latest --update-db
RUN npm install babel-plugin-transform-class-properties --save-dev
RUN npm install

COPY extractor-interface/ ./
RUN npm run build

ENV GIT_PYTHON_REFRESH quiet

# SSH CONFIGURATION
RUN useradd -rm -d /home/extractor/ -s /bin/bash -g root -G sudo -u 1001 extractor
USER extractor
WORKDIR /home/extractor

RUN mkdir -p /home/extractor/.ssh/ && \
    chmod 0700 /home/extractor/.ssh  && \
    touch /home/extractor/.ssh/authorized_keys && \
    chmod 600 /home/extractor/.ssh/authorized_keys && \
    touch /home/extractor/.ssh/config && \
    chmod 600 /home/extractor/.ssh/config

COPY --chown=extractor:root ssh_config/ /keys/
RUN cat /keys/ssh_key.pub >> /home/extractor/.ssh/authorized_keys
RUN cat /keys/config >> /home/extractor/.ssh/config
####
#ENTRYPOINT service ssh start && bash
EXPOSE 4000
USER root
WORKDIR /root/extractor-interface
CMD ["gunicorn", "--bind", "0.0.0.0:4000", "-k", "flask_sockets.worker", "wsgi:app"]
