# Use an official Python runtime as a parent image
FROM ubuntu:bionic

RUN apt-get update && apt-get install -y \
    cppcheck \
    cccc \
    python-pip \
    wget \
    curl \
    netbase


RUN pip install --upgrade pip
RUN apt-get update

RUN curl -sL https://deb.nodesource.com/setup_13.x | bash - \
&& apt-get install -y nodejs

SHELL ["bash", "-c"]

RUN mkdir -p ~/extractor-interface

# Set the working directory
WORKDIR /root/extractor-interface

# Python requirements
COPY requirements.txt .
RUN pip install -Ir requirements.txt

# Node requirements
COPY package.json ./
RUN npm -v
RUN npm install npm@latest -g

COPY . ./

# Build the front-end
RUN npm install
RUN npm run build

EXPOSE 4000
CMD ["gunicorn", "--bind", "0.0.0.0:4000", "-k", "flask_sockets.worker", "wsgi:app"]
