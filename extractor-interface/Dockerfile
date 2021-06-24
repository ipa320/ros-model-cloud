# Use an official Python runtime as a parent image
FROM ubuntu:bionic
ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update && apt-get install  -y \
    wget \
    curl \
    tree \
    python-pip \
    git \
    apache2-dev \
    apache2 \
    util-linux


RUN pip install --upgrade pip
RUN pip install gitpython
RUN pip install psutil
RUN apt-get install -y gunicorn

#RUN pip install gevent
#RUN apt-get install -y python-gevent
#RUN pip install mod-wsgi

RUN curl -sL https://deb.nodesource.com/setup_14.x | bash - \
&& apt-get install -y nodejs

RUN mkdir -p ~/extractor-interface
WORKDIR /root/extractor-interface
COPY flaskr /root/extractor-interface/flaskr
COPY wsgi.py /root/extractor-interface/wsgi.py

# Python requirements
COPY requirements.txt .
RUN pip install -Ir requirements.txt

COPY package.json ./
#RUN npm install sharp parcel@^2.0.0-nightly -D
#RUN npm install --save-dev cssnano
#RUN npm install --save-dev postcss
RUN npm install -g parcel
RUN npm install
RUN npm run build

ENV GIT_PYTHON_REFRESH quiet


EXPOSE 4000
CMD ["gunicorn", "--bind", "0.0.0.0:4000", "-k", "flask_sockets.worker", "wsgi:app"]
