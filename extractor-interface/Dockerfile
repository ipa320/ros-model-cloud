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
    util-linux


#RUN pip3 install --upgrade pip
RUN pip3 install gitpython
RUN pip3 install psutil
RUN apt-get install -y gunicorn
#RUN pip install gevent
#RUN apt-get install -y python-gevent
#RUN pip install mod-wsgi

RUN curl -sL https://deb.nodesource.com/setup_14.x | bash - \
&& apt-get install -y nodejs

RUN mkdir -p ~/extractor-interface
WORKDIR /root/extractor-interface

#COPY flaskr /root/extractor-interface/flaskr
#COPY wsgi.py /root/extractor-interface/wsgi.py

# Python requirements
COPY requirements.txt .
RUN pip install -Ir requirements.txt


COPY package*.json ./
RUN npm install -g parcel
RUN npm install materialize-css@next
RUN npx browserslist@latest --update-db
RUN npm install babel-plugin-transform-class-properties --save-dev

RUN npm install

COPY . ./
RUN npm run build

ENV GIT_PYTHON_REFRESH quiet


EXPOSE 4000
CMD ["gunicorn", "--bind", "0.0.0.0:4000", "-k", "flask_sockets.worker", "wsgi:app"]
