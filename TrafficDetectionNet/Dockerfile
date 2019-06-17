# Udacity capstone project dockerfile
FROM tensorflow/tensorflow:latest-gpu

# Install tools
RUN apt-get update
RUN apt-get install vim git tree -y

# install python dependencies
RUN pip install pillow lxml matplotlib cython
RUN apt-get install python-tk python-yaml -y

# install protobuf-compiler
RUN mkdir prot
RUN cd prot
RUN curl -OL https://github.com/google/protobuf/releases/download/v3.2.0/protoc-3.2.0-linux-x86_64.zip
RUN unzip protoc-3.2.0-linux-x86_64.zip -d protoc3
RUN mv protoc3/bin/* /usr/local/bin/
RUN mv protoc3/include/* /usr/local/include/
RUN ln -s /usr/local/bin/protoc /usr/bin/protoc
RUN cd ..
RUN rm prot -rf

RUN export PYTHONPATH=$PYTHONPATH:/capstone/models/research/object_detection:/capstone/models/research:/capstone/models/research/slim

# prepare environment
RUN mkdir /capstone
VOLUME ["/capstone"]
WORKDIR /capstone
