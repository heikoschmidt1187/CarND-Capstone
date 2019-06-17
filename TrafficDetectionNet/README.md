sudo docker build . -t capstone-gpu
docker run --runtime=nvidia -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone-gpu
