sudo docker build . -t capstone-gpu
optirun docker run --runtime=nvidia -p 4567:4567 -p 8888:8888 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone-gpu


