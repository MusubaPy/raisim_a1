#!/usr/bin/env bash


if [[ $1 = "--nvidia" ]] || [[ $1 = "-n" ]]
  then
    docker build -t raisim-img -f Dockerfile . \
                                  --network=host \
                                  --build-arg from=nvidia/cudagl:10.1-base-ubuntu18.04

else
    echo "[!] If you use nvidia gpu, please rebuild with -n or --nvidia argument"
    docker build -t raisim-img -f Dockerfile . \
                                  --network=host \
                                  --build-arg from=ubuntu:18.04
fi


