docker run -it --name ativision \
    -v $(pwd):/home/at_vision \
    -p 2222:22 \
    ativision:latest /bin/bash